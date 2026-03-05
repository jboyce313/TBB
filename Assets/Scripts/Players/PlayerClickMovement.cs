using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Tilemaps;

[RequireComponent(typeof(Rigidbody2D))]
public class PlayerClickMovement : MonoBehaviour
{
    [Header("Movement")]
    public float moveSpeed = 5f;
    public float waypointReachDistance = 0.15f;

    [Header("Pathfinding")]
    [Tooltip("Tilemap representing walkable ground tiles")]
    public Tilemap groundTilemap;
    [Tooltip("Layer(s) containing obstacle objects (e.g. other units with Collider2D). Must not include this object's layer.")]
    public LayerMask obstacleLayer;
    [Tooltip("Radius used for physics obstacle check at each tile center")]
    public float tileCheckRadius = 0.2f;

    [Tooltip("How often (seconds) to check if the current path is still clear and reroute if blocked")]
    public float rerouteInterval = 0.25f;

    [Header("Debug")]
    public bool drawPathGizmos = true;

    private Rigidbody2D rb;
    private List<Vector3> path;
    private int waypointIndex;
    private bool isMoving;
    private Vector3Int goalCell;
    private float rerouteTimer;

    // Cached so GC doesn't allocate per-frame
    private static readonly Vector3Int[] Directions =
    {
        new Vector3Int( 1,  0, 0), // right
        new Vector3Int(-1,  0, 0), // left
        new Vector3Int( 0,  1, 0), // up
        new Vector3Int( 0, -1, 0), // down
        new Vector3Int( 1,  1, 0), // diagonal
        new Vector3Int(-1,  1, 0),
        new Vector3Int( 1, -1, 0),
        new Vector3Int(-1, -1, 0),
    };

    private const float DiagonalCost = 1.41421f;

    // ---------------------------------------------------------------
    // A* Node
    // ---------------------------------------------------------------
    private sealed class Node
    {
        public readonly Vector3Int Cell;
        public Node Parent;
        public float G;           // cost from start
        public readonly float H;  // heuristic to goal (immutable after creation)
        public float F => G + H;

        public Node(Vector3Int cell, Node parent, float g, float h)
        {
            Cell   = cell;
            Parent = parent;
            G      = g;
            H      = h;
        }
    }

    // ---------------------------------------------------------------
    // Unity lifecycle
    // ---------------------------------------------------------------
    private void Awake()
    {
        rb = GetComponent<Rigidbody2D>();
        rb.constraints = RigidbodyConstraints2D.FreezeRotation;
    }

    private void Update()
    {
        if (Input.GetMouseButtonDown(0))
            HandleClick();
    }

    private void FixedUpdate()
    {
        FollowPath();

        if (isMoving)
        {
            rerouteTimer -= Time.fixedDeltaTime;
            if (rerouteTimer <= 0f)
            {
                rerouteTimer = rerouteInterval;
                CheckForReroute();
            }
        }
    }

    // ---------------------------------------------------------------
    // Input
    // ---------------------------------------------------------------
    private void HandleClick()
    {
        if (groundTilemap == null || Camera.main == null) return;

        Vector3 worldPos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
        worldPos.z = 0f;

        Vector3Int targetCell = groundTilemap.WorldToCell(worldPos);
        Vector3Int startCell  = groundTilemap.WorldToCell(transform.position);

        if (startCell == targetCell || !IsWalkable(targetCell)) return;

        List<Vector3> newPath = AStar(startCell, targetCell);
        if (newPath != null && newPath.Count > 0)
        {
            path          = newPath;
            waypointIndex = 0;
            isMoving      = true;
            goalCell      = targetCell;
            rerouteTimer  = rerouteInterval;
        }
    }

    // ---------------------------------------------------------------
    // Path following
    // ---------------------------------------------------------------
    private void FollowPath()
    {
        if (!isMoving || path == null || waypointIndex >= path.Count)
        {
            rb.linearVelocity = Vector2.zero;
            isMoving = false;
            return;
        }

        Vector2 target   = path[waypointIndex];
        Vector2 toTarget = target - rb.position;

        if (toTarget.sqrMagnitude <= waypointReachDistance * waypointReachDistance)
        {
            waypointIndex++;
            if (waypointIndex >= path.Count)
            {
                rb.linearVelocity = Vector2.zero;
                rb.MovePosition(target); // snap cleanly to final tile center
                isMoving = false;
                return;
            }
            // Refresh direction toward next waypoint
            toTarget = (Vector2)path[waypointIndex] - rb.position;
        }

        rb.linearVelocity = toTarget.normalized * moveSpeed;
    }

    // ---------------------------------------------------------------
    // Dynamic rerouting
    // ---------------------------------------------------------------
    private void CheckForReroute()
    {
        // Scan remaining waypoints for a newly blocked tile
        bool blocked = false;
        for (int i = waypointIndex; i < path.Count; i++)
        {
            if (Physics2D.OverlapCircle(path[i], tileCheckRadius, obstacleLayer) != null)
            {
                blocked = true;
                break;
            }
        }

        if (!blocked) return;

        Vector3Int currentCell = groundTilemap.WorldToCell(transform.position);
        List<Vector3> newPath  = AStar(currentCell, goalCell);

        if (newPath != null && newPath.Count > 0)
        {
            path          = newPath;
            waypointIndex = 0;
        }
        else
        {
            // Destination fully surrounded — stop and wait for next reroute tick
            rb.linearVelocity = Vector2.zero;
            isMoving = false;
        }
    }

    // ---------------------------------------------------------------
    // A* pathfinding
    // ---------------------------------------------------------------
    private List<Vector3> AStar(Vector3Int start, Vector3Int goal)
    {
        // openList sorted by F; openDict allows O(1) lookup by cell
        var openList = new List<Node>();
        var openDict = new Dictionary<Vector3Int, Node>();
        var closed   = new HashSet<Vector3Int>();

        var startNode = new Node(start, null, 0f, Heuristic(start, goal));
        openList.Add(startNode);
        openDict[start] = startNode;

        while (openList.Count > 0)
        {
            Node current = PopLowestF(openList);
            openDict.Remove(current.Cell);

            if (current.Cell == goal)
                return BuildPath(current);

            closed.Add(current.Cell);

            foreach (Vector3Int dir in Directions)
            {
                Vector3Int neighborCell = current.Cell + dir;

                if (closed.Contains(neighborCell)) continue;
                if (!IsWalkable(neighborCell))     continue;

                // Prevent diagonal movement through a blocked corner
                bool isDiagonal = dir.x != 0 && dir.y != 0;
                if (isDiagonal)
                {
                    if (!IsWalkable(current.Cell + new Vector3Int(dir.x, 0, 0))) continue;
                    if (!IsWalkable(current.Cell + new Vector3Int(0, dir.y, 0))) continue;
                }

                float moveCost    = isDiagonal ? DiagonalCost : 1f;
                float tentativeG  = current.G + moveCost;

                if (openDict.TryGetValue(neighborCell, out Node existing))
                {
                    if (tentativeG < existing.G)
                    {
                        existing.G      = tentativeG;
                        existing.Parent = current;
                    }
                }
                else
                {
                    var neighbor = new Node(neighborCell, current, tentativeG, Heuristic(neighborCell, goal));
                    openList.Add(neighbor);
                    openDict[neighborCell] = neighbor;
                }
            }
        }

        return null; // no path found
    }

    // Swap-remove to avoid O(n) list shifts
    private static Node PopLowestF(List<Node> list)
    {
        int bestIdx = 0;
        for (int i = 1; i < list.Count; i++)
        {
            Node a = list[i], b = list[bestIdx];
            if (a.F < b.F || (a.F == b.F && a.H < b.H))
                bestIdx = i;
        }
        Node best = list[bestIdx];
        list[bestIdx] = list[list.Count - 1];
        list.RemoveAt(list.Count - 1);
        return best;
    }

    // Octile distance — correct heuristic for 8-directional grid movement
    private static float Heuristic(Vector3Int a, Vector3Int b)
    {
        int dx = Mathf.Abs(a.x - b.x);
        int dy = Mathf.Abs(a.y - b.y);
        return Mathf.Max(dx, dy) + (DiagonalCost - 1f) * Mathf.Min(dx, dy);
    }

    private bool IsWalkable(Vector3Int cell)
    {
        if (!groundTilemap.HasTile(cell)) return false;

        Vector3 worldCenter = groundTilemap.GetCellCenterWorld(cell);
        return Physics2D.OverlapCircle(worldCenter, tileCheckRadius, obstacleLayer) == null;
    }

    private List<Vector3> BuildPath(Node endNode)
    {
        var result = new List<Vector3>();
        for (Node n = endNode; n != null; n = n.Parent)
            result.Add(groundTilemap.GetCellCenterWorld(n.Cell));
        result.Reverse();
        return result;
    }

    // ---------------------------------------------------------------
    // Gizmos
    // ---------------------------------------------------------------
    private void OnDrawGizmos()
    {
        if (!drawPathGizmos || path == null) return;

        Gizmos.color = Color.cyan;
        for (int i = waypointIndex; i < path.Count; i++)
        {
            Gizmos.DrawSphere(path[i], 0.07f);
            if (i + 1 < path.Count)
                Gizmos.DrawLine(path[i], path[i + 1]);
        }

        if (waypointIndex < path.Count)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(path[waypointIndex], 0.1f);
        }
    }
}
