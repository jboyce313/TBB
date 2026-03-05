using UnityEngine;

[RequireComponent(typeof(Rigidbody2D))]
public class Defender : MonoBehaviour
{
    [Header("References")]
    [Tooltip("The player this defender is guarding")]
    public Transform player;
    [Tooltip("The hoop the defender is protecting")]
    public Transform hoop;

    [Header("Positioning")]
    [Tooltip("How far from the player the defender stands, toward the hoop (world units)")]
    public float guardDistance = 1f;

    [Header("Movement")]
    public float moveSpeed = 4f;
    public float stoppingDistance = 0.05f;

    [Header("Obstacle Avoidance")]
    [Tooltip("Layer(s) containing objects to avoid (other defenders, players, etc.)")]
    public LayerMask obstacleLayer;
    [Tooltip("Radius within which nearby obstacles exert a separation force")]
    public float avoidanceRadius = 1.5f;
    [Tooltip("How strongly avoidance steers away from obstacles relative to the seek force")]
    public float avoidanceWeight = 2f;

    private Rigidbody2D rb;
    private Collider2D ownCollider;

    // Pre-allocated buffer — avoids GC allocation each FixedUpdate
    private readonly Collider2D[] _overlapBuffer = new Collider2D[16];

    private void Awake()
    {
        rb          = GetComponent<Rigidbody2D>();
        ownCollider = GetComponent<Collider2D>();
        rb.constraints = RigidbodyConstraints2D.FreezeRotation;
    }

    private void FixedUpdate()
    {
        if (player == null || hoop == null) return;

        Vector2 target   = GetGuardPosition();
        Vector2 toTarget = target - rb.position;
        Vector2 seek     = toTarget.sqrMagnitude > 0.001f ? toTarget.normalized : Vector2.zero;

        Vector2 avoidance = ComputeAvoidance();
        Vector2 combined  = seek + avoidance * avoidanceWeight;

        if (combined.sqrMagnitude < 0.001f)
        {
            rb.linearVelocity = Vector2.zero;
            return;
        }

        // Only stop if both close to target AND nothing is pushing us away
        bool nearTarget      = toTarget.sqrMagnitude <= stoppingDistance * stoppingDistance;
        bool avoidanceActive = avoidance.sqrMagnitude > 0.001f;

        if (nearTarget && !avoidanceActive)
        {
            rb.linearVelocity = Vector2.zero;
            return;
        }

        rb.linearVelocity = combined.normalized * moveSpeed;
    }

    // Stand guardDistance units away from the player, along the player→hoop line
    private Vector2 GetGuardPosition()
    {
        Vector2 playerPos    = player.position;
        Vector2 hoopPos      = hoop.position;
        Vector2 playerToHoop = hoopPos - playerPos;

        if (playerToHoop.sqrMagnitude < 0.001f)
            return playerPos;

        return playerPos + playerToHoop.normalized * guardDistance;
    }

    // Returns a separation vector pushing away from all nearby obstacles
    private Vector2 ComputeAvoidance()
    {
        int count     = Physics2D.OverlapCircleNonAlloc(rb.position, avoidanceRadius, _overlapBuffer, obstacleLayer);
        Vector2 total = Vector2.zero;

        for (int i = 0; i < count; i++)
        {
            Collider2D col = _overlapBuffer[i];
            if (col == ownCollider) continue; // skip self

            Vector2 away = rb.position - (Vector2)col.bounds.center;
            float dist   = away.magnitude;
            if (dist < 0.001f) continue;

            // Linear falloff: full strength at dist=0, zero at dist=avoidanceRadius
            float strength = 1f - (dist / avoidanceRadius);
            total += away.normalized * strength;
        }

        return total;
    }
}
