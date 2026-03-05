# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**TBB** is a 2D top-down basketball game built in Unity. The current prototype has click-to-move player movement and AI defenders. It uses Unity's Universal Render Pipeline (URP) 2D renderer. The court is pure background art — there is no tilemap.

## Unity Version & Rendering

- Unity project with URP 2D (see `Assets/Settings/` for render pipeline assets)
- 2D physics (Rigidbody2D, Physics2D) throughout — do not use 3D physics components
- No Tilemap — the court is background art only; walkability is determined purely by physics colliders and a configurable grid boundary
- Pixel art assets from PixelArtStudio (SkeletonsPack / CommonSoldier) with directional animations (8 directions: Top, Bot, Left, Right, LeftTop, LeftBot, RightTop, RightBot)

## Project Structure

```
Assets/
  Scripts/
    Players/    - Player-controlled character scripts
    Defenders/  - AI defender scripts
  Scenes/
    TestGame.unity  - Main development scene
  Unity_Assets/
    PixelArtStudio/SkeletonsPack/CommonSoldier/  - Animated skeleton enemy sprites/prefabs
    Free 32x32 Isometric Tileset Pack/           - Tilemap tiles
```

## Core Scripts

### `Assets/Scripts/Players/PlayerClickMovement.cs`
Click-to-move controller with self-contained A* pathfinding over a virtual grid (no Tilemap).
- The grid is defined by `gridOrigin` (world-space bottom-left), `gridSize` (total extent), and `cellSize` — set these in the Inspector to match the court bounds
- A cell is walkable if it is within bounds AND has no physics collider on `obstacleLayer` at its center; `obstacleLayer` must exclude the player's own layer
- Supports 8-directional movement with diagonal corner-cutting prevention
- Dynamically reroutes every `rerouteInterval` seconds if the current path becomes blocked
- Enable `drawPathGizmos` to visualise the active path; select the object in the Editor to see the grid boundary as a green wire rectangle

### `Assets/Scripts/Defenders/Defender.cs`
Steering-behaviour AI that positions itself between a player and a hoop.
- Requires `player` and `hoop` Transform references set in the Inspector
- Guard position = `player.position + (player→hoop direction) * guardDistance`
- Uses `Physics2D.OverlapCircleNonAlloc` with a pre-allocated 16-element buffer to separate from nearby obstacles (other defenders, players) — avoidance is a linear falloff weighted by `avoidanceWeight`
- `obstacleLayer` should include all units that defenders should separate from

## Development Workflow

There are no CLI build/test commands — development is done entirely through the Unity Editor:

1. Open the project in Unity Hub
2. Load `Assets/Scenes/TestGame.unity` for the main prototype scene
3. Enter Play mode to test; path gizmos are visible in Scene view during play when `drawPathGizmos` is enabled on the player

## Key Conventions

- All movement uses `Rigidbody2D.linearVelocity` (Unity 6 API) — not `velocity`
- Physics is applied in `FixedUpdate`; input is read in `Update`
- `[RequireComponent]` attributes enforce component dependencies on all character scripts
- Tilemap cell coordinates (`Vector3Int`) are used internally for pathfinding; world positions (`Vector3`) are used for actual movement
