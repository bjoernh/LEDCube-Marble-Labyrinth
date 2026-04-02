# Marble Labyrinth — Implementation Plan

## Context
We're implementing a physics-based marble labyrinth puzzle game for the LEDCube platform. The player tilts the physical LED cube (or a phone in the simulator) to guide a marble through a maze spanning all 6 faces. The project builds on the LEDCube-App-Template, which provides a `CubeApplication` base class with a frame-based `loop()` method and drawing API.

The implementation will be split into **4 incremental PRs**, each building on the last. Dependencies: nlohmann/json vendored as a single header, Catch2 v3 as a git submodule.

---

## PR 1: Spec + Project Skeleton + Geometry Module

### Files to create/modify:

1. **`spec/game-spec.md`** — Save the full game specification document
2. **`src/main.cpp`** — Change `#include "AppTemplate.h"` to `#include "MarbleMazeApp.h"`, instantiate `MarbleMazeApp`
3. **`src/MarbleMazeApp.h`** / **`src/MarbleMazeApp.cpp`** — Minimal skeleton inheriting `CubeApplication`, empty `loop()` that just calls `clear()` + `render()` + returns true
4. **`src/geometry/CubeFace.h`** / **`src/geometry/CubeFace.cpp`** — Single face with UV-to-world transforms
5. **`src/geometry/CubeGeometry.h`** / **`src/geometry/CubeGeometry.cpp`** — All 6 faces + face-transition logic
6. **`CMakeLists.txt`** — Rename project to `MarbleLabyrinth`, update executable name, add new source files
7. **`third_party/nlohmann/json.hpp`** — Vendored single header (downloaded)
8. **`third_party/catch2/`** — Catch2 v3 git submodule
9. **`tests/GeometryTests.cpp`** — Unit tests for UV-to-world round trips, face transitions
10. Delete **`src/AppTemplate.h`** and **`src/AppTemplate.cpp`**

### CubeFace design:
```cpp
struct CubeFace {
    int index;                    // 0-5 mapping to ScreenNumber
    Eigen::Vector3f origin;       // corner of face in world space
    Eigen::Vector3f normal;       // outward-pointing normal
    Eigen::Vector3f uAxis;        // tangent U direction (unit vector)
    Eigen::Vector3f vAxis;        // tangent V direction (unit vector)

    Eigen::Vector3f uvToWorld(float u, float v) const;
    Eigen::Vector2f worldToUV(const Eigen::Vector3f& worldPos) const;
    float distanceToPlane(const Eigen::Vector3f& point) const;
    bool containsUV(float u, float v, float margin = 0.0f) const;
};
```

### CubeGeometry design:
```cpp
class CubeGeometry {
    std::array<CubeFace, 6> faces_;
public:
    CubeGeometry();  // initializes all 6 faces of unit cube centered at origin
    const CubeFace& face(int index) const;

    // Face transition: given world pos outside current face, find new face
    struct TransitionResult {
        int faceIndex;
        Eigen::Vector2f uv;
        Eigen::Vector2f velocity;  // transformed to new face UV
    };
    TransitionResult findFace(const Eigen::Vector3f& worldPos) const;
    TransitionResult transition(int currentFace, const Eigen::Vector2f& uvPos,
                                const Eigen::Vector2f& uvVel) const;
};
```

### Face definitions (unit cube centered at origin, half-extent = 0.5):
| Face   | ScreenNumber | Normal    | Origin            | U-axis    | V-axis    |
|--------|-------------|-----------|-------------------|-----------|-----------|
| front  | 0           | (0,0,+1)  | (-0.5,-0.5,+0.5) | (+1,0,0)  | (0,+1,0) |
| right  | 1           | (+1,0,0)  | (+0.5,-0.5,+0.5) | (0,0,-1)  | (0,+1,0) |
| back   | 2           | (0,0,-1)  | (+0.5,-0.5,-0.5) | (-1,0,0)  | (0,+1,0) |
| left   | 3           | (-1,0,0)  | (-0.5,-0.5,-0.5) | (0,0,+1)  | (0,+1,0) |
| top    | 4           | (0,+1,0)  | (-0.5,+0.5,+0.5) | (+1,0,0)  | (0,0,-1) |
| bottom | 5           | (0,-1,0)  | (-0.5,-0.5,-0.5) | (+1,0,0)  | (0,0,+1) |

UV axes chosen so that when looking at each face from outside the cube, U goes right and V goes up (consistent handedness).

### Face transition algorithm:
1. Convert current UV pos to world: `worldPos = face.uvToWorld(u, v)`
2. For each of 6 faces, compute `worldToUV(worldPos)`
3. Select face where UV is closest to [0,1] range (smallest max overshoot)
4. Transform velocity: decompose `worldVel = uvVel.x * currentFace.uAxis + uvVel.y * currentFace.vAxis` into new face's U/V components via dot products

### Geometry tests:
- UV-to-world-to-UV round trip for all 6 faces at corners (0,0), (1,0), (0,1), (1,1)
- Face transition across all 12 edges (e.g., front UV=(1.05, 0.5) should land on right face)
- Face transition at all 8 corners
- Velocity transformation preserves magnitude

---

## PR 2: Physics + IMU + Ball

### Files to create/modify:

1. **`src/imu/IImu.h`** — Abstract interface
2. **`src/imu/Mpu6050Imu.h`** / **`src/imu/Mpu6050Imu.cpp`** — Framework adapter
3. **`src/physics/Ball.h`** / **`src/physics/Ball.cpp`** — Ball state struct
4. **`src/physics/PhysicsEngine.h`** / **`src/physics/PhysicsEngine.cpp`** — Per-frame update
5. **`tests/PhysicsTests.cpp`** — Physics unit tests
6. **`CMakeLists.txt`** — Add new source files

### IImu interface:
```cpp
class IImu {
public:
    virtual ~IImu() = default;
    virtual Eigen::Vector3f getGravityDirection() = 0;
};
```

### Mpu6050Imu:
```cpp
class Mpu6050Imu : public IImu {
    Mpu6050 mpu_;
    Eigen::Quaternionf calibrationOffset_;
    bool calibrated_ = false;
public:
    void init();
    void calibrate();  // capture current orientation as reference
    Eigen::Vector3f getGravityDirection() override;
};
```

### Ball state:
```cpp
struct Ball {
    int faceIndex = 0;
    Eigen::Vector2f position{0.5f, 0.5f};  // UV coords
    Eigen::Vector2f velocity{0.0f, 0.0f};  // UV/s
    float radius = 0.02f;                    // UV units (~1.3 pixels)
    float friction = 0.98f;                  // per-frame multiplier
};
```

### PhysicsEngine per-frame update:
```cpp
class PhysicsEngine {
    CubeGeometry& geometry_;
    float gravityStrength_ = 1.0f;

public:
    void update(Ball& ball, const Eigen::Vector3f& gravityDir, float dt,
                std::function<CollisionResult(int face, Eigen::Vector2f pos,
                              Eigen::Vector2f proposedPos, float radius)> collisionFn);
};
```

Steps in `update()`:
1. Project gravity onto current face plane: `gravWorld - (gravWorld.dot(normal)) * normal`
2. Decompose into UV: `accelU = tangential.dot(uAxis)`, `accelV = tangential.dot(vAxis)`
3. `velocity += accel * gravityStrength * dt`
4. `velocity *= friction`
5. `proposedPos = position + velocity * dt`
6. Call collision function to get corrected pos & velocity
7. If corrected pos outside [0,1], do face transition via geometry
8. Clamp to [0+radius, 1-radius]

### Physics tests:
- Gravity projection: gravity aligned with face normal produces zero acceleration
- Gravity along U axis produces acceleration only in U
- Friction decay over N frames
- Velocity reflection off a wall normal

---

## PR 3: Maze + Collision + JSON Loading

### Files to create/modify:

1. **`src/maze/WallSegment.h`** — Simple struct
2. **`src/maze/MazeLayout.h`** / **`src/maze/MazeLayout.cpp`** — Wall collections + JSON loader
3. **`src/maze/CollisionSolver.h`** / **`src/maze/CollisionSolver.cpp`** — Circle vs line segment collision
4. **`mazes/maze_01.json`** — Sample playable maze
5. **`tests/MazeTests.cpp`** — Collision resolution tests
6. **`CMakeLists.txt`** — Add new source files, add `third_party/nlohmann` to include path

### WallSegment:
```cpp
struct WallSegment {
    Eigen::Vector2f a, b;  // UV endpoints
};
```

### MazeLayout:
```cpp
struct FaceMaze {
    std::vector<WallSegment> walls;
    Eigen::Vector2f startPosition;  // ball start UV (only on start face)
    Eigen::Vector2f goalPosition;   // goal UV (only on goal face)
    float goalRadius;
};

class MazeLayout {
    std::array<FaceMaze, 6> faces_;
    int startFace_ = 0;
    int goalFace_ = 0;
public:
    static MazeLayout loadFromFile(const std::string& path);
    const FaceMaze& face(int index) const;
    int startFace() const;
    int goalFace() const;
    Eigen::Vector2f startPosition() const;
    Eigen::Vector2f goalPosition() const;
    float goalRadius() const;
};
```

### JSON format for maze files:
```json
{
  "startFace": 0,
  "startPosition": [0.1, 0.1],
  "goalFace": 2,
  "goalPosition": [0.9, 0.9],
  "goalRadius": 0.05,
  "faces": [
    {
      "walls": [
        [[0.0, 0.3], [0.7, 0.3]],
        [[0.3, 0.3], [0.3, 0.7]]
      ]
    }
  ]
}
```

### CollisionSolver algorithm:
```cpp
struct CollisionResult {
    Eigen::Vector2f position;
    Eigen::Vector2f velocity;
};

class CollisionSolver {
public:
    static CollisionResult resolve(
        const Eigen::Vector2f& currentPos,
        const Eigen::Vector2f& proposedPos,
        const Eigen::Vector2f& velocity,
        float ballRadius,
        const std::vector<WallSegment>& walls);
};
```

For each wall segment:
1. Find closest point on segment to ball center (clamp projection to segment extent)
2. Compute distance from ball center to closest point
3. If distance < ballRadius:
   - Compute wall normal (perpendicular to segment direction, pointing toward ball)
   - Push ball out: `position += normal * (ballRadius - distance)`
   - Reflect velocity: `velocity -= 2 * velocity.dot(normal) * normal`
   - Apply damping to reflected component (e.g., 0.5x) for energy loss

### Collision tests:
- Ball approaching wall head-on: reflected correctly
- Ball sliding along wall: no false collision
- Ball hitting segment endpoint: correct resolution
- Ball outside segment extent: no collision

---

## PR 4: Rendering + Game Logic + Integration

### Files to create/modify:

1. **`src/renderer/MazeRenderer.h`** / **`src/renderer/MazeRenderer.cpp`** — Drawing walls, ball, goal
2. **`src/MarbleMazeApp.h`** / **`src/MarbleMazeApp.cpp`** — Full game integration
3. **`CMakeLists.txt`** — Add renderer source files

### MazeRenderer:
```cpp
class MazeRenderer {
    CubeApplication& app_;  // reference to call drawing methods
    CubeGeometry& geometry_;

public:
    void drawWalls(const MazeLayout& maze, Color wallColor);
    void drawBall(const Ball& ball, Color ballColor);
    void drawGoal(const MazeLayout& maze, Color goalColor);
    void drawWinAnimation(float progress);  // 0.0-1.0 animation progress
};
```

- `drawWalls()`: For each face, for each wall segment, scale UV by CUBEMAXINDEX, call `drawLine2D(screenNumber, x0, y0, x1, y1, color)`
- `drawBall()`: Convert ball UV to world via `geometry.face(ball.faceIndex).uvToWorld()`, scale by CUBEMAXINDEX for 3D coords, call `setPixelSmooth3D()`
- `drawGoal()`: `drawCircle2D()` on goal face at goal position scaled to pixels

### MarbleMazeApp full integration:
```cpp
class MarbleMazeApp : public CubeApplication {
    CubeGeometry geometry_;
    Mpu6050Imu imu_;
    PhysicsEngine physics_;
    MazeRenderer renderer_;
    Ball ball_;
    std::vector<MazeLayout> levels_;
    int currentLevel_ = 0;

    // Game state
    enum class State { Calibrating, Playing, Win };
    State state_ = State::Calibrating;
    float calibrationTimer_ = 0.0f;
    float winTimer_ = 0.0f;
    float levelTimer_ = 0.0f;  // for reset detection

public:
    MarbleMazeApp(std::string serverUri);
    bool loop() override;

private:
    void loadLevels();
    void startLevel(int index);
    void resetBall();
    bool checkWin();
};
```

### Constructor registers AnimationParams:
- `params.registerFloat("gravity", "Gravity Strength", 0.1f, 5.0f, 1.0f, 0.1f, "Physics")`
- `params.registerFloat("friction", "Friction", 0.9f, 1.0f, 0.98f, 0.01f, "Physics")`
- `params.registerFloat("ballRadius", "Ball Radius", 0.01f, 0.05f, 0.02f, 0.005f, "Physics")`
- `params.registerFloat("resetThreshold", "Reset Threshold", 0.01f, 0.2f, 0.05f, 0.01f, "Game")`
- `params.registerBool("resetBall", "Reset Ball", false, "Game")` (button-style param)

### loop() flow:
1. Read tunable params
2. Read IMU gravity direction
3. **Calibrating state**: accumulate gravity samples for ~2s, compute offset quaternion, transition to Playing
4. **Playing state**:
   - `physics_.update(ball_, gravity, dt, collisionSolver)`
   - Check reset conditions (level timer for cube held level, or resetBall param)
   - Check win condition (distance from ball to goal < goalRadius)
   - `clear()` then `renderer_.drawWalls()` then `renderer_.drawBall()` then `renderer_.drawGoal()` then `render()`
5. **Win state**: Flash animation for ~2s, then advance to next level

### Level loading:
- Scan `mazes/` directory for `maze_*.json` files, sort alphabetically
- Load all at startup into `levels_` vector
- Cycle through with wrap-around

---

## CMakeLists.txt final state:

```cmake
project(MarbleLabyrinth VERSION 1.0.0)

# ... existing find_package calls unchanged ...

include_directories(${EIGEN3_INCLUDE_DIRS} ${IMLIB2_INCLUDE_DIRS} third_party/nlohmann)

set(MAINSRC
    src/main.cpp
    src/MarbleMazeApp.cpp src/MarbleMazeApp.h
    src/geometry/CubeFace.cpp src/geometry/CubeFace.h
    src/geometry/CubeGeometry.cpp src/geometry/CubeGeometry.h
    src/physics/Ball.cpp src/physics/Ball.h
    src/physics/PhysicsEngine.cpp src/physics/PhysicsEngine.h
    src/maze/WallSegment.h
    src/maze/MazeLayout.cpp src/maze/MazeLayout.h
    src/maze/CollisionSolver.cpp src/maze/CollisionSolver.h
    src/imu/IImu.h
    src/imu/Mpu6050Imu.cpp src/imu/Mpu6050Imu.h
    src/renderer/MazeRenderer.cpp src/renderer/MazeRenderer.h
)

add_executable(MarbleLabyrinth ${MAINSRC})
target_link_libraries(MarbleLabyrinth ${MAINLIBS} ${Boost_LIBRARIES})

# Tests
add_subdirectory(third_party/catch2)
add_executable(MarbleMazeTests
    tests/GeometryTests.cpp
    tests/PhysicsTests.cpp
    tests/MazeTests.cpp
    src/geometry/CubeFace.cpp
    src/geometry/CubeGeometry.cpp
    src/physics/PhysicsEngine.cpp
    src/maze/CollisionSolver.cpp
    src/maze/MazeLayout.cpp
)
target_include_directories(MarbleMazeTests PRIVATE src third_party/nlohmann)
target_link_libraries(MarbleMazeTests Catch2::Catch2WithMain)
```

---

## Verification

### Per-PR testing:
- **PR 1**: `cmake --build . && ./MarbleLabyrinth` connects and shows empty black cube. `./MarbleMazeTests` passes geometry tests.
- **PR 2**: Physics tests pass. Ball responds to simulated gravity in tests.
- **PR 3**: Maze JSON loads. Collision tests pass. Integration test: ball doesn't pass through walls.
- **PR 4**: Full game runs in simulator. Open `https://<host>:5173` on phone, tilt to control. Ball navigates maze, transitions between faces, reaches goal, triggers win animation.

### End-to-end:
1. Start matrixserver simulator: `docker run -it --rm -p 2017:2017 -p 1337:1337 -p 5173:5173 ghcr.io/bjoernh/matrixserver-simulator:latest`
2. Build and run: `mkdir build && cd build && cmake .. && make && ./MarbleLabyrinth`
3. Open `https://localhost:5173` in browser to see 3D visualization
4. Open same URL on phone for tilt control
5. Verify: walls render on all faces, ball moves with tilt, collisions work, face transitions are smooth, reaching goal triggers win animation and level advance
