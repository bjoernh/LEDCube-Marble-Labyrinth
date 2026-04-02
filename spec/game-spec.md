# Marble Labyrinth -- Game Specification (v2)

**Target Platform:** LEDCube via bjoernh/matrixserver
**Language:** C++17
**Architecture:** matrixserver Client Application, based on bjoernh/LEDCube-App-Template

## 1. Overview

Marble Labyrinth is a physics-based puzzle game running as a CubeApplication on the matrixserver platform. The player tilts the physical LED cube to guide a marble through a maze that extends across all six faces of the cube. The cube's embedded MPU-6050 IMU provides orientation data; the game responds by simulating gravity and ball physics on the cube surface.

## 2. System Architecture

### 2.1 Role within matrixserver

The game is a standalone matrixserver client application built from the LEDCube-App-Template. It connects to a running server_simulator or hardware server on TCP port 2017, inherits from CubeApplication, and drives pixel output exclusively via the drawing API provided by the application library. The server handles all rendering backend concerns transparently.

### 2.2 Source Structure

The application follows the flat source layout of the template. All files reside under src/, organized into subdirectories for logical grouping. There is a single CMakeLists.txt at the project root, which lists all source files explicitly. No subdirectory-level CMakeLists.txt files are introduced.

The logical modules and their source locations are as follows:

**src/ (top level)** -- MarbleMazeApp.h and MarbleMazeApp.cpp form the top-level application class, plus the unchanged main.cpp from the template.

**src/geometry/** -- Cube surface model. Defines the six faces as embedded 3D objects with world-space normal vectors, origin points, and UV axis vectors. Provides coordinate transformations between local face UV space and 3D world space in both directions. Has no dependencies on physics or rendering.

**src/physics/** -- Ball simulation. Owns the ball state (current face, UV position, UV velocity) and advances it each frame. Reads gravity from the IMU, applies friction and velocity integration, delegates collision detection to the maze module, and triggers face transitions through the geometry module.

**src/maze/** -- Maze data and collision. Stores wall geometry as collections of 2D line segments in face-local UV coordinates, one set per face. Exposes a function that takes the ball's current and proposed position and returns the resolved position and reflected velocity after all wall collisions. Loads maze layouts from JSON files using nlohmann/json.

**src/imu/** -- IMU interface. Wraps the framework's Mpu6050 class behind a thin abstract interface IImu, providing a testable seam and a clean boundary between the physics engine and hardware specifics. A single concrete implementation adapts Mpu6050::getAcceleration() into the gravity vector expected by the physics engine.

### 2.3 Integration Point

The top-level MarbleMazeApp class inherits from CubeApplication, overrides the loop() virtual method as the per-frame entry point, and holds instances of all game modules. The loop() method returns true each frame to continue execution and false only to terminate. render() is called at the end of every frame.

## 3. Surface Model

### 3.1 Face Definition

Each of the six cube faces is defined by three orthonormal vectors in 3D world space: a surface normal pointing outward, and two tangent vectors forming the local U and V axes. Together with a face origin point, these define an affine mapping between the face's 2D UV coordinate space (ranging from 0.0 to 1.0) and 3D world positions on the cube surface. The cube is modeled as a unit cube centered at the world origin.

### 3.2 Coordinate Conventions

UV coordinates on each face run from (0, 0) at the face origin corner to (1, 1) at the diagonally opposite corner. The orientation of UV axes on each face must be chosen consistently so that all six faces share a coherent handedness when viewed from outside the cube. The mapping from UV to pixel coordinates scales by CUBEMAXINDEX (63), which is the constant defined in the framework for the maximum valid pixel index on a 64x64 face.

### 3.3 Face Transitions

Face transitions are handled geometrically rather than through a lookup table. When the ball's proposed world position falls outside the current face's valid UV range, the geometry module projects that world position onto each of the six face planes and selects the face whose plane the point most closely lies on while remaining within a small margin of the face boundary. Velocity is transformed to the new face by decomposing the 3D velocity vector -- expressed in world coordinates -- into components along the new face's U and V axes. This eliminates the need for per-edge rotation tables and handles cube corners correctly by construction.

## 4. Physics Simulation

### 4.1 State Representation

The ball state consists of its current face index, its UV position on that face, its UV velocity vector, a scalar radius (in face UV units), and a friction coefficient. All physics quantities are expressed in face-local UV coordinates except during face transitions, where world-space vectors serve as the intermediate representation.

### 4.2 Per-Frame Update

Each frame, the physics module performs the following steps in order. First, the gravity vector in world space is obtained from the IMU module and projected onto the tangential plane of the current face by subtracting its component along the face normal. The result is decomposed into U and V components to yield a 2D acceleration vector. Second, the UV velocity is updated by integrating the acceleration over the frame timestep, then scaled by the friction coefficient. Third, the proposed new UV position is computed. Fourth, the maze module is called to resolve all wall collisions, returning a corrected position and reflected velocity. Fifth, if the corrected position lies outside the face's UV range, the geometry module performs a face transition. Sixth, the ball position is clamped to the valid UV range as a numerical safety measure.

### 4.3 Timestep

The physics simulation runs at the target frame rate configured in the CubeApplication constructor. The timestep passed to the integrator is the inverse of that frame rate, yielding consistent behavior regardless of the chosen FPS.

## 5. Maze Representation

### 5.1 Wall Geometry

Walls are stored as ordered lists of 2D line segments, one list per face. Each segment is defined by two UV endpoint coordinates. Walls are authored in UV space independently per face and are not required to be aligned to any grid.

### 5.2 Cross-Edge Walls

A wall that conceptually continues across a cube edge is represented as two separate segments: one on each face, each ending exactly at the edge (UV value 0 or 1). The geometric face transition ensures that the ball's trajectory is continuous across the edge without any explicit cross-face wall representation.

### 5.3 Data Format

Maze layouts are loaded from JSON files at startup, one file per maze level. The file contains an array of six face entries, each with an array of wall segments as pairs of UV endpoint coordinates. This format is resolution-independent and works regardless of the CUBESIZE constant. JSON parsing uses the nlohmann/json library, added as a vendored single header under third_party/nlohmann/.

### 5.4 Collision Resolution

The maze module tests the ball (modeled as a circle with a radius in UV space) against each wall segment on the current face. For each segment, it computes the signed distance from the ball center to the infinite line containing the segment, checks that the projection falls within the segment extents, and if the ball is closer than its radius, pushes the ball back along the wall normal and reflects the velocity component along that normal.

## 6. IMU Integration

### 6.1 Abstract Interface

The physics engine reads gravity through the IImu interface, which exposes a single method returning the current gravity direction as a normalized 3D world-space vector. This interface has one concrete implementation that adapts the Mpu6050 class from the framework.

### 6.2 Hardware Implementation

The concrete implementation calls Mpu6050::getAcceleration(), which returns an Eigen::Vector3f in g-units representing the accelerometer measurement. This vector is normalized to produce a unit gravity direction. Mpu6050::init() is called from the MarbleMazeApp constructor to start the background I2C polling thread.

### 6.3 Simulator Support

No separate simulator stub class is required. The CubeWebapp's smartphone mode forwards the phone's built-in accelerometer data to the matrixserver over WebSocket, where it becomes available to the Mpu6050 class through the same static data pathway used for physical hardware. During development, opening https://<host>:5173 on a smartphone and tilting it provides full tilt-control input to the simulator without any code changes.

### 6.4 Calibration

At startup, during a brief calibration phase while the cube is held still, the measured gravity direction defines the world-space "down" reference. This offset quaternion is stored and applied to all subsequent IMU readings before they are passed to the physics engine.

## 7. Rendering

### 7.1 Coordinate Mapping

For each game entity, the rendering step converts UV position on a face to a 2D pixel coordinate by scaling by CUBEMAXINDEX. Walls are rasterized per face using drawLine2D(ScreenNumber, x0, y0, x1, y1, Color), where the endpoints are the UV coordinates of each wall segment scaled to pixel space. The ball is drawn using setPixelSmooth3D applied to the 3D world position obtained from the geometry module's UV-to-world transform followed by getPointOnScreen, to support sub-pixel positioning for smooth movement.

### 7.2 Frame Lifecycle

Each frame begins with clear() to reset all faces to the background color. Walls are drawn for all six faces, then the ball is drawn, then the goal marker is drawn. The frame ends with render().

### 7.3 Color Scheme

The color palette is configurable at runtime via AnimationParams registered in the MarbleMazeApp constructor, with the following defaults: background black, walls white, ball yellow, goal marker green. All colors are expressed as Color values using the framework's Color class.

### 7.4 Visual Feedback

When the ball reaches the goal area, the game enters a win state and displays a brief flashing animation before loading the next level. The fade() method may be used during this animation to create a glow effect.

## 8. Game Logic

### 8.1 Level Progression

Levels are numbered sequentially and correspond to JSON maze files named maze_01.json, maze_02.json, etc., placed in a mazes/ directory relative to the application binary. All available files are loaded at startup, and the game cycles through them in order, looping back to the first after the last.

### 8.2 Ball Reset

If the player holds the cube level for two seconds (gravity tangential component below a threshold), or explicitly resets via a registered button parameter, the ball is returned to the start position of the current level with zero velocity.

### 8.3 Tunable Parameters

The following values are registered via params in the constructor and are adjustable from the CubeWebapp Parameter Configuration Panel without recompilation: gravity strength scalar, friction coefficient, ball radius, level reset threshold, and all color values.

## 9. Build Integration

### 9.1 CMakeLists.txt Changes

The root CMakeLists.txt inherits from the template. The changes required are: rename the project, update the executable name, add all source files from src/geometry/, src/physics/, src/maze/, and src/imu/ to the MAINSRC list, and add the nlohmann/json include directory to include_directories. No additional find_package calls are needed beyond those already present in the template, as all required libraries (Eigen3, Boost, Protobuf, libmatrixapplication) are already located by the template. The C++ standard remains C++17 as set by the template.

### 9.2 JSON Dependency

nlohmann/json is vendored as a single header file under third_party/nlohmann/ and included as a header-only library. No additional CMake find_package or target_link_libraries entry is required.

### 9.3 Testing

Unit tests for the geometry module (UV-to-world and world-to-UV round trips, face transition correctness across all 12 edges and 8 corners), the physics module (gravity projection, friction decay, velocity reflection), and the maze module (wall collision resolution, segment endpoint cases) are placed in a tests/ directory. A separate add_executable(MarbleMazeTests ...) target is added in the root CMakeLists.txt using the Catch2 v3 library added as a git submodule under third_party/catch2/.

## 10. File and Class Overview

| File                               | Class             | Responsibility                                     |
|------------------------------------|-------------------|---------------------------------------------------|
| `src/main.cpp`                     | --                | Entry point (from template, unchanged)             |
| `src/MarbleMazeApp.h/.cpp`         | `MarbleMazeApp`   | Top-level app, owns all modules, drives `loop()`   |
| `src/geometry/CubeFace.h/.cpp`     | `CubeFace`        | Single face: UV-to-world transforms                |
| `src/geometry/CubeGeometry.h/.cpp` | `CubeGeometry`    | All six faces, face-transition logic               |
| `src/physics/Ball.h/.cpp`          | `Ball`            | Ball state: face, UV position, UV velocity, radius |
| `src/physics/PhysicsEngine.h/.cpp` | `PhysicsEngine`   | Per-frame update: gravity, friction, integration   |
| `src/maze/WallSegment.h`           | `WallSegment`     | UV endpoint pair                                   |
| `src/maze/MazeLayout.h/.cpp`       | `MazeLayout`      | Six-face wall collections, JSON loader             |
| `src/maze/CollisionSolver.h/.cpp`  | `CollisionSolver` | Wall collision detection and resolution            |
| `src/imu/IImu.h`                   | `IImu`            | Abstract IMU interface                             |
| `src/imu/Mpu6050Imu.h/.cpp`        | `Mpu6050Imu`      | Adapts framework `Mpu6050` to `IImu`               |
| `src/renderer/MazeRenderer.h/.cpp` | `MazeRenderer`    | Draws walls, ball, goal via framework drawing API  |
