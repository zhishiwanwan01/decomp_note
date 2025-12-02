# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a ROS workspace containing DecompROS, a ROS wrapper for convex decomposition of free space in cluttered environments. The workspace implements Safe Flight Corridor generation using ellipsoid-based regional inflation for motion planning in obstacle-rich spaces.

## Build System

### Prerequisites
- ROS (Indigo or later)
- catkin_simple
- Qt4+ (for RViz plugins)
- Eigen3
- cmake_modules

### Initial Setup

If the DecompUtil submodule is not initialized:
```bash
cd catkin_ws/src/DecompROS
git submodule update --init
```

### Building

Using catkin tools (recommended):
```bash
cd catkin_ws
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
```

Using catkin_make_isolated:
```bash
cd catkin_ws
catkin_make_isolated -DCMAKE_BUILD_TYPE=Release
```

### Testing

DecompUtil has CTest tests. To run them:
```bash
cd catkin_ws/src/DecompROS/DecompUtil/build
make test
```

## Repository Structure

The workspace is organized into four main packages:

### 1. DecompUtil (Submodule)
Header-only C++ library implementing core convex decomposition algorithms. Located at `catkin_ws/src/DecompROS/DecompUtil/`.

**Key classes:**
- `DecompBase<Dim>`: Base class for all decomposition methods
- `EllipsoidDecomp<Dim>`: Path-based decomposition using ellipsoid inflation
- `SeedDecomp<Dim>`: Point-based decomposition with spherical inflation
- `LineSegment<Dim>`: Line segment decomposition
- `IterativeDecomp<Dim>`: Iterative refinement of decomposition

**Key data structures:**
- `Ellipsoid<Dim>`: Represents ellipsoids with center and covariance matrix
- `Polyhedron<Dim>`: Represents convex polyhedra as collections of hyperplanes
- `Hyperplane<Dim>`: Half-space representation (point + normal vector)
- `LinearConstraint<Dim>`: Constraint representation as Ax ≤ b

### 2. decomp_ros_msgs
ROS message definitions for serializing and communicating decomposition results.

**Messages:**
- `Ellipsoid.msg`: Contains center (d[3]) and covariance matrix (E[9])
- `Polyhedron.msg`: Arrays of points and normals defining hyperplanes
- `EllipsoidArray.msg`: Array of ellipsoids
- `PolyhedronArray.msg`: Array of polyhedra

### 3. decomp_ros_utils
ROS utilities and RViz visualization plugins.

**Key components:**
- `data_ros_utils.h`: Conversion functions between DecompUtil and ROS types
  - `vec_to_path()`: Convert point vectors to nav_msgs::Path
  - `cloud_to_vec()` / `vec_to_cloud()`: Point cloud conversions
  - `polyhedron_to_ros()` / `ros_to_polyhedron()`: Polyhedron conversions
  - `ellipsoid_array_to_ros()`: Ellipsoid conversions

**RViz plugins:**
- `EllipsoidArrayDisplay`: Visualizes ellipsoid arrays in RViz
- `PolyhedronArrayDisplay`: Visualizes polyhedron arrays in RViz
- Visual components: `BoundVisual`, `MeshVisual`, `VectorVisual`

### 4. decomp_test_node
Example nodes demonstrating usage patterns.

**Test nodes:**
- `test_path_decomp_2d`: 2D path decomposition using EllipsoidDecomp2D
- `test_path_decomp_3d`: 3D path decomposition using EllipsoidDecomp3D
- `test_seed_decomp`: Single-point decomposition using SeedDecomp3D

## Running Examples

Launch visualization:
```bash
roscd decomp_test_node/launch
roslaunch rviz.launch
```

In separate terminals, launch test nodes:
```bash
roslaunch decomp_test_node test_path_decomp_2d.launch
# or
roslaunch decomp_test_node test_path_decomp_3d.launch
# or
roslaunch decomp_test_node test_seed_decomp.launch
```

## Algorithm Architecture

The decomposition pipeline follows this pattern:

1. **Input**: Obstacle point cloud and a path (or seed point)
2. **Local bounding box**: Optional constraint limiting the search region around each segment
3. **Ellipsoid inflation**: Inflate ellipsoid around path segment until obstacles are reached
4. **Polyhedron extraction**: Find supporting hyperplanes from ellipsoid to obstacles
5. **Output**: Safe Flight Corridor as polyhedra or ellipsoids, convertible to linear constraints (Ax ≤ b)

### Typical Usage Pattern

```cpp
// Create decomposition object
EllipsoidDecomp2D decomp_util;

// Set obstacles (point cloud)
decomp_util.set_obs(obstacle_points);

// Set local bounding box (optional)
decomp_util.set_local_bbox(Vec2f(width, height));

// Perform decomposition along path
decomp_util.dilate(path);

// Extract results
auto polyhedra = decomp_util.get_polyhedrons();
auto ellipsoids = decomp_util.get_ellipsoids();
auto constraints = decomp_util.get_constraints(); // Ax <= b form
```

## Code Organization Notes

- **Template dimension**: All classes are templated on dimension (2 or 3), with typedefs like `EllipsoidDecomp2D`, `Polyhedron3D`, etc.
- **Eigen dependency**: Heavy use of Eigen for linear algebra. Types like `Vec2f`, `Vec3f`, `Matf<Dim, Dim>` are Eigen-based.
- **Header-only core**: DecompUtil is header-only, so changes to core algorithms don't require library rebuilding.
- **RViz integration**: The Qt-based RViz plugins require MOC code generation, handled by CMake's `CMAKE_AUTOMOC`.

## Common Development Tasks

### Adding a new decomposition algorithm
1. Inherit from `DecompBase<Dim>` in DecompUtil
2. Implement virtual method `dilate()`
3. Implement virtual method `add_local_bbox()`
4. Add test in `DecompUtil/test/`

### Adding ROS message types
1. Define `.msg` file in `decomp_ros_msgs/msg/`
2. Add conversion functions in `decomp_ros_utils/include/decomp_ros_utils/data_ros_utils.h`
3. Rebuild catkin workspace

### Creating visualization plugins
1. Create visual class (inherit from RViz visual base)
2. Create display class (inherit from `rviz::MessageFilterDisplay`)
3. Add to `decomp_ros_utils/src/` and update CMakeLists.txt
4. Register plugin in `plugin_description.xml`

## Important Implementation Details

- **2D to 3D conversion**: 2D polyhedra are converted to 3D by adding thin z-axis constraints (±0.01) for visualization
- **Obstacle filtering**: `set_obs()` filters points to only those inside the local bounding box
- **Hyperplane orientation**: Normals point outward from the free space
- **Constraint generation**: Use `LinearConstraint<Dim>` with an interior point to ensure correct inequality orientation
