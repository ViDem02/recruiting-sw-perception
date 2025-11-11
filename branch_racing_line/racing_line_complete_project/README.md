# Racing Line Complete Project

This project demonstrates the refactoring of procedural cone recognition code into a class-based design to reduce parameter passing and improve maintainability.

## Overview

The project includes:
- **ConesRecognizer**: Refactored class-based implementation
- **cones_recog**: Original procedural implementation (for comparison)
- **DistinguishLeftRight**: Utility for separating left/right cones with distance filtering
- **racingline_nlopt**: Main application demonstrating usage

## Key Improvements

### Before: Procedural Design (cones_recog.h/cpp)
The original design had functions with excessive parameters:
```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr recognizeCones(
    const std::string& scan_file,
    const std::string& model_file,
    double voxel_leaf_size,
    int sor_mean_k,
    double sor_stddev_mul_thresh,
    double plane_distance_threshold,
    double min_points_ratio,
    double rotation_x_deg,
    double rotation_z_deg,
    double cluster_tolerance,
    int min_cluster_size,
    int max_cluster_size,
    double icp_max_correspondence_distance,
    int icp_max_iterations,
    double icp_transformation_epsilon,
    double icp_fitness_threshold,
    double filter_ref_x,
    double filter_ref_y,
    double filter_max_distance);  // 19 parameters!
```

### After: Class-Based Design (ConesRecognizer.h/cpp)
The refactored design encapsulates configuration as member state:
```cpp
cones::RecognizerConfig config;
config.filter_ref_x = robot_x;
config.filter_ref_y = robot_y;
config.filter_max_distance = max_distance;

cones::ConesRecognizer recognizer(config);
auto cones = recognizer.detectCones(scan_file);  // Simple API!
```

## Benefits of Refactoring

1. **Reduced Parameter Passing**: Configuration stored as member variables
2. **Better Encapsulation**: Private helper methods operate on member state
3. **Easier to Test**: Can configure once and run multiple detections
4. **More Maintainable**: Adding new parameters doesn't break all function signatures
5. **State Preservation**: Access to intermediate results (clusters, full cloud, centers)
6. **Cleaner API**: Public methods have minimal, intuitive parameters

## Components

### ConesRecognizer Class
Main class for cone detection and recognition:
- **Public API**:
  - `detectCones(file)` - Detect cones from PCD file
  - `detectCones(cloud)` - Detect cones from point cloud
  - `getCenters()` - Get detected cone centers
  - `getConeCloud()` - Get full cone point cloud
  - `getClusters()` - Get individual cone clusters
  - `processFrame(file)` - Convenience method
  - `setConfig(config)` - Update configuration
  - `loadModel(file)` - Load cone model for ICP matching

- **Configuration** (RecognizerConfig):
  - Preprocessing: voxel filtering, outlier removal, plane segmentation
  - Transformation: coordinate system adjustments
  - Clustering: tolerance, size constraints
  - ICP: matching parameters
  - Filtering: distance-based filtering

### DistinguishLeftRight
Separates cones into left and right based on robot position:
```cpp
auto result = cones::distinguishLeftRight(
    cones, 
    robot_x, robot_y,
    -45.0, 45.0,        // angle range (degrees)
    max_distance);      // distance threshold
```

Features:
- Angle-based filtering (forward field of view)
- Distance-based filtering (ignore distant cones)
- Coordinate transformation handling
- Backward compatibility (default parameters)

## Building

```bash
mkdir build
cd build
cmake ..
make
```

## Usage

```bash
# Run with default parameters (no distance filtering)
./racingline_nlopt cones.pcd

# Run with robot position
./racingline_nlopt cones.pcd 0.0 0.0

# Run with distance filtering (10 meter radius)
./racingline_nlopt cones.pcd 0.0 0.0 10.0
```

## Distance Filtering

Both `ConesRecognizer` and `distinguishLeftRight` support distance-based filtering:

### In ConesRecognizer
```cpp
config.filter_max_distance = 10.0;  // meters
config.filter_ref_x = 0.0;
config.filter_ref_y = 0.0;
```

### In DistinguishLeftRight
```cpp
auto result = cones::distinguishLeftRight(
    cones, robot_x, robot_y,
    -45.0, 45.0,
    10.0);  // max_distance
```

Default: `std::numeric_limits<double>::infinity()` (no filtering)

## Thread Safety

The current implementation is not thread-safe. If concurrent detection is needed:
1. Create separate `ConesRecognizer` instances per thread
2. Consider making model loading static/shared
3. Protect member state access with mutexes

## Future Improvements

1. Unit tests for distance filtering edge cases
2. Thread-safe implementation option
3. Performance profiling and optimization
4. Support for additional filtering criteria
5. Integration with racing line generation
