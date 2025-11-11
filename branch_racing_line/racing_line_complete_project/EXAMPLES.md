# Usage Examples: Before and After Refactoring

## Example 1: Basic Cone Detection

### Before (Procedural API)
```cpp
#include "cones_recog.h"

// Need to pass ALL 19 parameters
auto cones = cones::recognizeCones(
    "lidar_scan.pcd",           // scan_file
    "cone_model.ply",           // model_file
    0.01,                       // voxel_leaf_size
    50,                         // sor_mean_k
    1.0,                        // sor_stddev_mul_thresh
    0.01,                       // plane_distance_threshold
    0.4,                        // min_points_ratio
    -90.0,                      // rotation_x_deg
    90.0,                       // rotation_z_deg
    0.05,                       // cluster_tolerance
    10,                         // min_cluster_size
    10000,                      // max_cluster_size
    0.05,                       // icp_max_correspondence_distance
    50,                         // icp_max_iterations
    1e-8,                       // icp_transformation_epsilon
    0.05,                       // icp_fitness_threshold
    0.0,                        // filter_ref_x
    0.0,                        // filter_ref_y
    std::numeric_limits<double>::infinity()  // filter_max_distance
);

// Only get final result - no access to intermediate data
std::cout << "Detected " << cones->size() << " cones\n";
```

### After (Class-Based API)
```cpp
#include "ConesRecognizer.h"

// Configure once
cones::RecognizerConfig config;
// Only set non-default values
// (all others use sensible defaults)

// Create recognizer
cones::ConesRecognizer recognizer(config);

// Simple detection
auto cones = recognizer.detectCones("lidar_scan.pcd");

// Access all results
std::cout << "Detected " << cones->size() << " cone centers\n";
auto clusters = recognizer.getClusters();
std::cout << "Found " << clusters.size() << " clusters\n";
```

**Improvement:** 19 parameters → 1 parameter, plus access to intermediate results

---

## Example 2: Distance Filtering

### Before (Procedural API)
```cpp
#include "cones_recog.h"

// Must pass all parameters including distance filter
auto cones = cones::recognizeCones(
    "lidar_scan.pcd", "cone_model.ply",
    0.01, 50, 1.0, 0.01, 0.4,
    -90.0, 90.0,
    0.05, 10, 10000,
    0.05, 50, 1e-8, 0.05,
    0.0,   // filter_ref_x (robot position)
    0.0,   // filter_ref_y
    10.0   // filter_max_distance (only cones within 10m)
);
```

### After (Class-Based API)
```cpp
#include "ConesRecognizer.h"

cones::RecognizerConfig config;
config.filter_ref_x = 0.0;         // Robot X
config.filter_ref_y = 0.0;         // Robot Y
config.filter_max_distance = 10.0; // 10 meter radius

cones::ConesRecognizer recognizer(config);
auto cones = recognizer.detectCones("lidar_scan.pcd");
```

**Improvement:** Clear, named parameters instead of positional arguments

---

## Example 3: Processing Multiple Scans

### Before (Procedural API)
```cpp
#include "cones_recog.h"

// Must repeat ALL parameters for each scan
auto cones1 = cones::recognizeCones(
    "scan1.pcd", "model.ply",
    0.01, 50, 1.0, 0.01, 0.4, -90.0, 90.0,
    0.05, 10, 10000, 0.05, 50, 1e-8, 0.05,
    0.0, 0.0, 10.0);

auto cones2 = cones::recognizeCones(
    "scan2.pcd", "model.ply",
    0.01, 50, 1.0, 0.01, 0.4, -90.0, 90.0,
    0.05, 10, 10000, 0.05, 50, 1e-8, 0.05,
    0.0, 0.0, 10.0);

auto cones3 = cones::recognizeCones(
    "scan3.pcd", "model.ply",
    0.01, 50, 1.0, 0.01, 0.4, -90.0, 90.0,
    0.05, 10, 10000, 0.05, 50, 1e-8, 0.05,
    0.0, 0.0, 10.0);

// Very repetitive and error-prone!
```

### After (Class-Based API)
```cpp
#include "ConesRecognizer.h"

// Configure once
cones::RecognizerConfig config;
config.filter_max_distance = 10.0;

// Reuse for multiple scans
cones::ConesRecognizer recognizer(config);
auto cones1 = recognizer.detectCones("scan1.pcd");
auto cones2 = recognizer.detectCones("scan2.pcd");
auto cones3 = recognizer.detectCones("scan3.pcd");

// Much cleaner!
```

**Improvement:** Configure once, reuse many times

---

## Example 4: Custom Clustering Parameters

### Before (Procedural API)
```cpp
#include "cones_recog.h"

// Must pass all parameters even if only changing clustering
auto cones = cones::recognizeCones(
    "scan.pcd", "model.ply",
    0.01, 50, 1.0, 0.01, 0.4, -90.0, 90.0,
    0.10,    // Changed: cluster_tolerance
    5,       // Changed: min_cluster_size
    5000,    // Changed: max_cluster_size
    0.05, 50, 1e-8, 0.05, 0.0, 0.0,
    std::numeric_limits<double>::infinity());
```

### After (Class-Based API)
```cpp
#include "ConesRecognizer.h"

cones::RecognizerConfig config;
// Only set what you want to change
config.cluster_tolerance = 0.10;
config.min_cluster_size = 5;
config.max_cluster_size = 5000;
// All other parameters use defaults

cones::ConesRecognizer recognizer(config);
auto cones = recognizer.detectCones("scan.pcd");
```

**Improvement:** Only specify what changes, defaults handle the rest

---

## Example 5: Accessing Intermediate Results

### Before (Procedural API)
```cpp
#include "cones_recog.h"

// Can only get final cone centers
auto centers = cones::recognizeCones(
    "scan.pcd", "model.ply",
    0.01, 50, 1.0, 0.01, 0.4, -90.0, 90.0,
    0.05, 10, 10000, 0.05, 50, 1e-8, 0.05,
    0.0, 0.0, 10.0);

// No way to access:
// - Individual clusters
// - Full point cloud
// - Intermediate processing results
```

### After (Class-Based API)
```cpp
#include "ConesRecognizer.h"

cones::ConesRecognizer recognizer;
auto centers = recognizer.detectCones("scan.pcd");

// Access everything!
auto clusters = recognizer.getClusters();
auto full_cloud = recognizer.getConeCloud();

std::cout << "Cone centers: " << centers->size() << "\n";
std::cout << "Clusters: " << clusters.size() << "\n";
std::cout << "Total points: " << full_cloud->size() << "\n";

// Can analyze individual clusters
for (size_t i = 0; i < clusters.size(); ++i) {
    std::cout << "Cluster " << i << ": " 
              << clusters[i]->size() << " points\n";
}
```

**Improvement:** Full access to processing pipeline results

---

## Example 6: Integration with DistinguishLeftRight

### Before (Multiple Function Calls with Parameters)
```cpp
#include "cones_recog.h"
#include "DistinguishLeftRight.h"

// Step 1: Detect cones (19 parameters)
auto cones = cones::recognizeCones(
    "scan.pcd", "model.ply",
    0.01, 50, 1.0, 0.01, 0.4, -90.0, 90.0,
    0.05, 10, 10000, 0.05, 50, 1e-8, 0.05,
    0.0, 0.0, 10.0);

// Step 2: Separate left/right (6 parameters)
auto result = cones::distinguishLeftRight(
    cones, 0.0, 0.0, -45.0, 45.0, 10.0);
```

### After (Clean Pipeline)
```cpp
#include "ConesRecognizer.h"
#include "DistinguishLeftRight.h"

// Step 1: Configure and detect
cones::RecognizerConfig config;
config.filter_max_distance = 10.0;

cones::ConesRecognizer recognizer(config);
auto cones = recognizer.detectCones("scan.pcd");

// Step 2: Separate left/right
double robot_x = 0.0, robot_y = 0.0;
auto result = cones::distinguishLeftRight(
    cones, robot_x, robot_y, -45.0, 45.0, 10.0);

// Use results
std::cout << "Left: " << result.left_cones->size() << "\n";
std::cout << "Right: " << result.right_cones->size() << "\n";
```

**Improvement:** Clear separation of concerns, readable code

---

## Example 7: Testing with Different Configurations

### Before (Procedural API)
```cpp
#include "cones_recog.h"

void testConfiguration1() {
    auto cones = cones::recognizeCones(
        "test.pcd", "model.ply",
        0.01, 50, 1.0, 0.01, 0.4, -90.0, 90.0,
        0.05, 10, 10000, 0.05, 50, 1e-8, 0.05,
        0.0, 0.0, 10.0);
    // Test assertions...
}

void testConfiguration2() {
    auto cones = cones::recognizeCones(
        "test.pcd", "model.ply",
        0.02, 30, 2.0, 0.02, 0.3, -90.0, 90.0,
        0.10, 5, 5000, 0.10, 100, 1e-6, 0.10,
        0.0, 0.0, 5.0);
    // Test assertions...
}
```

### After (Class-Based API)
```cpp
#include "ConesRecognizer.h"

void testConfiguration1() {
    cones::RecognizerConfig config;
    config.filter_max_distance = 10.0;
    
    cones::ConesRecognizer recognizer(config);
    auto cones = recognizer.detectCones("test.pcd");
    // Test assertions...
}

void testConfiguration2() {
    cones::RecognizerConfig config;
    config.voxel_leaf_size = 0.02;
    config.sor_mean_k = 30;
    config.cluster_tolerance = 0.10;
    config.filter_max_distance = 5.0;
    
    cones::ConesRecognizer recognizer(config);
    auto cones = recognizer.detectCones("test.pcd");
    // Test assertions...
}
```

**Improvement:** Self-documenting configuration, easy to create test scenarios

---

## Example 8: Real-Time Processing Loop

### Before (Procedural API)
```cpp
#include "cones_recog.h"

while (running) {
    std::string scan = getCurrentScan();
    
    // Must pass all parameters every frame
    auto cones = cones::recognizeCones(
        scan, "model.ply",
        0.01, 50, 1.0, 0.01, 0.4, -90.0, 90.0,
        0.05, 10, 10000, 0.05, 50, 1e-8, 0.05,
        robot_x, robot_y, 15.0);
    
    processConesForRacingLine(cones);
}
```

### After (Class-Based API)
```cpp
#include "ConesRecognizer.h"

// Configure once before loop
cones::RecognizerConfig config;
config.filter_max_distance = 15.0;
cones::ConesRecognizer recognizer(config);

while (running) {
    std::string scan = getCurrentScan();
    
    // Update dynamic parameters
    config.filter_ref_x = robot_x;
    config.filter_ref_y = robot_y;
    recognizer.setConfig(config);
    
    // Simple detection
    auto cones = recognizer.detectCones(scan);
    processConesForRacingLine(cones);
}
```

**Improvement:** Efficient processing, update only what changes

---

## Example 9: Debugging and Analysis

### Before (Procedural API)
```cpp
#include "cones_recog.h"

// Limited visibility into processing steps
auto cones = cones::recognizeCones(/* 19 parameters */);

// Can't inspect:
// - How many clusters were found?
// - What was filtered out?
// - What are the cluster sizes?
```

### After (Class-Based API)
```cpp
#include "ConesRecognizer.h"

cones::ConesRecognizer recognizer(config);
auto centers = recognizer.detectCones("scan.pcd");

// Full visibility for debugging
std::cout << "=== Detection Results ===\n";
std::cout << "Cone centers: " << centers->size() << "\n";

auto clusters = recognizer.getClusters();
std::cout << "Clusters found: " << clusters.size() << "\n";

for (size_t i = 0; i < clusters.size(); ++i) {
    auto& cluster = clusters[i];
    
    // Compute cluster statistics
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);
    
    std::cout << "Cluster " << i << ":\n";
    std::cout << "  Points: " << cluster->size() << "\n";
    std::cout << "  Center: (" << centroid[0] << ", " 
              << centroid[1] << ", " << centroid[2] << ")\n";
}

auto full_cloud = recognizer.getConeCloud();
std::cout << "Total points in all cones: " 
          << full_cloud->size() << "\n";
```

**Improvement:** Complete visibility for debugging and analysis

---

## Summary

| Feature | Before (Procedural) | After (Class-Based) |
|---------|-------------------|-------------------|
| Parameter count | 19 | 1 |
| Configuration reuse | No | Yes |
| Intermediate results | No access | Full access |
| Code readability | Poor | Excellent |
| Maintainability | Difficult | Easy |
| Testing | Hard | Easy |
| Debugging | Limited | Full visibility |

The refactored design makes the code:
- **Easier to use**: Simple, clean API
- **Easier to maintain**: Clear organization
- **Easier to test**: Configurable behavior
- **Easier to debug**: Access to all results
- **Easier to extend**: Add features without breaking API
