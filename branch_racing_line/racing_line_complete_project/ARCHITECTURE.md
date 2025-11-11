# Architecture Documentation

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                      Racing Line System                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌──────────────┐      ┌─────────────────┐      ┌────────────┐ │
│  │   LiDAR      │─────▶│ ConesRecognizer │─────▶│ Racing     │ │
│  │   Scanner    │      │     (Class)     │      │ Line Gen   │ │
│  │   (.pcd)     │      └─────────────────┘      └────────────┘ │
│  └──────────────┘              │                                │
│                                 │                                │
│                                 ▼                                │
│                      ┌──────────────────────┐                   │
│                      │ DistinguishLeftRight │                   │
│                      │     (Function)       │                   │
│                      └──────────────────────┘                   │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
```

## Class Architecture: ConesRecognizer

```
╔═══════════════════════════════════════════════════════════╗
║                     ConesRecognizer                        ║
╠═══════════════════════════════════════════════════════════╣
║ + Public Interface                                         ║
║   ┌───────────────────────────────────────────────────┐   ║
║   │ + ConesRecognizer()                               │   ║
║   │ + ConesRecognizer(config)                         │   ║
║   │ + setConfig(config)                               │   ║
║   │ + getConfig()                                     │   ║
║   │ + loadModel(file)                                 │   ║
║   │ + detectCones(file) → PointCloud                  │   ║
║   │ + detectCones(cloud) → PointCloud                 │   ║
║   │ + getCenters() → PointCloud                       │   ║
║   │ + getConeCloud() → PointCloud                     │   ║
║   │ + getClusters() → vector<PointCloud>              │   ║
║   │ + processFrame(file) → PointCloud                 │   ║
║   └───────────────────────────────────────────────────┘   ║
║                                                             ║
║ + Configuration (RecognizerConfig)                         ║
║   ┌───────────────────────────────────────────────────┐   ║
║   │ Preprocessing:                                    │   ║
║   │   - voxel_leaf_size                               │   ║
║   │   - sor_mean_k, sor_stddev_mul_thresh            │   ║
║   │   - plane_distance_threshold                      │   ║
║   │   - min_points_ratio                              │   ║
║   │                                                    │   ║
║   │ Transformation:                                   │   ║
║   │   - rotation_x_deg, rotation_z_deg                │   ║
║   │                                                    │   ║
║   │ Clustering:                                       │   ║
║   │   - cluster_tolerance                             │   ║
║   │   - min_cluster_size, max_cluster_size           │   ║
║   │                                                    │   ║
║   │ ICP:                                              │   ║
║   │   - icp_max_correspondence_distance               │   ║
║   │   - icp_max_iterations                            │   ║
║   │   - icp_transformation_epsilon                    │   ║
║   │   - icp_fitness_threshold                         │   ║
║   │                                                    │   ║
║   │ Filtering:                                        │   ║
║   │   - filter_ref_x, filter_ref_y                    │   ║
║   │   - filter_max_distance                           │   ║
║   └───────────────────────────────────────────────────┘   ║
║                                                             ║
║ - Private State                                            ║
║   ┌───────────────────────────────────────────────────┐   ║
║   │ - config_: RecognizerConfig                       │   ║
║   │ - model_cone_: PointCloud::Ptr                    │   ║
║   │ - cone_centers_: PointCloud::Ptr                  │   ║
║   │ - cone_cloud_: PointCloud::Ptr                    │   ║
║   │ - clusters_: vector<PointCloud::Ptr>              │   ║
║   └───────────────────────────────────────────────────┘   ║
║                                                             ║
║ - Private Helpers (operate on member state)                ║
║   ┌───────────────────────────────────────────────────┐   ║
║   │ - loadAndPreprocess(file)                         │   ║
║   │ - applyTransformation(cloud)                      │   ║
║   │ - clusterCones(cloud)                             │   ║
║   │ - extractConeCenters()                            │   ║
║   │ - filterByDistance()                              │   ║
║   │ - matchWithModel(cone, transform)                 │   ║
║   └───────────────────────────────────────────────────┘   ║
╚═══════════════════════════════════════════════════════════╝
```

## Processing Pipeline

```
┌─────────────────────────────────────────────────────────────┐
│                  ConesRecognizer Pipeline                    │
└─────────────────────────────────────────────────────────────┘

Input: LiDAR scan (.pcd file)
   │
   ▼
┌────────────────────────────┐
│  loadAndPreprocess()       │  ← voxel_leaf_size
│  - Load PCD file           │  ← sor_mean_k, sor_stddev_mul_thresh
│  - Voxel grid filter       │  ← plane_distance_threshold
│  - Statistical outlier     │  ← min_points_ratio
│  - Ground plane removal    │
└────────────────────────────┘
   │
   ▼
┌────────────────────────────┐
│  applyTransformation()     │  ← rotation_x_deg
│  - Rotate coordinate system│  ← rotation_z_deg
└────────────────────────────┘
   │
   ▼
┌────────────────────────────┐
│  clusterCones()            │  ← cluster_tolerance
│  - Euclidean clustering    │  ← min_cluster_size
│  - Separate individual     │  ← max_cluster_size
│    cones                   │
└────────────────────────────┘
   │
   ▼
┌────────────────────────────┐
│  extractConeCenters()      │
│  - Compute centroids       │
│  - Store in cone_centers_  │
└────────────────────────────┘
   │
   ▼
┌────────────────────────────┐
│  filterByDistance()        │  ← filter_max_distance
│  - Remove distant cones    │  ← filter_ref_x, filter_ref_y
└────────────────────────────┘
   │
   ▼
Output: Cone centers (PointCloud)
```

## Data Flow: Before vs After Refactoring

### Before (Procedural)
```
main()
  │
  ├─ Call recognizeCones(19 params)
  │    │
  │    ├─ loadAndPreprocess(6 params) ────────┐
  │    │                                       │
  │    ├─ applyTransformation(3 params) ──────┤
  │    │                                       │
  │    ├─ clusterCones(4 params) ─────────────┤  Parameters
  │    │                                       │  passed
  │    ├─ extractConeCenters(1 param) ────────┤  repeatedly
  │    │                                       │
  │    └─ filterByDistance(4 params) ─────────┘
  │
  └─ Get result: PointCloud
     (No access to intermediate results)
```

### After (Class-Based)
```
main()
  │
  ├─ Create RecognizerConfig ────────┐
  │    (set once, reuse multiple)    │
  │                                   │
  ├─ Create ConesRecognizer(config) ─┤  Config stored
  │                                   │  as member
  │                                   │
  ├─ Call detectCones("file") ───────┘
  │    │
  │    ├─ loadAndPreprocess()  ─┐
  │    ├─ applyTransformation() │
  │    ├─ clusterCones()        ├─ Access config_
  │    ├─ extractConeCenters()  │  member variable
  │    └─ filterByDistance()    ┘  (no param passing)
  │
  ├─ Get results:
  │    │
  │    ├─ getCenters()    → Cone centers
  │    ├─ getConeCloud()  → Full point cloud
  │    └─ getClusters()   → Individual clusters
  │
  └─ Reuse for next frame (config preserved)
```

## Integration with DistinguishLeftRight

```
┌──────────────────────────────────────────────────────────────┐
│                    Complete Processing Flow                   │
└──────────────────────────────────────────────────────────────┘

LiDAR Scan (.pcd)
   │
   ▼
┌─────────────────────────────────┐
│   ConesRecognizer               │
│   - Preprocessing               │
│   - Clustering                  │
│   - Center extraction           │
│   - Distance filtering          │
└─────────────────────────────────┘
   │
   ▼
Detected Cone Centers
   │
   ▼
┌─────────────────────────────────┐
│   distinguishLeftRight()        │
│   - Coordinate transform        │
│   - Angle-based filtering       │
│   - Distance filtering          │
│   - Left/Right classification   │
└─────────────────────────────────┘
   │
   ├─────────────┬──────────────┐
   ▼             ▼              ▼
Left Cones   Right Cones   (Filtered out)
   │             │
   └─────┬───────┘
         │
         ▼
   Racing Line
   Generation
```

## Module Dependencies

```
┌─────────────────────────────────────────────────────────┐
│                   Dependency Graph                       │
└─────────────────────────────────────────────────────────┘

racingline_nlopt.cpp
   │
   ├──────────────┬──────────────┐
   │              │              │
   ▼              ▼              ▼
ConesRecognizer  DistinguishLeftRight  PCL
   │              │                     │
   │              └────────┬────────────┘
   │                       │
   └───────────────────────┴───────────▶ Eigen3
                                         Boost
                                         VTK
```

## Memory Management

```
┌─────────────────────────────────────────────────────────┐
│              Smart Pointer Usage                         │
└─────────────────────────────────────────────────────────┘

ConesRecognizer
   │
   ├─ model_cone_: shared_ptr<PointCloud>
   │   └─ Loaded once, reused
   │
   ├─ cone_centers_: shared_ptr<PointCloud>
   │   └─ Updated each detection
   │
   ├─ cone_cloud_: shared_ptr<PointCloud>
   │   └─ Updated each detection
   │
   └─ clusters_: vector<shared_ptr<PointCloud>>
       └─ Updated each detection

Benefits:
✓ Automatic cleanup
✓ Safe sharing
✓ No manual delete needed
✓ Exception safe
```

## Configuration Management

```
┌─────────────────────────────────────────────────────────┐
│           Configuration Lifecycle                        │
└─────────────────────────────────────────────────────────┘

1. Create Config
   RecognizerConfig config;
   config.filter_max_distance = 10.0;
   config.cluster_tolerance = 0.05;

2. Initialize Recognizer
   ConesRecognizer recognizer(config);
   
3. Use Multiple Times
   auto cones1 = recognizer.detectCones("scan1.pcd");
   auto cones2 = recognizer.detectCones("scan2.pcd");
   auto cones3 = recognizer.detectCones("scan3.pcd");

4. Update Config (if needed)
   config.filter_max_distance = 15.0;
   recognizer.setConfig(config);

5. Continue Processing
   auto cones4 = recognizer.detectCones("scan4.pcd");
```

## Thread Safety Considerations

```
┌─────────────────────────────────────────────────────────┐
│              Current: Not Thread-Safe                    │
└─────────────────────────────────────────────────────────┘

Single Thread Usage (Current):
   Thread 1: recognizer.detectCones() ✓

Multi-Thread Usage (NOT Safe):
   Thread 1: recognizer.detectCones() ✗
   Thread 2: recognizer.detectCones() ✗
   (Shared state corruption)

Recommended Multi-Thread Pattern:
   Thread 1: recognizer1.detectCones() ✓
   Thread 2: recognizer2.detectCones() ✓
   (Separate instances)

Future Enhancement:
   Add mutex protection:
   - std::mutex detection_mutex_
   - Lock in detectCones()
   - Lock in state accessors
```

## Performance Characteristics

```
┌─────────────────────────────────────────────────────────┐
│            Time Complexity Analysis                      │
└─────────────────────────────────────────────────────────┘

loadAndPreprocess():
   - File I/O:           O(n)
   - Voxel filtering:    O(n)
   - Outlier removal:    O(n log n)
   - Plane removal:      O(n * iterations)
   Total: O(n log n)

applyTransformation():
   - Matrix multiply:    O(n)

clusterCones():
   - KD-Tree build:      O(n log n)
   - Clustering:         O(n log n)
   Total: O(n log n)

extractConeCenters():
   - Centroid compute:   O(k * avg_cluster_size)
                        ≈ O(n)

filterByDistance():
   - Distance check:     O(k) where k = # clusters
                        ≈ O(n) in worst case

Overall: O(n log n) where n = # points in scan
```

## Error Handling

```
┌─────────────────────────────────────────────────────────┐
│              Error Handling Strategy                     │
└─────────────────────────────────────────────────────────┘

File I/O Errors:
   - PCL functions return -1 on error
   - Check return values
   - Log errors with PCL_ERROR
   - Return empty point cloud

Invalid Parameters:
   - Defensive clamping (e.g., max_distance)
   - Validate in config setters (future)
   - Use sensible defaults

Processing Errors:
   - Check empty clouds
   - Validate cluster sizes
   - Handle ICP convergence failure

Memory Errors:
   - Smart pointers prevent leaks
   - RAII ensures cleanup
   - No manual memory management
```

## Design Patterns Used

```
┌─────────────────────────────────────────────────────────┐
│              Design Patterns Applied                     │
└─────────────────────────────────────────────────────────┘

1. Configuration Object
   RecognizerConfig - groups related parameters

2. Facade
   Simple public API hides complex pipeline

3. Template Method (implicit)
   detectCones() orchestrates steps in order

4. Factory (implicit)
   Constructors create configured instances

5. RAII
   Smart pointers manage resources

6. Null Object
   Return empty clouds instead of null/exceptions

7. Encapsulation
   Private helpers access member state
```

## Future Architecture Enhancements

```
┌─────────────────────────────────────────────────────────┐
│              Potential Improvements                      │
└─────────────────────────────────────────────────────────┘

1. Strategy Pattern
   Pluggable clustering algorithms:
   - EuclideanClustering (current)
   - RegionGrowing
   - ConditionalClustering

2. Observer Pattern
   Progress callbacks:
   - onPreprocessingComplete()
   - onClusteringComplete()
   - onDetectionComplete()

3. Builder Pattern
   Fluent configuration:
   RecognizerConfig()
       .withVoxelSize(0.01)
       .withClusterTolerance(0.05)
       .build()

4. Command Pattern
   Undo/redo configuration changes

5. Visitor Pattern
   Process detected cones with different strategies

6. Pipeline Pattern
   Composable processing stages
```

This architecture provides a solid foundation for autonomous vehicle perception while maintaining clean separation of concerns and extensibility.
