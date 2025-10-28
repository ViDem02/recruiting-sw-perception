# REQUIREMENTS

## OVERVIEW

The **goal**: implement a **perception module** for a **Formula Student Driverless vehicle**. 

The **Tools**: **C++, CMake, OpenCV**, **PCL**, **git**.

Activities:
- process sensor inputs, 
- extracting meaningful information to simulate a **perception pipeline for an autonomous racing car**. 


The challenge consists of **five levels** plus an additional bonus tasks. 

LiDAR path: levels 1 - 4 should be completed using **cones.pcd**, while level 5 should be tackled using **first.pcd and second.pcd** .


## LEVELS 


L1: Load, display, and preprocess LiDAR data
L2: Detect cones
L3: Classify objects (cones or obstacles) in the scene
L4: Extract track edges or racing line
L5: Perform odometry using image or LiDAR data
BONUS: Implement interactive visualization for debugging

## IN DEPTH

### L1 Load and Display Captured Data
GOAL: Load and display a 3D point cloud dataset from a LiDAR sensor.
STEPS: Display the raw data using **OpenCV** or visualize the point cloud using **PCL visualizer**.

### L2 Cone Detection

GOAL: Extract points corresponding to objects in the scan. and classify them as either cones or obstacles on the track.
STEPS: Use clustering and classification algorithms (e.g., **K-Means**, **DBSCAN**) to detect from the point cloud.

---

### L3 Object Classification
GOAL: Classify the objects obtained previously as either cones or general obstacles.
STEPS: Use point cloud geometry fitting such as **RANSAC** to determine the shape, or perform **ICP** using an ideal cone model to infer its shape.

### L4 Extract Track Edges
GOAL: Identify the **racing line** based on cone positions.
STEPS: Use clustering algorithms (e.g., **RANSAC**) to fit lines to detected cone clusters or boundaries, helping to define the racing line.

### L5 Odometry
GOAL:  Given two point clouds, perform a **pose estimation** relying on **registration algorithms**.00
STEPS: Use a point cloud registration algorithm like **ICP (Iterative Closest Point)** to align the two point clouds, and extract the relative **transformation** between the two. 

### L6 Bonus: Interactive Visualization for Debugging
TASK: Build an interactive interface to visualize the pipeline described above.  

Create a GUI where the user can adjust parameters dynamically (e.g., thresholds for cone detection, edge detection). The processed results should update in **real-time** as the parameters change.


## DELIVERY

### DEAD: 2 weeks

### Material required 
1. Code, via github.
2. CMakeLists.txt for easy compilation
3. Short report (PDF or Markdown) covering:
   - Approach used for each task.
   - Challenges faced and how they were overcome.
   - Any possible improvements or optimizations.
4. Screenshots or GIFs showcasing results (especially for interactive GUI)