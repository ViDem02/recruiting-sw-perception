# Diary

project link: https://github.com/users/ViDem02/projects/7

## First phases

I will now parse the requirements. 
I've done that. 

### R&D suggested: essential theory

#### 1 - LiDAR & Point Cloud Fundamentals

You need to know:

What a point cloud represents (XYZ, intensity, frame of reference)

Common LiDAR distortions and noise

Filtering techniques
→ Voxel grid downsampling
→ Radius & statistical outlier removal

📌 Keywords: Point Cloud Coordinate Frames, LiDAR Preprocessing, PCL Filters


#### 2 - Segmentation & Clustering

This is key for Levels 2 & 3.

You should understand:

Euclidean clustering (PCL implementation)

Density-based clustering (DBSCAN)

Estimating cluster centroid, size, bounding box

📌 Keywords: PCL EuclideanClusterExtraction, DBSCAN, KD-Trees

#### 3 - Track Boundary Estimation / Racing Line

You should understand:

How racing cones define left & right track edges

Line fitting from clusters
→ RANSAC line fitting

Optional: curve fitting for racing line (e.g., splines)

📌 Keywords: Clustering track sides, Polynomial / Spline Fitting, Path Extraction

#### 4 -  Pose Estimation with ICP (Odometry)

Critical for Level 5.

Understand:

Iterative Closest Point basics

Rigid transformation: rotation matrix + translation

Frame-to-frame registration

📌 Keywords: ICP, Point Cloud Registration, SE(3) Transformations, Homogeneous Coordinates


#### Visualization Tools

In-depth knowledge helps debugging.

PCL Visualizer (3D)

Optional: OpenCV for projections

📌 Keywords: PCLVisualizer, Colorizing clusters, Interactive widgets


#### Suggestions on structure 

You want a clean perception pipeline:

Load → Filter → Cluster → Classify → Track Extraction → Odometry → Visualization

Modular design means each level becomes a unit test.

### To do

I have set up a project to keep track of tasks.

Doing research, topic 1

LiDAR Basics: The Coordinate System
https://hackernoon.com/lidar-basics-the-coordinate-system-a26529615df9


![coordinates system](<imgs/Capture d’écran 2025-10-28 à 18.08.17.png>)


## Progress

Created second project. Worked out structure and workflow.

### How to structure a project 
Seen https://www.youtube.com/watch?v=HXd7g3RlCIs&list=PLS_iNJJVTtiRV0DZRDcTHnvAuDrKGPN40&index=6

Seen https://learn.microsoft.com/en-us/cpp/cpp/header-files-cpp?view=msvc-170
Seen https://learn.microsoft.com/en-us/cpp/cpp/modules-cpp?view=msvc-170


# Saturday progress

Done some experimentation on filtering and working 
with point clouds.

# Sunday plan

Work with 