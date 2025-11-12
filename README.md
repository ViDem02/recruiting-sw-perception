# Project Report

## Levels 

L1: Load, display, and preprocess LiDAR data

L2: Detect cones

L3: Classify objects (cones or obstacles) in the scene

L4: Extract track edges or racing line

L5: Perform odometry using image or LiDAR data

BONUS: Implement interactive visualization for debugging


## In depth

### Prior work 

In the first phases of the work, I heavily relied on the [website of PCL](https://pcl.readthedocs.io/projects/tutorials/en/master/index.html#basic-usage), seeing all the components of the library.
I set up a GitHub project but in the end I felt like the task were sequential, so I didn't use it. 

### General Approach

I started by implementing the examples and tutorials provided by the PCL website. I used CLion and I used the std system toolchain. I explored the possibility of using modules but they're not compatible with AppleClang so I didn't use them. After I got the logic of the examples, I applied them to the task at end. 

### L1 Load and Display Captured Data

GOAL: Load and display a 3D point cloud dataset from a LiDAR sensor.
STEPS: Display the raw data using  **PCL visualizer**.

For this task I used both the [tutorial on rotations](https://pcl.readthedocs.io/projects/tutorials/en/master/matrix_transform.html#matrix-transform) and the CLI tool that the library provides (even though for MacOS it's a bit tricky to make it reachable by zsh).

### L2 Cone Detection & L3 Object Classification

GOAL: Extract points corresponding to objects in the scan. and classify them as either cones or obstacles on the track.
STEPS: Use clustering and classification algorithms (e.g., **K-Means**, **DBSCAN**) to detect from the point cloud.

GOAL: Classify the objects obtained previously as either cones or general obstacles.
STEPS: Use point cloud geometry fitting such as **RANSAC** to determine the shape, or perform **ICP** using an ideal cone model to infer its shape.

These levels proved to be challenging, as, the cones are generally made up of a small number number of points, sometimes surrounded by noisy points. 

I've tackled these levels almost simultaneously. I initially focused on the cones, and then I went to the obstacles. 

I started by removing the surfaces of points. I started by working with [Euclidean Cluster Extraction](https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html#cluster-extraction). It gave good results but this algorithm general purpose. The idea for the application of this algorithm was: I recognize the 3 biggest clusters in terms of size (the walls) and I remove them. 

After a while I started working with pass filters: those filters implied that there were only two big clusters (as the upper wall was removed). I wanted a more dynamic solution. 

I found the [Region Growing Segmentation](https://pcl.readthedocs.io/projects/tutorials/en/master/region_growing_segmentation.html#region-growing-segmentation) algorithm so I started playing with its parameters. At the start it proved to be tricky, as often the cones were also removed. 

I put together these two approaches to see the best performing one. With the addition of the [Statistical Outlier Remover](https://pcl.readthedocs.io/projects/tutorials/en/latest/statistical_outlier.html) filter, the Region Growing Segm algorithm gave in general better and more robust result. One useful parameter is the minimum number of points which can form a cluster. Correctly set, this param implies that clusters will contain only walls. Therefore, if one takes the whole cloud and removes the points added to the cluster, the result one gets is a wall-less cloud point -- exactly what we need. 

Now, taking into account the fact that we are using the SOR filter, we now have a lot of clusters with few points. Some of these cluster resemble cones, some do not. The good thing is that, by eye, it's clear which clusters are cones and which ones are not. 

Therefore I started exploring the [Iterative Closest Point](https://pcl.readthedocs.io/projects/tutorials/en/latest/interactive_icp.html) algorithm. Not having blender installed, I created a script in python that creates a PLY file with an ideal cone inside of it. Using this method in the cones recognition pipeline, combined with a good calibration of the SOR algorithm, provided a good-enough recognition of cones. 

There was a problem related to a small cluster of points in the north wall of the point cloud. Part of the north wall was mistaken by a cone. This was no good, as, the cluster in question created a very tight bend, rendering the racing-line estimation tricky. Using an ad-hoc visualizer, I saw that this cluster was rotated extensively so that it fit the ideal cluster. Because all cones have similar inclination, I therefore implemented a yaw check. If a cluster is rotated on the Y axis for more than a certain threshold (now set at 13 degrees), it would be rejected. 

Once I recognized the cones, I removed them from the whole cloud. The cloud now contains only the obstacles. 



### L4 Extract Track Edges
GOAL: Identify the **racing line** based on cone positions.
STEPS: Use clustering algorithms (e.g., **RANSAC**) to fit lines to detected cone clusters or boundaries, helping to define the racing line.

For this task, I explored the possibility of fitting a racing line on the cones clusters. Consulting some papers, I saw that most of the times, the differentiation between the left and the right cone is made easier by the fact that the left cone colored differently from the right cone. This case was tricky for two reason: firstly, there are a small number of cones and secondly, they're placed in a right-bend sharp curve.

I started with prototyping an algorithm on MATLAB: I explored two greedy solutions, one based on relative angles and competitive allocation and the other based on a simulated movement of the robot. The latter applies more generally but works well when there a lot of cones placed one near the other. The former is more naïve but in this particular case seemed more promising. 

One I explored left-right cones separation, I explored curve fitting between cones. I started looking at the library CasADi, in python. It gave good results but the C++ API lacked the documentation I needed. Therefore I looked for alternatives and I found the library [NLopt](https://nlopt.readthedocs.io/en/latest/), which is open source and easy to implement. Additionally, it yearned good results. 

I applied the first fo the left-right algorithms in conjunction with the NLopt library. I then created a point cloud which allows the user to see visually the racing line fitted by NLopt. The results can be improved: the area that would need more attention is recognizing which cones are the left cones and which cones are the right cones. Given the point cloud at hand, that task is tricker than racing line fitting. 

One huge problem that I've encountered is that, for some reason, in the class pcl::PointXYZ, the Z represents the north direction and X the east direction. Because there is this constraint of north-east relationship between the two axis, I couldn't find a good rotation that allowed me to have Z pointing to the north and X to the east. I therefore had to awkwardly swap x and z coordinates on points before applying the left-right algorithm. I then swapped them back for visualization.


### L5 Odometry
GOAL:  Given two point clouds, perform a **pose estimation** relying on **registration algorithms**.00
STEPS: Use a point cloud registration algorithm like **ICP (Iterative Closest Point)** to align the two point clouds, and extract the relative **transformation** between the two. 

I carried out this task in a stand-alone fashion. Having already explored some techniques related to ICP the calculation of the displacement of the robot seemed to me quite straight forward. To maximize the precision of the odometry, I aligned the two clouds with respect to the Y axis, so that the displacement is only related to the horizontal movement of the vehicle. 

### L6 Bonus: Interactive Visualization for Debugging
TASK: Build an interactive interface to visualize the pipeline described above.  

Create a GUI where the user can adjust parameters dynamically (e.g., thresholds for cone detection, edge detection). The processed results should update in **real-time** as the parameters change.

For this step, I use the [PCL + QT](https://pcl.readthedocs.io/projects/tutorials/en/master/qt_colorize_cloud.html#qt-colorize-cloud) tutorial and I tailored it to my needs.
The GUI allowed me to have a clear structure in mind for the project. For the GUI I decided to put into it just the levels 1-4, leaving the level 5 as a stand alone file. I divided files into .cpp source files. Ideally, I would have organized the script in classes as I profoundly believe in the superiority of them. However, I did not have the time to do that, therefore, I tried to organize functions in the most linear way possible. I also made the pipeline quite robust. 

The GUI tool proves very useful, as there are some aspects that need further investigation. If the lateral positioning of the cloud is changed the cone_cloud isn't, but the racing line is. 

## Improvements

In the development of this project, the R&D work took most of development time, so the code is mostly the result of adapted prototyping. This implies that code organization, code readability and memory management is suboptimal. Those areas are the ones where improvement is most needed. 

Secondly, the algorithm for recognizing left and right cones is to be improved and made more robust. It would we worth to exploring the method which simulated the movement of the robots. I'd suggest using a perception method that combines LiDAR data with uncertainty, as the cones which are further from the robots are distorted in size and possibly in position. 

Additionally, it's worth exploring the possibility of non-linear stretching in the data, as from the x-y positions of the cones there seems to be some distortion on distance. The distance between points which are far from the sensor may not be accurately preserved. 

Finally, an interesting area of improvement would be the exploitation of the obstacles for the calculation of the racing line. 

### Graphical Material 

The graphical material is available in the screenshots folder 

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
