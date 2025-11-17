# LiDAR Odometry with ICP (`main.cpp`)

This C++ script performs point cloud odometry using the Iterative Closest Point (ICP) algorithm with PCL. It aligns two `.pcd` files, estimates the transformation, and visualizes the result.

## Features

- Loads two point cloud files (`.pcd`)
- Preprocesses clouds (pass-through filter, outlier removal, transforms)
- Runs ICP to estimate relative pose
- Prints transformation matrix and north/east displacement
- Visualizes original and aligned clouds side-by-side

## Usage

### Build

For the building of this project using CMake is the easiest root. 

```
cmake -DCMAKE_BUILD_TYPE=Debug -G Ninja -S . -B cmake-build-debug  
cd cmake-build-debug  
cmake --build .
./sample_consensus first.pcd second.pcd
```

### Run

```sh
./odometry_icp first.pcd second.pcd
```

## Output

- Console: ICP score, transformation matrix, north/east displacement of vehicle
- GUI: Side-by-side visualization of clouds before/after alignment

## Notes

- Axes: x = East, y = Up, z = North
- Adjust file names and parameters as needed
