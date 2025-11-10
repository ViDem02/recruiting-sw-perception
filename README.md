# E-Agle Formula Student Driverless

Welcome to the portfolio for the E-Agle Formula Student Driverless Perception Task. This repository showcases a modular, multi-level approach to autonomous vehicle perception using C++, CMake, and Point Cloud Library (PCL).

## 🚗 Project Overview
The goal is to build a robust perception pipeline for a Formula Student Driverless vehicle, processing LiDAR data to extract actionable information for autonomous racing. The project is organized into progressive levels, each focusing on a core aspect of perception:

| Level | Task |
|-------|------|
| 1     | Load, display, and preprocess LiDAR data |
| 2     | Detect cones |
| 3     | Classify objects (cones/obstacles) |
| 4     | Extract racing line |
| 5     | Odometry (pose estimation) |
| Bonus | Interactive visualization for debugging |

## 🛠️ Technologies Used
- **C++**
- **CMake**
- **PCL (Point Cloud Library)** (LiDAR processing)
- **Git** (version control)

## 📂 Structure & Highlights
- **LiDAR Path:** Levels 1-4 use `cones.pcd`; Level 5 uses `first.pcd` and `second.pcd`.
- **Modular Design:** Each level is implemented as a separate module or folder, making it easy to follow progress and reuse code.
- **CMake Integration:** All code is buildable via CMake for cross-platform compatibility.
- **Bonus:** Interactive GUI for real-time parameter tuning and visualization.

## 🏁 Task Breakdown
### Level 1: Load & Display
- Load and visualize LiDAR data using PCL.

### Level 2: Cone Detection
- Detect cones using geometry thresholding, clustering, or ML classifiers.

### Level 3: Object Classification
- Classify cones by shape (LiDAR).

### Level 4: Racing Line Extraction
- Identify the racing line using clustering and geometric fitting.

### Level 5: Odometry
- Estimate pose using ICP registration (LiDAR).

### Bonus: Interactive Visualization
- GUI for dynamic parameter adjustment and real-time feedback.

## 📜 Submission & Documentation
- **Code:** Well-documented, modular, and version-controlled.
- **CMakeLists.txt:** Provided for easy compilation.
- **Report:** Approach, challenges, and improvements (PDF/Markdown).
- **Visuals:** Screenshots/GIFs of results and GUI.

## 📅 Timeline
- **Completion Target:** 2 weeks

## 💡 Portfolio Value
This project demonstrates:
- Advanced perception algorithms for autonomous vehicles
- Practical use of C++ and modern libraries
- Modular, scalable code organization
- Effective use of version control and build systems
- Interactive tools for debugging and visualization

---

For details on each level, see the corresponding folders and documentation. Contributions, questions, and feedback are welcome!