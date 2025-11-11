# Installation and Build Instructions

## Prerequisites

### System Requirements
- Ubuntu 20.04+ or similar Linux distribution
- CMake 3.10 or higher
- GCC/G++ with C++20 support
- Git

### Required Libraries

#### Point Cloud Library (PCL)
The project requires PCL 1.3 or higher for point cloud processing.

**Ubuntu/Debian:**
```bash
sudo apt-get update
sudo apt-get install libpcl-dev
```

This will install:
- libpcl-common
- libpcl-features
- libpcl-filters
- libpcl-io
- libpcl-kdtree
- libpcl-keypoints
- libpcl-registration
- libpcl-sample-consensus
- libpcl-search
- libpcl-segmentation
- libpcl-surface
- libpcl-visualization

**Verify installation:**
```bash
pkg-config --modversion pcl_common
```

#### Eigen3
PCL depends on Eigen3, which is typically installed with PCL:
```bash
sudo apt-get install libeigen3-dev
```

#### Boost
PCL also requires Boost libraries:
```bash
sudo apt-get install libboost-all-dev
```

#### VTK (for visualization)
```bash
sudo apt-get install libvtk9-dev
```

## Building the Project

### Step 1: Clone Repository
```bash
git clone https://github.com/ViDem02/recruiting-sw-perception.git
cd recruiting-sw-perception/branch_racing_line/racing_line_complete_project
```

### Step 2: Create Build Directory
```bash
mkdir build
cd build
```

### Step 3: Configure with CMake
```bash
cmake ..
```

**Expected output:**
```
-- The C compiler identification is GNU X.X.X
-- The CXX compiler identification is GNU X.X.X
-- Found PCL: 1.14.0
-- Configuring done
-- Generating done
-- Build files have been written to: .../build
```

### Step 4: Build
```bash
make -j$(nproc)
```

**This will create:**
- `racingline_nlopt` - Main demo application
- `libcones_recognizer.a` - ConesRecognizer library
- `libdistinguish_left_right.a` - DistinguishLeftRight library
- `libcones_recog.a` - Legacy procedural library

### Step 5: Verify Build
```bash
ls -lh racingline_nlopt
```

## Troubleshooting

### CMake Cannot Find PCL
**Error:**
```
CMake Error: By not providing "FindPCL.cmake" in CMAKE_MODULE_PATH...
```

**Solution:**
```bash
sudo apt-get install libpcl-dev
# or specify PCL_DIR manually:
cmake -DPCL_DIR=/usr/lib/x86_64-linux-gnu/cmake/pcl ..
```

### Missing Eigen3
**Error:**
```
fatal error: Eigen/Dense: No such file or directory
```

**Solution:**
```bash
sudo apt-get install libeigen3-dev
```

### C++20 Not Supported
**Error:**
```
CMake Error: CMAKE_CXX_STANDARD is set to 20 but...
```

**Solution:**
Update your compiler:
```bash
sudo apt-get install g++-11
export CXX=g++-11
cmake ..
```

### Boost Not Found
**Error:**
```
Could not find the following Boost libraries: system filesystem...
```

**Solution:**
```bash
sudo apt-get install libboost-all-dev
```

## Running the Application

### Basic Usage
```bash
./racingline_nlopt scan.pcd
```

### With Robot Position
```bash
./racingline_nlopt scan.pcd 0.0 0.0
```

### With Distance Filtering
```bash
# Only detect cones within 10 meters
./racingline_nlopt scan.pcd 0.0 0.0 10.0
```

### Test Data
If you don't have test data, you can use any PCD file from the RD examples:
```bash
./racingline_nlopt ../../RD/5-extract_indices/cones.pcd 0.0 0.0 15.0
```

## Development Setup

### IDE Configuration (VS Code)
Create `.vscode/c_cpp_properties.json`:
```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/usr/include/pcl-1.14",
                "/usr/include/eigen3",
                "/usr/include/vtk-9.1"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/g++",
            "cStandard": "c17",
            "cppStandard": "c++20",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}
```

### Debugging Configuration
Create `.vscode/launch.json`:
```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Debug racingline_nlopt",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/build/racingline_nlopt",
            "args": ["cones.pcd", "0.0", "0.0", "10.0"],
            "stopAtEntry": false,
            "cwd": "${workspaceFolder}/build",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb"
        }
    ]
}
```

## Testing

### Unit Tests (Future Enhancement)
Unit tests are not yet implemented. To add them:

1. Install Google Test:
```bash
sudo apt-get install libgtest-dev
```

2. Create test files and update CMakeLists.txt

### Manual Testing
```bash
# Test with different configurations
./racingline_nlopt test_scan.pcd 0.0 0.0 5.0   # 5m radius
./racingline_nlopt test_scan.pcd 0.0 0.0 10.0  # 10m radius
./racingline_nlopt test_scan.pcd 0.0 0.0       # No distance limit
```

## Docker Setup (Alternative)

If you prefer a containerized environment:

```dockerfile
FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libpcl-dev \
    libeigen3-dev \
    libboost-all-dev \
    libvtk9-dev

WORKDIR /workspace
COPY . .

RUN mkdir -p build && cd build && cmake .. && make

CMD ["/bin/bash"]
```

Build and run:
```bash
docker build -t cones-recognizer .
docker run -it cones-recognizer
```

## Continuous Integration

### GitHub Actions Example
Create `.github/workflows/build.yml`:
```yaml
name: Build

on: [push, pull_request]

jobs:
  build:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3
      
      - name: Install dependencies
        run: |
          sudo apt-get update
          sudo apt-get install -y libpcl-dev cmake
      
      - name: Build
        run: |
          cd branch_racing_line/racing_line_complete_project
          mkdir build && cd build
          cmake ..
          make -j$(nproc)
      
      - name: Test
        run: |
          cd branch_racing_line/racing_line_complete_project/build
          ./racingline_nlopt --help || true
```

## Performance Optimization

### Build Types

**Debug Build** (default):
```bash
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
```

**Release Build** (optimized):
```bash
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

**Release with Debug Info:**
```bash
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make
```

### Compiler Flags
For maximum performance, edit CMakeLists.txt:
```cmake
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")
```

## Next Steps

After successful installation:
1. Read [README.md](README.md) for usage overview
2. Review [EXAMPLES.md](EXAMPLES.md) for code examples
3. Study [REFACTORING.md](REFACTORING.md) to understand design decisions
4. Check [SUMMARY.md](SUMMARY.md) for quick reference

## Support

For issues or questions:
- Check existing documentation
- Review troubleshooting section
- Create an issue on GitHub
- Review PCL documentation: https://pointclouds.org/

## License

See repository LICENSE file for details.
