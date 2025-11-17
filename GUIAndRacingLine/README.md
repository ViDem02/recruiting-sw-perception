# GUIAndRacingLine

To build this project, the use of CLion is advised. 

This process has been tested for MacOS 15, M3 processor. 

```
cmake -DCMAKE_BUILD_TYPE=Debug -G Ninja -S . -B cmake-build-debug  
cd cmake-build-debug  
cmake --build .
./sample_consensus first.pcd second.pcd
```