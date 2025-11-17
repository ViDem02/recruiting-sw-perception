# GUIAndRacingLine

To build this project, the use of CLion is advised. 

```
cmake -DCMAKE_BUILD_TYPE=Debug -G Ninja -S . -B cmake-build-debug  
cd cmake-build-debug  
cmake --build .
./sample_consensus first.pcd second.pcd
```