# How to run the project

## Manual mode

```bash
 cmake -S . -B build -G Ninja \
  -DCMAKE_CXX_COMPILER=/opt/homebrew/opt/llvm/bin/clang++ \
  -DCMAKE_C_COMPILER=/opt/homebrew/opt/llvm/bin/clang \
  -DCMAKE_CXX_FLAGS="--sysroot=$(xcrun --show-sdk-path)"

cmake --build build -v

cd build
./4_modules
```

## CLion

Go to File → Settings → Build, Execution, Deployment → CMake → CMake options
and write:
`-DCMAKE_TOOLCHAIN_FILE=llvm-toolchain.cmake`

Also set the Generator to Ninja.

Everything should work.




