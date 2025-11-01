set(CMAKE_C_COMPILER /opt/homebrew/opt/llvm/bin/clang)
set(CMAKE_CXX_COMPILER /opt/homebrew/opt/llvm/bin/clang++)

# Trova automaticamente il path dell'SDK macOS
execute_process(
        COMMAND xcrun --show-sdk-path
        OUTPUT_VARIABLE MACOS_SDK_PATH
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(CMAKE_CXX_FLAGS "--sysroot=${MACOS_SDK_PATH}")
set(CMAKE_C_FLAGS "--sysroot=${MACOS_SDK_PATH}")