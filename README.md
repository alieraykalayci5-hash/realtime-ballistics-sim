## Build (Windows / MSYS2 UCRT64)

This project is built with CMake + Ninja and a MinGW-w64 (UCRT) toolchain.

### Prerequisites (MSYS2 UCRT64)
Install packages:
- `mingw-w64-ucrt-x86_64-toolchain`
- `mingw-w64-ucrt-x86_64-cmake`
- `mingw-w64-ucrt-x86_64-ninja`

### Configure + build
```bash
cd /c/Users/AliEray/Desktop/Staj-Proje/realtime-ballistics-sim
cmake -S . -B build -G Ninja
cmake --build build -j