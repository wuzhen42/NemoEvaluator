## How to build
```
mkdir build
cd build
conan install ..
cmake -G "Visual Studio 17 2022" -A x64 -DHOUDINI_VERSION=20 ..
cmake --build . --config Release
```