# Simple Ray tracer

## Follow these steps to run the program on Linux Subsystem for Windows (WSL) in VSCode
- sudo apt install cmake (install Cmake)
- sudo apt install g++ (install g++)
- cd Simple Ray Tracer
- cd raytrace
- mkdir build
- cd build
- cmake -DCMAKE_BUILD_TYPE=Debug .. 
(If too slow, replace Debug with Release)
- make
- ./executable
(May take a few minutes to run)
Compare the result with the image in the img directory