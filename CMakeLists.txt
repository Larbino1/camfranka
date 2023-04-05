cmake_minimum_required(VERSION 3.4)

project(CamFranka)

find_package(Eigen3 REQUIRED)
add_library(Eigen3::Eigen3 INTERFACE IMPORTED)
set_target_properties(Eigen3::Eigen3 PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS}
    INTERFACE_COMPILE_DEFINITIONS "${EIGEN3_DEFINITIONS}"
)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

## Submodules
add_subdirectory(submodules)
## My libraries
add_subdirectory(lib)
## My robot control examples
add_subdirectory(examples)
