cmake_minimum_required(VERSION 3.15)
project(kinetic_delaunay)

set(CMAKE_CXX_STANDARD 14)
set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/build/bin)

find_package(Opencv REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(include ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_subdirectory(src)
add_subdirectory(tests)
add_subdirectory(lib/googletest)

