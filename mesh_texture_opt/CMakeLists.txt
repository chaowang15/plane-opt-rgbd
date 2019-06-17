cmake_minimum_required(VERSION 3.1)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(gflags REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_BUILD_TYPE "Release")
SET(EIGEN_INCLUDE_DIR "/usr/include/eigen3")
# message("Eigen path: ${EIGEN_INCLUDE_DIR}")
include_directories(${EIGEN_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})


file(GLOB SRC "*.cpp")
# message("cpp file list: ${SRC}")
add_executable(mesh_texture_opt ${SRC} ../common/covariance.cpp)
target_link_libraries( mesh_texture_opt ${OpenCV_LIBS} gflags)
