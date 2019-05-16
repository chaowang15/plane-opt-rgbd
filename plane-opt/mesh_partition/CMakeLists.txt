cmake_minimum_required(VERSION 3.0)

# set(Boost_INCLUDE_DIR /usr/local/include)
# set(Boost_LIBRARY_DIR /usr/local/lib)
find_package(Boost COMPONENTS system filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})
# message(${Boost_LIBRARIES})

SET(EIGEN_INCLUDE_DIR "/usr/include/eigen3")
message("Eigen path: ${EIGEN_INCLUDE_DIR}")
include_directories(${EIGEN_INCLUDE_DIR})

file(GLOB SRC "*.cpp")
# message("cpp file list: ${SRC}")
add_executable(mesh_partition ${SRC})
target_link_libraries( mesh_partition ${Boost_LIBRARIES} )
