cmake_minimum_required (VERSION 3.1)

project (blur_estimation)

set(CMAKE_BUILD_TYPE "Release")

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
file(GLOB sources "*.cpp")

add_executable(blur_estimation ${sources})
target_link_libraries(blur_estimation ${OpenCV_LIBS})
