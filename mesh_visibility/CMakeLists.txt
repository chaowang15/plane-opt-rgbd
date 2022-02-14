cmake_minimum_required (VERSION 3.1)

project (mesh_visibility)

# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_FLAGS "-fpermissive")
# set(CMAKE_CXX_COMPILER /usr/bin/g++)
set(CMAKE_BUILD_TYPE "Release")

find_package(OpenCV REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS} glm)


# This is to copy shader files to your build folder
configure_file(${CMAKE_SOURCE_DIR}/rendermode.vert ${CMAKE_BINARY_DIR}/rendermode.vert COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/rendermode.frag ${CMAKE_BINARY_DIR}/rendermode.frag COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/savemode.vert ${CMAKE_BINARY_DIR}/savemode.vert COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/savemode.frag ${CMAKE_BINARY_DIR}/savemode.frag COPYONLY)


file(GLOB sources "*.cpp")

add_executable(mesh_visibility ${sources})

target_link_libraries(mesh_visibility ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLEW_LIBRARIES} glfw dl X11 pthread Xrandr Xi Xxf86vm Xinerama Xcursor)
