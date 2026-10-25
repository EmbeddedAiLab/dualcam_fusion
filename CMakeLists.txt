cmake_minimum_required(VERSION 3.13)
project(dualcam_fusion CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# All modules are built as one static library (see lib/CMakeLists.txt)
add_subdirectory(lib)

add_executable(dualcam_fusion main.cpp)
target_link_libraries(dualcam_fusion PRIVATE dualcam_lib)
