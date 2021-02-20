cmake_minimum_required(VERSION 3.1)

set (CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# For Ubuntu 20.04 or custom OpenCV build set appropriate path
# set (OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4/)

find_package( OpenCV REQUIRED )
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories( ${OpenCV_INCLUDE_DIRS} )

project(
    hybrid-a-star
)

# those are our main targets
add_executable(simple-example node_se2.cpp examples/simple_example.cpp)
target_link_libraries(simple-example ompl ${OpenCV_LIBS} Eigen3::Eigen)

add_executable(gui-example node_se2.cpp examples/gui_example.cpp)
target_link_libraries(gui-example ompl ceres ${OpenCV_LIBS} Eigen3::Eigen)

