cmake_minimum_required(VERSION 3.1)

set (CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

project(
    hybrid-a-star
)

# those are our main targets

add_executable(simple-example node_se2.cpp examples/simple_example.cpp)
target_link_libraries(simple-example ompl ${OpenCV_LIBS})

add_executable(gui-example node_se2.cpp examples/gui_example.cpp)
target_link_libraries(gui-example ompl ${OpenCV_LIBS})
