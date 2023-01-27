cmake_minimum_required(VERSION 3.10)
project(binpcd)

set(CMAKE_CXX_STANDARD 17)
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

FIND_PACKAGE( Boost 1.58 COMPONENTS program_options REQUIRED )
#add_executable (pcl_visualizer_demo pcl_visualizer_demo.cpp)
#target_link_libraries (pcl_visualizer_demo ${PCL_LIBRARIES})

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
