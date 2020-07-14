cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_to_2dmap)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

###########
## Build ##
###########

include_directories(
  include
  ${CV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pointcloud_to_2dmap src/pointcloud_to_2dmap.cpp)
target_link_libraries(pointcloud_to_2dmap
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
