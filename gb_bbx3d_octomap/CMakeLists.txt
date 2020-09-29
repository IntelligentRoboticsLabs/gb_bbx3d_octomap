cmake_minimum_required(VERSION 3.5)
project(gb_bbx3d_octomap)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(gb_visual_detection_3d_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2
  octomap
  octomap_msgs
  gb_visual_detection_3d_msgs
)

include_directories(
  include/
)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/Bbx3D2Octomaps.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(bbx3d2octomaps_node src/bbx3d2octomaps_node.cpp)
ament_target_dependencies(bbx3d2octomaps_node ${dependencies})
target_link_libraries(bbx3d2octomaps_node ${PROJECT_NAME} octomap)

target_include_directories(bbx3d2octomaps_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


#########
#INSTALL#
#########

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  bbx3d2octomaps_node
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

#ament_export_include_directories(include)

ament_export_dependencies(${dependencies})

ament_package()
