cmake_minimum_required(VERSION 2.8.3)
project(lidar_scanner_node)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  tf
  tf_conversions
  roslib
  urdf
  interactive_markers
  visualization_msgs
  )
find_package(VTK 7.1 CONFIG)
find_package(VTK 7.1 REQUIRED)
include(${VTK_USE_FILE})

find_package(cmake_modules REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES simulated_lidar_scanner
  CATKIN_DEPENDS
    roscpp
    tf
  DEPENDS
    VTK
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(synthetic_lidar_scanner SHARED
 src/synthetic_lidar_scanner/vtkLidarScanner.cxx
 src/synthetic_lidar_scanner/vtkRay.cxx
 src/synthetic_lidar_scanner/vtkLidarPoint.cxx
)

target_link_libraries(synthetic_lidar_scanner
 ${catkin_LIBRARIES}
 ${VTK_LIBRARIES}
)

add_executable(lidar_scanner_node src/lidar_scanner_node.cpp
                                  src/lidar_scanner_simulator.cpp
                                  src/scene_builder.cpp)

#add_dependencies(lidar_scanner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(lidar_scanner_node
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  synthetic_lidar_scanner)

add_executable(scanner_relocator src/scanner_relocator.cpp)

target_link_libraries(scanner_relocator
  ${catkin_LIBRARIES})
