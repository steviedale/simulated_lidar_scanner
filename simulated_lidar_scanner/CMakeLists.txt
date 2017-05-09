cmake_minimum_required(VERSION 2.8.3)
project(simulated_lidar_scanner)

find_package(catkin REQUIRED cmake_modules)
find_package(VTK 7.1 CONFIG)
find_package(VTK 7.1 REQUIRED)
include(${VTK_USE_FILE})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES simulated_lidar_scanner
  DEPENDS VTK
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
add_library(simulated_lidar_scanner SHARED
 src/vtkLidarScanner.cxx
 src/vtkRay.cxx
 src/vtkLidarPoint.cxx
)

target_link_libraries(simulated_lidar_scanner
 ${catkin_LIBRARIES}
 ${VTK_LIBRARIES}
)
