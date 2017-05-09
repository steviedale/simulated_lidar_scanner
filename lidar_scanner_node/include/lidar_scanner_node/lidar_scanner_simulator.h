#ifndef LIDAR_SCANNER_SIMULATOR_H
#define LIDAR_SCANNER_SIMULATOR_H

#include <lidar_scanner_node/synthetic_lidar_scanner/vtkLidarScanner.h>
#include <lidar_scanner_node/scanner_params.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LidarScannerSim
{
public:
  LidarScannerSim(const scanner_params& sim);
  void setScannerScene(const vtkSmartPointer<vtkPolyData>& scene) {scanner_->SetScene(scene);}
  void setScannerTransform(const tf::StampedTransform& frame);
  void getNewScanData(const tf::StampedTransform& scanner_transform);
  vtkSmartPointer<vtkPolyData> getScanPolyData() const {return scan_data_vtk_;}
  pcl::PointCloud<pcl::PointNormal>::Ptr getScanDataPointCloud() const {return scan_data_cloud_;}
private:
  vtkSmartPointer<vtkLidarScanner> scanner_;
  vtkSmartPointer<vtkPolyData> scan_data_vtk_;
  pcl::PointCloud<pcl::PointNormal>::Ptr scan_data_cloud_;
  tf::StampedTransform scanner_frame_;
  bool enable_incidence_filter_ = false;
  double max_incidence_angle_;
  bool enable_distance_filter_ = false;
  double max_distance_;
};

#endif // LIDAR_SCANNER_SIMULATOR_H
