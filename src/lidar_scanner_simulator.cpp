#include <lidar_scanner_node/lidar_scanner_simulator.h>
#include <lidar_scanner_node/scanner_params.h>
#include <vtk_viewer/vtk_utils.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <vtkTransform.h>
#include <vtkPolyData.h>

bool incidenceFilter(const pcl::PointNormal& pt,
                     const double max_angle)
{
  Eigen::Vector3d ray (pt.x, pt.y, pt.z);
  Eigen::Vector3d normal (pt.normal_x, pt.normal_y, pt.normal_z);
  const double c_theta = normal.dot(ray.normalized());
  return (c_theta > max_angle);
}

bool distanceFilter(const pcl::PointNormal& pt,
                    const double max_dist)
{
  const double dist2 = pt.x*pt.x + pt.y*pt.y + pt.z*pt.z;
  return (dist2 > max_dist*max_dist);
}

LidarScannerSim::LidarScannerSim(const scanner_params& sim)
{
  // Initialize scanner and scan data
  scanner_ = vtkSmartPointer<vtkLidarScanner>::New();
  scan_data_vtk_ = vtkSmartPointer<vtkPolyData>::New();
  scan_data_cloud_.reset(new pcl::PointCloud<pcl::PointNormal> ());

  // Set scanner parameters
  scanner_->SetPhiSpan(sim.phi_span);
  scanner_->SetThetaSpan(sim.theta_span);
  scanner_->SetNumberOfThetaPoints(sim.theta_points);
  scanner_->SetNumberOfPhiPoints(sim.phi_points);
  scanner_->SetLOSVariance(sim.los_variance);
  scanner_->SetOrthogonalVariance(sim.orthogonal_variance);
  scanner_->SetStoreRays(false);
  scanner_->SetCreateMesh(false);

  if(sim.max_incidence_angle > 0.0)
  {
    enable_incidence_filter_ = true;
    max_incidence_angle_ = -std::cos(sim.max_incidence_angle * M_PI / 180.0);
  }

  if(sim.max_distance > 0.0)
  {
    enable_distance_filter_ = true;
    max_distance_ = sim.max_distance;
  }
}

void LidarScannerSim::setScannerTransform(const tf::StampedTransform& frame)
{
  tf::Vector3 origin = frame.getOrigin();
  tf::Matrix3x3 orientation_mat = frame.getBasis();

  vtkSmartPointer<vtkMatrix4x4> vtk_matrix = vtkSmartPointer<vtkMatrix4x4>::New();
  for(unsigned int j = 0; j < 4; ++j)
  {
    if(j != 3)
    {
      vtk_matrix->SetElement(0, j, orientation_mat.getColumn(j).getX());
      vtk_matrix->SetElement(1, j, orientation_mat.getColumn(j).getY());
      vtk_matrix->SetElement(2, j, orientation_mat.getColumn(j).getZ());
    }
    else
    {
      vtk_matrix->SetElement(0, j, origin.getX());
      vtk_matrix->SetElement(1, j, origin.getY());
      vtk_matrix->SetElement(2, j, origin.getZ());
    }
  }

  // Create transform from matrix; rotate such that sensor axis is Z axis
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->SetMatrix(vtk_matrix);
  transform->RotateX(90.0f);

  // Set and update transform in scanner object
  scanner_->SetTransform(transform);
  scanner_frame_ = frame;
}

void LidarScannerSim::getNewScanData(const tf::StampedTransform& scanner_transform)
{
  // Set the scanner transform
  setScannerTransform(scanner_transform);

  // Perform the scan and get the points
  scanner_->PerformScan();
  scanner_->GetValidOutputPoints(scan_data_vtk_);

  pcl::PointCloud<pcl::PointNormal> cloud;
  vtk_viewer::VTKtoPCL(scan_data_vtk_, cloud);

  // Transform point cloud into scanner frame
  Eigen::Affine3d transform;
  tf::transformTFToEigen(scanner_frame_.inverse(), transform);
  pcl::transformPointCloudWithNormals(cloud, *scan_data_cloud_, transform);

  // Cull points whose angle of incidence is greater than the specified tolerance
  if(enable_incidence_filter_)
  {
    scan_data_cloud_->erase(std::remove_if(scan_data_cloud_->points.begin(),
                                          scan_data_cloud_->points.end(),
                                          boost::bind(incidenceFilter, _1, max_incidence_angle_)),
                           scan_data_cloud_->end());
  }

  if(enable_distance_filter_)
  {
    scan_data_cloud_->erase(std::remove_if(scan_data_cloud_->points.begin(),
                                          scan_data_cloud_->points.end(),
                                          boost::bind(distanceFilter, _1, max_distance_)),
                           scan_data_cloud_->end());
  }
}
