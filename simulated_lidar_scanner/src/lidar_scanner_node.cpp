#include <ros/ros.h>
#include <ros/package.h>
#include <simulated_lidar_scanner/lidar_scanner_simulator.h>
#include <simulated_lidar_scanner/scene_builder.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkAppendPolyData.h>
#include <urdf/model.h>

const static double SCAN_FREQUENCY = 10.0; // Hz
const static double TF_TIMEOUT = 10.0; // seconds
const static double TF_FILTER_DIST = 0.025; // meters
const static double TF_FILTER_DIST_SQR = TF_FILTER_DIST * TF_FILTER_DIST;
const static double TF_FILTER_ANGLE = 1.0*(M_PI/180.0);

static bool distanceComparator(const tf::StampedTransform& a,
                               const tf::StampedTransform& b)
{
  const tf::Transform diff = a.inverseTimes(b);
  const double d2 = diff.getOrigin().length2();
  const tfScalar q = diff.getRotation().getAngle();
  return (d2 > TF_FILTER_DIST_SQR || q > TF_FILTER_ANGLE);
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "lidar_sim_node");

  // Set up ROS node handle
  ros::NodeHandle pnh("~");

  // Get scanner parameters
  scanner_params sim;

  if(!pnh.getParam("scanner/theta_span", sim.theta_span))
  {
    ROS_FATAL("Theta span parameter must be set");
    return 1;
  }
  sim.theta_span *= M_PI / 180.0;
  if(!pnh.getParam("scanner/phi_span", sim.phi_span))
  {
    ROS_FATAL("Phi span parameter must be set");
    return 1;
  }
  sim.phi_span *= M_PI / 180.0;
  if(!pnh.getParam("scanner/theta_points", sim.theta_points))
  {
    ROS_FATAL("Theta points parameter must be set");
    return 1;
  }
  if(!pnh.getParam("scanner/phi_points", sim.phi_points))
  {
    ROS_FATAL("Phi points parameter must be set");
    return 1;
  }
  if(!pnh.getParam("scanner/los_variance", sim.los_variance))
  {
    ROS_FATAL("LOS variance parameter must be set");
    return 1;
  }
  if(!pnh.getParam("scanner/orthogonal_variance", sim.orthogonal_variance))
  {
    ROS_FATAL("Orthogonal variance parameter must be set");
    return 1;
  }
  if(!pnh.getParam("scanner/max_incidence_angle", sim.max_incidence_angle))
  {
    ROS_INFO("max_incidence_angle parameter not set; disabling incidence check");
    sim.max_incidence_angle = 0.0;
  }
  if(!pnh.getParam("scanner/max_distance", sim.max_distance))
  {
    ROS_INFO("max_distance parameter not set; disabling distance filter");
    sim.max_distance = 0.0;
  }

  std::string world_frame;
  if(!pnh.getParam("world_frame", world_frame))
  {
    ROS_FATAL("World frame name parameter must be set");
    return 1;
  }

  std::string scanner_frame;
  if(!pnh.getParam("scanner_frame", scanner_frame))
  {
    ROS_FATAL("Scanner frame name parameter must be set");
    return 1;
  }

  // Create a ROS tf listener to get the scanner frame
  tf::TransformListener listener;
  if(!listener.waitForTransform(world_frame, scanner_frame, ros::Time(0), ros::Duration(TF_TIMEOUT)))
  {
    ROS_FATAL("TF listener timeout for '%s' to '%s' transform", world_frame.c_str(), scanner_frame.c_str());
    return 0;
  }

  // Create scanner class
  LidarScannerSim scanner(sim);

  // Set static scene
  SceneBuilder builder;
  if(!builder.createVTKScene())
  {
    ROS_FATAL("Unable to create VTK scene");
    return 1;
  }

  scanner.setScannerScene(builder.getVTKScene());

  ROS_INFO("Simulated LIDAR scanner initialized. Beginning data acquisition...");

  // Create a ROS publisher to publish the scan data
  ros::Publisher scan_pub = pnh.advertise<sensor_msgs::PointCloud2>("/sensor_data/" + scanner_frame, 1, false);

  // Set initial previous transform to identity matrix
  tf::StampedTransform previous_transform;
  previous_transform.setIdentity();

  sensor_msgs::PointCloud2 scan_data_msg;

  ros::Rate loop_rate(SCAN_FREQUENCY);
  while(pnh.ok())
  {
    // Get TF frame from absolute static world origin to scanner focal point
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform(world_frame, scanner_frame, ros::Time(0), transform);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      continue;
    }

    // Update scan information if TF frame has moved significantly
    if(distanceComparator(previous_transform, transform))
    {
      // Set scanner transform and dynamically changing elements of the scene
//      scanner.setScannerScene(scene);
      scanner.getNewScanData(transform);

      // Convert scan data from private class object to ROS msg
      pcl::PointCloud<pcl::PointNormal>::Ptr data = scanner.getScanDataPointCloud();
      pcl::toROSMsg(*data, scan_data_msg);
    }

    scan_data_msg.header.stamp = ros::Time::now();
    scan_data_msg.header.frame_id = scanner_frame;
    scan_pub.publish(scan_data_msg);

    previous_transform = transform;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
