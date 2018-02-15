// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <urdf/model.h>

// VTK Lidar Scanner
#include <lidar_scanner_node/scanner_params.h>
#include <lidar_scanner_node/lidar_scanner_simulator.h>
#include <lidar_scanner_node/scene_builder.h>

// VTK Utils
#include <vtk_viewer/vtk_utils.h>
#include <vtk_viewer/vtk_viewer.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkAppendPolyData.h>

#include <memory>

const static double SCAN_FREQUENCY = 10.0; // Hz
const static double TF_TIMEOUT = 30.0; // seconds
const static double TF_FILTER_DIST = 0.001; // meters
const static double TF_FILTER_DIST_SQR = TF_FILTER_DIST * TF_FILTER_DIST;

static bool distanceComparator(const tf::StampedTransform& a,
                               const tf::StampedTransform& b)
{
  const tf::Transform diff = a.inverseTimes(b);
  const double d2 = diff.getOrigin().length2();
  return d2 > TF_FILTER_DIST_SQR;
}


bool getParams(const XmlRpc::XmlRpcValue& resources,
               std::vector<SceneObject>& scene_objects)
{
  XmlRpc::XmlRpcValue r = resources;

  if(r.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Mesh resources parameter is not an array!");
    return false;
  }
  if(r.size() < 1)
  {
    ROS_ERROR("Mesh resources parameter has no mesh items");
    return false;
  }

  for(int i = 0; i < r.size(); ++i)
  {
    XmlRpc::XmlRpcValue& mesh = r[i];
    if(!mesh.hasMember("file"))
    {
      ROS_WARN("No file path specified; skipping mesh resource %d...", i);
      continue;
    }
    else if(!mesh.hasMember("pose"))
    {
      ROS_WARN("No pose parameter specified; skipping mesh resource %d...", i);
      continue;
    }
    else if(mesh.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_WARN("Mesh parameter is not a struct; skipping mesh resource %d...", i);
      continue;
    }
    else
    {
      XmlRpc::XmlRpcValue& file = mesh["file"];
      if(file.getType() != XmlRpc::XmlRpcValue::TypeString)
      {
        ROS_WARN("File parameter is not a string; skipping mesh resource %d...", i);
        continue;
      }

      XmlRpc::XmlRpcValue& pose = mesh["pose"];
      if(pose.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_WARN("Pose parameter is not an array; skipping mesh resource %d...", i);
        continue;
      }
      if(pose.size() != 6)
      {
        ROS_WARN("Pose parameter does not have 6 elements; skipping mesh resource %d...", i);
        continue;
      }

      std::string path = static_cast<std::string>(file);
      std::vector<double> vals;
      for(int j = 0; j < 6; ++j)
      {
        double val = static_cast<double>(pose[j]);
        vals.push_back(val);
      }

      SceneObject obj (path, vals);
      scene_objects.push_back(obj);
    }
  }

  if(scene_objects.empty())
  {
    return false;
  }

  return true;
}

bool loadScannerParams(ros::NodeHandle& nh,
                       scanner_params& sim)
{
  if(!nh.getParam("scanner/theta_span", sim.theta_span))
  {
    ROS_ERROR("Theta span parameter must be set");
    return false;
  }
  if(!nh.getParam("scanner/phi_span", sim.phi_span))
  {
    ROS_ERROR("Phi span parameter must be set");
    return false;
  }
  if(!nh.getParam("scanner/theta_points", sim.theta_points))
  {
    ROS_ERROR("Theta points parameter must be set");
    return false;
  }
  if(!nh.getParam("scanner/phi_points", sim.phi_points))
  {
    ROS_ERROR("Phi points parameter must be set");
    return false;
  }
  if(!nh.getParam("scanner/los_variance", sim.los_variance))
  {
    ROS_ERROR("LOS variance parameter must be set");
    return false;
  }
  if(!nh.getParam("scanner/orthogonal_variance", sim.orthogonal_variance))
  {
    ROS_ERROR("Orthogonal variance parameter must be set");
    return false;
  }
  if(!nh.getParam("scanner/max_incidence_angle", sim.max_incidence_angle))
  {
    ROS_INFO("max_incidence_angle parameter not set; disabling incidence check");
    sim.max_incidence_angle = 0.0;
  }
  if(!nh.getParam("scanner/max_distance", sim.max_distance))
  {
    ROS_INFO("max_distance parameter not set; disabling distance filter");
    sim.max_distance = 0.0;
  }

  return true;
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "lidar_sim_node");

  // Set up ROS node handle
  ros::NodeHandle pnh("~");

  // Load the general parameters
  std::string world_frame;
  if(!pnh.getParam("world_frame", world_frame))
  {
    ROS_ERROR("World frame name parameter must be set");
    return false;
  }

  std::string scanner_frame;
  if(!pnh.getParam("scanner_frame", scanner_frame))
  {
    ROS_ERROR("Scanner frame name parameter must be set");
    return false;
  }

  // Get scanner parameters
  scanner_params sim;
  if(!loadScannerParams(pnh, sim))
  {
    return -1;
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

  XmlRpc::XmlRpcValue p;
  if(pnh.getParam("resources", p))
  {
    std::vector<SceneObject> scene_objects;
    if(getParams(p, scene_objects))
    {
      if(!builder.createVTKSceneFromMeshResources(scene_objects))
      {
        return -1;
      }
    }
    else
    {
      ROS_WARN("Failed to parse mesh resource parameters correctly");
      ROS_WARN("Building scene from static geometry in URDF");
      if(!builder.createVTKSceneFromURDF())
      {
        return -1;
      }
    }
  }
  else
  {
    ROS_WARN("Failed to find mesh resources parameter");
    ROS_WARN("Building scene from static geometry in URDF");
    if(!builder.createVTKSceneFromURDF())
    {
      return -1;
    }
  }

  scanner.setScannerScene(builder.scene);
  ROS_INFO("Simulated LIDAR scanner initialization completed. Looking up data from scanner frame %s in world frame %s."
           "Beginning data acquisition...", scanner_frame.c_str(), world_frame.c_str());

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

    // We time stamp the data prior the moment prior to performing raytracing
    // instead of after its done b/c the raytracing can take a while.
    scan_data_msg.header.stamp = ros::Time::now()-ros::Duration(1.0);
    scan_data_msg.header.frame_id = scanner_frame;

    // Update scan information if TF frame has moved significantly
    if(distanceComparator(previous_transform, transform))
    {
      // Set scanner transform and dynamically changing elements of the scene
      scanner.getNewScanData(transform);

      // Convert scan data from private class object to ROS msg
      pcl::PointCloud<pcl::PointNormal>::Ptr data = scanner.getScanDataPointCloud();
      pcl::toROSMsg(*data, scan_data_msg);
    }

    scan_pub.publish(scan_data_msg);

    previous_transform = transform;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
