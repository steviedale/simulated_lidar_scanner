#include <lidar_scanner_node/scene_builder.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <urdf/model.h>
#include <boost/filesystem.hpp>

// VTK Utils
#include <vtk_viewer/vtk_utils.h>
#include <vtk_viewer/vtk_viewer.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkAppendPolyData.h>

namespace
{
  vtkSmartPointer<vtkTransform> urdfPoseToVTKTransform(const urdf::Pose& pose)
  {
    urdf::Rotation q = pose.rotation;
    urdf::Vector3 p = pose.position;
    double w, x, y, z, denom;

    if(q.w > 1) {q.normalize();}
    w = 2*std::acos(q.w);
    denom = std::sqrt(1 - q.w*q.w);
    if(denom < 0.001)
    {
      // Choose an arbitrary axis since the rotation angle ~= 0
      x = 0.0; //q.x;
      y = 0.0; //q.y;
      z = 1.0; //q.z;
    }
    else
    {
      x = q.x / denom;
      y = q.y / denom;
      z = q.z / denom;
    }

    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->Translate(p.x, p.y, p.z);
    transform->RotateWXYZ(w*180/M_PI, x, y, z);

    return transform;
  }

  Eigen::Affine3d urdfPoseToEigen(const urdf::Pose& pose)
  {
    Eigen::Affine3d transform (Eigen::Affine3d::Identity());
    transform.translate(Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z));
    transform.rotate(Eigen::Quaterniond(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z));

    return transform;
  }

  vtkSmartPointer<vtkTransform> eigenTransformToVTK(const Eigen::Affine3d& transform)
  {
    vtkSmartPointer<vtkTransform> vtk_transform = vtkSmartPointer<vtkTransform>::New();

    Eigen::Translation3d translation (transform.translation());
    vtk_transform->Translate(translation.x(), translation.y(), translation.z());

    Eigen::AngleAxisd rotation (transform.rotation());
    vtk_transform->RotateWXYZ(rotation.angle()*180.0/M_PI, rotation.axis()[0], rotation.axis()[1], rotation.axis()[2]);

    return vtk_transform;
  }

  visualization_msgs::Marker createMeshResourceMarker(const std::string& filename,
                                                      const Eigen::Affine3d& transform,
                                                      const std::string& frame,
                                                      const int id)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();
    marker.id = id;

    marker.action = marker.ADD;
    marker.type = marker.MESH_RESOURCE;
    marker.lifetime = ros::Duration(0);
    marker.frame_locked = false;
    marker.mesh_resource = filename;
    marker.mesh_use_embedded_materials = true;

    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;

    Eigen::Translation3d translation (transform.translation());
    Eigen::AngleAxisd rotation (transform.rotation());

    marker.pose.position.x = translation.x();
    marker.pose.position.x = translation.y();
    marker.pose.position.x = translation.z();
    marker.pose.orientation.w = rotation.angle();
    marker.pose.orientation.x = rotation.axis()[0];
    marker.pose.orientation.y = rotation.axis()[1];
    marker.pose.orientation.z = rotation.axis()[2];

    return marker;
  }
}

SceneBuilder::SceneBuilder()
  : scene(vtkSmartPointer<vtkPolyData>::New())
{
  if(!urdf_model_.initParam("/robot_description"))
  {
    std::string err_msg = "'robot_description' parameter must be set";
    ROS_ERROR("%s", err_msg.c_str());
    throw std::runtime_error(err_msg);
  }
}

bool SceneBuilder::createVTKSceneFromURDF()
{
  scene_data_.clear();
//  scene->Reset();

  // Get filenames for the meshes of all static links
  if(!getSceneGeometry())
  {
    ROS_FATAL("'/robot_description' parameter must be set");
    return false;
  }

  // Concatenate all vtkPolyData objects into one if multiple static meshes exist
  if(scene_data_.size() == 0)
  {
    ROS_INFO("No static geometry present in URDF");
    return false;
  }

  if(!vtkSceneFromMeshFiles())
  {
    ROS_FATAL("Unable to load scene from URDF into VTK");
    return false;
  }
  return true;
}

bool SceneBuilder::createVTKSceneFromMeshResources(const std::vector<SceneObject>& scene_objects)
{
  scene_data_.clear();
//  scene->Reset();
  mesh_resource_marker_array_.markers.clear();

  scene_data_ = scene_objects;

  // Create a Marker message for each resource and add it to the marker array
  int id = 0;
  for(auto it = scene_data_.begin(); it != scene_data_.end(); ++it)
  {
    visualization_msgs::Marker marker = createMeshResourceMarker(it->filename, it->transform, urdf_model_.getRoot()->name, id);
    ++id;
    mesh_resource_marker_array_.markers.push_back(marker);
  }

  // Replace package URI in filenames
  changeFilenames();

  //
  if(!vtkSceneFromMeshFiles())
  {
    ROS_ERROR("Failed to load scene from mesh resource files into VTK");
    return false;
  }

  return true;
}

bool SceneBuilder::getSceneGeometry()
{
  // Get the root link of the URDF
  urdf::LinkConstSharedPtr root_link = urdf_model_.getRoot();
  if(!root_link)
  {
    ROS_FATAL("No root link set in URDF");
    return false;
  }

  // Check if the root link has any mesh geometry
  std::vector<urdf::VisualSharedPtr> root_link_visuals = root_link->visual_array;
  urdf::Pose base_pose;
  base_pose.position.x = base_pose.position.y = base_pose.position.z = 0.0f;
  base_pose.rotation.w = base_pose.rotation.x = base_pose.rotation.z = base_pose.rotation.w = 0.0f;
  getLinkGeometry(root_link_visuals, base_pose);

  // Check joints of the root link for static links
  std::vector<urdf::JointSharedPtr> root_joints = root_link->child_joints;
  getLinkChildGeometry(root_joints);

  // Resolve package:// URI in filenames to get full file paths
  changeFilenames();

  return true;
}

void SceneBuilder::getLinkGeometry(const std::vector<urdf::VisualSharedPtr>& visuals,
                                   const urdf::Pose& joint_pose)
{
  for(auto it = visuals.begin(); it != visuals.end(); ++it)
  {
    urdf::VisualConstSharedPtr vis = *it;
    if(vis->geometry->type == urdf::Geometry::MESH)
    {
      Eigen::Affine3d link_transform = urdfPoseToEigen(vis->origin);
      Eigen::Affine3d joint_transform  = urdfPoseToEigen(joint_pose);
      Eigen::Affine3d transform (joint_transform * link_transform);

      SceneObject obj (boost::dynamic_pointer_cast<const urdf::Mesh>(vis->geometry)->filename, transform);
      scene_data_.push_back(obj);
    }
  }
}

void SceneBuilder::getLinkChildGeometry(const std::vector<urdf::JointSharedPtr>& joints)
{
  for(auto it = joints.begin(); it != joints.end(); ++it)
  {
    urdf::JointSharedPtr joint = *it;
    if(joint->type == urdf::Joint::FIXED)
    {
      urdf::LinkSharedPtr link;
      urdf_model_.getLink(joint->child_link_name, link);
      getLinkGeometry(link->visual_array, joint->parent_to_joint_origin_transform);

      // Check if this static link has static link children
      if(link->child_joints.size() > 0)
      {
        getLinkChildGeometry(link->child_joints);
      }
    }
  }
}

void SceneBuilder::changeFilenames()
{
  // Create a vector of iterators to filenames that aren't in the correct format
  std::vector<std::vector<SceneObject>::iterator> erase_its;

  // Define the word to search for
  const std::string prefix = "package://";

  // Iterate through all filenames to find the defined prefix
  for(auto it = scene_data_.begin(); it != scene_data_.end(); ++it)
  {
    // Create a reference to the current file name
    SceneObject &obj = *it;

    // Find the prefix to remove in the current filename
    size_t pos = obj.filename.find(prefix);
    if(pos != std::string::npos)
    {
      // Erase prefix from filename
      obj.filename.erase(pos, prefix.length());

      // Find the first "/" in the filename (indicating end of package name)
      size_t start_pos = obj.filename.find_first_of("/", 0);

      // Create package name string and get the full package name file path
      std::string pkg_name = obj.filename.substr(pos, start_pos);
      std::string pkg_path = ros::package::getPath(pkg_name);
      if(pkg_path.length() == 0)
      {
        // Save the iterator to the filename if the ROS package where it came from is not found
        ROS_INFO("Package not found: %s", pkg_name.c_str());
        auto erase_it = std::find_if(scene_data_.begin(), scene_data_.end(),
                                 [&obj](const SceneObject& x){return x.filename == obj.filename;});
        erase_its.push_back(erase_it);
        continue;
      }

      // Erase the package name from the front of the filename and replace with full package path
      obj.filename.erase(0, start_pos);
      obj.filename.insert(obj.filename.begin(), pkg_path.begin(), pkg_path.end());
    }
    else
    {
      // Save the iterator to the filename if it doesn't start with the defined prefix
      ROS_INFO("Filename not in correct format: %s", obj.filename.c_str());
      auto erase_it = std::find_if(scene_data_.begin(), scene_data_.end(),
                               [&obj](const SceneObject& x){return x.filename == obj.filename;});
      erase_its.push_back(erase_it);
    }
  }

  // Erase all filenames that weren't in the correct format
  for(auto it = erase_its.begin(); it != erase_its.end(); ++it)
  {
    scene_data_.erase(*it);
  }
}

bool SceneBuilder::vtkSceneFromMeshFiles()
{
  if(scene_data_.empty())
  {
    ROS_ERROR("Scene data object is empty");
    return false;
  }

  vtkSmartPointer<vtkAppendPolyData> append_filter = vtkSmartPointer<vtkAppendPolyData>::New();
  for(auto it = scene_data_.begin(); it != scene_data_.end(); ++it)
  {
    boost::filesystem::path path (it->filename);
    if(!boost::filesystem::exists(path))
    {
      ROS_ERROR("File at %s does not exist", it->filename.c_str());
      return false;
    }

    vtkSmartPointer<vtkPolyData> vtk_poly = vtk_viewer::readSTLFile(it->filename);
    if(!vtk_poly)
    {
      ROS_ERROR("Unable to read input .stl file");
      return false;
    }

    // Apply the input transformation to the VTK poly data
    vtkSmartPointer<vtkTransformPolyDataFilter> transform_filter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    vtkSmartPointer<vtkTransform> transform = eigenTransformToVTK(it->transform);
    transform_filter->SetInputData(vtk_poly);
    transform_filter->SetTransform(transform);
    transform_filter->Update();

    // Append this VTK object to the previous geometries
    append_filter->AddInputData(transform_filter->GetOutput());
  }
  append_filter->Update();
  scene = append_filter->GetOutput();

//  vtk_viewer::VTKViewer viz;
//  std::vector<float> color = {0.0, 0.0, 1.0};
//  viz.addPolyDataDisplay(scene.GetPointer(), color);
//  viz.renderDisplay();

  return true;
}
