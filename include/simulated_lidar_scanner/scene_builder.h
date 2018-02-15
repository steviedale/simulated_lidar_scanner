#ifndef SCENE_BUILDER_H
#define SCENE_BUILDER_H

#include <Eigen/Eigen>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <urdf/model.h>
#include <visualization_msgs/MarkerArray.h>

struct SceneObject
{
  SceneObject()
  {
  }

  SceneObject(const std::string& filename,
              const Eigen::Affine3d& transform)
    : filename(filename)
    , transform(transform)
  {
  }

  SceneObject(const std::string& filename,
              const std::vector<double>& pose)
    : filename(filename)
    , transform(Eigen::Affine3d::Identity())
  {
    if(pose.size() != 6)
    {
      throw std::runtime_error("Pose does not have 6 elements!");
    }
    transform.translate(Eigen::Vector3d(pose[0], pose[1], pose[2]));
    transform.rotate(Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX()));
    transform.rotate(Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()));
    transform.rotate(Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()));
  }

  std::string filename;
  Eigen::Affine3d transform;
};

class SceneBuilder
{
public:
  SceneBuilder();

  bool createVTKSceneFromURDF();

  bool createVTKSceneFromMeshResources(const std::vector<SceneObject>& scene_objects);

  void getMarkerArray(visualization_msgs::MarkerArray& marker_array) const {marker_array = mesh_resource_marker_array_;}

  vtkSmartPointer<vtkPolyData> scene;
private:
  bool getSceneGeometry();

  void getLinkGeometry(const std::vector<urdf::VisualSharedPtr>& root_link_visuals,
                       const urdf::Pose& joint_pose);

  void getLinkChildGeometry(const std::vector<urdf::JointSharedPtr>& joints);

  void changeFilenames();

  bool vtkSceneFromMeshFiles();

  bool meshResourcesToVTK();

  std::vector<SceneObject> scene_data_;
  urdf::Model urdf_model_;
  visualization_msgs::MarkerArray mesh_resource_marker_array_;
};

#endif // SCENE_BUILDER_H
