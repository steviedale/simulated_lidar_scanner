#ifndef SCENE_BUILDER_H
#define SCENE_BUILDER_H

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <urdf/model.h>

struct scene_object
{
  std::string filename;
  urdf::Pose link_pose;
  urdf::Pose joint_pose;
};

class SceneBuilder
{
public:
  SceneBuilder();
  bool createVTKScene();
  vtkSmartPointer<vtkPolyData> scene;
private:
  bool getSceneGeometry();
  void getLinkGeometry(std::vector<urdf::VisualSharedPtr>& root_link_visuals,
                       urdf::Pose& joint_pose);
  void getLinkChildGeometry(std::vector<urdf::JointSharedPtr>& joints);
  void changeFilenames();
  bool vtkSceneFromMeshFiles();

  std::vector<scene_object> scene_data_;
  urdf::Model urdf_model_;
};

#endif // SCENE_BUILDER_H
