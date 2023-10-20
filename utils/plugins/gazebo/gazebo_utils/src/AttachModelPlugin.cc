/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <map>
#include <vector>
#include <ignition/math/Pose3.hh>
#include <ros/ros.h>

#include <gazebo/common/Events.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include "AttachModelPlugin.hh"

namespace servicesim
{
  /// \brief Private data class for the AttachModelPlugin class
  class AttachModelPluginPrivate
  {
    /// \brief Event connections
  public:
    std::vector<gazebo::event::ConnectionPtr> connections;

    /// \brief Pointer to this model
  public:
    gazebo::physics::ModelPtr model;

    /// \brief Pointer to the world
  public:
    gazebo::physics::WorldPtr world;

    /// \brief List of link and model pointers and the model pose offset
  public:
    std::map<gazebo::physics::LinkPtr,
             std::map<gazebo::physics::ModelPtr, ignition::math::Pose3d>>
        linkModels;
  };
}

using namespace servicesim;
GZ_REGISTER_MODEL_PLUGIN(servicesim::AttachModelPlugin)

/////////////////////////////////////////////////
AttachModelPlugin::AttachModelPlugin()
    : dataPtr(new AttachModelPluginPrivate)
{
}

/////////////////////////////////////////////////
void AttachModelPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->dataPtr->model = _model;
  this->dataPtr->world = _model->GetWorld();

  if (!_sdf->HasElement("link"))
  {
    gzerr << "No <link> sdf elements found." << std::endl;
    return;
  }

  // load links
  sdf::ElementPtr linkElem = _sdf->GetElement("link");
  while (linkElem)
  {
    std::string linkName;
    if (linkElem->HasElement("link_name"))
    {
      linkName = linkElem->Get<std::string>("link_name");

      auto link = this->dataPtr->model->GetLink(linkName);
      if (!link)
      {
        gzerr << "Link: '" << linkName << "' not found." << std::endl;
      }
      else
      {
        // load models
        if (linkElem->HasElement("model"))
        {
          sdf::ElementPtr modelElem = linkElem->GetElement("model");
          while (modelElem)
          {
            auto modelName = modelElem->Get<std::string>("model_name");
            auto model = this->dataPtr->world->ModelByName(modelName);

            // TODO: what if model hasn't been loaded yet
            if (!model)
            {
              ROS_WARN("98 AttachModelPlugin.cc No Model found");
              gzerr << "Model: '" << modelName << "' not found, make sure it is loaded before '"
                    << _model->GetName() << "'." << std::endl;
            }
            else
            {
              // pose is optional
              ROS_WARN("105 AttachModelPlugin.cc No pose found?");
              ignition::math::Pose3d pose;
              if (modelElem->HasElement("pose"))
                pose = modelElem->Get<ignition::math::Pose3d>("pose");

              auto &models = this->dataPtr->linkModels[link];
              models[model] = pose;
            }
            modelElem = modelElem->GetNextElement("model");
          }
        }
      }
    }
    linkElem = linkElem->GetNextElement("link");
  }

  if (this->dataPtr->linkModels.empty())
    return;

  this->dataPtr->connections.push_back(
      gazebo::event::Events::ConnectWorldUpdateEnd(
          std::bind(&AttachModelPlugin::OnUpdate, this)));
}

/////////////////////////////////////////////////
void AttachModelPlugin::OnUpdate()
{
  // update model pose based on link pose
  for (auto &it : this->dataPtr->linkModels)
  {
    auto link = it.first;
    auto &models = it.second;
    for (auto &modelIt : models)
    {
      auto model = modelIt.first;
      auto pose = modelIt.second;
      model->SetWorldPose(pose + link->WorldPose());
    }
  }
}
