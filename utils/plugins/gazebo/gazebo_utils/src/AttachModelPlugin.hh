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

#ifndef SERVICESIM_ATTACHMODELPLUGIN_HH_
#define SERVICESIM_ATTACHMODELPLUGIN_HH_

#include <memory>

#include <gazebo/common/Plugin.hh>

namespace servicesim
{
  // forward declarations
  class AttachModelPluginPrivate;

  /// \brief A model plugin that enables multiple models in the world to be
  /// attached to links within this model.
  ///
  /// The plugin has the following SDF description:
  /// <link>            SDF element. More than one <link> can be defined.
  ///   <link_name>     Name of the link in this model.
  ///   <light>         SDF element. More than one <model> can be attached.
  ///     <pose>        Offset pose of the model in the link frame.
  ///     <model_name>  Name of the other model in the world.
  class AttachModelPlugin : public gazebo::ModelPlugin
  {
    /// \brief Constructor
  public:
    AttachModelPlugin();

    // Documentation inherited
  public:
    virtual void Load(gazebo::physics::ModelPtr _model,
                      sdf::ElementPtr _sdf);

    /// \brief Main loop to update the pose of lights
  private:
    void OnUpdate();

    /// \brief Pointer to private data
  private:
    std::unique_ptr<AttachModelPluginPrivate> dataPtr;
  };
}
#endif
