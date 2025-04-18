#ifndef HUNAV_ACTOR_PLUGIN_H
#define HUNAV_ACTOR_PLUGIN_H

// C++
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include <unordered_map>

// Gazebo
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Actor.hh>
#include <gz/common/Console.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/AxisAlignedBox.hh>
#include <gz/math/AxisAlignedBox.hh>
#include <sdf/Geometry.hh>
#include <sdf/Box.hh>
#include <sdf/Sphere.hh>
#include <sdf/Cylinder.hh>
#include <gz/transport/Node.hh>
#include <gz/sim/World.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport.hh>
#include <gz/msgs.hh>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include "hunav_msgs/msg/agent.hpp"
#include "hunav_msgs/msg/agents.hpp"
#include "hunav_msgs/srv/get_agents.hpp"
#include "hunav_msgs/srv/reset_agents.hpp"
#include "hunav_msgs/srv/move_agent.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class HuNavActorPluginIGN : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate
{
public:
  /// \brief Constructor
  HuNavActorPluginIGN();

  /// \brief Destructor
  ~HuNavActorPluginIGN() override;

  /// \brief Configure the plugin
  void Configure(const gz::sim::Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
                         gz::sim::EntityComponentManager& _ecm, gz::sim::EventManager& _eventMgr) final;

  /// \brief Update at every simulation step
  virtual void PreUpdate(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm) override;

private:
  /// Helper functions
  bool getHumanState(const gz::sim::EntityComponentManager& _ecm);
  void updateGazeboHuman(gz::sim::EntityComponentManager& _ecm, const hunav_msgs::msg::Agent& _agent);

  void computeLinearVel(gz::math::Pose3d& prevPose, const gz::math::Pose3d& currentPose, std::chrono::steady_clock::time_point& prevTime, gz::math::Vector3d& linearVelocity);
  void computeAngularVel(gz::math::Pose3d& prevPose, const gz::math::Pose3d& currentPose, std::chrono::steady_clock::time_point& prevTime, gz::math::Vector3d& angularVelocity);

  /// ROS node and services
  rclcpp::Node::SharedPtr rosnode_;
  rclcpp::Client<hunav_msgs::srv::MoveAgent>::SharedPtr rosSrvClient_;

  /// Gazebo entity information
  gz::sim::Entity actorEntity_;
  std::string actorName_;
  hunav_msgs::msg::Agent humanAgent_;
  sdf::ElementPtr sdf_;

  /// State tracking
  std::chrono::steady_clock::time_point lastUpdateTime_;
  std::chrono::steady_clock::time_point pedLastTime_;
  double update_rate_secs_;
  bool reset_;
  int counter_;
  bool initialized_;

  /// Configuration
  std::string robotName_;
  int agentId_;

  /// Helper functions
  inline double normalizeAngle(double a)
  {
    double value = a;
    while (value <= -M_PI)
      value += 2 * M_PI;
    while (value > M_PI)
      value -= 2 * M_PI;
    return value;
  }

  // Function to compute the closest position from a point to an AxisAlignedBox
  inline gz::math::Vector3d ClosestPointOnBox(const gz::math::Vector3d& point, const gz::math::AxisAlignedBox& box)
  {
    gz::math::Vector3d closestPoint;
    closestPoint.X() = std::max(box.Min().X(), std::min(point.X(), box.Max().X()));
    closestPoint.Y() = std::max(box.Min().Y(), std::min(point.Y(), box.Max().Y()));
    closestPoint.Z() = std::max(box.Min().Z(), std::min(point.Z(), box.Max().Z()));
    return closestPoint;
  }
};

#endif // HUNAV_ACTOR_PLUGIN_H