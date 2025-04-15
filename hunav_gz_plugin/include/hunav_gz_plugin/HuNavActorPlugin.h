/***********************************************************************/
/**                                                                    */
/** HuNavActorPlugin_fortress.h                                        */
/**                                                                    */
/** Copyright (c) 2025, Service Robotics Lab (SRL).                    */
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Noé Pérez-Higueras (maintainer)                                    */
/** email: noeperez@upo.es                                             */
/**                                                                    */
/** This software may be modified and distributed under the terms      */
/** of the MIT license. See the LICENSE file for details.              */
/**                                                                    */
/**                                                                    */
/***********************************************************************/

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
//#include <gz/sim/components/AnimationUpdateData.hh>
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
//#include "hunav_msgs/srv/compute_agents.hpp"
#include "hunav_msgs/srv/get_agents.hpp"
#include "hunav_msgs/srv/reset_agents.hpp"
#include "hunav_msgs/srv/move_agent.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// namespace gazebo_ros {

// class HuNavPluginPrivate;

class HuNavActorPluginIGN : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate
                    /*public gz::sim::ISystemPostUpdate*/
                    /*public gz::sim::ISystemReset*/          
{
public:
  /// \brief Constructor
  HuNavActorPluginIGN();

  /// \brief Destructor
  ~HuNavActorPluginIGN() override;

  /// \brief Configuración del plugin
  void Configure(const gz::sim::Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
                         gz::sim::EntityComponentManager& _ecm, gz::sim::EventManager& _eventMgr) final;

  // Implement PostUpdate callback, provided by ISystemPostUpdate
  // and called at every iteration, after physics is done
  //void PostUpdate(const gz::sim::UpdateInfo& _info, const gz::sim::EntityComponentManager& _ecm) override;
  virtual void PreUpdate(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm) override;
  //virtual void Reset(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override;
  

  /// \brief Reiniciar el plugin
  // void Reset(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm);

  // void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

private:

  /// Helper functions
  void initializeAgents(gz::sim::EntityComponentManager& _ecm);
  void initializeRobot(gz::sim::EntityComponentManager& _ecm);
  //void getObstacles(const gz::sim::EntityComponentManager& _ecm);
  void getHumanObstacles(const gz::sim::EntityComponentManager& _ecm);
  bool getPedestrianStates(const gz::sim::EntityComponentManager& _ecm);
  bool getHumanState(const gz::sim::EntityComponentManager& _ecm);
  bool getRobotState(const gz::sim::EntityComponentManager& _ecm);
  void updateGazeboHuman(gz::sim::EntityComponentManager& _ecm, const gz::sim::UpdateInfo& _info, const hunav_msgs::msg::Agent& _agent);
  void updateGazeboHumans(gz::sim::EntityComponentManager& _ecm, const gz::sim::UpdateInfo& _info, const hunav_msgs::msg::Agents& _agents);

  void computeLinearVel(gz::math::Pose3d& prevPose, const gz::math::Pose3d& currentPose, std::chrono::steady_clock::time_point& prevTime, gz::math::Vector3d& linearVelocity);
  void computeAngularVel(gz::math::Pose3d& prevPose, const gz::math::Pose3d& currentPose, std::chrono::steady_clock::time_point& prevTime, gz::math::Vector3d& angularVelocity);

  rclcpp::Node::SharedPtr rosnode_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros_test_pub_;
  //rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr ros_bbs_pub1;
  //::gz::transport::Node gz_node;

  /// ROS Services
  //rclcpp::Client<hunav_msgs::srv::ComputeAgents>::SharedPtr rosSrvClient_;
  rclcpp::Client<hunav_msgs::srv::MoveAgent>::SharedPtr rosSrvClient_;
  rclcpp::Client<hunav_msgs::srv::GetAgents>::SharedPtr rosSrvGetAgentsClient_;
  rclcpp::Client<hunav_msgs::srv::ResetAgents>::SharedPtr rosSrvResetClient_;

  /// ROS Subscriber
  // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;

  /// Robot and Pedestrian Information
  hunav_msgs::msg::Agent robotAgent_;
  hunav_msgs::msg::Agent initRobotAgent_;
  std::chrono::steady_clock::time_point robotLastTime_;
  std::unordered_map<gz::sim::Entity, hunav_msgs::msg::Agent> pedestrians_;
  hunav_msgs::msg::Agents initPedestrians_;
  std::chrono::steady_clock::time_point pedLastTime_;

  /// Gazebo Fortress Entities
  gz::sim::Entity actorEntity_;
  std::string actorName_;
  hunav_msgs::msg::Agent humanAgent_;
  sdf::ElementPtr sdf_;

  /// Simulation Info
  std::chrono::steady_clock::time_point lastUpdate_;
  /// \brief Time of the last update.
  //gz::common::Time lastUpdate_;
  //rclcpp::Time rostime_;
  //double dt_;
  double update_rate_secs_;
  bool reset_;

  int counter_;
  bool agentsInitialized_;
  bool robotInitialized_;

  /// Robot Configuration
  std::string robotName_;
  gz::sim::Entity robotEntity_;
  std::string globalFrame_;

  /// Navigation Goals
  bool waitForGoal_;
  bool goalReceived_;
  std::string goalTopic_;

  /// Models to Ignore
  std::vector<std::string> ignoreModels_;
  // ignition::common::ConnectionPtr updateConnection;

  inline double normalizeAngle(double a)
  {
    double value = a;
    while (value <= -M_PI)
      value += 2 * M_PI;
    while (value > M_PI)
      value -= 2 * M_PI;
    return value;
  }

  // Function to calculate the distance between two axis-aligned bounding boxes
  inline double CalculateDistance(const ignition::math::AxisAlignedBox& box1, const ignition::math::AxisAlignedBox& box2)
  {
    gz::math::Vector3d min1 = box1.Min();
    gz::math::Vector3d max1 = box1.Max();
    gz::math::Vector3d min2 = box2.Min();
    gz::math::Vector3d max2 = box2.Max();

    double dx = std::max({0.0, min2.X() - max1.X(), min1.X() - max2.X()});
    double dy = std::max({0.0, min2.Y() - max1.Y(), min1.Y() - max2.Y()});
    double dz = std::max({0.0, min2.Z() - max1.Z(), min1.Z() - max2.Z()});

    return std::sqrt(dx * dx + dy * dy + dz * dz);
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

