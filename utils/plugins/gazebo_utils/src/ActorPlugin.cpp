#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>

#include "rclcpp/rclcpp.hpp"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <thread>

#include <pedsim_msgs/msg/tracked_persons.hpp>
#include <pedsim_msgs/msg/agent_states.hpp>

namespace gazebo
{
  class ActorPosePlugin : public ModelPlugin
  {
  public:
    ActorPosePlugin() : ModelPlugin() {}

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

      this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);

      // Create custom trajectory
      this->trajectoryInfo.reset(new physics::TrajectoryInfo());
      this->trajectoryInfo->type = "animation";
      this->trajectoryInfo->duration = 1.0;
      this->actor->SetCustomTrajectory(this->trajectoryInfo);
      if (!ros::isInitialized())
      {
        RCLCPP_ERROR(rclcpp::get_logger("GazeboUtils"), "ROS not initialized");
        return;
      }
      rosNode.reset(new rclcpp::Node(this->actor->GetName()));
      this->actor_height = 1.1;
      ros::SubscribeOptions so = ros::SubscribeOptions::create<pedsim_msgs::msg::AgentStates>("/pedsim_simulator/simulated_agents", 1, boost::bind(&ActorPosePlugin::OnRosMsg, this, _1), ros::VoidPtr(), &rosQueue);
      rosSub = rosNode->subscribe(so);
      rosQueueThread = std::thread(std::bind(&ActorPosePlugin::QueueThread, this));
    }

  public:
    // call back function when receive rosmsg
    void OnRosMsg(const pedsim_msgs::msg::AgentStates::ConstSharedPtr msg)
    {
      // RCLCPP_ERROR(rclcpp::get_logger("GazeboUtils"), msg);
      double distanceTraveled;
      bool actorFound = false;
      for (auto& actor : msg->agent_states)
      {
        if (this->actor->GetName() == actor.id)
        {
          actorFound = true;
          ignition::math::Pose3d pose = this->actor->WorldPose();
          ignition::math::Pose3d gzb_pose;
          // Getting the direction angle of the agent
          ignition::math::Quaterniond quat(actor.pose.orientation.w, actor.pose.orientation.x, actor.pose.orientation.y, actor.pose.orientation.z);
          ignition::math::Angle yaw = quat.Yaw();
          gzb_pose.Pos().Set(actor.pose.position.x,
                             actor.pose.position.y,
                             actor.pose.position.z + actor_height);

          // Rotating the actor in the correct direction -> yaw, keeping in mind that the actor is oriented Y-up and Z-front
          gzb_pose.Rot() = ignition::math::Quaterniond(1.5707, 0.0, yaw.Radian() + 1.5707);
          try
          {
            distanceTraveled = (gzb_pose.Pos() -
                                pose.Pos())
                                   .Length();
            this->actor->SetWorldPose(gzb_pose, true, false);
            this->actor->SetScriptTime(this->actor->ScriptTime() + (distanceTraveled * 5.1)); // 5.1 == Animation factor
            return;
          }
          catch (gazebo::common::Exception gz_ex)
          {
            gzerr << "Problem\n";
          }
        }
      }
      if (!actorFound)
      // Actor not found in pedsim simulation -> place him far away in Gazebo so it doesn't intervene
      {
        RCLCPP_WARN(rclcpp::get_logger("GazeboUtils"), "Actor not found in pedsim simulation -> place him far away in Gazebo so it doesn't intervene");
        ignition::math::Pose3d pose = this->actor->WorldPose();
        pose.Pos().Z() = -20.0;
        this->actor->SetWorldPose(pose, true, false);
      }
    }

    // ROS helper function that processes messages
  private:
    void QueueThread()
    {
      static const double timeout = 0.1;
      while (rosNode->ok())
      {
        rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

  private:
    std::unique_ptr<rclcpp::Node> rosNode;
    ros::Subscriber rosSub;
    ros::CallbackQueue rosQueue;
    std::thread rosQueueThread;
    physics::WorldPtr world_;
    physics::ActorPtr actor;
    physics::TrajectoryInfoPtr trajectoryInfo;
    event::ConnectionPtr updateConnection_;
    double actor_height;
  };
  GZ_REGISTER_MODEL_PLUGIN(ActorPosePlugin)
}
