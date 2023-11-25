/*
Created on Mon Dec  2

@author: mahmoud
*/

#include <pedsim/types.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <thread>

#include <pedsim_msgs/TrackedPersons.h>
#include <pedsim_msgs/AgentStates.h>

namespace gazebo
{
    class PedsimGazeboActorPlugin : public ModelPlugin
    {
    public:

        PedsimGazeboActorPlugin() : ModelPlugin() {}

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            if (!ros::isInitialized())
            {
                ROS_ERROR("ROS not initialized");
                return;
            }

            this->model = _model;

            if(!_sdf->GetAttribute("name")->Get<std::string>(this->id))
                ROS_FATAL("PedsimGazeboActorPlugin: could not parse id");
                        
            

            this->animationFactor = _sdf->Get<double>("animation_factor", 1.0).first;

            this->modelHeight = _sdf->Get<double>("model_height", 2.0).first / 2.0;

            std::string node_id = this->id;
            node_id.erase(remove_if(node_id.begin(),node_id.end(), [](char c){return !(c>=0 && c <128);}), node_id.end());
            this->rosNode.reset(new ros::NodeHandle("PEDSIM_ACTOR_" + node_id));

            ros::SubscribeOptions so = ros::SubscribeOptions::create<pedsim_msgs::AgentStates>("/pedsim_simulator/simulated_agents", 1, boost::bind(&PedsimGazeboActorPlugin::OnRosMsg, this, _1), ros::VoidPtr(), &this->rosQueue);
            this->rosSub = this->rosNode->subscribe(so);
            this->rosQueueThread = std::thread(std::bind(&PedsimGazeboActorPlugin::QueueThread, this));
            // in case you need to change/modify model on update
            // this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&actorPosePlugin::OnUpdate, this));

            physics::ActorPtr actor = boost::dynamic_pointer_cast<physics::Actor>(this->model);
            if (!actor)
                return;

            physics::TrajectoryInfoPtr trajectoryInfo(new physics::TrajectoryInfo);

            trajectoryInfo->id = 0;
            trajectoryInfo->type = std::string("walking");
            trajectoryInfo->duration = 1.0;
            trajectoryInfo->startTime = 0.0;
            trajectoryInfo->endTime = 1.0;
            trajectoryInfo->translated = false;

            actor->SetCustomTrajectory(trajectoryInfo);
        }

    public:
        // call back function when receive rosmsg
        void OnRosMsg(const pedsim_msgs::AgentStatesConstPtr msg)
        {

            physics::ActorPtr actor = boost::dynamic_pointer_cast<physics::Actor>(this->model);
            if (!actor)
                return;

            for (auto agentState : msg->agent_states)
            {
                if (agentState.id != this->id)
                    continue;

                ignition::math::Angle yaw = ignition::math::Quaterniond(agentState.pose.orientation.w, agentState.pose.orientation.x, agentState.pose.orientation.y, agentState.pose.orientation.z).Yaw();

                ignition::math::Pose3d oldPose = actor->WorldPose();
                ignition::math::Pose3d newPose;

                newPose.Pos().Set(
                    agentState.pose.position.x,
                    agentState.pose.position.y,
                    agentState.pose.position.z + this->modelHeight);
                newPose.Rot() = ignition::math::Quaterniond(M_PI_2, 0, yaw.Radian() + M_PI_2); // P-R-Y

                try
                {
                    double newScriptTime = actor->ScriptTime() + (newPose.Pos() - oldPose.Pos()).Length() * this->animationFactor;
                    actor->SetWorldPose(newPose, true, true);
                    actor->SetScriptTime(newScriptTime);
                }
                catch (gazebo::common::Exception gz_ex)
                {
                    ROS_ERROR("Error setting actor pose %s - %s", actor->GetName().c_str(), gz_ex.GetErrorStr().c_str());
                }
                break;
            }
        }

        // ROS helper function that processes messages
    private:
        void QueueThread()
        {
            static const double timeout = 0.1;
            while (!this->shutdown && this->rosNode->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

    private:
        std::unique_ptr<ros::NodeHandle> rosNode;
        ros::Subscriber rosSub;
        ros::CallbackQueue rosQueue;
        std::thread rosQueueThread;
        event::ConnectionPtr updateConnection_;

        physics::ModelPtr model;
        pedsim::id id;
        double animationFactor;
        double modelHeight;

        bool shutdown = false;
        
    };
    GZ_REGISTER_MODEL_PLUGIN(PedsimGazeboActorPlugin)
}
