#include "hunav_gz_plugin/HuNavActorPlugin.h"

HuNavActorPluginIGN::HuNavActorPluginIGN() : reset_(false), counter_(0), initialized_(false)
{
}

HuNavActorPluginIGN::~HuNavActorPluginIGN()
{
}

void HuNavActorPluginIGN::Configure(const gz::sim::Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
                            gz::sim::EntityComponentManager& _ecm, gz::sim::EventManager& _eventMgr)
{
  (void)_eventMgr;
  counter_ = 0;

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  
  // Create ROS node
  std::string nodename = "hunav_plugin_node_" + std::to_string(_entity);
  this->rosnode_ = std::make_shared<rclcpp::Node>(nodename.c_str());

  actorEntity_ = _entity; 

  // Get actor name
  auto nameComp = _ecm.Component<gz::sim::components::Name>(_entity);
  if (nameComp)
  {
    ignmsg << "Plugin loaded for actor: " << nameComp->Data() << std::endl;
    actorName_ = nameComp->Data();
  }

  // Get plugin parameters
  sdf_ = _sdf->Clone();

  // Get agent ID from name if possible
  if (actorName_.find("agent") != std::string::npos)
  {
    try {
      std::string id_str = actorName_.substr(5); // "agent1" -> "1"
      agentId_ = std::stoi(id_str);
    }
    catch (...) {
      agentId_ = 1; // Default if not specified
    }
  }
  else {
    agentId_ = 1;
  }

  // Robot name parameter
  if (sdf_->HasElement("robot_name"))
    robotName_ = sdf_->Get<std::string>("robot_name");
  else
    robotName_ = "robot";

  // Set update rate
  auto update_rate = sdf_->Get<double>("update_rate", 30.0).first;
  if (update_rate > 0.0)
  {
    update_rate_secs_ = 1.0 / update_rate;
  }
  else
  {
    update_rate_secs_ = 0.01;
  }
  
  ignmsg << "update_rate: " << update_rate << ", secs: " << update_rate_secs_ << std::endl;

  // Create the MoveAgent service client
  rosSrvClient_ = this->rosnode_->create_client<hunav_msgs::srv::MoveAgent>("move_agent");

  // Initialize time tracking
  lastUpdateTime_ = std::chrono::steady_clock::now();
  pedLastTime_ = std::chrono::steady_clock::now();
  
  // Initialize agent state
  humanAgent_.id = agentId_;
  humanAgent_.name = actorName_;
  humanAgent_.type = 1; // PERSON
  
  // Get initial pose
  gz::math::Pose3d pose = worldPose(actorEntity_, _ecm);
  double yaw = normalizeAngle(pose.Rot().Yaw() - M_PI_2);
  
  humanAgent_.position.position.x = pose.Pos().X();
  humanAgent_.position.position.y = pose.Pos().Y();
  humanAgent_.position.position.z = pose.Pos().Z();
  
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, yaw);
  humanAgent_.position.orientation = tf2::toMsg(myQuaternion);
  humanAgent_.yaw = yaw;
  
  // Set desired velocity (this could be a parameter)
  humanAgent_.desired_velocity = 1.0;
  
  // Set up trajectory pose to prevent default movement
  auto trajPoseComp = _ecm.Component<gz::sim::components::TrajectoryPose>(actorEntity_);
  if (nullptr == trajPoseComp)
  {
    gz::math::Pose3d initialPose = pose;
    initialPose.Pos().Z(0.03);
    _ecm.CreateComponent(actorEntity_, gz::sim::components::TrajectoryPose(initialPose));
  }
  
  initialized_ = true;
  ignmsg << "HuNavActorPlugin initialized for actor: " << actorName_ << " (ID: " << agentId_ << ")" << std::endl;
}

void HuNavActorPluginIGN::PreUpdate(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm)
{
  if (_info.paused || !initialized_)
    return;

  // Rate limiting
  auto currentTime = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = currentTime - lastUpdateTime_;
  if (elapsed.count() < update_rate_secs_)
    return;
    
  counter_++;

  // Get current state
  if (!getHumanState(_ecm)) {
    ignwarn << "Failed to get human state" << std::endl;
    return;
  }
  
  // Create service request
  auto request = std::make_shared<hunav_msgs::srv::MoveAgent::Request>();
  
  // Create robot message (minimal)
  hunav_msgs::msg::Agent robot;
  robot.id = 0;
  robot.name = robotName_;
  robot.type = 2; // ROBOT
  robot.position.position.x = 0.0;
  robot.position.position.y = 0.0;
  
  // Prepare agents message
  hunav_msgs::msg::Agents agents;
  agents.header.stamp = this->rosnode_->get_clock()->now();
  agents.header.frame_id = "map";
  agents.agents.push_back(humanAgent_);
  
  // Build request
  request->robot = robot;
  request->current_agents = agents;
  request->agent_id = agentId_;
  
  // Call service
  std::chrono::milliseconds timeout(100);
  if (rosSrvClient_->wait_for_service(timeout)) {
    auto result = rosSrvClient_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(this->rosnode_, result, timeout) == 
        rclcpp::FutureReturnCode::SUCCESS) {
      
      auto updatedAgent = result.get()->updated_agent;
      updateGazeboHuman(_ecm, updatedAgent);
      
      // Occasional debugging
      if (counter_ % 100 == 0) {
        double goalx = updatedAgent.goals.empty() ? 0 : updatedAgent.goals[0].position.x;
        double goaly = updatedAgent.goals.empty() ? 0 : updatedAgent.goals[0].position.y;
        double currx = updatedAgent.position.position.x;
        double curry = updatedAgent.position.position.y;
        double dist = sqrt(pow(goalx - currx, 2) + pow(goaly - curry, 2));
        RCLCPP_INFO(rosnode_->get_logger(), "Agent %s. Goal: %.2f, %.2f, Current pos:[%.2f, %.2f], Dist: %.2f",
          updatedAgent.name.c_str(), goalx, goaly, currx, curry, dist);
      }
    }
  }
  
  lastUpdateTime_ = currentTime;
}

bool HuNavActorPluginIGN::getHumanState(const gz::sim::EntityComponentManager& _ecm)
{
  gz::math::Pose3d pose = worldPose(actorEntity_, _ecm);
  double yaw = normalizeAngle(pose.Rot().Yaw() - M_PI_2);
  gz::math::Vector3d pos = pose.Pos();
  
  // Actors in Gazebo don't have physical dynamics, so compute velocities manually
  gz::math::Pose3d prevPose(humanAgent_.position.position.x, humanAgent_.position.position.y, humanAgent_.position.position.z, 0, 0, humanAgent_.yaw);
  
  // Get elapsed time
  auto currentTime = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsedTime = currentTime - pedLastTime_;
  double dt = elapsedTime.count();
  
  // Compute velocities
  gz::math::Vector3d linearVelocity;
  computeLinearVel(prevPose, pose, pedLastTime_, linearVelocity);
  gz::math::Vector3d angularVelocity;
  computeAngularVel(prevPose, pose, pedLastTime_, angularVelocity);
  
  double xi = humanAgent_.position.position.x;
  double yi = humanAgent_.position.position.y;
  double xf = pos.X();
  double yf = pos.Y();
  double lvel = linearVelocity.Length();
  double vx = linearVelocity.X();
  double vy = linearVelocity.Y();
  double anvel = angularVelocity.Z();
  
  if (reset_)
  {
    lvel = 0.0;
    vx = 0.0;
    vy = 0.0;
    anvel = 0.0;
  }
  else if (lvel > humanAgent_.desired_velocity)
  {
    lvel = humanAgent_.desired_velocity;
    double maxd = lvel * update_rate_secs_;
    if (fabs(xf - xi) > maxd)
      vx = ((xf - xi) / fabs(xf - xi)) * maxd / dt;
    if (fabs(yf - yi) > maxd)
      vy = ((yf - yi) / fabs(yf - yi)) * maxd / dt;
  }
  
  humanAgent_.velocity.linear.x = vx;
  humanAgent_.velocity.linear.y = vy;
  humanAgent_.linear_vel = lvel;
  humanAgent_.velocity.angular.z = anvel;
  humanAgent_.angular_vel = anvel;
  
  // Update position
  humanAgent_.position.position.x = xf;
  humanAgent_.position.position.y = yf;
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, yaw);
  humanAgent_.position.orientation = tf2::toMsg(myQuaternion);
  humanAgent_.yaw = yaw;
  
  reset_ = false;
  return true;
}

void HuNavActorPluginIGN::updateGazeboHuman(gz::sim::EntityComponentManager& _ecm, const hunav_msgs::msg::Agent& _agent)
{
  // Get current pose for animation
  gz::math::Pose3d actorPose = worldPose(actorEntity_, _ecm);
  gz::math::Pose3d prevPose = actorPose;
  
  // Smooth orientation changes
  double yaw = normalizeAngle(_agent.yaw + M_PI_2);
  double currAngle = actorPose.Rot().Yaw();
  double diff = normalizeAngle(yaw - currAngle);
  if (std::fabs(diff) > GZ_DTOR(10))
  {
    yaw = normalizeAngle(currAngle + (diff * 0.1));  // 0.01, 0.005
  }
  
  // Set new position
  actorPose.Pos().X(_agent.position.position.x);
  actorPose.Pos().Y(_agent.position.position.y);
  actorPose.Rot() = gz::math::Quaterniond(1.5707, 0, yaw);
  
  // Update position in ECM
  auto pose = _ecm.Component<gz::sim::components::Pose>(actorEntity_);
  if(pose)
  {
    pose->Data() = actorPose;
    _ecm.SetChanged(actorEntity_, gz::sim::components::Pose::typeId);
  }
  else{
    RCLCPP_ERROR(rosnode_->get_logger(), "[updateGazeboHuman] Failed to get pose component for actor %s", _agent.name.c_str());
  }
  
  // Update animation based on distance traveled
  double distanceTraveled = (actorPose.Pos() - prevPose.Pos()).Length();
  
  // Update trajectory pose to prevent default movement
  auto trajPoseComp = _ecm.Component<gz::sim::components::TrajectoryPose>(actorEntity_);
  if (trajPoseComp) {
    trajPoseComp->Data() = actorPose;
    _ecm.SetChanged(actorEntity_, gz::sim::components::TrajectoryPose::typeId);
  }
  
  // Update the actor state with goals from service response
  humanAgent_.goals.clear();
  humanAgent_.goals = _agent.goals;
  if (_agent.behavior.state != humanAgent_.behavior.state)
  {
    humanAgent_.behavior.state = _agent.behavior.state;
  }
}

void HuNavActorPluginIGN::computeLinearVel(gz::math::Pose3d& prevPose, const gz::math::Pose3d& currentPose, std::chrono::steady_clock::time_point& prevTime, gz::math::Vector3d& linearVelocity)
{
  // Get current time
  auto currentTime = std::chrono::steady_clock::now();

  // Elapsed time
  std::chrono::duration<double> elapsedTime = currentTime - prevTime;
  double dt = elapsedTime.count();
  
  // Prevent division by zero
  if (dt < 0.0001) dt = 0.0001;

  // Position difference
  gz::math::Vector3d deltaPosition = currentPose.Pos() - prevPose.Pos();

  // Linear velocity
  linearVelocity = deltaPosition / dt;

  // Update previous values
  prevPose = currentPose;
  prevTime = currentTime;
}

void HuNavActorPluginIGN::computeAngularVel(gz::math::Pose3d& prevPose, const gz::math::Pose3d& currentPose, std::chrono::steady_clock::time_point& prevTime, gz::math::Vector3d& angularVelocity)
{
  // Get current time
  auto currentTime = std::chrono::steady_clock::now();

  // Elapsed time
  std::chrono::duration<double> elapsedTime = currentTime - prevTime;
  double dt = elapsedTime.count();
  
  // Prevent division by zero
  if (dt < 0.0001) dt = 0.0001;

  // Orientation difference
  double deltaYaw = currentPose.Rot().Yaw() - prevPose.Rot().Yaw();

  // Angle normalization
  deltaYaw = atan2(sin(deltaYaw), cos(deltaYaw));

  // Angular velocity
  angularVelocity.Z(deltaYaw / dt);

  // Update previous values
  prevPose = currentPose;
  prevTime = currentTime;
}

// Register the plugin
GZ_ADD_PLUGIN(
    HuNavActorPluginIGN,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(HuNavActorPluginIGN, "HuNavActorPluginIGN")