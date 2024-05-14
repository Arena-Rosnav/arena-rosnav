/*
    Use global path of move_base and publish Subgoals based on SpacialHorizon
   algorithm
*/
#include <spacial_horizon/spacial_horizon_node.h>

void SpacialHorizon::init(ros::NodeHandle &nh)
{
    has_odom = false;
    has_goal = false;

    nh.param("/train_mode", train_mode, false);

    /*  fsm param  */
    nh.param("/disable_intermediate_planner", disable_intermediate_planner, false);
    nh.param("fsm/goal_tolerance", goal_tolerance, 0.2);
    nh.param("fsm/subgoal_tolerance", subgoal_tolerance, 0.2);
    nh.param("fsm/subgoal_pub_period", subgoal_pub_period, 0.2);
    nh.param("fsm/planning_horizon", planning_horizon, 5.0);

    if (!train_mode)
    {
        // if not in train mode, create timers
        ROS_INFO_STREAM("Spacial Horizon: Creating Global Plan Timer");
        update_global_plan_timer = nh.createTimer(
            ros::Duration(0.1), &SpacialHorizon::getGlobalPath, this
        );
    }
    subgoal_timer = nh.createTimer(
            ros::Duration(subgoal_pub_period), &SpacialHorizon::updateSubgoalCallback, this
    );
    
    /* ros communication with public node */
    ros::NodeHandle public_nh; // sim1/goal
    sub_goal =
        public_nh.subscribe(SUB_TOPIC_GOAL, 1, &SpacialHorizon::goalCallback, this);
    sub_odom =
        public_nh.subscribe(SUB_TOPIC_ODOM, 1, &SpacialHorizon::odomCallback, this);

    pub_subgoal =
        public_nh.advertise<geometry_msgs::PoseStamped>(PUB_TOPIC_SUBGOAL, 10);
    pub_global_plan = public_nh.advertise<nav_msgs::Path>(PUB_TOPIC_GLOBAL_PLAN, 10);

    initializeGlobalPlanningService();
}

void SpacialHorizon::initializeGlobalPlanningService()
{
    ROS_INFO_STREAM("[Spacial Horizon - INIT] Initializing MBF service client");
    ros::NodeHandle nh;
    std::string service_name = ros::this_node::getNamespace() + "/" + SERVICE_GLOBAL_PLANNER;

    while (!ros::service::waitForService(service_name, ros::Duration(3.0)))
    {
        ROS_INFO("[SpacialHorizon - INIT] Waiting for service %s to become available",
                service_name.c_str());
    }
    global_planner_srv = nh.serviceClient<nav_msgs::GetPlan>(service_name, true);
}

void SpacialHorizon::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    ROS_INFO_STREAM("[Spacial Horizon] Received new odom");
    odom_pos =
        Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
    odom_vel =
        Eigen::Vector2d(msg->twist.twist.linear.x, msg->twist.twist.linear.y);

    has_odom = true;
}

void SpacialHorizon::goalCallback(const geometry_msgs::PoseStampedPtr &msg)
{
    ROS_INFO_STREAM("[Spacial Horizon] Received new goal");

    end_pos = Eigen::Vector2d(msg->pose.position.x, msg->pose.position.y);

    if (!has_odom)
    {
        ROS_WARN("[SpacialHorizon] Received goal before receiving odom");
        return;
    }

    has_goal = true;

    getGlobalPath();

    // when disable_intermediate_planner is true, the goal is the subgoal
    if (disable_intermediate_planner){
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = end_pos(0);
        pose_stamped.pose.position.y = end_pos(1);
        pose_stamped.pose.position.z = 0.0;

        pub_subgoal.publish(pose_stamped);
        std::cout << " SUBGOAL = GOAL" << std::endl;
    }
}

/**
 * @brief Retrieves the subgoal for the SpacialHorizon object.
 *
 * This function retrieves the subgoal, which is a 2D vector, for the SpacialHorizon object.
 *
 * @param subgoal A reference to an Eigen::Vector2d object where the subgoal will be stored.
 * @return bool Returns true if the subgoal was successfully retrieved, false otherwise.
 */
bool SpacialHorizon::getSubgoal(Eigen::Vector2d &subgoal)
{   
    double dist_to_goal = (odom_pos - end_pos).norm();

    if (dist_to_goal <= goal_tolerance)
    {
        return false;
    }

    if (dist_to_goal < planning_horizon)
    {
        subgoal = end_pos;

        return true;
    }

    for (size_t i = 0; i < global_plan.response.plan.poses.size(); i++)
    {
        Eigen::Vector2d wp_pt =
            Eigen::Vector2d(global_plan.response.plan.poses[i].pose.position.x,
                            global_plan.response.plan.poses[i].pose.position.y);
        double dist_to_robot = (odom_pos - wp_pt).norm();

        // If dist to robot is somewhere in planning_horizon +- subgoal_tolerance

        if (abs(dist_to_robot - planning_horizon) < subgoal_tolerance)
        {
            subgoal = wp_pt;

            return true;
        }
    }

    return false;
}

void SpacialHorizon::updateSubgoalCallback(const ros::TimerEvent &e)
{
    if (disable_intermediate_planner){
        return;
    }
    else
    {
        ROS_INFO_STREAM("[Spacial Horizon] Updating subgoal");

        if (!has_goal) {
            ROS_WARN("[SpacialHorizon] No goal received yet");
            return;
        }
        Eigen::Vector2d subgoal;
        bool subgoal_success = getSubgoal(subgoal);

        // if to far away from subgoal -> recompute global path and subgoal
        double dist_to_subgoal = (odom_pos - subgoal).norm();
        if (dist_to_subgoal > planning_horizon + 1.0)
        {
            ROS_INFO_STREAM("[Spacial Horizon]: Too far away from subgoal! Recomputing global path: " 
                            << end_pos << " " << odom_pos);
            getGlobalPath();
            subgoal_success = getSubgoal(subgoal);
        }

        if (!subgoal_success)
        {
            ROS_WARN_STREAM("[Spacial Horizon] No subgoal found. No global plan received or goal reached!");
            return;
        }

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = subgoal(0);
        pose_stamped.pose.position.y = subgoal(1);
        pose_stamped.pose.position.z = 0.0;

        ROS_INFO_STREAM("[Spacial Horizon] Publishing new subgoal");

        pub_subgoal.publish(pose_stamped);
    }
}

void SpacialHorizon::getGlobalPath(const ros::TimerEvent &e) {
    getGlobalPath();
}

/* Get global plan from move_base */
void SpacialHorizon::getGlobalPath()
{
    /* get global path from move_base */
    if (!global_planner_srv)
    {
        ROS_FATAL("[SpacialHorizon - GET_PATH] Could not initialize get plan "
                  "service from %s",
                  global_planner_srv.getService().c_str());
    }

    fillPathRequest(global_plan.request);
    callPlanningService(global_planner_srv, global_plan);
}

void SpacialHorizon::fillPathRequest(nav_msgs::GetPlan::Request &request)
{
    request.start.header.frame_id = "map";
    request.start.pose.position.x =
        odom_pos[0]; // x coordinate of the initial position
    request.start.pose.position.y =
        odom_pos[1];                        // y coordinate of the initial position
    request.start.pose.orientation.w = 1.0; // direction
    request.goal.header.frame_id = "map";
    request.goal.pose.position.x = end_pos[0]; // End point coordinates
    request.goal.pose.position.y = end_pos[1];
    request.goal.pose.orientation.w = 1.0;
    request.tolerance = goal_tolerance; // If the goal cannot be reached, the
                                        // most recent constraint
}

void SpacialHorizon::callPlanningService(ros::ServiceClient &serviceClient,
                                         nav_msgs::GetPlan &srv)
{
    if (serviceClient.call(srv))
    {
        if (srv.response.plan.poses.empty())
        {
            ROS_WARN("[SpacialHorizon - GET_PATH] Global plan was empty!");
            return;
        }

        pub_global_plan.publish(srv.response.plan);
    }
    else
    {
        ROS_ERROR("[SpacialHorizon - GET_PATH] Failed to call service %s - is the "
                  "robot moving?",
                  serviceClient.getService().c_str());
    }
}

int main(int argc, char **argv)
{
    std::cout << "Spacial Horizon node started" << std::endl;
    ros::init(argc, argv, "spacial_horizon_node");

    ros::NodeHandle nh("~");
    SpacialHorizon spacial_horizon;
    spacial_horizon.init(nh);

    std::string ns = ros::this_node::getNamespace();
    ROS_INFO_STREAM(":\tSpacial_Horizon successfully loaded for namespace\t"
                    << ns);

    ros::spin();
}
