/*
    Use global path of move_base and publish Subgoals based on SpacialHorizon
   algorithm
*/
#include <spacial_horizon/spacial_horizon_node.h>

void SpacialHorizon::init(ros::NodeHandle &nh)
{
    has_odom = false;
    has_goal = false;

    /*  fsm param  */
    nh.param("fsm/goal_tolerance", goal_tolerance, 0.5);
    nh.param("fsm/subgoal_tolerance", subgoal_tolerance, 0.2);
    nh.param("fsm/subgoal_pub_period", subgoal_pub_period, 0.5);
    nh.param("fsm/planning_horizon", planning_horizon, 3.0);

    subgoal_timer = nh.createTimer(
        ros::Duration(0.05), &SpacialHorizon::updateSubgoalCallback, this);
    update_global_plan_timer = nh.createTimer(
        ros::Duration(0.1), &SpacialHorizon::getGlobalPath, this);
    

    /* ros communication with public node */
    ros::NodeHandle public_nh; // sim1/goal
    sub_goal =
        public_nh.subscribe("goal", 1, &SpacialHorizon::goalCallback, this);
    sub_odom =
        public_nh.subscribe("odom", 1, &SpacialHorizon::odomCallback, this);

    sub_initial_pose = public_nh.subscribe(
        "initialpose", 0, &SpacialHorizon::initialPoseCallback, this);

    pub_subgoal =
        public_nh.advertise<geometry_msgs::PoseStamped>("subgoal", 10);
    pub_global_plan = public_nh.advertise<nav_msgs::Path>("global_plan", 10);
}

void SpacialHorizon::initialPoseCallback(
    const geometry_msgs::PoseWithCovarianceStampedPtr &msg)
{
    initial_pos =
        Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
}

void SpacialHorizon::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    odom_pos =
        Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
    odom_vel =
        Eigen::Vector2d(msg->twist.twist.linear.x, msg->twist.twist.linear.y);

    has_odom = true;
}

void SpacialHorizon::goalCallback(const geometry_msgs::PoseStampedPtr &msg)
{
    if (!has_odom)
        return;

    end_pos = Eigen::Vector2d(msg->pose.position.x, msg->pose.position.y);

    has_goal = true;

    getGlobalPath();
}

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
    std::cout << "GEN NEW SUBGOAL" << std::endl;


    if (!has_goal) {
    std::cout << "NO GOAL" << std::endl;

        return;

    }
    Eigen::Vector2d subgoal;
    bool subgoal_success = getSubgoal(subgoal);
    ;

    // if to far away from subgoal -> recompute global path and subgoal
    double dist_to_subgoal = (odom_pos - subgoal).norm();
    if (dist_to_subgoal > planning_horizon + 1.0)
    {
        std::cout << "[Spacial Horizon]: Too far away from subgoal! Recomputing "
                     "global path " << end_pos << " " << odom_pos
                  << std::endl;
        getGlobalPath();
        subgoal_success = getSubgoal(subgoal);
    }

    if (!subgoal_success)
    {
    std::cout << "SUBGOAL SUCCESS" << std::endl;
        
        return;
    }

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose.position.x = subgoal(0);
    pose_stamped.pose.position.y = subgoal(1);
    pose_stamped.pose.position.z = 0.0;

    std::cout << "PUBLISHING SUBGOAL" << std::endl;

    pub_subgoal.publish(pose_stamped);
}

void SpacialHorizon::getGlobalPath(const ros::TimerEvent &e) {
    getGlobalPath();
}

/* Get global plan from move_base */
void SpacialHorizon::getGlobalPath()
{
    /* get global path from move_base */
    ros::NodeHandle nh;
    std::string service_name = "/move_base/NavfnROS/make_plan";

    while (!ros::service::waitForService(service_name, ros::Duration(3.0)))
    {
        ROS_INFO("[SpacialHorizon - GET_PATH] Waiting for service "
                 "/move_base/NavfnROS/make_plan to become available");
    }

    ros::ServiceClient serviceClient =
        nh.serviceClient<nav_msgs::GetPlan>(service_name, true);

    if (!serviceClient)
    {
        ROS_FATAL("[SpacialHorizon - GET_PATH] Could not initialize get plan "
                  "service from %s",
                  serviceClient.getService().c_str());
    }

    fillPathRequest(global_plan.request);
    callPlanningService(serviceClient, global_plan);
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
            ROS_WARN("[SpacialHorizon - GET_PATH] Got empty plan");
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
