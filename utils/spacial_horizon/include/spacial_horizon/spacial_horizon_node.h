#ifndef _SPACIAL_HORIZON_NODE_H_
#define _SPACIAL_HORIZON_NODE_H_

#include <Eigen/Eigen>
#include <iostream>
#include <ros/ros.h>
#include <algorithm>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

#include <geometry_msgs/Twist.h>

#define PI 3.14159265

class SpacialHorizon
{
private:
    // enum FSM_EXEC_STATE {INIT, WAIT_GOAL};
    /* planning data */
    bool has_goal, has_odom;

    geometry_msgs::PoseStamped ps_odom;

    Eigen::Vector2d odom_pos, odom_vel, initial_pos;

    Eigen::Vector2d end_pos;

    // subscriber
    ros::Subscriber sub_goal, sub_odom, sub_initial_pose;

    // publisher
    ros::Publisher pub_subgoal, pub_global_plan;

    // plan with global path from move base
    nav_msgs::GetPlan global_plan;

    /* parameters */
    double goal_tolerance;    // meter
    double subgoal_tolerance; // meter
    double subgoal_pub_period;
    double planning_horizon;

    /* ROS utils */
    ros::Timer subgoal_timer, update_global_plan_timer;

    /* ros related callback*/
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    void goalCallback(const geometry_msgs::PoseStampedPtr &msg);
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedPtr &msg);

    bool getSubgoal(Eigen::Vector2d &subgoal);
    void updateSubgoalCallback(const ros::TimerEvent &e);

    /* get global plan from move base */
    void getGlobalPath();
    void getGlobalPath(const ros::TimerEvent &e);
    void fillPathRequest(nav_msgs::GetPlan::Request &request);
    void callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv);

public:
    SpacialHorizon(/* args */) {}
    ~SpacialHorizon() {}

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
