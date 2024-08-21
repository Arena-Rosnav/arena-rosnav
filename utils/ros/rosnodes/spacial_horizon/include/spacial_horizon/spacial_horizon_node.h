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
#include <std_srvs/Empty.h>

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

#include <geometry_msgs/Twist.h>

#define SUB_TOPIC_GOAL "move_base_simple/goal"
#define SUB_TOPIC_ODOM "odom"
#define PUB_TOPIC_SUBGOAL "subgoal"
#define PUB_TOPIC_GLOBAL_PLAN "global_plan"
#define SERVICE_GLOBAL_PLANNER "move_base_flex/NavfnROS/make_plan"
#define SERVICE_STEP_WORLD "step_world"

class SpacialHorizon
{
private:
    ros::NodeHandle nh_;
    // enum FSM_EXEC_STATE {INIT, WAIT_GOAL};
    /* planning data */
    bool has_timers, has_goal, has_odom;

    geometry_msgs::PoseStamped ps_odom;

    Eigen::Vector2d odom_pos, odom_vel, initial_pos;

    Eigen::Vector2d end_pos;
    Eigen::Vector2d subgoal_pos;


    // subscriber
    ros::Subscriber sub_goal, sub_odom, sub_initial_pose;

    // publisher
    ros::Publisher pub_subgoal, pub_global_plan;

    // service
    ros::ServiceClient global_planner_srv, step_world_srv;

    // plan with global path from move base
    nav_msgs::GetPlan global_plan;

    /* parameters */
    bool train_mode;
    bool disable_intermediate_planner;
    bool publish_goal_on_subgoal_fail = false;
    double goal_tolerance;    // meter
    double subgoal_tolerance; // meter
    double subgoal_reach_tolerance; // meter
    double subgoal_pub_period;
    double planning_horizon;
    double update_global_period;

    /* ROS utils */
    ros::Timer subgoal_timer, update_global_plan_timer;

    /* init methods */
    void initializeGlobalPlanningService();
    void initializeStepWorldService();
    void initializeTimers();

    /* ros related callback*/
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    void goalCallback(const geometry_msgs::PoseStampedPtr &msg);
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedPtr &msg);

    bool getSubgoal(Eigen::Vector2d &subgoal);
    void updateSubgoalCallback(const ros::TimerEvent &e);
    void publishSubgoal(Eigen::Vector2d &subgoal);
    bool tryUpdateGlobalplanAndSubgoal(int try_count = 0);

    /* get global plan from move base */
    void getGlobalPath();
    void getGlobalPath(const ros::TimerEvent &e);
    void fillPathRequest(nav_msgs::GetPlan::Request &request);
    bool callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv);

public:
    SpacialHorizon(/* args */) {}
    ~SpacialHorizon() {}

    void init(ros::NodeHandle &nh);
};

#endif
