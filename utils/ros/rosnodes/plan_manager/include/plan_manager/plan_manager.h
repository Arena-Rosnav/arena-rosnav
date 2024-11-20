#ifndef PLAN_MANAGER_H
#define PLAN_MANAGER_H

#include <random> // for test
#include <Eigen/Eigen>
#include <iostream>

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"  // goal, subgoal needs time stamp
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <nav_msgs/srv/get_plan.h>
#include <plan_msgs/msg/RobotStateStamped.h>

#include <visualization_msgs/Marker.h>

#include <plan_manager/robot_state.h>
#include <plan_manager/plan_collector.h>
#include <plan_visualization/planning_visualization.h>
#include <thread>






class PlanManager{

private:
    /*
        INIT: wait for odom
        WAIT_GOAL: wait for goal(global goal)
        GEN_NEW_GLOBAL: generate new global path
        REPLAN_MID: replan for a new subgoal
        EXEC_LOCAL: no interference, let local planner do his job
    */
    enum FSM_EXEC_STATE { INIT, WAIT_GOAL, GEN_NEW_GLOBAL, REPLAN_MID, EXEC_LOCAL };
    enum MODE_TYPE { TRAIN = 1, TEST = 2};

    /* planning utils */
    PlanCollector::Ptr planner_collector_;
    PlanningVisualization::Ptr visualization_;

    /* parameters */
    int mode_;  // 1 train , 2 test
    
    
    /* planning data */
    bool have_goal_, have_odom_;
    FSM_EXEC_STATE exec_state_;

    geometry_msgs::PoseStamped goal_;              // global goal
    geometry_msgs::PoseStamped subgoal_;           // mid goal(waypoint/subgoal)
    
    //Eigen::Vector3d odom_pos_, odom_vel_;  
    //Eigen::Vector3d start_pt_, start_vel_;  
    //Eigen::Vector3d end_pt_, end_vel_;      
    RobotStatePtr cur_state_;        // robot odometry state: x y theta, vx vy w_theta
    RobotStatePtr start_state_;      // start state
    RobotStatePtr end_state_;        // goal state
    ros::Time start_time_;
    ros::Time subgoal_start_time_;
    double look_ahead_distance_;
    double tolerance_approach_;
    double timeout_goal_,timeout_subgoal_;


    
    /* ROS utils */
    auto node_ = std::make_shared<rclcpp::Node>("node_");;
    
    ros::Timer exec_timer_, safety_timer_, vis_timer_;
    
    ros::Subscriber goal_sub_, odom_sub_;
    
    ros::Publisher global_plan_pub_;
    ros::Publisher subgoal_pub_;

    ros::Publisher robot_state_pub_;


    /* helper functions */
    void changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call);
    void printFSMExecState();

    /* ROS functions */
    void goalCallback(const geometry_msgs::PoseStampedPtr& msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
    
    void execFSMCallback(const ros::TimerEvent& e);
    //void checkCollisionCallback(const ros::TimerEvent& e);



    /* test purpose*/
    

public:
    PlanManager(/* args */) {
    }

    ~PlanManager() {
    }

    void init(ros::NodeHandle & nh);


};


#endif