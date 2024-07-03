#ifndef PLAN_COLLECTOR_H
#define PLAN_COLLECTOR_H

#include <Eigen/Eigen>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/srv/get_plan.h>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"  // goal, subgoal needs time stamp


#include <plan_manager/robot_state.h>
#include <plan_msgs/MakeGlobalPlan.h>

class PlanCollector{

public:

    //PlanParameters pp_;
    //LocalTrajData local_data_;
    //GlobalTrajData global_data_;
    //MidPlanData plan_data_;
    //EDTEnvironment::Ptr edt_environment_;

    // global 
    geometry_msgs::PoseStamped goal_;              // global goal
    nav_msgs::Path global_path_;                // global path
    

    // mid
    geometry_msgs::PoseStamped subgoal_;              // subgoal
    RobotStatePtr subgoal_state_;

    // local

private:
    ros::ServiceClient global_plan_client_;

    double look_ahead_distance_;
    double tolerance_approach_;



public:
    PlanCollector();
    ~PlanCollector();
    void initPlanModules(ros::NodeHandle& nh);
    

    bool generate_global_plan(RobotState &start_state,RobotState &end_state);

    bool generate_subgoal(RobotStatePtr cur_state, RobotStatePtr end_state, nav_msgs::Path global_path_ptr, double obstacle_info, double sensor_info);



public:
  typedef std::unique_ptr<PlanCollector> Ptr;

};


#endif