#include <plan_visualization/planning_visualization.h>
#include <iostream> // Include for std::cout (used in getColor)

PlanningVisualization::PlanningVisualization(const rclcpp::NodeOptions & options) : Node("planning_visualization", options) {
  goal_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning_vis/goal", 20);
  pubs_.push_back(goal_pub_);

  subgoal_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning_vis/subgoal", 20);
  pubs_.push_back(subgoal_pub_);

  global_path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning_vis/global_path", 20);
  pubs_.push_back(global_path_pub_);
}

void PlanningVisualization::displaySphereList(const std::vector<Eigen::Vector3d>& list, double resolution,
                                              const Eigen::Vector4d& color, int id, int pub_id) {
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp    = this->now();
  mk.type            = visualization_msgs::msg::Marker::SPHERE_LIST;
  mk.action          = visualization_msgs::msg::Marker::DELETE;
  mk.id              = id;
  pubs_[pub_id]->publish(mk);

  mk.action             = visualization_msgs::msg::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::msg::Point pt;
  for (size_t i = 0; i < list.size(); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id]->publish(mk);
}



void PlanningVisualization::displayLineList(const std::vector<Eigen::Vector3d>& list1, double line_width,
                                            const Eigen::Vector4d& color, int id, int pub_id) {
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp    = this->now();
  mk.type            = visualization_msgs::msg::Marker::LINE_LIST;
  mk.action          = visualization_msgs::msg::Marker::DELETE;
  mk.id              = id;
  pubs_[pub_id]->publish(mk);

  mk.action             = visualization_msgs::msg::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = line_width;

  geometry_msgs::msg::Point pt;
  for (size_t i = 0; i < list1.size(); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id]->publish(mk);
}

void PlanningVisualization::drawGoal(const geometry_msgs::msg::PoseStamped& goal, double resolution,
                                     const Eigen::Vector4d& color, int id) {
            
    Eigen::Vector3d goal_pt(goal.pose.position.x, goal.pose.position.y, 0.0);
    std::vector<Eigen::Vector3d> goal_vec = { goal_pt };
    displaySphereList(goal_vec, resolution, color, GOAL + id % 100);
}

void PlanningVisualization::drawSubgoal(const geometry_msgs::msg::PoseStamped& subgoal, double resolution,
                                        const Eigen::Vector4d& color, int id) {

    Eigen::Vector3d subgoal_pt(subgoal.pose.position.x, subgoal.pose.position.y, 0.0);
    std::vector<Eigen::Vector3d> subgoal_vec = { subgoal_pt };
    displaySphereList(subgoal_vec, resolution, color, SUBGOAL + id % 100);

}

void PlanningVisualization::drawGlobalPath(const nav_msgs::msg::Path& global_path, double resolution,
                                           const Eigen::Vector4d& color, int id) {
  // draw new path
  std::vector<Eigen::Vector3d> edge_pt;
  for(size_t i = 0; i < global_path.poses.size(); i++){
        Eigen::Vector3d wp_pose;
        wp_pose(0) = global_path.poses[i].pose.position.x;
        wp_pose(1) = global_path.poses[i].pose.position.y;
        wp_pose(2) = 0.0;
        edge_pt.push_back(wp_pose);      
  }
  
  displayLineList(edge_pt, resolution, color, GLOBAL_PATH + id % 100);
}


Eigen::Vector4d PlanningVisualization::getColor(double h, double alpha) {
  if (h < 0.0 || h > 1.0) {
    std::cout << "h out of range" << std::endl;
    h = 0.0;
  }

  double          lambda;
  Eigen::Vector4d color1, color2;
  if (h >= -1e-4 && h < 1.0 / 6) {
    lambda = (h - 0.0) * 6;
    color1 = Eigen::Vector4d(1, 0, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 1, 1);

  } else if (h >= 1.0 / 6 && h < 2.0 / 6) {
    lambda = (h - 1.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 0, 1, 1);

  } else if (h >= 2.0 / 6 && h < 3.0 / 6) {
    lambda = (h - 2.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 1, 1);

  } else if (h >= 3.0 / 6 && h < 4.0 / 6) {
    lambda = (h - 3.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 0, 1);

  } else if (h >= 4.0 / 6 && h < 5.0 / 6) {
    lambda = (h - 4.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 1, 0, 1);

  } else if (h >= 5.0 / 6 && h <= 1.0 + 1e-4) {
    lambda = (h - 5.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 0, 1);
  }

  Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
  fcolor(3)              = alpha;

  return fcolor;
}