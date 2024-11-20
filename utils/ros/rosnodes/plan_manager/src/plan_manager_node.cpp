#include <plan_manager/plan_manager.h>




int main(int argc, char** argv) {
    // DEBUG
    rclcpp::init(argc, argv, "plan_manager");
    std::string ns = ros::this_node::getNamespace();
    //auto node_handle = std::make_shared<rclcpp::Node>("node_handle");"~"); every topic will be with namespace
    auto node_handle = std::make_shared<rclcpp::Node>("node_handle");"");
    //ros::WallRate r(100);
    PlanManager plan_manager;
    plan_manager.init(node_handle);
    
    rclcpp::spin(node);
}