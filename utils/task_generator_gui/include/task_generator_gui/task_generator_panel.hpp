#ifndef TASK_GENERATOR_GUI_TASK_GENERATOR_PANEL_HPP
#define TASK_GENERATOR_GUI_TASK_GENERATOR_PANEL_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/node.hpp"

#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/properties/property_tree_model.hpp>
#include "task_generator_msgs/srv/get_environments.hpp"
#include "task_generator_msgs/srv/get_parametrizeds.hpp"
#include "task_generator_msgs/srv/get_randoms.hpp"
#include "task_generator_msgs/srv/get_scenarios.hpp"
#include "task_generator_msgs/srv/get_worlds.hpp"
#include "task_generator_msgs/srv/get_robots.hpp"

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

#include <QLabel>
#include <QPushButton>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QTreeView>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QTreeView>
#include <QStandardItemModel>
#include <QDir>
#include <QHeaderView>
#include <QSpinBox>
#include <QCheckBox>
#include <QGroupBox>
#include <QFontMetrics>

namespace task_generator_gui
{
    using rviz_common::properties::PropertyTreeModel;

    class TaskGeneratorPanel
        : public rviz_common::Panel
    {
        Q_OBJECT

    public:
        explicit TaskGeneratorPanel(QWidget *parent = 0);
        ~TaskGeneratorPanel() override;
        void onInitialize() override;

        // Get available robot models
        void getRobots();
        // Get available worlds
        void getWorlds();

        void getCurrentTaskGeneratorNodeParams();

        void getTMObstaclesParams();
        void getScenarios(const std::string &world_name);
        void getTMRobotsParams();

        void setTMObstaclesParamsRequest(rcl_interfaces::srv::SetParameters::Request::SharedPtr request);
        void setTMRobotsParamsRequest(rcl_interfaces::srv::SetParameters::Request::SharedPtr request);
        void setParams();

        void setupUi();
        QComboBox *setupComboBoxWithLabel(QLayout *parent, const QStringList &combobox_values, const QString &label);
        QTabWidget *setupTabs(QLayout *Parent);
        QTreeWidget *setupTree(QLayout *parent);
        void setupObstaclesTreeItem();
        void setupRobotsTreeItem();
        QWidget *setupMinMaxSpinBox(std::vector<std::int64_t, std::allocator<std::int64_t>> *connected_values);
        QGroupBox *setupGroupCheckBox(std::vector<std::string> check_box_texts, std::vector<int> *connected_hash_map);

    protected:
        std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr;
        rclcpp::Node::SharedPtr node;

        // Node to get configs
        std::shared_ptr<rclcpp::Node> service_node;
        // Client to get list of all available environments
        rclcpp::Client<task_generator_msgs::srv::GetEnvironments>::SharedPtr get_environments_client;
        // Client to get list of all available parametrizeds
        rclcpp::Client<task_generator_msgs::srv::GetParametrizeds>::SharedPtr get_parametrizeds_client;
        // Client to get all parameters for Random Obstacles Task Mode
        rclcpp::Client<task_generator_msgs::srv::GetRandoms>::SharedPtr get_randoms_client;
        // Client to get list of all available scenarios for given world
        rclcpp::Client<task_generator_msgs::srv::GetScenarios>::SharedPtr get_scenarios_client;
        // Client to get list of all available worlds
        rclcpp::Client<task_generator_msgs::srv::GetWorlds>::SharedPtr get_worlds_client;
        // Client to get list of all available robots models
        rclcpp::Client<task_generator_msgs::srv::GetRobots>::SharedPtr get_robots_client;
        // Client to get ROS parameters from Node "/task_generator_node"
        std::shared_ptr<rclcpp::SyncParametersClient> parameters_client;
        // Client to set ROS parameters for Node "/task_generator_node"
        rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_param_client;
        // Client to reset task
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_task_client;

        // Selected robot model
        std::string selected_robot_model;
        // Selected world
        std::string selected_world;

        // All available robot models
        std::vector<std::string> robot_models;
        // All available robot worlds
        std::vector<std::string> worlds;

        // Parameters for Obstacles Task Mode = "Random"
        std::vector<std::int64_t, std::allocator<std::int64_t>> n_static_obstacles_range, n_interactive_obstacles_range, n_dynamic_obstacles_range;
        // Parameters for Obstacles Task Mode = "Random"
        std::vector<std::string> static_obstacles_all_models, interactive_obstacles_all_models, dynamic_obstacles_all_models;
        // Selected obstacles models
        std::vector<std::string> static_obstacles_models, interactive_obstacles_models, dynamic_obstacles_models;
        // Hash map for seletected obstacles models
        std::vector<int> static_obstacles_models_selected, interactive_obstacles_models_selected, dynamic_obstacles_models_selected;

        // Parameters for Obstacles Task Mode = "Environment" or "Parametrized" or "Scenario"
        std::vector<std::string> environment_config_files;
        QStringList environment_config_files_qstringlist;
        std::vector<std::string> parametrized_config_files;
        QStringList parametrized_config_files_qstringlist;
        std::vector<std::string> scenario_config_files;
        QStringList scenario_config_files_qstringlist;

        std::string selected_environment_config_file;
        std::string selected_parametrized_config_file;
        std::string selected_scenario_config_file;

        // UI Components
        QVBoxLayout *root_layout;
        QString obstacles_task_mode;
        QString robots_task_mode;
        QTabWidget *tabs;
        QTreeWidget *obstacles_tree;
        QTreeWidget *robots_tree;
        QComboBox *obstacles_task_mode_combobox;
        QComboBox *robot_task_mode_combobox;
        QComboBox *robot_combobox;
        QComboBox *world_combobox;
        QPushButton *reset_scenario_button;

    private Q_SLOTS:
        void resetScenarioButtonActivated();

        void onRobotChanged(const QString &text);
        void onWorldChanged(const QString &text);

        void onObstaclesTaskModeChanged(const QString &text);
        void onRobotsTaskModeChanged(const QString &text);
    };
} // namespace arena_rosnav
#endif // ARENA_ROSNAV_TASK_GENERATOR_PANEL_HPP