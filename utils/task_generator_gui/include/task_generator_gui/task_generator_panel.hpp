#ifndef TASK_GENERATOR_GUI_TASK_GENERATOR_PANEL_HPP
#define TASK_GENERATOR_GUI_TASK_GENERATOR_PANEL_HPP

#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <std_msgs/msg/string.hpp>

#include <QLabel>
#include <QPushButton>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>

namespace task_generator_gui
{
    class TaskGeneratorPanel
        : public rviz_common::Panel
    {
        Q_OBJECT

    public:
        explicit TaskGeneratorPanel(QWidget *parent = 0);
        ~TaskGeneratorPanel() override;

        void onInitialize() override;

    protected:
        std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
        // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

        void topicCallback(const std_msgs::msg::String &msg);

        // Worlds selector
        QWidget *world_selector;
        QHBoxLayout *world_selector_layout;
        QLabel *world_selector_label;
        QComboBox *world_selector_combobox;

        // Robot spawner
        QWidget *robot_spawner;
        QHBoxLayout *robot_spawner_layout;
        QLabel *robot_spawner_label;
        QComboBox *robot_spawner_combobox;
        QPushButton *robot_spawner_button;

        // Obstacles tab
        QWidget *obstacles_widget;
        QVBoxLayout *obstacles_tab_layout;
        QHBoxLayout *obstacles_task_mode_layout;
        QWidget *obstacles_task_mode_widget;
        QLabel *obstacles_task_mode_label;
        QComboBox *obstacle_task_mode_combobox;

        // Robots tab
        QWidget *robots_widget;
        QVBoxLayout *robots_tab_layout;
        QHBoxLayout *robots_task_mode_layout;
        QWidget *robots_task_mode_widget;
        QLabel *robots_task_mode_label;
        QComboBox *robot_task_mode_combobox;

        QPushButton *reset_scenario_button;
        QTabWidget *obstacles_robots_tab_widget;

    private Q_SLOTS:
        void buttonActivated();
    };
} // namespace arena_rosnav
#endif // ARENA_ROSNAV_TASK_GENERATOR_PANEL_HPP