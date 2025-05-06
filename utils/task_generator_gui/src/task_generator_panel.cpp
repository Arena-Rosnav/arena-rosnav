#include <task_generator_gui/task_generator_panel.hpp>
#include <rviz_common/display_context.hpp>

namespace task_generator_gui
{
    TaskGeneratorPanel::TaskGeneratorPanel(QWidget *parent) : Panel(parent)
    {
        // Set the main layout
        const auto layout = new QVBoxLayout(this);
        reset_scenario_button = new QPushButton("Reset Scenario");

        // "Worlds" selector
        world_selector = new QWidget();
        world_selector_layout = new QHBoxLayout();
        world_selector_label = new QLabel("World");
        world_selector_combobox = new QComboBox();

        world_selector_layout->addWidget(world_selector_label);
        world_selector_combobox->addItems({"World 1", "World 2", "World 3"});
        world_selector_layout->addWidget(world_selector_combobox);
        world_selector->setLayout(world_selector_layout);

        // Robot spawner
        robot_spawner = new QWidget();
        robot_spawner_layout = new QHBoxLayout();
        robot_spawner_label = new QLabel("Robot");
        robot_spawner_combobox = new QComboBox();
        robot_spawner_button = new QPushButton("Spawn");

        robot_spawner_layout->addWidget(robot_spawner_label);
        robot_spawner_combobox->addItems({"Robot 1", "Robot 2", "Robot 3"});
        robot_spawner_layout->addWidget(robot_spawner_combobox);
        robot_spawner_layout->addWidget(robot_spawner_button);
        robot_spawner->setLayout(robot_spawner_layout);

        // Obstacles tab
        obstacles_widget = new QWidget();
        obstacles_tab_layout = new QVBoxLayout();
        obstacles_task_mode_widget = new QWidget();
        obstacles_task_mode_layout = new QHBoxLayout();
        obstacles_task_mode_label = new QLabel("Task mode");
        obstacle_task_mode_combobox = new QComboBox();

        obstacle_task_mode_combobox->addItems({"Environment", "Parametrized", "Random", "Scenario"});
        obstacles_task_mode_layout->addWidget(obstacles_task_mode_label);
        obstacles_task_mode_layout->addWidget(obstacle_task_mode_combobox);
        obstacles_task_mode_widget->setLayout(obstacles_task_mode_layout);
        obstacles_tab_layout->addWidget(obstacles_task_mode_widget);
        obstacles_widget->setLayout(obstacles_tab_layout);

        // Robots tab
        robots_widget = new QWidget();
        robots_tab_layout = new QVBoxLayout();
        robots_task_mode_widget = new QWidget();
        robots_task_mode_layout = new QHBoxLayout();
        robots_task_mode_label = new QLabel("Task mode");
        robot_task_mode_combobox = new QComboBox();

        robot_task_mode_combobox->addItems({"Explore", "Guided", "Random", "Scenario"});
        robots_task_mode_layout->addWidget(robots_task_mode_label);
        robots_task_mode_layout->addWidget(robot_task_mode_combobox);
        robots_task_mode_widget->setLayout(robots_task_mode_layout);
        robots_tab_layout->addWidget(robots_task_mode_widget);
        robots_widget->setLayout(robots_tab_layout);

        // Add tabs to the tab widget
        obstacles_robots_tab_widget = new QTabWidget(this);
        obstacles_robots_tab_widget->addTab(obstacles_widget, "Obstacles");
        obstacles_robots_tab_widget->addTab(robots_widget, "Robots");

        layout->addWidget(world_selector);
        layout->addWidget(robot_spawner);
        layout->addWidget(obstacles_robots_tab_widget);
        layout->addWidget(reset_scenario_button);
        // setLayout(layout);

        // QObject::connect(button_, &QPushButton::released, this, &DemoPanel::buttonActivated);
    }

    TaskGeneratorPanel::~TaskGeneratorPanel() = default;

    void TaskGeneratorPanel::onInitialize()
    {
        // Access the abstract ROS Node and
        // in the process lock it for exclusive use until the method is done.
        node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

        // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
        // (as per normal rclcpp code)
        rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();

        // // Create a String publisher for the output
        // publisher_ = node->create_publisher<std_msgs::msg::String>("/output", 10);

        // // Create a String subscription and bind it to the topicCallback inside this class.
        // subscription_ = node->create_subscription<std_msgs::msg::String>("/input", 10, std::bind(&DemoPanel::topicCallback, this, std::placeholders::_1));
    }

    // When the subscriber gets a message, this callback is triggered,
    // and then we copy its data into the widget's label
    // void DemoPanel::topicCallback(const std_msgs::msg::String &msg)
    // {
    //     label_->setText(QString(msg.data.c_str()));
    // }

    void TaskGeneratorPanel::buttonActivated()
    {
        // auto message = std_msgs::msg::String();
        // message.data = "Button clicked!";
        // publisher_->publish(message);
    }

} // namespace task_generator_gui

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(task_generator_gui::TaskGeneratorPanel, rviz_common::Panel)