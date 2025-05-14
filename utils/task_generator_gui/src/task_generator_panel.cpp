#include "task_generator_gui/task_generator_panel.hpp"
#include "rviz_common/display_context.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rcl_interfaces/srv/set_parameters.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
namespace task_generator_gui
{
    TaskGeneratorPanel::TaskGeneratorPanel(QWidget *parent) : Panel(parent)
    {
        root_layout = new QVBoxLayout(this);
    }

    TaskGeneratorPanel::~TaskGeneratorPanel() = default;

    void TaskGeneratorPanel::onInitialize()
    {
        // Access the abstract ROS Node and
        // in the process lock it for exclusive use until the method is done.
        node_ptr = getDisplayContext()->getRosNodeAbstraction().lock();

        // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
        // (as per normal rclcpp code)
        node = node_ptr->get_raw_node();

        // Create a new node for the service clients
        service_node = std::make_shared<rclcpp::Node>("tm_service_node");

        // Create a String client to call to the "/task_generator_node/get_environments" service
        get_environments_client = service_node->create_client<task_generator_msgs::srv::GetEnvironments>("/task_generator_node/get_environments");

        // Create a String client to call to the "/task_generator_node/get_parametrizeds" service
        get_parametrizeds_client = service_node->create_client<task_generator_msgs::srv::GetParametrizeds>("/task_generator_node/get_parametrizeds");

        // Create a String client to call to the "/task_generator_node/get_randoms" service
        get_randoms_client = service_node->create_client<task_generator_msgs::srv::GetRandoms>("/task_generator_node/get_randoms");

        // Create a String client to call to the "/task_generator_node/get_scenarios" service
        get_scenarios_client = service_node->create_client<task_generator_msgs::srv::GetScenarios>("/task_generator_node/get_scenarios");

        // Create a String client to call to the "/task_generator_node/get_param" service
        get_param_client = service_node->create_client<rcl_interfaces::srv::GetParameters>("/task_generator_node/get_parameters");

        // Create a String client to call to the "/task_generator_node/set_parameters" service
        set_param_client = service_node->create_client<rcl_interfaces::srv::SetParameters>("/task_generator_node/set_parameters");

        // Create a String client to call to the "/task_generator_node/reset_task" service
        reset_task_client = service_node->create_client<std_srvs::srv::Empty>("/task_generator_node/reset_task");

        get_worlds_client = service_node->create_client<task_generator_msgs::srv::GetWorlds>("/task_generator_node/get_worlds");

        get_robots_client = service_node->create_client<task_generator_msgs::srv::GetRobots>("/task_generator_node/get_robots");

        parameters_client = std::make_shared<rclcpp::SyncParametersClient>(service_node, "task_generator_node");

        getRobots();
        getWorlds();
        getTMObstaclesParams();
        getTMRobotsParams();
        getCurrentTaskGeneratorNodeParams();

        setupUi();
    }

    void TaskGeneratorPanel::getRobots()
    {
        auto request = std::make_shared<task_generator_msgs::srv::GetRobots::Request>();

        while (!get_robots_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(service_node->get_logger(), "Interrupted while watiting for the /task_generator_node/get_robots service!. Exiting.");
                return;
            }
            RCLCPP_INFO(service_node->get_logger(), "Service is not available, waiting again...");
        }

        auto future = get_robots_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(service_node, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(service_node->get_logger(), "Got response from get_robots_client!");
        }
        else
        {
            RCLCPP_ERROR(service_node->get_logger(), "Failed to call service!");
        }

        auto response = future.get();
        robot_models = response->robots;
        selected_robot_model = robot_models[0];
    }

    void TaskGeneratorPanel::getWorlds()
    {
        auto request = std::make_shared<task_generator_msgs::srv::GetWorlds::Request>();

        while (!get_worlds_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(service_node->get_logger(), "Interrupted while watiting for the /task_generator_node/get_worlds service!. Exiting.");
                return;
            }
            RCLCPP_INFO(service_node->get_logger(), "Service is not available, waiting again...");
        }

        auto future = get_worlds_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(service_node, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(service_node->get_logger(), "Got response from get_worlds_client!");
        }
        else
        {
            RCLCPP_ERROR(service_node->get_logger(), "Failed to call service!");
        }

        auto response = future.get();
        worlds = response->worlds;
        selected_world = worlds[0];
    }

    void TaskGeneratorPanel::getCurrentTaskGeneratorNodeParams()
    {
        if (!parameters_client)
        {
            RCLCPP_WARN(rclcpp::get_logger("TaskGeneratorPanel"),
                        "Parameter client is not available.");
            return;
        }

        while (!parameters_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(service_node->get_logger(), "Interrupted while watiting for parameters_client service!. Exiting.");
                return;
            }
            RCLCPP_INFO(service_node->get_logger(), "Service is not available, waiting again...");
        }

        try
        {
            RCLCPP_INFO(service_node->get_logger(), "Getting parameters from /task_generator_node");
            if (parameters_client->has_parameter("tm_obstacles"))
            {
                auto current_obstacles_tm = parameters_client->get_parameter<std::string>("tm_obstacles");

                RCLCPP_INFO(service_node->get_logger(), "Current Obstacles Task Mode: %s", current_obstacles_tm.c_str());

                if (current_obstacles_tm == "environment")
                {
                    obstacles_task_mode = QString("Environment");

                    auto config_file = parameters_client->get_parameter<std::string>("task.environment.file");
                    selected_environment_config_file = config_file;
                }
                else if (current_obstacles_tm == "parametrized")
                {
                    obstacles_task_mode = QString("Parametrized");

                    auto config_file = parameters_client->get_parameter<std::string>("task.parametrized.file");
                    selected_parametrized_config_file = config_file;
                }
                else if (current_obstacles_tm == "random")
                {
                    obstacles_task_mode = QString("Random");

                    auto current_static_models = parameters_client->get_parameter<std::vector<std::string>>("task.random.static.models");
                    auto current_interactive_models = parameters_client->get_parameter<std::vector<std::string>>("task.random.interactive.models");
                    auto current_dynamic_models = parameters_client->get_parameter<std::vector<std::string>>("task.random.dynamic.models");

                    for (int i = 0; i < int(static_obstacles_all_models.size()); i++)
                    {
                        for (auto &model : current_static_models)
                        {
                            if (static_obstacles_all_models[i] == model)
                            {
                                static_obstacles_models_selected[i] = 1;
                                RCLCPP_INFO(service_node->get_logger(), "Static obstacles model selected: %s", static_obstacles_all_models[i].c_str());
                            }
                        }
                    }

                    for (int i = 0; i < int(interactive_obstacles_all_models.size()); i++)
                    {
                        for (auto &model : current_interactive_models)
                        {
                            if (interactive_obstacles_all_models[i] == model)
                            {
                                interactive_obstacles_models_selected[i] = 1;
                            }
                        }
                    }

                    for (int i = 0; i < int(dynamic_obstacles_all_models.size()); i++)
                    {
                        for (auto &model : current_dynamic_models)
                        {
                            if (dynamic_obstacles_all_models[i] == model)
                            {
                                dynamic_obstacles_models_selected[i] = 1;
                                RCLCPP_INFO(service_node->get_logger(), "Dynamic obstacles model selected: %s", dynamic_obstacles_all_models[i].c_str());
                            }
                        }
                    }

                    RCLCPP_INFO(service_node->get_logger(), "Static obstacles models selected: ");

                    n_static_obstacles_range = parameters_client->get_parameter<std::vector<int64_t>>("task.random.static.n");

                    n_interactive_obstacles_range = parameters_client->get_parameter<std::vector<int64_t>>("task.random.interactive.n");

                    n_dynamic_obstacles_range = parameters_client->get_parameter<std::vector<int64_t>>("task.random.dynamic.n");
                }
                else if (current_obstacles_tm == "scenario")
                {
                    obstacles_task_mode = QString("Scenario");

                    auto config_file = parameters_client->get_parameter<std::string>("task.scenario.file");

                    selected_scenario_config_file = config_file;
                }
            }

            if (parameters_client->has_parameter("tm_robots"))
            {
                auto current_robots_tm = parameters_client->get_parameter<std::string>("tm_robots");

                if (current_robots_tm == "explore")
                {
                    robots_task_mode = QString("Explore");
                }
                else if (current_robots_tm == "guided")
                {
                    robots_task_mode = QString("Guided");
                }
                else if (current_robots_tm == "random")
                {
                    robots_task_mode = QString("Random");
                }
                else if (current_robots_tm == "scenario")
                {
                    robots_task_mode = QString("Scenario");
                }

                RCLCPP_INFO(service_node->get_logger(), "Current Robot Task Mode: %s", current_robots_tm.c_str());
            }

            if (parameters_client->has_parameter("robot"))
            {
                auto current_robot_model = parameters_client->get_parameter<std::string>("robot");
                selected_robot_model = current_robot_model;
            }

            if (parameters_client->has_parameter("world"))
            {
                auto current_world = parameters_client->get_parameter<std::string>("world");
                selected_world = current_world;
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to get parameters: %s", e.what());
        }
    }

    void TaskGeneratorPanel::getTMObstaclesParams()
    {
        try
        {
            // Get configs for Environment Obstacles Task Mode
            auto environment_request = std::make_shared<task_generator_msgs::srv::GetEnvironments::Request>();

            while (!get_environments_client->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(service_node->get_logger(), "Interrupted while watiting for the service!. Exiting.");
                    return;
                }
                RCLCPP_INFO(service_node->get_logger(), "Service is not available, waiting again...");
            }

            auto environment_future = get_environments_client->async_send_request(environment_request);
            if (rclcpp::spin_until_future_complete(service_node, environment_future) == rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(service_node->get_logger(), "Got response from get_environments_client!");
            }
            else
            {
                RCLCPP_ERROR(service_node->get_logger(), "Failed to call service!");
            }

            auto environment_response = environment_future.get();
            environment_config_files = environment_response->environments;

            environment_config_files_qstringlist = QStringList();
            for (const auto &environment : environment_config_files)
            {
                environment_config_files_qstringlist << QString::fromStdString(environment);
            }

            // Get configs for Parametrized Obstacles Task Mode
            auto parametrized_request = std::make_shared<task_generator_msgs::srv::GetParametrizeds::Request>();

            while (!get_parametrizeds_client->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(service_node->get_logger(), "Interrupted while watiting for the service!. Exiting.");
                    return;
                }
                RCLCPP_INFO(service_node->get_logger(), "Service is not available, waiting again...");
            }

            auto parametrized_future = get_parametrizeds_client->async_send_request(parametrized_request);
            if (rclcpp::spin_until_future_complete(service_node, parametrized_future) == rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(service_node->get_logger(), "Got response from get_parametrizeds_client!");
            }
            else
            {
                RCLCPP_ERROR(service_node->get_logger(), "Failed to call service!");
            }

            auto parametrized_response = parametrized_future.get();
            parametrized_config_files = parametrized_response->parametrizeds;

            parametrized_config_files_qstringlist = QStringList();
            for (const auto &parametrized : parametrized_config_files)
            {
                parametrized_config_files_qstringlist << QString::fromStdString(parametrized);
            }

            // Get configs for Random Obstacles Task Mode
            auto random_request = std::make_shared<task_generator_msgs::srv::GetRandoms::Request>();

            while (!get_randoms_client->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(service_node->get_logger(), "Interrupted while watiting for the service!. Exiting.");
                    return;
                }
                RCLCPP_INFO(service_node->get_logger(), "Service is not available, waiting again...");
            }

            auto random_future = get_randoms_client->async_send_request(random_request);
            if (rclcpp::spin_until_future_complete(service_node, random_future) == rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(service_node->get_logger(), "Got response from get_randoms_client!");
            }
            else
            {
                RCLCPP_ERROR(service_node->get_logger(), "Failed to call service!");
            }

            auto random_response = random_future.get();
            n_static_obstacles_range = random_response->n_static_obstacles;
            n_interactive_obstacles_range = random_response->n_interactive_obstacles;
            n_dynamic_obstacles_range = random_response->n_dynamic_obstacles;

            static_obstacles_all_models = random_response->models_static_obstacles;
            static_obstacles_models_selected = std::vector<int>(static_obstacles_all_models.size(), 0);
            interactive_obstacles_all_models = random_response->models_interactive_obstacles;
            interactive_obstacles_models_selected = std::vector<int>(interactive_obstacles_all_models.size(), 0);
            dynamic_obstacles_all_models = random_response->models_dynamic_obstacles;
            dynamic_obstacles_models_selected = std::vector<int>(dynamic_obstacles_all_models.size(), 0);

            // Get configs for Scenario Obstacles Task Mode
            getScenarios(selected_world);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void TaskGeneratorPanel::getScenarios(const std::string &world_name)
    {
        auto request = std::make_shared<task_generator_msgs::srv::GetScenarios::Request>();
        request->world = world_name;

        while (!get_scenarios_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(service_node->get_logger(), "Interrupted while watiting for the service!. Exiting.");
                return;
            }
            RCLCPP_INFO(service_node->get_logger(), "Service is not available, waiting again...");
        }

        auto future = get_scenarios_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(service_node, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(service_node->get_logger(), "Got response from get_scenarios_client!");
        }
        else
        {
            RCLCPP_ERROR(service_node->get_logger(), "Failed to call service!");
        }

        auto response = future.get();
        scenario_config_files = response->scenarios;

        scenario_config_files_qstringlist = QStringList();
        for (const auto &scenario : scenario_config_files)
        {
            scenario_config_files_qstringlist << QString::fromStdString(scenario);
        }
    }

    void TaskGeneratorPanel::getTMRobotsParams()
    {
        try
        {
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void TaskGeneratorPanel::setupUi()
    {
        // Setup Combobox for choosing Robot model
        auto robots_combobox_values = QStringList();
        for (const auto &robot : robot_models)
        {
            robots_combobox_values << QString::fromStdString(robot);
        }
        robot_combobox = setupComboBoxWithLabel(this->root_layout, robots_combobox_values, QString("Robot"));
        robot_combobox->setCurrentText(QString::fromStdString(selected_robot_model));
        connect(robot_combobox, &QComboBox::currentTextChanged, this, &TaskGeneratorPanel::onRobotChanged);

        // Setup Combobox for choosing World
        auto worlds_combobox_values = QStringList();
        for (const auto &world : worlds)
        {
            worlds_combobox_values << QString::fromStdString(world);
        }
        world_combobox = setupComboBoxWithLabel(this->root_layout, worlds_combobox_values, QString("World"));
        world_combobox->setCurrentText(QString::fromStdString(selected_world));
        connect(world_combobox, &QComboBox::currentTextChanged, this, &TaskGeneratorPanel::onWorldChanged);

        setupTabs(this->root_layout);

        reset_scenario_button = new QPushButton("Reset Scenario");
        connect(reset_scenario_button, &QPushButton::clicked, this, &TaskGeneratorPanel::resetScenarioButtonActivated);
        root_layout->addWidget(reset_scenario_button);
    }

    QComboBox *TaskGeneratorPanel::setupComboBoxWithLabel(QLayout *parent, const QStringList &combobox_values, const QString &label)
    {
        auto placeholder_widget = new QWidget();
        auto placeholder_layout = new QHBoxLayout();
        auto label_label = new QLabel(label);
        auto combobox = new QComboBox();

        combobox->addItems(combobox_values);
        placeholder_layout->addWidget(label_label);
        placeholder_layout->addWidget(combobox);
        placeholder_widget->setLayout(placeholder_layout);

        parent->addWidget(placeholder_widget);

        return combobox;
    }

    QTabWidget *TaskGeneratorPanel::setupTabs(QLayout *parent)
    {
        auto tabs = new QTabWidget();
        auto obstacles_tab_widget = new QWidget();
        auto robot_tab_widget = new QWidget();

        tabs->addTab(obstacles_tab_widget, "Obstacles");
        tabs->addTab(robot_tab_widget, "Robots");

        auto obstacles_tab_layout = new QVBoxLayout();
        auto robot_tab_layout = new QVBoxLayout();

        obstacles_task_mode_combobox = setupComboBoxWithLabel(
            obstacles_tab_layout,
            QStringList({"Environment", "Parametrized", "Random", "Scenario"}),
            QString("Obstacles Task Mode"));
        obstacles_task_mode_combobox->setCurrentText(obstacles_task_mode);
        connect(
            obstacles_task_mode_combobox,
            &QComboBox::currentTextChanged,
            this,
            &TaskGeneratorPanel::onObstaclesTaskModeChanged);

        robot_task_mode_combobox = setupComboBoxWithLabel(
            robot_tab_layout,
            QStringList({"Explore", "Guided", "Random", "Scenario"}),
            QString("Robots Task Mode"));
        robot_task_mode_combobox->setCurrentText(robots_task_mode);
        connect(robot_task_mode_combobox,
                &QComboBox::currentTextChanged,
                this,
                &TaskGeneratorPanel::onRobotsTaskModeChanged);

        obstacles_tree = setupTree(obstacles_tab_layout);
        obstacles_tree->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
        obstacles_tree->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
        robots_tree = setupTree(robot_tab_layout);
        robots_tree->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
        robots_tree->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);

        obstacles_tab_widget->setLayout(obstacles_tab_layout);
        robot_tab_widget->setLayout(robot_tab_layout);

        setupObstaclesTreeItem();
        setupRobotsTreeItem();

        parent->addWidget(tabs);
        return tabs;
    }

    QTreeWidget *TaskGeneratorPanel::setupTree(QLayout *parent)
    {
        auto tree = new QTreeWidget();
        tree->setColumnCount(2);
        tree->setHeaderLabels({"Parameter", "Value"});
        tree->header()->resizeSection(0, int(0.5 * tree->width()));
        tree->header()->resizeSection(1, int(0.5 * tree->width()));
        tree->setColumnWidth(0, int(0.5 * tree->width()));
        tree->setColumnWidth(1, int(0.5 * tree->width()));

        parent->addWidget(tree);

        return tree;
    }

    void TaskGeneratorPanel::onRobotChanged(const QString &text)
    {
        selected_robot_model = text.toStdString();
    }

    void TaskGeneratorPanel::onWorldChanged(const QString &text)
    {
        selected_world = text.toStdString();

        getScenarios(selected_world);
        setupObstaclesTreeItem();
    }

    void TaskGeneratorPanel::setupObstaclesTreeItem()
    {
        // Clear the widget tree
        obstacles_tree->clear();
        if (obstacles_task_mode == "Environment")
        {
            auto param_config_file_combobox = new QComboBox();
            param_config_file_combobox->addItems(environment_config_files_qstringlist);
            param_config_file_combobox->setCurrentText(QString::fromStdString(selected_environment_config_file));
            auto item = new QTreeWidgetItem(obstacles_tree);
            item->setText(0, "Configuration File");
            obstacles_tree->setItemWidget(item, 1, param_config_file_combobox);
            connect(param_config_file_combobox, &QComboBox::currentTextChanged, this, [this](const QString &text)
                    { selected_environment_config_file = text.toStdString(); });
        }

        else if (obstacles_task_mode == "Parametrized")
        {
            auto param_config_file_combobox = new QComboBox();

            param_config_file_combobox->addItems(parametrized_config_files_qstringlist);
            param_config_file_combobox->setCurrentText(QString::fromStdString(selected_parametrized_config_file));
            auto item = new QTreeWidgetItem(obstacles_tree);
            item->setText(0, "Configuration File");
            obstacles_tree->setItemWidget(item, 1, param_config_file_combobox);
            connect(param_config_file_combobox, &QComboBox::currentTextChanged, this, [this](const QString &text)
                    { selected_parametrized_config_file = text.toStdString(); });
        }

        else if (obstacles_task_mode == "Random")
        {
            // Set up the spinbox for n_static_obstacles
            auto n_static_obstacles_widgetitem = new QTreeWidgetItem(obstacles_tree);
            n_static_obstacles_widgetitem->setText(0, "Number of Static Obstacles");

            auto n_static_obstacles_widget = new QWidget();
            auto n_static_obstacles_hbox = new QHBoxLayout();

            n_static_obstacles_hbox->addWidget(new QLabel("Min"));

            auto n_min_static_obstacles_spinbox = new QSpinBox();
            n_min_static_obstacles_spinbox->setRange(0, std::numeric_limits<int>::max());
            n_min_static_obstacles_spinbox->setValue(n_static_obstacles_range[0]); // Default value
            connect(n_min_static_obstacles_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value)
                    { n_static_obstacles_range[0] = value; });
            n_static_obstacles_hbox->addWidget(n_min_static_obstacles_spinbox);

            n_static_obstacles_hbox->addWidget(new QLabel("Max"));

            auto n_max_static_obstacles_spinbox = new QSpinBox();
            n_max_static_obstacles_spinbox->setRange(0, std::numeric_limits<int>::max());
            n_max_static_obstacles_spinbox->setValue(n_static_obstacles_range[1]); // Default value
            connect(n_max_static_obstacles_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value)
                    { n_static_obstacles_range[1] = value; });
            n_static_obstacles_hbox->addWidget(n_max_static_obstacles_spinbox);
            n_static_obstacles_hbox->addStretch();
            n_static_obstacles_hbox->setAlignment(Qt::AlignLeft);
            n_static_obstacles_hbox->setSpacing(4);

            n_static_obstacles_widget->setLayout(n_static_obstacles_hbox);
            obstacles_tree->setItemWidget(n_static_obstacles_widgetitem, 1, n_static_obstacles_widget);

            // Set up the spinbox for n_interactive_obstacles
            auto n_interactive_obstacles_widgetitem = new QTreeWidgetItem(obstacles_tree);
            n_interactive_obstacles_widgetitem->setText(0, "Number of Interactive Obstacles");

            auto n_interactive_obstacles_widget = new QWidget();
            auto n_interactive_obstacles_hbox = new QHBoxLayout();

            n_interactive_obstacles_hbox->addWidget(new QLabel("Min"));

            auto n_min_interactive_obstacles_spinbox = new QSpinBox();
            n_min_interactive_obstacles_spinbox->setRange(0, std::numeric_limits<int>::max());
            n_min_interactive_obstacles_spinbox->setValue(n_interactive_obstacles_range[0]); // Default value
            connect(n_min_interactive_obstacles_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value)
                    { n_interactive_obstacles_range[0] = value; });
            n_interactive_obstacles_hbox->addWidget(n_min_interactive_obstacles_spinbox);

            n_interactive_obstacles_hbox->addWidget(new QLabel("Max"));

            auto n_max_interactive_obstacles_spinbox = new QSpinBox();
            n_max_interactive_obstacles_spinbox->setRange(0, std::numeric_limits<int>::max());
            n_max_interactive_obstacles_spinbox->setValue(n_interactive_obstacles_range[1]); // Default value
            connect(n_max_interactive_obstacles_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value)
                    { n_interactive_obstacles_range[1] = value; });
            n_interactive_obstacles_hbox->addWidget(n_max_interactive_obstacles_spinbox);

            n_interactive_obstacles_hbox->addStretch();
            n_interactive_obstacles_hbox->setAlignment(Qt::AlignLeft);
            n_interactive_obstacles_hbox->setSpacing(4);
            n_interactive_obstacles_widget->setLayout(n_interactive_obstacles_hbox);
            obstacles_tree->setItemWidget(n_interactive_obstacles_widgetitem, 1, n_interactive_obstacles_widget);

            // Set up the spinbox for n_dynamic_obstacles
            auto n_dynamic_obstacles_widgetitem = new QTreeWidgetItem(obstacles_tree);
            n_dynamic_obstacles_widgetitem->setText(0, "Number of Dynamic Obstacles");

            auto n_dynamic_obstacles_widget = new QWidget();
            auto n_dynamic_obstacles_hbox = new QHBoxLayout();

            n_dynamic_obstacles_hbox->addWidget(new QLabel("Min"));

            auto n_min_dynamic_obstacles_spinbox = new QSpinBox();
            n_min_dynamic_obstacles_spinbox->setRange(0, std::numeric_limits<int>::max());
            n_min_dynamic_obstacles_spinbox->setValue(n_dynamic_obstacles_range[0]); // Default value
            connect(n_min_dynamic_obstacles_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value)
                    { n_dynamic_obstacles_range[0] = value; });
            n_dynamic_obstacles_hbox->addWidget(n_min_dynamic_obstacles_spinbox);

            n_dynamic_obstacles_hbox->addWidget(new QLabel("Max"));

            auto n_max_dynamic_obstacles_spinbox = new QSpinBox();
            n_max_dynamic_obstacles_spinbox->setRange(0, std::numeric_limits<int>::max());
            n_max_dynamic_obstacles_spinbox->setValue(n_dynamic_obstacles_range[1]); // Default value
            connect(n_max_dynamic_obstacles_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value)
                    { n_dynamic_obstacles_range[1] = value; });
            n_dynamic_obstacles_hbox->addWidget(n_max_dynamic_obstacles_spinbox);

            n_dynamic_obstacles_hbox->addStretch();
            n_dynamic_obstacles_hbox->setAlignment(Qt::AlignLeft);
            n_dynamic_obstacles_hbox->setSpacing(4);
            n_dynamic_obstacles_widget->setLayout(n_dynamic_obstacles_hbox);
            obstacles_tree->setItemWidget(n_dynamic_obstacles_widgetitem, 1, n_dynamic_obstacles_widget);

            // Set up check boxes to choose static obstacles models
            auto static_obstacles_widgetitem = new QTreeWidgetItem(obstacles_tree);
            static_obstacles_widgetitem->setText(0, "Static Obstacles Models");

            auto static_obstacles_models_groupbox = new QGroupBox(obstacles_tree);
            auto vbox = new QVBoxLayout();

            for (int i = 0; i < int(static_obstacles_all_models.size()); i++)
            {
                auto static_obstacle_model_checkbox = new QCheckBox(QString::fromStdString(static_obstacles_all_models[i]));
                static_obstacle_model_checkbox->setChecked(static_obstacles_models_selected[i]);
                connect(static_obstacle_model_checkbox, &QCheckBox::stateChanged, this, [this, i](int state)
                        { static_obstacles_models_selected[i] = (state == Qt::Checked); });
                vbox->addWidget(static_obstacle_model_checkbox);
            }

            static_obstacles_models_groupbox->setLayout(vbox);
            obstacles_tree->setItemWidget(static_obstacles_widgetitem, 1, static_obstacles_models_groupbox);

            // Set up check boxes to choose interactive obstacles models
            auto interactive_obstacles_widgetitem = new QTreeWidgetItem(obstacles_tree);
            interactive_obstacles_widgetitem->setText(0, "Interactive Obstacles Models");

            auto interactive_obstacles_models_groupbox = new QGroupBox(obstacles_tree);
            vbox = new QVBoxLayout();

            for (int i = 0; i < int(interactive_obstacles_all_models.size()); i++)
            {
                auto interactive_obstacle_model_checkbox = new QCheckBox(QString::fromStdString(interactive_obstacles_all_models[i]));
                interactive_obstacle_model_checkbox->setChecked(interactive_obstacles_models_selected[i]);
                connect(interactive_obstacle_model_checkbox, &QCheckBox::stateChanged, this, [this, i](int state)
                        { interactive_obstacles_models_selected[i] = (state == Qt::Checked); });
                vbox->addWidget(interactive_obstacle_model_checkbox);
            }
            interactive_obstacles_models_groupbox->setLayout(vbox);
            obstacles_tree->setItemWidget(interactive_obstacles_widgetitem, 1, interactive_obstacles_models_groupbox);

            // Set up check boxes to choose dynamic obstacles models
            auto dynamic_obstacles_widgetitem = new QTreeWidgetItem(obstacles_tree);
            dynamic_obstacles_widgetitem->setText(0, "Dynamic Obstacles Models");

            auto dynamic_obstacles_models_groupbox = new QGroupBox(obstacles_tree);
            vbox = new QVBoxLayout();

            for (int i = 0; i < int(dynamic_obstacles_all_models.size()); i++)
            {
                auto dynamic_obstacle_model_checkbox = new QCheckBox(QString::fromStdString(dynamic_obstacles_all_models[i]));
                dynamic_obstacle_model_checkbox->setChecked(dynamic_obstacles_models_selected[i]);
                connect(dynamic_obstacle_model_checkbox, &QCheckBox::stateChanged, this, [this, i](int state)
                        { dynamic_obstacles_models_selected[i] = (state == Qt::Checked); });
                vbox->addWidget(dynamic_obstacle_model_checkbox);
            }

            dynamic_obstacles_models_groupbox->setLayout(vbox);
            obstacles_tree->setItemWidget(dynamic_obstacles_widgetitem, 1, dynamic_obstacles_models_groupbox);
        }

        else if (obstacles_task_mode == "Scenario")
        {
            auto param_config_file_combobox = new QComboBox();
            param_config_file_combobox->addItems(scenario_config_files_qstringlist);
            auto item = new QTreeWidgetItem(obstacles_tree);
            item->setText(0, "Configuration File");
            obstacles_tree->setItemWidget(item, 1, param_config_file_combobox);
            connect(param_config_file_combobox, &QComboBox::currentTextChanged, this, [this](const QString &text)
                    { selected_scenario_config_file = text.toStdString(); });
        }
    }

    void TaskGeneratorPanel::setupRobotsTreeItem()
    {
    }

    void TaskGeneratorPanel::onObstaclesTaskModeChanged(const QString &text)
    {
        obstacles_task_mode = text;

        setupObstaclesTreeItem();
    }

    void TaskGeneratorPanel::onRobotsTaskModeChanged(const QString &text)
    {
        robots_task_mode = text;
    }

    void TaskGeneratorPanel::resetScenarioButtonActivated()
    {
        while (!set_param_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(service_node->get_logger(), "Interrupted while waiting for service. Exiting.");
                return;
            }
            RCLCPP_INFO(service_node->get_logger(), "Waiting for set_parameters service...");
        }

        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

        rcl_interfaces::msg::Parameter world_param;
        world_param.name = "world";
        world_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        world_param.value.string_value = selected_world;
        request->parameters.push_back(world_param);

        rcl_interfaces::msg::Parameter robot_model_param;
        robot_model_param.name = "robot";
        robot_model_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        robot_model_param.value.string_value = selected_robot_model;
        request->parameters.push_back(robot_model_param);

        // Obstacles Task Mode
        if (obstacles_task_mode == "Environment")
        {
            rcl_interfaces::msg::Parameter parameter;
            parameter.name = "tm_obstacles";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = "environment";
            request->parameters.push_back(parameter);

            parameter = rcl_interfaces::msg::Parameter();
            parameter.name = "task.environment.file";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = selected_environment_config_file;
            request->parameters.push_back(parameter);
        }
        else if (obstacles_task_mode == "Parametrized")
        {
            rcl_interfaces::msg::Parameter parameter;
            parameter.name = "tm_obstacles";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = "parametrized";
            request->parameters.push_back(parameter);

            parameter = rcl_interfaces::msg::Parameter();
            parameter.name = "task.parametrized.file";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = selected_parametrized_config_file;
            request->parameters.push_back(parameter);
        }
        else if (obstacles_task_mode == "Random")
        {
            rcl_interfaces::msg::Parameter parameter;
            parameter.name = "tm_obstacles";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = "random";

            parameter = rcl_interfaces::msg::Parameter();
            parameter.name = "task.random.static.models";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
            std::vector<std::string> selected_static_obstacles_models;
            for (int i = 0; i < int(static_obstacles_all_models.size()); i++)
            {
                if (static_obstacles_models_selected[i] == 1)
                {
                    selected_static_obstacles_models.push_back(static_obstacles_all_models[i]);
                }
            }
            parameter.value.string_array_value = selected_static_obstacles_models;
            request->parameters.push_back(parameter);

            parameter = rcl_interfaces::msg::Parameter();
            parameter.name = "task.random.interactive.models";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
            std::vector<std::string> selected_interactive_obstacles_models;
            for (int i = 0; i < int(interactive_obstacles_all_models.size()); i++)
            {
                if (interactive_obstacles_models_selected[i] == 1)
                {
                    selected_interactive_obstacles_models.push_back(interactive_obstacles_all_models[i]);
                }
            }
            parameter.value.string_array_value = selected_interactive_obstacles_models;
            request->parameters.push_back(parameter);

            parameter = rcl_interfaces::msg::Parameter();
            parameter.name = "task.random.dynamic.models";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
            std::vector<std::string> selected_dynamic_obstacles_models;
            for (int i = 0; i < int(dynamic_obstacles_all_models.size()); i++)
            {
                if (dynamic_obstacles_models_selected[i] == 1)
                {
                    selected_dynamic_obstacles_models.push_back(dynamic_obstacles_all_models[i]);
                }
            }
            parameter.value.string_array_value = selected_dynamic_obstacles_models;
            request->parameters.push_back(parameter);

            parameter = rcl_interfaces::msg::Parameter();
            parameter.name = "task.random.static.n";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
            parameter.value.integer_array_value = n_static_obstacles_range;
            request->parameters.push_back(parameter);

            parameter = rcl_interfaces::msg::Parameter();
            parameter.name = "task.random.interactive.n";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
            parameter.value.integer_array_value = n_interactive_obstacles_range;
            request->parameters.push_back(parameter);

            parameter = rcl_interfaces::msg::Parameter();
            parameter.name = "task.random.dynamic.n";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
            parameter.value.integer_array_value = n_dynamic_obstacles_range;
            request->parameters.push_back(parameter);
        }
        else if (obstacles_task_mode == "Scenario")
        {
            rcl_interfaces::msg::Parameter parameter;
            parameter.name = "tm_obstacles";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = "scenario";
            request->parameters.push_back(parameter);

            parameter = rcl_interfaces::msg::Parameter();
            parameter.name = "task.scenario.file";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = selected_scenario_config_file;
            request->parameters.push_back(parameter);
        }

        // Robot Task Mode
        if (robots_task_mode == "Explore")
        {
            rcl_interfaces::msg::Parameter parameter;
            parameter.name = "tm_robots";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = "explore";
            request->parameters.push_back(parameter);
        }
        else if (robots_task_mode == "Guided")
        {
            rcl_interfaces::msg::Parameter parameter;
            parameter.name = "tm_robots";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = "guided";
            request->parameters.push_back(parameter);
        }
        else if (robots_task_mode == "Random")
        {
            rcl_interfaces::msg::Parameter parameter;
            parameter.name = "tm_robots";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = "random";
            request->parameters.push_back(parameter);
        }
        else if (robots_task_mode == "Scenario")
        {
            rcl_interfaces::msg::Parameter parameter;
            parameter.name = "tm_robots";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = "scenario";
            request->parameters.push_back(parameter);
        }

        auto future = set_param_client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(service_node, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(service_node->get_logger(), "Successfully set parameter");
        }
        else
        {
            RCLCPP_ERROR(service_node->get_logger(), "Failed to set parameter");
        }

        auto reset_task_future = reset_task_client->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
        if (rclcpp::spin_until_future_complete(service_node, reset_task_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(service_node->get_logger(), "Successfully reset task scenario");
        }
        else
        {
            RCLCPP_ERROR(service_node->get_logger(), "Failed to reset task scenario");
        }
    }

} // namespace task_generator_gui

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(task_generator_gui::TaskGeneratorPanel, rviz_common::Panel)