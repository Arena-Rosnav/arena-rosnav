#include "task_generator_gui/task_generator_panel.hpp"
#include "rviz_common/display_context.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rcl_interfaces/srv/set_parameters.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
namespace task_generator_gui
{
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

    void TaskGeneratorPanel::setTMObstaclesParamsRequest(rcl_interfaces::srv::SetParameters::Request::SharedPtr request)
    {
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
            std::vector<std::string> selected_static_obstacles_models = convert(static_obstacles_models_groupbox->currentText());
            parameter.value.string_array_value = selected_static_obstacles_models;
            request->parameters.push_back(parameter);

            parameter = rcl_interfaces::msg::Parameter();
            parameter.name = "task.random.interactive.models";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
            std::vector<std::string> selected_interactive_obstacles_models = convert(interactive_obstacles_models_groupbox->currentText());
            parameter.value.string_array_value = selected_interactive_obstacles_models;
            request->parameters.push_back(parameter);

            parameter = rcl_interfaces::msg::Parameter();
            parameter.name = "task.random.dynamic.models";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
            std::vector<std::string> selected_dynamic_obstacles_models = convert(dynamic_obstacles_models_groupbox->currentText());
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
    }

    void TaskGeneratorPanel::setTMRobotsParamsRequest(rcl_interfaces::srv::SetParameters::Request::SharedPtr request)
    {
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
    }

    void TaskGeneratorPanel::setParams()
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

        setTMObstaclesParamsRequest(request);
        setTMRobotsParamsRequest(request);

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

    // Convert QStringList to std::vector<std::string>
    std::vector<std::string> convert(const QStringList &qList)
    {
        std::vector<std::string> result;
        result.reserve(qList.size()); // optional, for efficiency
        for (const QString &item : qList)
        {
            result.push_back(item.toStdString());
        }
        return result;
    }
} // namespace task_generator_gui