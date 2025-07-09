#include "turret_aim_control_behaviors/aim_turret.hpp"

namespace turret_aim_control_behaviors {

AimTurret::AimTurret(const std::string name, const BT::NodeConfig &config)
: BT::StatefulActionNode(name, config), node_(rclcpp::Node::make_shared(name))
{}

BT::PortsList AimTurret::providedPorts()
{
    return {
        BT::InputPort<std::string>("namespace", "Namespace of called service. Leave empty to use namespace of this node."),
        BT::InputPort<int>("service_discovery_timeout", 1000, "Duration in ms waited for service to be available."),
        BT::InputPort<std::shared_ptr<geometry_msgs::msg::Vector3>>("direction_vector", "Vector to align with."),
        BT::InputPort<int>("motion_timeout", 2000, "Duration in ms waited for PTU motion to complete."),
        BT::InputPort<float>("pos_threshold", 0.1, "Max radian error for motion completion."),
    };
}

BT::NodeStatus AimTurret::onStart()
{
    std::string service_handle {"aim_turret"};
    auto ns {getInput<std::string>("namespace")};
    if (ns.has_value()) {
        service_client_ = node_->create_client<Trigger>("/" + ns.value() + "/" + service_handle);
    } else {
        service_client_ = node_->create_client<Trigger>(service_handle);
    }
    
    auto result {BT::NodeStatus::RUNNING};

    std::chrono::milliseconds timeout(getInput<int>("service_discovery_timeout").value());
    auto start_time {std::chrono::steady_clock::now()};
    while (!service_client_->wait_for_service(std::chrono::milliseconds(10))) {
        if (std::chrono::steady_clock::now() - start_time > timeout) {
            RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for Aim Turret service!");
            return BT::NodeStatus::FAILURE;
        }
    }

    auto req {std::make_shared<Trigger::Request>()};
    auto direction_vector {getInput<std::shared_ptr<geometry_msgs::msg::Vector3>>("direction_vector").value()};
    auto motion_timeout {getInput<int>("motion_timeout").value()};
    auto pos_threshold {getInput<float>("pos_threshold").value()};

    req->direction_vector = *direction_vector;
    req->motion_timeout = motion_timeout;
    req->pos_threshold = pos_threshold;
    
    request_future_ = service_client_->async_send_request(req);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Aim Turret " << result << "...");

    return result;
}

BT::NodeStatus AimTurret::onRunning()
{
    // TODO: need to ensure that the turret finishes the motion
    auto response {rclcpp::spin_until_future_complete(node_->get_node_base_interface(), request_future_.value(), std::chrono::milliseconds(5))};

    switch(response) {
        case rclcpp::FutureReturnCode::TIMEOUT: {
            RCLCPP_DEBUG(node_->get_logger(), "Timed waiting for Aim Turret response! Still running...");
            return BT::NodeStatus::RUNNING;
        }
        case rclcpp::FutureReturnCode::INTERRUPTED: {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted waiting for Aim Turret response. Aborting...");
            return BT::NodeStatus::FAILURE;
        }
        case rclcpp::FutureReturnCode::SUCCESS: {
            auto return_status {BT::NodeStatus::FAILURE}; 

            auto resp {request_future_->get()};
            if (resp->success) {
                return_status = BT::NodeStatus::SUCCESS;
            }

            request_future_ = std::nullopt;
            RCLCPP_INFO_STREAM(node_->get_logger(), "Aim Turret " << return_status << "!");
            return return_status;
        }
    }

    return BT::NodeStatus::FAILURE;
}

void AimTurret::onHalted()
{
    if (request_future_.has_value()) {
        service_client_->remove_pending_request(request_future_.value());
    }
}

} // namespace turret_aim_control_behaviors