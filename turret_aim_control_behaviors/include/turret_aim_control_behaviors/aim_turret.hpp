#pragma once

#include <future>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>

#include <geometry_msgs/msg/vector3.hpp>

#include "turret_aim_control_interfaces/srv/aim_turret.hpp"

namespace turret_aim_control_behaviors {

class AimTurret : public BT::StatefulActionNode
{
public:
    using Trigger = turret_aim_control_interfaces::srv::AimTurret;

    AimTurret(const std::string name, const BT::NodeConfig &config);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::Client<Trigger>> service_client_;
    std::optional<rclcpp::Client<Trigger>::FutureAndRequestId> request_future_;
}; // class AimTurret

} // namespace turret_aim_control_behaviors