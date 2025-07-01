#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <interbotix_xs_msgs/srv/robot_info.hpp>
#include <interbotix_xs_msg/joint_group_command.hpp>

#include "tracking_interfaces/srv/get_detection_point.hpp"

namespace turret_aim_control {

class TurretServer : public rclcpp::Node 
{
public:
    TurretServer(const rclcpp::NodeOptions &opts);

    void aimTurret(const std::shared_ptr<tracking_interfaces::srv::AimTurret::Request> request, std::shared_ptr<tracking_interfaces::srv::AimTurret::Response> response);

private:
    void initLimits();

    std::shared_ptr<rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>> joint_cmd_pub_;
    std::shared_ptr<rclcpp::Service<tracking_interfaces::srv::AimTurret>> aim_turret_service_;

    std::array<double, 2> pan_limits_;
    std::array<double, 2> tilt_limits_;
}; // class TurretServer

} // namespace turret_aim_control