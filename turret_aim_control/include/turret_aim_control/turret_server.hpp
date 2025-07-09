#pragma once

#include <memory>
#include <string>
#include <csignal>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <interbotix_xs_msgs/srv/robot_info.hpp>
#include <interbotix_xs_msgs/msg/joint_group_command.hpp>

#include "turret_aim_control_interfaces/srv/aim_turret.hpp"

namespace turret_aim_control {

class TurretServer : public rclcpp::Node 
{
public:
    TurretServer(const rclcpp::NodeOptions &opts);

    void aimTurret(const std::shared_ptr<turret_aim_control_interfaces::srv::AimTurret::Request> request, std::shared_ptr<turret_aim_control_interfaces::srv::AimTurret::Response> response);

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void publishJointGroupCommand();
    float wrapToRange(float val, float min, float max);
    void getRobotInfo();
    void initLimits(rclcpp::Client<interbotix_xs_msgs::srv::RobotInfo>::SharedFuture future);


    std::shared_ptr<rclcpp::Service<turret_aim_control_interfaces::srv::AimTurret>> aim_turret_service_;
    std::shared_ptr<rclcpp::Client<interbotix_xs_msgs::srv::RobotInfo>> info_client_;

    std::shared_ptr<rclcpp::CallbackGroup> service_cb_group_;
    std::shared_ptr<rclcpp::CallbackGroup> timer_cb_group_;

    std::shared_ptr<rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>> joint_cmd_pub_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>> joint_state_sub_;

    std::shared_ptr<rclcpp::TimerBase> command_publisher_timer_; 

    std::array<float, 2> pan_limits_; // -3.140 to 3.140
    std::array<float, 2> tilt_limits_; // -1.571 to 1.571
    bool limits_initialized_ {false};

    std::mutex joint_state_mutex_;
    std::condition_variable joint_state_cv_;

    float actual_pan_;
    float actual_tilt_;
    bool joint_state_received_ {false};

    std::mutex target_mutex_;
    float pan_target_;  
    float tilt_target_;
}; // class TurretServer

} // namespace turret_aim_control