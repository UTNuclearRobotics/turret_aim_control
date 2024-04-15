#ifndef TURRET_CONTROLLER_HPP
#define TURRET_CONTROLLER_HPP

#include <chrono>
#include <eigen3/Eigen/Dense>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "interbotix_xs_msgs/msg/joint_group_command.hpp"
#include "interbotix_xs_msgs/srv/motor_gains.hpp"

#include "turret_aim_control_interfaces/srv/aim_enable.hpp"

// Simplify message and service usage
using AimEnable = turret_aim_control_interfaces::srv::AimEnable;
using JointGroupCommand = interbotix_xs_msgs::msg::JointGroupCommand;
using JointState = sensor_msgs::msg::JointState;

class TurretController : public rclcpp::Node
{
public:
    TurretController();

private:
    // Frequency at which the ROS Timer updating the turret joint goal should run
    int timer_hz;

    // Boolean that determines if turret joint states should be published
    bool pub_turret_joint_states;

    // Dynamixel PID
    int32_t kp_pos;
    int32_t ki_pos;
    int32_t kd_pos;
    int32_t k1;
    int32_t k2;
    int32_t kp_vel;
    int32_t ki_vel;

    // End-effector PID
    float kp;
    float ki;
    float kd;
    int32_t buffer_n;
    Eigen::Matrix3Xf buffer;

    // Names
    std::string turret_name;
    std::string payload_name;

    // Links and joints
    std::string base_link;
    std::string turret_pan_link;
    std::string turret_tilt_link;
    std::string payload_aim_link;
    std::string payload_aim_joint;
    std::string target_link;

    // Topics
    std::string topic_joint_states_turret;
    std::string topic_joint_states_payload;

    // Transform topic listener
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    // Calculated pose to be published to the turret
    Eigen::Vector3f dq;
    Eigen::Vector3f q;

    // Transforms
    Eigen::Vector3f Z1;
    Eigen::Vector3f Y2;
    Eigen::Vector3f X3;
    Eigen::Vector3f t1_transform;
    Eigen::Vector3f t2_transform;
    Eigen::Vector3f t3_transform;
    Eigen::Vector3f td_transform;

    /// @brief Declare ROS parameters, publishers, timers, and services
    rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr pub_joint_group_command;
    rclcpp::Publisher<JointState>::SharedPtr pub_joint_states_turret;
    rclcpp::Publisher<JointState>::SharedPtr pub_joint_states_payload;
    rclcpp::TimerBase::SharedPtr tmr_joint_goal;
    rclcpp::Service<AimEnable>::SharedPtr srv_aim_enable;

    /// @brief Initialize ROS parameters, publishers, timers, and services
    void robot_init_parameters();
    void robot_init_publishers();
    void robot_init_timers();
    void robot_init_services();

    /// @brief Set custom PID values for the Dynamixel motors
    /// @details This service is created by Interbotix. Default PID values: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#control-table-of-ram-area
    void set_custom_dynamixel_motor_pid_gains();

    /// @brief ROS Service to turret aiming on/off
    /// @param request AimEnable service message request
    /// @param response AimEnable service message response
    /// @details refer to the service definition for details
    void robot_srv_aim_enable(
        const std::shared_ptr<AimEnable::Request> request,
        const std::shared_ptr<AimEnable::Response> response);

    /// @brief This is the robot_srv_aim_enable callback function
    /// @details This is the main turret controller method to calculate turret orientation and manages publishing turret commands and joint states. Only if the target frame is in the /tf tree, the turret will move to align with the target
    void update_turret_joint_goal();

    /// @brief Publishes commands to control the turret hardware
    /// @details Defined by Interbotix libraries
    void publish_turret_joint_goal();

    /// @brief Publishes the joint states for all simulated joints
    /// @details Always publishes the virtual end-effector joints to align with target. Only publishes the turret joint states if pub_turret_joint_states is true;
    void publish_sim_joint_states();

    // End-effector PID methods
    void update_buffer(Eigen::Matrix3Xf &buffer, Eigen::Vector3f error);
    Eigen::Vector3f get_pid();
    Eigen::Vector3f get_error_dt(Eigen::Matrix3Xf &buffer, int hz);
    Eigen::Vector3f get_error_integral(Eigen::Matrix3Xf &buffer, int hz);

    /// @brief Helper function to print vector values
    void print_vector3f(std::string var, Eigen::Vector3f vector)
    {
        RCLCPP_INFO(get_logger(), "%s: <%f, %f, %f>", var.c_str(), vector.coeff(0), vector.coeff(1), vector.coeff(2));
    }
};

#endif