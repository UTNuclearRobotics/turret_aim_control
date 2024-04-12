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

class TurretController : public rclcpp::Node
{
public:
    TurretController();

private:
    int32_t timer_hz;

    bool turret_simulate_joint_states;

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
    std::string turret_joint_states_topic;
    std::string payload_joint_states_topic;

    // Transform topic listener
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

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

    rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr joint_group_command_publisher;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr turret_joint_states_publisher;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr payload_joint_states_publisher;
    rclcpp::TimerBase::SharedPtr joint_goal_timer;

    /// @brief Declare all parameters needed by the node
    void robot_init_parameters();

    /// @brief Initialize ROS Publishers
    void robot_init_publishers();

    /// @brief Initialize ROS Timers
    void robot_init_timers();

    /// @brief Set custom PID values for the Dynamixel motors
    /// Default PID values: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#control-table-of-ram-area
    void set_custom_dynamixel_motor_pid_gains();

    // Joint publishers
    void publishTurretJointGoal();
    void publishSimJointStates();
    void setTurretJointGoal();

    // End-effector PID methods
    Eigen::Vector3f getPIDVelocity();
    void updateBuffer(Eigen::Matrix3Xf& buffer, Eigen::Vector3f error);
    Eigen::Vector3f calculateErrorDt(Eigen::Matrix3Xf& buffer, int hz);
    Eigen::Vector3f calculateErrorIntegral(Eigen::Matrix3Xf& buffer, int hz);

    /// @brief Helper function to print vector values
    void printVector(std::string var, Eigen::Vector3f vector);
};

#endif