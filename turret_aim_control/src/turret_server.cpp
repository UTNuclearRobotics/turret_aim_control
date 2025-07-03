#include "turret_aim_control/turret_server.hpp"

namespace turret_aim_control {

TurretServer::TurretServer(const rclcpp::NodeOptions &opts)
: Node("turret_controller", opts)
{
    joint_cmd_pub_ = this->create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>(
        "/pxxls/commands/joint_group", 1);

    aim_turret_service_ = this->create_service<turret_aim_control_interfaces::srv::AimTurret>(
        "aim_turret", std::bind(&TurretServer::aimTurret, this, std::placeholders::_1, std::placeholders::_2));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/pxxls/joint_states", 1,
        std::bind(&TurretServer::jointStateCallback, this, std::placeholders::_1));

    if (!initLimits()) {
        RCLCPP_ERROR(get_logger(), "initLimits failed — shutting down node");

        rclcpp::shutdown();

        // exit_timer_ = create_wall_timer(
        //     std::chrono::milliseconds(100),
        //     [this]() {
        //     RCLCPP_INFO(get_logger(), "Node shutting down");
        //     exit(0);
        //     });

        return;
    }
}

void TurretServer::aimTurret(const std::shared_ptr<turret_aim_control_interfaces::srv::AimTurret::Request> request, std::shared_ptr<turret_aim_control_interfaces::srv::AimTurret::Response> response)
{
    float current_pan, current_tilt;
    {
        std::unique_lock<std::mutex> lock(joint_state_mutex_);
        if (!joint_state_received_) {
            RCLCPP_WARN(this->get_logger(), "No joint state received yet. Cannot aim turret.");
            response->success = false;
            return;
        }
        current_pan = actual_pan_;
        current_tilt = actual_tilt_;
    }

    const auto &dir_vector = request->direction_vector;
    float pan_offset = static_cast<float>(std::atan2(dir_vector.y, dir_vector.x));
    float tilt_offset = static_cast<float>(std::atan2(dir_vector.z, std::sqrt(dir_vector.x * dir_vector.x + dir_vector.y * dir_vector.y)));

    float pan_target = current_pan + pan_offset;
    float tilt_target = current_tilt + tilt_offset;

    pan_target = std::clamp(pan_target, pan_limits_[0], pan_limits_[1]);
    tilt_target = std::clamp(tilt_target, tilt_limits_[0], tilt_limits_[1]);

    RCLCPP_INFO(this->get_logger(), 
                "Aiming - Current: pan=%.3f, tilt=%.3f | Offset: pan=%.3f, tilt=%.3f | Target: pan=%.3f, tilt=%.3f",
                current_pan, current_tilt, pan_offset, tilt_offset, pan_target, tilt_target);

    interbotix_xs_msgs::msg::JointGroupCommand cmd;
    cmd.name = "turret";
    cmd.cmd = {pan_target, tilt_target};
    joint_cmd_pub_->publish(cmd);

    const double POSITION_THRESHOLD = 0.02; 
    const int TIMEOUT_MS = 2000;
    auto start = std::chrono::steady_clock::now();

    std::unique_lock<std::mutex> lock(joint_state_mutex_);
    while (rclcpp::ok()) {
        if (!joint_state_received_) {
            joint_state_cv_.wait_for(lock, std::chrono::milliseconds(10));
            continue;
        }
        double pan_error = std::abs(actual_pan_ - pan_target);
        double tilt_error = std::abs(actual_tilt_ - tilt_target);
        if (pan_error < POSITION_THRESHOLD && tilt_error < POSITION_THRESHOLD) {
            response->success = true;
            return;
        }
        if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(TIMEOUT_MS)) {
            RCLCPP_WARN(this->get_logger(), "Timeout waiting for turret to reach goal.");
            RCLCPP_WARN(this->get_logger(), "Current pan: %.3f, target pan: %.3f. Current tilt: %.3f, target tilt: %.3f", 
                        actual_pan_, pan_target, actual_tilt_, tilt_target);
            response->success = false;
            return;
        }
        joint_state_cv_.wait_for(lock, std::chrono::milliseconds(10));
    }
    response->success = false;
}

void TurretServer::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "pan") actual_pan_ = msg->position[i];
        if (msg->name[i] == "tilt") actual_tilt_ = msg->position[i];
    }
    joint_state_received_ = true;
    joint_state_cv_.notify_all();
}

bool TurretServer::initLimits() 
{
    auto client = this->create_client<interbotix_xs_msgs::srv::RobotInfo>("/pxxls/get_robot_info");
    
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_WARN(this->get_logger(), "Interrupted while waiting for /pxxls/get_robot_info. Exiting.");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for robot info service…");
    }

    auto request = std::make_shared<interbotix_xs_msgs::srv::RobotInfo::Request>();
    request->cmd_type = "group";
    request->name = "turret";

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto info = result.get();
        pan_limits_ = {info->joint_lower_limits[0], info->joint_upper_limits[0]};
        tilt_limits_ = {info->joint_lower_limits[1], info->joint_upper_limits[1]};
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to get turret joint limits!");
        return false;
    }

    return true;
}

} // namespace turret_aim_control

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node {std::make_shared<turret_aim_control::TurretServer>(rclcpp::NodeOptions())};
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}