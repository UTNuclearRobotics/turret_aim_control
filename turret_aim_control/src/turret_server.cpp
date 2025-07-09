#include "turret_aim_control/turret_server.hpp"

namespace turret_aim_control {

TurretServer::TurretServer(const rclcpp::NodeOptions &opts)
: Node("turret_controller", opts)
{
    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    aim_turret_service_ = this->create_service<turret_aim_control_interfaces::srv::AimTurret>(
        "aim_turret", std::bind(&TurretServer::aimTurret, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_cb_group_);

    
    info_client_ = this->create_client<interbotix_xs_msgs::srv::RobotInfo>("/pxxls/get_robot_info");

    joint_cmd_pub_ = this->create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>(
        "/pxxls/commands/joint_group", 10);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/pxxls/joint_states", 10,
        std::bind(&TurretServer::jointStateCallback, this, std::placeholders::_1));

    pan_target_ = 0;
    tilt_target_ = 0;
    
    getRobotInfo();

    command_publisher_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), 
        std::bind(&TurretServer::publishJointGroupCommand, this),
        timer_cb_group_);

    RCLCPP_INFO(get_logger(), "TurretServer node successfully initialized. Waiting for joint limits...");

    // interbotix_xs_msgs::msg::JointGroupCommand cmd;
    // cmd.name = "turret";
    // cmd.cmd = {pan_target, tilt_target};

    // // raise(SIGTRAP);
    // joint_cmd_pub_->publish(cmd);
}

void TurretServer::aimTurret(const std::shared_ptr<turret_aim_control_interfaces::srv::AimTurret::Request> request, std::shared_ptr<turret_aim_control_interfaces::srv::AimTurret::Response> response)
{
    if (!limits_initialized_) {
        RCLCPP_WARN(this->get_logger(), "Joint limits not yet initialized. Cannot aim turret.");
        response->success = false;
        return;
    }

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

    pan_target = wrapToRange(pan_target, pan_limits_[0], pan_limits_[1]);
    tilt_target = wrapToRange(tilt_target, tilt_limits_[0], tilt_limits_[1]);

    RCLCPP_INFO(this->get_logger(), 
                "Aiming - Current: pan=%.3f, tilt=%.3f | Offset: pan=%.3f, tilt=%.3f | Target: pan=%.3f, tilt=%.3f",
                current_pan, current_tilt, pan_offset, tilt_offset, pan_target, tilt_target);

    { // Scoped lock for pan/tilt_target
        std::lock_guard<std::mutex> lock(target_mutex_);
        pan_target_ = pan_target;
        tilt_target_ = tilt_target;
    }    

    // interbotix_xs_msgs::msg::JointGroupCommand cmd;
    // cmd.name = "turret";
    // cmd.cmd = {pan_target, tilt_target};

    // // raise(SIGTRAP);
    // joint_cmd_pub_->publish(cmd);

    const double POSITION_THRESHOLD = 0.1; 
    const int TIMEOUT_MS = 10000;
    auto start = std::chrono::steady_clock::now();

    std::unique_lock<std::mutex> lock(joint_state_mutex_);
    while (rclcpp::ok()) {
        if (!joint_state_received_) {
            joint_state_cv_.wait_for(lock, std::chrono::milliseconds(10));
            continue;
        }

        double pan_error = std::abs(actual_pan_ - pan_target_);
        double tilt_error = std::abs(actual_tilt_ - tilt_target_);

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

void TurretServer::publishJointGroupCommand()
{
    if (!limits_initialized_) {
        getRobotInfo();
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, // Log at most once every 5 seconds
                    "Joint limits not yet initialized. Skipping joint command publication.");
        return;
    }

    float pan_to_publish, tilt_to_publish;
    { // Scoped lock for pan/tilt_target
        std::lock_guard<std::mutex> lock(target_mutex_);
        pan_to_publish = pan_target_;
        tilt_to_publish = tilt_target_;
    }

    interbotix_xs_msgs::msg::JointGroupCommand cmd;
    cmd.name = "turret";
    cmd.cmd = {pan_to_publish, tilt_to_publish};
    
    joint_cmd_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Periodically publishing command: pan=%.3f, tilt=%.3f",
                pan_to_publish, tilt_to_publish);
}

float TurretServer::wrapToRange(float val, float min, float max) 
{
    float range = max - min;

    while (val < min) val += range;
    while (val >= max) val -= range;

    return val;
}

void TurretServer::getRobotInfo() 
{    
    if (!info_client_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "Robot info service /pxxls/get_robot_info not ready. Will retry automatically.");
        return;
    }

    auto request = std::make_shared<interbotix_xs_msgs::srv::RobotInfo::Request>();
    request->cmd_type = "group";
    request->name = "turret";

    info_client_->async_send_request(request, std::bind(&TurretServer::initLimits, this, std::placeholders::_1));

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Sent request to get turret joint limits asynchronously.");
}

void TurretServer::initLimits(rclcpp::Client<interbotix_xs_msgs::srv::RobotInfo>::SharedFuture future)
{
    auto info = future.get();
    if (info) {
        pan_limits_ = {info->joint_lower_limits[0], info->joint_upper_limits[0]};
        tilt_limits_ = {info->joint_lower_limits[1], info->joint_upper_limits[1]};
        limits_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Pan limits: [%.3f, %.3f]", pan_limits_[0], pan_limits_[1]);
        RCLCPP_INFO(this->get_logger(), "Tilt limits: [%.3f, %.3f]", tilt_limits_[0], tilt_limits_[1]);
        RCLCPP_INFO(this->get_logger(), "Joint limits successfully initialized.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to get turret joint limits from service response!");
    }
}

} // namespace turret_aim_control

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node {std::make_shared<turret_aim_control::TurretServer>(rclcpp::NodeOptions())};

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2); // 2 threads

    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}