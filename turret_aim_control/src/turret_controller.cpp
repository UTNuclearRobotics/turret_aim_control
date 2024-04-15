#include "turret_controller.hpp"

TurretController::TurretController() : Node("turret_controller")
{
    robot_init_parameters();
    robot_init_publishers();
    robot_init_timers();
    robot_init_services();

    if (!pub_turret_joint_states)
    {
        set_custom_dynamixel_motor_pid_gains();
    }
}

void TurretController::robot_init_parameters()
{
    timer_hz = this->declare_parameter<int32_t>("timer_hz", 10);

    pub_turret_joint_states = this->declare_parameter<bool>("pub_turret_joint_states", true);

    // Turret velocity PID gain constants, currently set to Dynamixel default values
    kp_pos = this->declare_parameter<int32_t>("kp_pos", 800);
    ki_pos = this->declare_parameter<int32_t>("ki_pos", 0);
    kd_pos = this->declare_parameter<int32_t>("kd_pos", 0);
    k1 = this->declare_parameter<int32_t>("k1", 0);
    k2 = this->declare_parameter<int32_t>("k2", 0);
    kp_vel = this->declare_parameter<int32_t>("kp_vel", 100);
    ki_vel = this->declare_parameter<int32_t>("ki_vel", 1920);

    // End-effector velocity PID gain constants
    kp = this->declare_parameter<float>("kp", 5.0);
    ki = this->declare_parameter<float>("ki", 1.0);
    kd = this->declare_parameter<float>("kd", 0.0);

    // Initialize the end-effector PID buffer matrix size and fill with zeroes
    buffer_n = this->declare_parameter<int32_t>("buffer_n", 10);
    buffer.resize(3, buffer_n);
    buffer.setZero();

    // Names
    turret_name = this->declare_parameter<std::string>("turret_name", "");
    payload_name = this->declare_parameter<std::string>("payload_name", "");

    // Links and joints
    base_link = this->declare_parameter<std::string>("base_link", "");
    turret_pan_link = this->declare_parameter<std::string>("turret_pan_link", "");
    turret_tilt_link = this->declare_parameter<std::string>("turret_tilt_link", "");
    payload_aim_link = this->declare_parameter<std::string>("payload_aim_link", "");
    payload_aim_joint = this->declare_parameter<std::string>("payload_aim_joint", "");
    target_link = this->declare_parameter<std::string>("target_link", "");

    // Topics
    topic_joint_states_turret = this->declare_parameter<std::string>("topic_joint_states_turret", turret_name + "/joint_states");
    topic_joint_states_payload = this->declare_parameter<std::string>("topic_joint_states_payload", payload_name + "/joint_states");

    // Transform topic listener
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    tf_buffer->setUsingDedicatedThread(true);

    // Initial turret joint positions
    q << 0, 0, 0.45;
    dq << 0, 0, 0;
}

void TurretController::robot_init_publishers()
{
    // If disconnected from turret hardware, simulating updates turret joint states
    if (pub_turret_joint_states)
    {
        pub_joint_states_turret = this->create_publisher<JointState>(
            topic_joint_states_turret,
            rclcpp::QoS(1));
    }

    // Update virtual end-effector joint states
    pub_joint_states_payload = this->create_publisher<JointState>(
        topic_joint_states_payload,
        rclcpp::QoS(1));

    // Read by Interbotix SDK to control hardware
    pub_joint_group_command = this->create_publisher<JointGroupCommand>(
        turret_name + "/commands/joint_group",
        10);
}

void TurretController::robot_init_timers()
{
    // Timer that calculates the turret joint goal to aim at the target
    tmr_joint_goal = this->create_wall_timer(
        std::chrono::nanoseconds(static_cast<int>(1.0e9 / (timer_hz))),
        std::bind(
            &TurretController::update_turret_joint_goal,
            this));

    // Instantly stop timer and wait for service to enable
    tmr_joint_goal->cancel();
}

void TurretController::robot_init_services()
{
    using namespace std::placeholders;
    srv_aim_enable = this->create_service<AimEnable>("aim_enable", std::bind(&TurretController::robot_srv_aim_enable, this, _1, _2));
}

void TurretController::set_custom_dynamixel_motor_pid_gains()
{
    // Create a client for the service
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("set_motor_pid_gains_client");
    auto client = node->create_client<interbotix_xs_msgs::srv::MotorGains>(turret_name + "/set_motor_pid_gains");

    // Prepare the request
    auto request = std::make_shared<interbotix_xs_msgs::srv::MotorGains::Request>();
    request->cmd_type = std::string("group");
    request->name = turret_name;
    request->kp_pos = kp_pos;
    request->ki_pos = ki_pos;
    request->kd_pos = kd_pos;
    request->k1 = k1;
    request->k2 = k2;
    request->kp_vel = kp_vel;
    request->ki_vel = ki_vel;

    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(get_logger(), "Waiting for service '%s'...", client->get_service_name());
    }

    // Send the request
    client->async_send_request(request);
}

void TurretController::robot_srv_aim_enable(
    const std::shared_ptr<AimEnable::Request> request,
    const std::shared_ptr<AimEnable::Response> response)
{
    if (request->aim_enable)
    {
        // Start aim controller
        try
        {
            target_link = request->target_frame_id;
            tmr_joint_goal->reset();
            response->success = true;
            response->message = "Successfully started turret aim controller!";
        }
        catch (const std::exception &e)
        {
            response->success = false;
            response->message = "Failed to start turret aim controller:" + std::string(e.what());
        }
    }
    else
    {
        // Stop aim controller
        try
        {
            tmr_joint_goal->cancel();
            response->success = true;
            response->message = "Successfully stopped turret aim controller!";
        }
        catch (const std::exception &e)
        {
            response->success = false;
            response->message = "Failed to stop turret aim controller:" + std::string(e.what());
        }
    }
}

void TurretController::update_turret_joint_goal()
{
    // Fetch transforms
    geometry_msgs::msg::TransformStamped t1;
    geometry_msgs::msg::TransformStamped t2;
    geometry_msgs::msg::TransformStamped t3;
    geometry_msgs::msg::TransformStamped td;
    try
    {
        t1 = tf_buffer->lookupTransform(
            base_link,
            turret_pan_link,
            tf2::TimePointZero);
        t2 = tf_buffer->lookupTransform(
            base_link,
            turret_tilt_link,
            tf2::TimePointZero);
        t3 = tf_buffer->lookupTransform(
            base_link,
            payload_aim_link,
            tf2::TimePointZero);
        td = tf_buffer->lookupTransform(
            base_link,
            target_link,
            tf2::TimePointZero);

        // Rotation
        Eigen::Quaternionf Q1(t1.transform.rotation.w, t1.transform.rotation.x, t1.transform.rotation.y, t1.transform.rotation.z);
        Eigen::Quaternionf Q2(t2.transform.rotation.w, t2.transform.rotation.x, t2.transform.rotation.y, t2.transform.rotation.z);
        Eigen::Quaternionf Q3(t3.transform.rotation.w, t3.transform.rotation.x, t3.transform.rotation.y, t3.transform.rotation.z);

        // [3X1] vector
        Z1 = Q1.toRotationMatrix().block<3, 1>(0, 2);
        Y2 = Q2.toRotationMatrix().block<3, 1>(0, 1);
        X3 = Q3.toRotationMatrix().block<3, 1>(0, 0);
        // RCLCPP_INFO_STREAM(get_logger(), "X3: \n" << X3);

        // Translation
        t1_transform << t1.transform.translation.x, t1.transform.translation.y, t1.transform.translation.z;
        t2_transform << t2.transform.translation.x, t2.transform.translation.y, t2.transform.translation.z;
        t3_transform << t3.transform.translation.x, t3.transform.translation.y, t3.transform.translation.z;
        td_transform << td.transform.translation.x, td.transform.translation.y, td.transform.translation.z;

        // Inverse Kinematics variables
        Eigen::MatrixXf Jacobian(6, 3);
        Eigen::MatrixXf JacobianInv;
        Eigen::VectorXf dx(6, 1);

        // [6X3] matrix
        Jacobian << Z1, Y2, Eigen::Vector3f::Zero(3),
            Z1.cross(t3_transform - t1_transform), Y2.cross(t3_transform - t2_transform), X3;
        // RCLCPP_INFO_STREAM(get_logger(), "jacobian: \n" << Jacobian);

        JacobianInv = Jacobian.completeOrthogonalDecomposition().pseudoInverse();
        // dx << Eigen::Vector3f::Zero(3), kp * (td_transform - t3_transform);
        dx << Eigen::Vector3f::Zero(3), get_pid();
        dq = JacobianInv * dx;

        // Check if dq(2) would make q(2) go negative, don't let it
        if (q.coeff(2) + dq.coeff(2) < 0.05)
        {
            // RCLCPP_INFO(get_logger(), "Set dq to zero? %f + %f = %f", q.coeff(2), dq.coeff(2), q.coeff(2)+dq.coeff(2));
            dq(2) = 0.05 - q.coeff(2);
        }

        q += dq / timer_hz;

        publish_turret_joint_goal();
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_INFO(get_logger(), "Couldn't find transforms: %s", ex.what());
    }

    publish_sim_joint_states();
}

Eigen::Vector3f TurretController::get_pid()
{
    Eigen::Vector3f error = td_transform - t3_transform;
    // print_vector3f("error", error);

    update_buffer(buffer, error);

    Eigen::Vector3f error_dt = get_error_dt(buffer, timer_hz);
    // print_vector3f("error_dt", error_dt);

    Eigen::Vector3f error_integral = get_error_integral(buffer, timer_hz);

    return (kp * error) + (kd * error_dt) - (ki * error_integral);
}

void TurretController::update_buffer(Eigen::Matrix3Xf &buffer, Eigen::Vector3f value)
{
    // Left shift the buffer discarding the oldest value
    buffer.leftCols(buffer.cols() - 1) = buffer.rightCols(buffer.cols() - 1);

    // Assign new value to last column of the buffer
    buffer.col(buffer.cols() - 1) = value;
}

Eigen::Vector3f TurretController::get_error_dt(Eigen::Matrix3Xf &buffer, int hz)
{
    // Calculate time step between consecutive buffer elements
    float dt = 1.0 / hz;

    // Calculate the average rate of change over the buffer
    Eigen::Vector3f error_dt = (buffer.col(buffer.cols() - 1) - buffer.col(0)) / (dt * (buffer.cols() - 1));

    return error_dt;
}

Eigen::Vector3f TurretController::get_error_integral(Eigen::Matrix3Xf &buffer, int hz)
{
    // Calculate time step between consecutive buffer elements
    float dt = 1.0 / hz;

    Eigen::Vector3f error_integral = buffer.rowwise().sum() * dt / buffer.cols();

    return error_integral;
}

void TurretController::publish_turret_joint_goal()
{
    interbotix_xs_msgs::msg::JointGroupCommand joint_group_command_msg;
    joint_group_command_msg.name = turret_name;
    joint_group_command_msg.cmd = {
        dq.coeff(0),
        dq.coeff(1),
    };
    pub_joint_group_command->publish(joint_group_command_msg);
}

void TurretController::publish_sim_joint_states()
{
    JointState joint_states_msg;
    joint_states_msg.header.stamp = this->get_clock()->now();

    if (pub_turret_joint_states)
    {
        joint_states_msg.name = {
            "pan",
            "tilt",
        };
        joint_states_msg.position = {
            q.coeff(0),
            q.coeff(1),
        };
        pub_joint_states_turret->publish(joint_states_msg);
    }

    // payload aim joint
    joint_states_msg.name = {
        payload_aim_joint,
    };
    joint_states_msg.position = {
        q.coeff(2),
    };
    pub_joint_states_payload->publish(joint_states_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurretController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
