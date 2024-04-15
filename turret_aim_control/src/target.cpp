#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

class TargetTransformPublisher : public rclcpp::Node
{
public:
    TargetTransformPublisher() : Node("target_transform_publisher")
    {
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&TargetTransformPublisher::publishTransform, this));
    }

private:
    void publishTransform()
    {
        static double angle = 0.0;
        const double radius = 0.5;
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "target";
        transformStamped.transform.translation.x = 0;
        transformStamped.transform.translation.y = radius * cos(angle);
        transformStamped.transform.translation.z = radius * sin(angle);

        static tf2_ros::TransformBroadcaster broadcaster(this);
        broadcaster.sendTransform(transformStamped);

        angle += 0.05; // Increment the angle
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetTransformPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
