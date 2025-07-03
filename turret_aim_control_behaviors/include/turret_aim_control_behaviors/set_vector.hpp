#pragma once

#include <sstream> 
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include "behaviortree_cpp/bt_factory.h"

#include <geometry_msgs/msg/vector3.hpp>

namespace turret_aim_control_behaviors {

class SetVector : public BT::SyncActionNode
{
public:
    SetVector(const std::string name, const BT::NodeConfig &config);
    
    static BT::PortsList providedPorts();
    
    BT::NodeStatus tick() override;

private:
}; // class AimTurret

} // namespace turret_aim_control_behaviors