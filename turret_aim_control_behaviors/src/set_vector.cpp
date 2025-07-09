#include "turret_aim_control_behaviors/set_vector.hpp"

namespace turret_aim_control_behaviors {

SetVector::SetVector(const std::string name, const BT::NodeConfig &config)
: BT::SyncActionNode(name, config)
{}

BT::PortsList SetVector::providedPorts()
{
  return {
    BT::InputPort<std::string>("vector_string", "vector input"),
    BT::OutputPort<std::shared_ptr<geometry_msgs::msg::Vector3>>("output_vector", "vector output port")
  };
}

BT::NodeStatus SetVector::tick()
{
  // Get the string input from the port
  auto vector_string_optional = getInput<std::string>("vector_string");

  // Check if the input port has a valid value
  if (!vector_string_optional)
  {
      throw BT::RuntimeError("missing required input [vector_string]");
  }

  std::string vector_string = vector_string_optional.value();

  // Delimit and parse the string to create a geometry_msgs::msg::Vector3
  geometry_msgs::msg::Vector3 my_vector;
  std::stringstream ss(vector_string); // Use stringstream for parsing
  std::string segment;
  std::vector<std::string> seglist; // Use a vector to store the split parts

  while(std::getline(ss, segment, ';')) // Split the string by the delimiter ';'
  {
      seglist.push_back(segment);
  }

  if (seglist.size() != 3) // Validate the number of components
  {
      throw BT::RuntimeError("Invalid Vector3 string format: expected 'x;y;z'");
  }

  // Convert string segments to double and assign to Vector3 components
  try {
      my_vector.x = std::stod(seglist[0]);
      my_vector.y = std::stod(seglist[1]);
      my_vector.z = std::stod(seglist[2]);
  } catch (const std::invalid_argument& e) {
      throw BT::RuntimeError("Invalid Vector3 component value: " + std::string(e.what()));
  } catch (const std::out_of_range& e) {
      throw BT::RuntimeError("Vector3 component value out of range: " + std::string(e.what()));
  }

  // Set the output vector to the created Vector3
  setOutput<std::shared_ptr<geometry_msgs::msg::Vector3>>("output_vector", std::make_shared<geometry_msgs::msg::Vector3>(my_vector));

  return BT::NodeStatus::SUCCESS;
}
  
} // namespace turret_aim_control_behaviors
