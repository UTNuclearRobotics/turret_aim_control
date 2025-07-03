#include <behaviortree_cpp/bt_factory.h>
#include "turret_aim_control_behaviors/aim_turret.hpp"
#include "turret_aim_control_behaviors/set_vector.hpp"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory) {
    factory.registerNodeType<turret_aim_control_behaviors::AimTurret>("AimTurret");
    factory.registerNodeType<turret_aim_control_behaviors::SetVector>("SetVector");

    factory.addMetadataToManifest("AimTurret", {
        {"description", "Aims the Interbotix PTU in a given direction vector."},
    });
    factory.addMetadataToManifest("SetVector", {
        {"description", "Sets a Vector3 and puts it on the blackboard at a specified port."},
    });
}
