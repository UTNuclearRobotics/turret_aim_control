#include <behaviortree_cpp/bt_factory.h>
#include "turret_aim_control_behaviors/aim_turret.hpp"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory) {
    factory.registerNodeType<turret_aim_control_behaviors::AimTurret>("AimTurret");

    factory.addMetadataToManifest("AimTurret", {
        {"description", "Aims the Interbotix PTU in a given direction vector."},
    });
}
