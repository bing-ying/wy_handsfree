#ifndef GRIPPER_STATE_INTERFACE_H_
#define GRIPPER_STATE_INTERFACE_H_

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <cassert>
#include <string>

namespace hardware_interface
{
    typedef JointStateHandle GripperStateHandle;
class GripperStateInterface: public HardwareResourceManager<GripperStateHandle>{};
}

#endif