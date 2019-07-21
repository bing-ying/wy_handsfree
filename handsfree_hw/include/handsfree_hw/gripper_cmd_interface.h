#ifndef GRIPPER_CMD_INTERFACE_H_
#define GRIPPER_CMD_INTERFACE_H_

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <handsfree_hw/gripper_state_interface.h>

namespace hardware_interface
{
    class GripperCmdHandle: public GripperStateHandle
    {
        public:
            GripperCmdHandle(): GripperStateHandle(), gripper_cmd_(0), cmd_(0) {}
            GripperCmdHandle(const GripperStateHandle & gh,unsigned char *gripper_cmd,double* cmd)
                : GripperStateHandle(gh),gripper_cmd_(gripper_cmd), cmd_(cmd)
            {
                if(!gripper_cmd){
                    throw HardwareInterfaceException("Cannot creat handle '" + gh.getName()+"'. gripper_cmd data pointer is null");
                }
               if(!cmd){
                    throw HardwareInterfaceException("Cannot creat handle '" + gh.getName()+"'. cmd data pointer is null");
               }
            }
           
            void setGripperCmd(unsigned char gripper_cmd) const {assert(gripper_cmd_); *gripper_cmd_ = gripper_cmd;}
            void setCmd(double cmd) const {assert(cmd_); *cmd_ =cmd;}
          
        private:
            unsigned char *gripper_cmd_;
            double* cmd_;
           

    };

class GripperCmdInterface : public HardwareResourceManager<GripperCmdHandle, ClaimResources> {}; 
}

#endif