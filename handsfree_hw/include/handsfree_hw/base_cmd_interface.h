/*
 * base_cmd_interface.h
 *
 *  Created on: Jan 7, 2016
 *      Author: liao
 */

#ifndef BASE_CMD_INTERFACE_H_
#define BASE_CMD_INTERFACE_H_

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <handsfree_hw/base_state_interface.h>

namespace hardware_interface
{
class BaseVelocityHandle : public BaseStateHandle
{//：控制底盘的速度
public:
	BaseVelocityHandle() : BaseStateHandle(), x_cmd_(0), y_cmd_(0), theta_cmd_(0) {}

	BaseVelocityHandle(const BaseStateHandle& js, double* x_cmd, double* y_cmd, double* theta_cmd)
	    : BaseStateHandle(js), x_cmd_(x_cmd), y_cmd_(y_cmd), theta_cmd_(theta_cmd)
	{
	if (!x_cmd_)
	{
	  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command data pointer is null.");
	}
	if (!y_cmd_)
	{
	  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command data pointer is null.");
	}
	if (!theta_cmd_)
	{
	  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command data pointer is null.");
	}
	}
	void setXcmd(double cmd) {assert(x_cmd_); *x_cmd_ = cmd;}
	void setYcmd(double cmd) {assert(y_cmd_); *y_cmd_ = cmd;}
	void setThetacmd(double cmd) {assert(theta_cmd_); *theta_cmd_ = cmd;}

	double getXcmd() const {assert(x_cmd_); return *x_cmd_;}
	double getYcmd() const {assert(y_cmd_); return *y_cmd_;}
	double getThetacmd() const {assert(theta_cmd_); return *theta_cmd_;}

private://目标速度
	double* x_cmd_;
	double* y_cmd_;
	double* theta_cmd_;
};

class BaseVelocityInterface : public HardwareResourceManager<BaseVelocityHandle, ClaimResources> {};
//第二个继承类一般默认为dontclaimresource，这里改变了 xiao这里继承的是一个ros官方的类，不是电子组那边的操作

}
//多了三个cmd数据
//接口用来登记，具体还是handle来处理数据


#endif /* BASE_CMD_INTERFACE_H_ */
