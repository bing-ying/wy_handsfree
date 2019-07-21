/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: hf_link.cpp
* Contact:  QQ Exchange Group -- 521037187
* Version:  V2.0
*
* LICENSING TERMS:
* The Hands Free is licensed generally under a permissive 3-clause BSD license.
* Contributions are requiredto be made under the same license.
*
* History:
* <author>      <time>      <version>      <desc>
* luke liao       2016.4.1   V1.0           creat this file
*
* Description: define the handfree pc software interface
***********************************************************************************************************************/

#ifndef HF_HW_H_
#define HF_HW_H_

#include <fstream>
#include <handsfree_hw/transport_serial.h>
#include <hf_link.h>
#include <cstdlib>

namespace handsfree_hw {

class HF_HW{
public:
	HF_HW(std::string url, std::string config_addr);

	bool updateCommand(const Command &command, int count);

	void updateRobot();

	inline RobotAbstract* getRobotAbstract()//：返回抽象机器人指针
	{
		return &my_robot_;
	}

	inline boost::shared_ptr<boost::asio::io_service> getIOinstace()
	{//：返回io_service对象指针
		return port_->getIOinstace();
	}

	bool reconfig()
	{

	}

	inline bool initialize_ok () const
	{
		return initialize_ok_;
	}

	inline void checkHandshake()
	{
		if (hflink_->getReceiveRenewFlag(SHAKING_HANDS)==1)
		{
			sendCommand(SHAKING_HANDS);
			std::cout<<"send shake hands"<<std::endl;
		}//else std::cout<<"I have send shake hands"<<std::endl;
	}

private:
	boost::shared_ptr<Transport> port_;//：抽象基类的指针，提供串口读写功能
	boost::shared_ptr<HFLink> hflink_;//：用于上下位机通信,定义通信协议
	//：维护一个时间，即线程的阻塞等待
	boost::shared_ptr<boost::asio::deadline_timer> timer_;

	//for reading config file
	std::fstream file_;
	bool initialize_ok_;
	//for updating data：LAST_COMMAND_FLAG于hf_link.h中声明，意为命令的最大数
	int hflink_command_set_[LAST_COMMAND_FLAG];//如果需要发送什么命令，相应位置的值就会置为非0
	int hflink_freq_[LAST_COMMAND_FLAG];//发送相应命令的频率，最大频率是每10毫秒一次，也就是100hz
	int hflink_count_[LAST_COMMAND_FLAG];//：它在.cpp文件中被初始化为0就再也没有出现了，作用未知
	int hflink_command_set_current_[LAST_COMMAND_FLAG];//代表对应命令是否已被发送

	int time_out_;//命令更新超时时间
	bool time_out_flag_;//：超时标志
	boost::mutex wait_mutex_;//：等待锁
	bool ack_ready_;//：是否已经收到ack
	void timeoutHandler(const boost::system::error_code &ec);
	
	inline void sendCommand(const Command command_state)
	{
		// std::cout<<"send message  "<<command_state <<std::endl;
		hflink_->masterSendCommand(command_state);//通过状态机分类组装信息
		//：第一个参数：上传缓冲区所在位置，第二个参数：上传缓冲区大小+缓冲区所在位置，即缓冲区结尾位置，目的：拷贝该缓冲区信息
		Buffer data(hflink_->getSerializedData(), hflink_->getSerializedLength() + hflink_->getSerializedData());//包装组装好的tx_buffer
		//？？：取得缓冲区的信息，将它包装进一个vector类型
		port_->writeBuffer(data);//串口写入组装好的tx_buffer
		// for(int i=0;i<data.size();i++)
		// {
		// 	std::cout<<std::hex<<(int)data[i]<<" ";
		// }
		// std::cout<<std::endl;
	}
	
	inline uint8_t checkUpdate(const Command command_state)
	{//：是否已发送命令标志&是否接受命令标志   如果发送了相关命令，而却没有接受相关命令，返回就为0
		if (hflink_command_set_current_[command_state] & hflink_->getReceiveRenewFlag(command_state))
		{
			return 1;
		}
		if (hflink_command_set_current_[command_state] == 0 ) return 1;
		return 0;
	}

	

	// a single object for robot
	RobotAbstract my_robot_;
};

}


#endif /* HF_HW_H_ */
