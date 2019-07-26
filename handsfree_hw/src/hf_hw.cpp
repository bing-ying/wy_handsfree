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

//#include <ros/ros.h>注释于1.27
#include <handsfree_hw/hf_hw.h>

namespace handsfree_hw {

HF_HW::HF_HW(std::string url, std::string config_addr, bool use_sim_)
{                
    loop_counter = 0;
                     //：复制"://"前的字符串
    std::string transport_method = url.substr(0, url.find("://"));
    if (transport_method == "serial")
    {                    //：初始化基类串口
        port_ = boost::make_shared<TransportSerial>(url);
        time_out_ = 20;//真的是这个值导致底盘卡吗，对确实是   原值100，handsfree开源社区用的是500
        hflink_ = boost::make_shared<HFLink>(0x01, 0x11, &my_robot_);
                       //：第一个参数是一个io_service对象指针
        timer_.reset(new boost::asio::deadline_timer(*(port_->getIOinstace()),
                                            //：第二个参数是等待的绝对时间
                                                     boost::posix_time::milliseconds(time_out_)));
    }else if (transport_method == "udp")
    {
    }else if (transport_method == "tcp")
    {
    }

    //process the config file
    file_.open(config_addr.c_str(), std::fstream::in);
    if (file_.is_open())
    {
        for (int i = 0; i < LAST_COMMAND_FLAG; i++)
        {
            std::string temp;
            //hflink_command_set_[i]表示是否发送对应命令，hflink_freq_[i]表示相应命令的发送频率
            file_ >> temp >> hflink_command_set_[i] >> hflink_freq_[i];
            //std::cout<< temp << hflink_command_set_[i] << hflink_freq_[i]<<std::endl;
        }
        file_.close();
        initialize_ok_ = port_->initialize_ok();
    } else
    {
        std::cerr << "config file can't be opened, check your file load " <<std::endl;
        initialize_ok_ = false;
    }
    if(use_sim_ == 0)
    {
        try
        {
            thread_2 = boost::thread(boost::bind(&HF_HW::updateReadCommand, this));
        }
        catch(std::exception &e)
        {
            std::cerr << "read thread create failed " << std::endl;
            std::cerr << "Error Info: " << e.what() <<std::endl;
        }
    }
}

void HF_HW::timeoutHandler(const boost::system::error_code &ec)
{
    if (!ec)
    {
        std::cerr << "Time Out" <<std::endl;
        boost::mutex::scoped_lock lock(wait_mutex_);
        time_out_flag_ = true;
    }
}

// for QT client , not for ros update
void HF_HW::updateRobot()
{
    boost::asio::deadline_timer cicle_timer_(*(port_->getIOinstace()));
    //：将hflink_count_用0填充
    memset(hflink_count_, 0 ,sizeof hflink_count_);
    int count = 0;//用来控制命令发送频率
    while (true)
    {
        cicle_timer_.expires_from_now(boost::posix_time::millisec(10));//100hz 设置10毫秒得程序运行时间
        //std::cout<< "start a write" <<std::endl;
        //  check hand shake
        checkHandshake();
        //  uodate normal data：将hflink_command_set_current_用0填充
        memset(hflink_command_set_current_, 0, sizeof hflink_command_set_current_);
        for (int i = 1; i < LAST_COMMAND_FLAG; i++)
        {//该循环会把所有需要发的命令发送一遍
            if (hflink_command_set_[i] != 0)//不为0意味着这个命令会被发送
            {
                int cnt = count % 100;
                if (cnt %  (100 / hflink_freq_[i]) == 0)
                {//控制各命令发送频率
                    sendCommand((Command)i);
                    hflink_command_set_current_[i] = 1;
                }
            }
        }

        //std::cout<< "start a read" <<std::endl;
        Buffer data = port_->readBuffer();//：data为从串口读取缓冲区读取的信息
        ack_ready_ = false;
        while (!ack_ready_)
        {//超时或者发送的所有命令下位机都正常接受了才会退出循环
            for (int i = 0; i < data.size(); i++)
            {//该循环将把整个数据包分析一遍
                //std::cout<<"get byte   :"<< data[i]<< std::endl;
                if (hflink_->byteAnalysisCall(data[i]))
                //：只要收到了包且包没问题就为true。在整个数据包的分析过程中，只有分析到最后的效验阶段，并且通过了才会执行下面代码块
                {
                    // all receive package ack arrived
                    uint8_t temp = 1;
                    for (int i = 1; i < LAST_COMMAND_FLAG; i++)
                        temp = temp & checkUpdate((Command)i);//这里可以考虑成0001&0001或0001&0000
                    //std::cout<< temp <<std::endl;
                    /*如果发送的所有命令都被接收到了此处就为1，but打印出这个有什么卵用吗？下面反正都打印了
                    上位机在发送相关命令后下位机收到了会发送给看似没有什么卵用的ack，这个ack虽然不包含实际数据，但是包含有接收到的命令
                    可以让上位机知道命令有没有正常被下位机处理*/
                    if (temp)
                    {
                        ack_ready_ = true;
                        //std::cout<< "all package received" <<std::endl;
                    }
                }
            }
            if (cicle_timer_.expires_from_now().is_negative())
            {
                //std::cout<<"Timeout continue next circle"<<std::endl;
                break;
            }
            data = port_->readBuffer();//：从串口读取缓冲区读取数据，此时下位机应该要根据之前上位机发送的命令把相应数据包写入串口了
        }
        count++;
        cicle_timer_.wait();// 函数调用直到定时器终止
    }
}

bool HF_HW::updateWriteCommand(const Command &command, int count)
{//按照相应频率发送命令
    if (hflink_command_set_[command] != 0)
    {
        int cnt = count % 100;
        if (cnt %  (100 / hflink_freq_[command]) == 0)
        {
            //std::cout<<std::hex<<command;//注意此处输出的是16进制的command
            sendCommand(command);//根据command的类型发送数据，已写入串口
        } else
        {
            // skip this package
            return false;
        }
    }
    //上面的代码完成定时向stm32发送指令，下面的代码完成分析接收包的信息
    // 相当于每发送一次指令后,程序就卡在下述代码段等待数据反馈
    
    return true;
}

void HF_HW::updateReadCommand()
{
    while(1)
    {
            boost::asio::io_service io;
            boost::asio::deadline_timer t(io, boost::posix_time::millisec(15));  
            t.wait();  
           Buffer data = port_->readBuffer();
           /*for(int i=0;i<data.size();i++)
                {
                    std::cout<<+static_cast<unsigned char>(data[i])<<" ";
                }
                std::cout<<std::endl;*/
                //std::cout<< "lala"<<data.size()<<std::endl;
                //std::cout<<loop_counter<<std::endl;
                loop_counter ++;
            if(!data.empty())
            {
                loop_counter = 0;
                boost::asio::deadline_timer cicle_timer_(*(port_->getIOinstace()));
                cicle_timer_.expires_from_now(boost::posix_time::millisec(time_out_));
                //std::cout << "lalal"<<read_queue.size()<<std::endl;
                /*for(int i=0;i<data.size();i++)
                {
                    std::cout<<+static_cast<unsigned char>(data[i])<<" ";
                }
                std::cout<<std::endl;*/
                ack_ready_ = false;
                bool flag = 0;
                while (!ack_ready_)//：两种情况下会退出循环，一是超时，二是成功下位机成功接受了命令
                {
                    // for (int i = 0; i < data.size(); i++)
                    // {注释于1.27
                    //     unsigned int a= int(data[i]);
                    //     ROS_INFO_STREAM(std::hex<<a);
                    // }
                    for (int i = 0; i < data.size(); i++)//假如data为空，则size()运行结果为0
                    {
                        if (hflink_->byteAnalysisCall(data[i]))//分析包的完整性，完整时分析包并执行所属操作,对机器人ADT的变量更新即在这层调用完成
                        {
                            // one package ack arrived
                            // std::cerr<<"I complete a package command is: "<<command<<std::endl;
                            ack_ready_ = true;
                        }//else{
                        // std::cerr<<"mmp,update Command false,ready to updateCommand again"<<std::endl;
                        // return false;
                    // }
                    }
                   
                    if (cicle_timer_.expires_from_now().is_negative())//超时报错
                    {
                        ack_ready_ = true;//丢掉坏包，跳出循环
                        std::cerr<<"Timeout continue skip this package "<<+static_cast<unsigned char>(data[6])<<std::endl;
                    }
                }
            }
            if(loop_counter >= 20)
                    {
                       std::cout<<"OH! The link has been broken!" << std::endl; 
                    }
        }
    }
}


