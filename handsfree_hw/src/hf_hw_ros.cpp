/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: hf_link.cpp
* Contact:  QQ Exchange Group -- 521037187
* Version:  V2.0
*
* LICENSING TERMS:
* The Hands Free is licensed generally under a permissive 3-clause BSD license.
* Contributions are required to be made under the same license.
*
* History:
* <author>      <time>      <version>      <desc>
* luke liao       2016.4.1   V1.0           creat this file
*
* Description: handsfree ros ros_control framework
***********************************************************************************************************************/
/********************


                *******************/


#include <handsfree_hw/hf_hw_ros.h>

namespace handsfree_hw{
uint8_t plat_flag=0;

HF_HW_ros::HF_HW_ros(ros::NodeHandle &nh, std::string url, std::string config_addr,bool use_sim_) :
    hf_hw_(url, config_addr, use_sim_),
    nh_(nh)
{
    //get the parameter
    //设置这个以后就不用spin了
    //tempi=0;
    sim_xm_ = use_sim_;
    //：从参数服务器获取launch文件中设定的同名参数sim_xm
    if(!sim_xm_){
        
        ROS_ERROR("Started xm_robot connection on port /dev/ttyUSB0");          
       
    }
    else 
        ROS_ERROR("xm_robot being simulated.");
    nh_.setCallbackQueue(&queue_);
    base_mode_ = "2-wheel";
    with_arm_ = true;
    controller_freq_ = 50;//100
    nh_.getParam("base_mode", base_mode_);
    nh_.getParam("/handsfree_hw_node/with_arm", with_arm_);
    nh_.getParam("freq", controller_freq_);
    // robot_state_publisher_ = nh_.advertise<handsfree_msgs::robot_state>("robot_state", 10);////////////////////

    x_ = y_ = theta_ = x_cmd_ = y_cmd_ = theta_cmd_ = 0.0;
    x_vel_ = y_vel_ = theta_vel_ = 0.0;
    gripper_cmd_hw_ = 2;
    /*
    head1_servo1_cmd_ = head1_servo2_cmd_  = head2_servo1_cmd_ = head2_servo2_cmd_ = 0.0;
    head1_servo1_pos_ = head1_servo2_pos_ = head1_servo1_vel_ = head2_servo1_vel_ = head1_servo1_eff_ = head2_servo1_eff_ = 0;
    */
    //real_theta = -theta_;real_theta_cmd = theta_cmd_;real_theta_vel = theta_vel_;
    //register the hardware interface on the robothw
    hardware_interface::BaseStateHandle base_state_handle("mobile_base", &x_, &y_, &theta_, &x_vel_, &y_vel_, &theta_vel_);//7.23电子组更改坐标系，为协调故在软件与电子交互层更改xy顺序
    base_state_interface_.registerHandle(base_state_handle);
    registerInterface(&base_state_interface_);
    hardware_interface::BaseVelocityHandle base_handle(base_state_handle, &x_cmd_, &y_cmd_, &theta_cmd_);
    base_velocity_interface_.registerHandle(base_handle);
    registerInterface(&base_velocity_interface_);

    //if (with_arm_)
    //{
        arm_pos_.resize(6,0.0);
        arm_vel_.resize(6,0.0);
        arm_eff_.resize(6,0.0);
        arm_cmd_.resize(6,0.0);//第六个数表示对plat_height的命令,第五个数表示对wrist_angle的命令
        std::vector<std::string>joint_names;
        joint_names.push_back("joint_waist");
        joint_names.push_back("joint_big_arm");
        joint_names.push_back("joint_fore_arm");
        joint_names.push_back("joint_wrist");
        
        // the lifting_controller use the same interface with the arm
        // but use different controller
        joint_names.push_back("joint_lifting");
        joint_names.push_back("joint_small_arm");
        ROS_WARN("registerHandle begin");

        for (int i = 0;i < arm_pos_.size();i++)
        {
            //get the joint name
            hardware_interface::JointStateHandle arm_state_handle(joint_names[i], &arm_pos_[i], &arm_vel_[i], &arm_eff_[i]);
            joint_state_interface_.registerHandle(arm_state_handle);
            hardware_interface::JointHandle arm_handle(arm_state_handle , &arm_cmd_[i]);
            position_joint_interface_.registerHandle(arm_handle);
            //ROS_WARN("registerHandle %s",i);
        }


        gripper_pos_.resize(2,0.0);
        gripper_vel_.resize(2,0.0);
        gripper_eff_.resize(2,0.0);
        gripper_cmd_.resize(2,0.0);
        std::vector<std::string>gripper_names;
        gripper_names.push_back("joint_left_finger");
        gripper_names.push_back("joint_right_finger");
        for (int i = 0;i < gripper_pos_.size();i++)
        {
            //get the joint name
            hardware_interface::JointStateHandle gripper_state_handle(gripper_names[i], &gripper_pos_[i], &gripper_vel_[i], &gripper_eff_[i]);
            joint_state_interface_.registerHandle(gripper_state_handle);
            hardware_interface::GripperCmdHandle gripper_handle(gripper_state_handle ,&gripper_cmd_hw_, &gripper_cmd_[i]);
            gripper_cmd_interface_.registerHandle(gripper_handle);
          
        }

       
        registerInterface(&joint_state_interface_);
        registerInterface(&position_joint_interface_);
        registerInterface(&gripper_cmd_interface_);

    //}


    /*hardware_interface::HeadServoStateHandle head_servo_state_handle("xm_head",&head_servo_yaw_pos_,&head_servo_pitch_pos_);
    head_servo_state_interface_.registerHandle(head_servo_state_handle);
    registerInterface(&head_servo_state_interface_);
    hardware_interface::HeadServoHandle head_servo_handle(head_servo_state_handle,&head_servo_yaw_cmd_,&head_servo_pitch_cmd_);
    head_servo_cmd_interface.registerHandle(head_servo_handle);
    
    registerInterface(&head_servo_cmd_interface);*/
    
    //write_update_thread_.start(&HF_HW_ros::writeBufferUpdate, *this);
    //read_update_thread_.start(&HF_HW_ros::readBufferUpdate, *this);
   
    if(!sim_xm_){
              if (hf_hw_.initialize_ok())
            {
                ROS_INFO("system initialized succeed, ready for communication");
            } else
            {
                ROS_ERROR("hf link initialized failed, please check the hardware");
            }
    }
    else 
     ROS_ERROR("hf link initialized is no need, take it easy");
   
}
// 上次比赛写的简易版本控制方法，还是可以用的
// 本来是写了一个控制器，但是偶尔会出现智能指针错误导致程序崩溃，所以保险起见还是暂时先用service来实现对gripper的控制
bool HF_HW_ros::serviceCallBack(xm_msgs::xm_Gripper::Request &req,xm_msgs::xm_Gripper::Response &res)
{
// 1 open 0 close

    gripper_cmd_hw_ = req.command;
    float gripper_state = (gripper_cmd_hw_==true)? 0.0:-0.01;
    // update the gripper_state
    gripper_cmd_[0]= gripper_state;
    gripper_cmd_[1]= gripper_state;
    res.result = true;
    res.message = "ok";
    
    return true;
}

// bool HF_HW_ros::platCallBack(xm_msgs::xm_Plat::Request &req,xm_msgs::xm_Plat::Response &res)
// {
//     plat_height_command=req.height;
//     if(req.height>0)
//     {
        
//         res.result = true;
//         res.message = "up";
//         ROS_ERROR("plat_up");
//     }
//     else{
       
//         res.result = true;
//         res.message = "down";
//          ROS_ERROR("plat_down");
//     }
//    plat_flag =1;
//     return true;
// }

void HF_HW_ros::mainloop()
{
    ros::CallbackQueue cm_callback_queue;
    ros::NodeHandle cm_nh("mobile_base");
    cm_nh.setCallbackQueue(&cm_callback_queue);
    controller_manager::ControllerManager cm(this, cm_nh);//传入一个和controller同一工作空间的ros句柄

    ros::AsyncSpinner cm_spinner(1, &cm_callback_queue);//:mmp你这里开个1线程是要闹哪样
    //ros::AsyncSpinner hw_spinner(1, &queue_);

    if(with_arm_)
        server_ = cm_nh.advertiseService("gripper_command", &HF_HW_ros::serviceCallBack,this);
    // plat_server_ =cm_nh.advertiseService("plat_command",&HF_HW_ros::platCallBack,this);
    ros::Rate loop(controller_freq_);
    //ros::Rate hehe(20);//去你mmp的呵呵
    cm_spinner.start();
    //hw_spinner.start();

    int count = 0;
    ros::Time currentTime = ros::Time::now();
    while (ros::ok())
    {
        if(!sim_xm_){
            hf_hw_.checkHandshake();
            /*// if(!hf_hw_.updateCommand(READ_GLOBAL_COORDINATE, count))
                hf_hw_.updateCommand(READ_GLOBAL_COORDINATE, count);
            // if(!hf_hw_.updateCommand(READ_ROBOT_SPEED, count))
                hf_hw_.updateCommand(READ_ROBOT_SPEED, count);
           
            // hf_hw_.updateCommand(READ_HEAD_1, count);
            if (with_arm_){
                hf_hw_.updateCommand(READ_ARM_TOTAL, count);/////
            //2017.7.9注释下述代码,因为stm32未写platform的数据反馈 
                hf_hw_.updateCommand(READ_PLAT, count);

               
            }
            // if(!hf_hw_.updateCommand(SET_ROBOT_SPEED, count))
            /////  
            */
            readBufferUpdate();
            
            //std::cout<< "* *"<<std::endl;
            hf_hw_.updateWriteCommand(SET_ROBOT_SPEED, count);
            //ROS_INFO("head1_servo1_cmd_ = %.4f  head1_servo2_cmd_=%.4f" , head1_servo1_cmd_ ,head1_servo2_cmd_);
            if(with_arm_){
                hf_hw_.updateWriteCommand(SET_ARM_TOTAL, count);             
                hf_hw_.updateWriteCommand(PLAT_MOVE, count);
                hf_hw_.updateWriteCommand(SET_CLAW, count);
            //if (with_arm_)
                hf_hw_.updateWriteCommand(SET_WRIST,count);
                    // due to we donnot call the chassis to return the gripper state to the handle
            // so we should manual update the handle for the /joint_state topic
                gripper_pos_[0]= gripper_cmd_[0];
                gripper_pos_[1]= gripper_cmd_[1];  
            }
            writeBufferUpdate();
     
            cm.update(ros::Time::now(), ros::Duration(1 / controller_freq_));
         
            // hf_hw_.updateCommand(SET_HEAD_1, count);
            
                                           
        }else{
            //底盘数据的模拟更新
            x_     += x_vel_*cos(theta_)/controller_freq_;
            y_     += x_vel_*sin(theta_)/controller_freq_;
            theta_ += theta_cmd_/controller_freq_;

            x_vel_ =x_cmd_;
            y_vel_ =y_cmd_;
            theta_vel_ = theta_cmd_;
            //机械臂数据的模拟更新
       
            arm_pos_[0]= arm_cmd_[0];
            arm_pos_[1]= arm_cmd_[1];
            arm_pos_[2]= arm_cmd_[2];
            arm_pos_[3]= arm_cmd_[3];
            arm_pos_[4]= arm_cmd_[4];
            arm_pos_[5]= arm_cmd_[5];
            gripper_pos_[0]= gripper_cmd_[0];
            gripper_pos_[1]= gripper_cmd_[1];
            cm.update(ros::Time::now(), ros::Duration(1 / controller_freq_));
            
            //hehe.sleep();注释于1.27如果不注释，仿真时每次更新完都会睡眠一次，显示在rviz上就会感觉机器人特别卡
                } 

        loop.sleep();
        count++;
    }

    cm_spinner.stop();
    //hw_spinner.stop();
}
};


