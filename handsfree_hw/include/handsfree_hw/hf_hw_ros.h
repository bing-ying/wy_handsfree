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

#ifndef HF_HW_ROS_
#define HF_HW_ROS_

#include <vector>

#include <handsfree_hw/base_cmd_interface.h>
//#include <handsfree_hw/base_state_interface.h>被包含在<handsfree_hw/base_cmd_interface.h>了
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <handsfree_hw/head_servo_state_interface.h>
#include <handsfree_hw/head_servo_command_interface.h>
#include <handsfree_hw/gripper_state_interface.h>
#include <handsfree_hw/gripper_cmd_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
// #include <handsfree_msgs/robot_state.h>

#include <xm_msgs/xm_Gripper.h>
#include <xm_msgs/xm_Plat.h>

#include <controller_manager/controller_manager.h>
// for ros headers
#include <ros/ros.h>
#include <ros/callback_queue.h>

// for hf link and transport
//#include <handsfree_hw/transport.h>包含在了<handsfree_hw/transport_serial.h>
//#include <handsfree_hw/transport_serial.h>
//#include <hf_link.h>包含在了<handsfree_hw/hf_hw.h>
#include <handsfree_hw/hf_hw.h>
#include <math.h>

//for tf 
#include <tf/transform_broadcaster.h>

#include <ecl/threads.hpp>
namespace handsfree_hw{

class HF_HW_ros:public hardware_interface::RobotHW{

public:
    HF_HW_ros(ros::NodeHandle &nh, std::string url, std::string config_addr , bool use_sim_);

    double getFreq()const
    {
        return controller_freq_;
    }

    void mainloop();
    bool serviceCallBack(xm_msgs::xm_Gripper::Request &req,xm_msgs::xm_Gripper::Response &res);
    bool platCallBack(xm_msgs::xm_Plat::Request &req,xm_msgs::xm_Plat::Response &res);

private:
    //communication with embeded system
    boost::mutex read_mutex_3;

    HF_HW hf_hw_;
    ros::NodeHandle nh_;
    ros::CallbackQueue queue_;
    //int tempi;//
    // publish the robot state for diagnose system
    // ros::Publisher robot_state_publisher_;
    ros::ServiceServer server_;
    ros::ServiceServer plat_server_;
    ros::ServiceServer getparam_srv_;
    ros::ServiceServer setparam_srv_;
    tf::TransformBroadcaster transform_arm;
    //parameter list 肖：在launch文件中声明了with_arm_、sim_xm_的同名参数(除去后面的下划线)并赋值
    std::string base_mode_;
    bool with_arm_;
    double controller_freq_;
    bool sim_xm_;
    //hardware resource
    // handsfree_msgs::robot_state robot_state;

    std::vector<double> arm_pos_  , arm_vel_  , arm_eff_, arm_cmd_;
    std::vector<double> gripper_pos_  , gripper_vel_  , gripper_eff_, gripper_cmd_,lifting_cmd_,lifting_pos_;
    unsigned char gripper_cmd_hw_;
   

    //char endEffort_command_;
    float plat_height_command;

    double x_, y_, theta_, x_cmd_, y_cmd_, theta_cmd_;
    double x_vel_, y_vel_, theta_vel_;//机器人当前速度和转角
   
    double head_servo_yaw_pos_ ,head_servo_pitch_pos_;
    double head_servo_yaw_cmd_ ,head_servo_pitch_cmd_;

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

    /*hardware_interface::HeadServoStateInterface head_servo_state_interface_;
    hardware_interface::HeadServoInterface head_servo_cmd_interface ;*/

    hardware_interface::BaseStateInterface base_state_interface_;
    hardware_interface::BaseVelocityInterface base_velocity_interface_;
    
    hardware_interface::GripperStateInterface gripper_state_interface_;
    hardware_interface::GripperCmdInterface gripper_cmd_interface_;

    // //ecl::Thread write_update_thread_;  
    // ecl::Thread read_update_thread_;

    inline void writeBufferUpdate()
    {//将机器人期望状态发送给下位机
        
      
        /*
        hf_hw_.getRobotAbstract()->expect_motor_speed.servo1 = wheel_cmd_[0];
        hf_hw_.getRobotAbstract()->expect_motor_speed.servo2 = wheel_cmd_[1];
        hf_hw_.getRobotAbstract()->expect_motor_speed.servo3 = wheel_cmd_[2];
        */
        hf_hw_.getRobotAbstract()->expect_robot_speed.x = x_cmd_;
        hf_hw_.getRobotAbstract()->expect_robot_speed.y = y_cmd_;
        hf_hw_.getRobotAbstract()->expect_robot_speed.z = theta_cmd_;
        if (with_arm_)
        {
            hf_hw_.getRobotAbstract()->expect_arm_state.servo1 = arm_cmd_[0];
            hf_hw_.getRobotAbstract()->expect_arm_state.servo2 = arm_cmd_[1];
            hf_hw_.getRobotAbstract()->expect_arm_state.servo3 = arm_cmd_[2];
           // arm_cmd_[3]*=-1;
            hf_hw_.getRobotAbstract()->expect_arm_state.servo4 = arm_cmd_[3];
            // std::cout<<gripper_cmd_hw_<<std::endl;
            // ROS_ERROR_STREAM("hehe"<<gripper_cmd_hw_);
            hf_hw_.getRobotAbstract()->claw_state_= gripper_cmd_hw_;
            
           hf_hw_.getRobotAbstract()->wrist_angle_ = arm_cmd_[5];

            hf_hw_.getRobotAbstract()->plat_height = arm_cmd_[4];

             
        }
        // the servo num is differentROS_ERRORmd_;
        /*
        hf_hw_.getRobotAbstract()->expect_head2_state.pitch  = head2_servo1_cmd_;
        hf_hw_.getRobotAbstract()->expect_head2_state.yaw  = head2_servo2_cmd_;
        */
        
    }

    inline void readBufferUpdate()
    {
        boost::mutex::scoped_lock lock(read_mutex_3);
      
        x_     = hf_hw_.getRobotAbstract()->measure_global_coordinate.x;
        y_     = hf_hw_.getRobotAbstract()->measure_global_coordinate.y;
        theta_ = hf_hw_.getRobotAbstract()->measure_global_coordinate.z;
       // if(tempi%30==0)
       //ROS_ERROR("measure_global_coordinate:         x_:%f ;y_:%f ;theta_:%f",x_,y_,theta_);

        x_vel_ = hf_hw_.getRobotAbstract()->measure_robot_speed.x;
        y_vel_ = hf_hw_.getRobotAbstract()->measure_robot_speed.y;
        theta_vel_ = hf_hw_.getRobotAbstract()->measure_robot_speed.z;
       // if(tempi%50==0)
        //ROS_ERROR("measure_robot_speed: x_vel_:%f ;y_vel_:%f ;theta_vel_:%f",x_vel_,y_vel_,theta_vel_);
       // tempi++;
        /*
        wheel_pos_[0] = hf_hw_.getRobotAbstract()->measure_motor_mileage.servo1;
        wheel_pos_[1] = hf_hw_.getRobotAbstract()->measure_motor_mileage.servo2;
        wheel_pos_[2] = hf_hw_.getRobotAbstract()->measure_motor_mileage.servo3;
        */
        // robot_state.battery_voltage = hf_hw_.getRobotAbstract()->robot_parameters.robot_body_radius;
        // robot_state.cpu_temperature = hf_hw_.getRobotAbstract()->robot_system_info.cpu_temperature;
        // robot_state.cpu_usage = hf_hw_.getRobotAbstract()->robot_system_info.cpu_usage;
        // robot_state.system_time = hf_hw_.getRobotAbstract()->robot_system_info.system_time;

        if (with_arm_)
        {
            arm_pos_[0] = hf_hw_.getRobotAbstract()->measure_arm_state.servo1;
            arm_pos_[1] = hf_hw_.getRobotAbstract()->measure_arm_state.servo2;
            arm_pos_[2] = hf_hw_.getRobotAbstract()->measure_arm_state.servo3;
            arm_pos_[3] = (hf_hw_.getRobotAbstract()->measure_arm_state.servo4);
            arm_pos_[5] = hf_hw_.getRobotAbstract()->wrist_angle_back;
            arm_pos_[4] = hf_hw_.getRobotAbstract()->plat_height_back;//升降测量高度返回值
       

           
        }

       /*head_servo_yaw_pos_ = hf_hw_.getRobotAbstract()->measure_head1_state.yaw;//暂无卵用
        head_servo_pitch_pos_ = hf_hw_.getRobotAbstract()->measure_head1_state.pitch;*/
        /*
        wheel_vel_[0] = hf_hw_.getRobotAbstract()->measure_motor_speed.servo1;
        wheel_vel_[1] = hf_hw_.getRobotAbstract()->measure_motor_speed.servo2;
        wheel_vel_[2] = hf_hw_.getRobotAbstract()->measure_motor_speed.servo3;

        head1_servo1_pos_ = hf_hw_.getRobotAbstract()->measure_head1_state.pitch ;
        head1_servo1_vel_ = 0 ;
        head1_servo1_eff_ = 0 ;

        head1_servo2_pos_ = hf_hw_.getRobotAbstract()->measure_head1_state.yaw ;
        head1_servo2_vel_ = 0 ;
        head1_servo2_eff_ = 0 ;

        head2_servo1_pos_ = hf_hw_.getRobotAbstract()->measure_head2_state.pitch ;
        head2_servo1_vel_ = 0 ;
        head2_servo1_eff_ = 0 ;

        head2_servo2_pos_ = hf_hw_.getRobotAbstract()->measure_head2_state.yaw ;
        head2_servo2_vel_ = 0 ;
        head2_servo2_eff_ = 0 ;
        */

    }
};

}

#endif
