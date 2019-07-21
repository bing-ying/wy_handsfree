#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv){
    ros::init(argc, argv,"testvelocity");
    ros::NodeHandle nh;
    ros::Publisher  vel_pub;
    vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/mobile_base_controller/cmd_vel",100);
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.25;
    ros::Time begin = ros::Time::now();
    while((ros::Time::now()-begin<ros::Duration(100.0))&& ros::ok() ){
       // ROS_ERROR("hhhhh");
        vel_pub.publish(cmd);
    }
}