#include <handsfree_hw/hf_hw_ros.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "robothw");
    ros::NodeHandle nh("mobile_base");
    std::string config_filename = "/config.txt" ;
    std::string config_filepath = CONFIG_PATH+config_filename ; 
    std::cerr<<"the configure file path is: "<<config_filepath<<std::endl;
    ros::NodeHandle nh_private("~");
    std::string serial_port;
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0"); 
    std::string serial_port_path="serial://" + serial_port;

    bool sim_xm_;
    nh.getParam("/handsfree_hw_node/sim_xm",sim_xm_);//优先获取是否仿真
    handsfree_hw::HF_HW_ros hf(nh, serial_port_path , config_filepath , sim_xm_);

    hf.mainloop();
    return 0;
}
