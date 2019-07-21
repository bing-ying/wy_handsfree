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
* mawenke       2015.10.1   V1.0           creat this file
*
* Description: This file defined hands_free_robot simple communications protocol
*              please read Hands Free Link Manua.doc for detail
***********************************************************************************************************************/


#ifndef HF_LINK_H
#define HF_LINK_H

#include "robot_abstract.h"

#define HF_LINK_NODE_MODEL  1    //1master  0slave

#if HF_LINK_NODE_MODEL==0
#include "hf_link_port.h"
#endif

/*肖：用于上位机与下位机的通信，具体用途就是上位机向下位机发送命令让其更新机器人相关部件数据
或是下位机接收了上位机的请求后发送机器人相关部件信息给上位机*/
static const unsigned short int MESSAGE_BUFER_SIZE = 100;  //limite one message's size
/****structure for communications protocol , read Hands Free Link Manua.doc for detail****/
typedef struct HFMessage{//肖：传递的信息
    unsigned char sender_id;
    unsigned char receiver_id;
    unsigned short int length;
    unsigned char data[MESSAGE_BUFER_SIZE];//肖：0位存储Command，从1位开始存储对应的数据
}HFMessage;

//communication status 肖：检验包
enum Recstate{
    WAITING_FF1,
    WAITING_FF2,
    SENDER_ID,
    RECEIVER_ID,
    RECEIVE_LEN_H,
    RECEIVE_LEN_L,
    RECEIVE_PACKAGE,
    RECEIVE_CHECK
};

//comand type 肖：对于同一种命令，上下位机采取不同的执行方式来彼此配合
enum Command{
    SHAKING_HANDS,//0
    SET_ROBOT_SPEED,//1
    READ_ROBOT_SPEED,   //2
    READ_GLOBAL_COORDINATE,//3
    //READ_ROBOT_COORDINATE,//4
    //CLEAR_COORDINATE_DATA,   //5
    //SET_HEAD_1,//6
    //READ_HEAD_1,//7
    //SET_HEAD_2,//8
    //READ_HEAD_2,    //9
    SET_CLAW,	//10
	SET_ARM_TOTAL,//11
	READ_ARM_TOTAL,//12
	PLAT_MOVE,//13
    READ_PLAT,//14
        SET_WRIST,//XIAO 15
        READ_WRIST,//16
    LAST_COMMAND_FLAG};//17
//
class HFLink
{
public://肖：默认参数为是自己是下位机
    HFLink(unsigned char my_id_=0x11 , unsigned char friend_id_=0x01 , RobotAbstract* my_robot_=0)
    {
        hf_link_node_model = HF_LINK_NODE_MODEL ;
        port_num = 1 ;
        my_id = my_id_;         //0x11 means slave ,  read Hands Free Link Manua.doc for detail
        friend_id = friend_id_;
        hf_link_ack_en = 0;
        my_robot=my_robot_;
        //enable hflink ack , generally, master disable and slave enable
        //and slave also can disable to reduce communication burden
        if(hf_link_node_model==0) hf_link_ack_en = 1;
        shaking_hands_state=0;

        receive_state_=WAITING_FF1;
        command_state_=SHAKING_HANDS;
        rx_message_.sender_id=0;
        rx_message_.receiver_id=0;
        rx_message_.length=0;
        tx_message_.sender_id=0;
        tx_message_.receiver_id=0;
        tx_message_.length=0;

        receive_package_count=0;
        package_update_frequency=0;

        send_packag_count=0;
        tx_buffer[0]=0;
        tx_buffer_length=0;
    }

public:   // only for master
    //the master can use masterSendCommand function to send data to slave
    //like SET_GLOBAL_SPEED , READ_ROBOT_SYSTEM_INFO, READ_ROBOT_SPEED...
    unsigned char masterSendCommand(const Command command_state);
    inline unsigned char getReceiveRenewFlag(const Command command_state) const
    {
        return receive_package_renew[command_state];
    }
    inline unsigned char* getSerializedData(void) 
    {
        return tx_buffer;
    }
    inline int getSerializedLength(void)
    {
        return tx_buffer_length;
    }

public: // only for slave
    //command updata flag , the robot need to traverse These flag to decide update his own behavior
    //肖：命令接受标志，上位机发布命令时置为0，上、下位机接收命令时置为1，在其他类中会被当作判断条件
    unsigned char receive_package_renew[LAST_COMMAND_FLAG];
    unsigned char shaking_hands_state;    //1 Success   0 Failed

public:  // common
    unsigned char byteAnalysisCall(const unsigned char rx_byte);//肖：参数是接收信息比特位

    inline void set_my_id(unsigned char id){my_id =id ;}
    inline void set_friend_id(unsigned char id){friend_id =id ;}
    inline void set_port_num(unsigned char num){port_num =num ;}
    //肖：下位机确认接收数据
    inline void enable_ack(void){if(hf_link_ack_en != 1) hf_link_ack_en=1;}
    //肖：下位机未接收到数据
    inline void disable_ack(void){hf_link_ack_en=0;}

private:

    unsigned char hf_link_node_model;   // 0 slave , 1 master
    unsigned char port_num;
    unsigned char my_id;
    unsigned char friend_id;
    unsigned char hf_link_ack_en;         //enable hflink ack

    // robot abstract pointer to hflink
    RobotAbstract* my_robot;
    Recstate   receive_state_;//肖：表示希望接收的通信状态，这个是自动累加的，希望的状态收到了就自动等待后面的状态了
    Command    command_state_;//肖：接收到的命令
    HFMessage rx_message_ , tx_message_;//肖：前者为接收信息，后者为上传信息

    float receive_package_count;//肖：收到包的数量
    float package_update_frequency;   // how many message receive in one second 肖：mmp目前这个似乎没用上


    float send_packag_count;//肖：发送的包的数量
    unsigned char tx_buffer[MESSAGE_BUFER_SIZE + 20];//肖：上传数据缓冲区
    unsigned tx_buffer_length;//肖：上传缓冲区大小

    unsigned int receive_check_sum_;//xiao作为最后效验数据的依据存在
    short int receive_message_length_;//肖：记录包数据区长度，它在协议核对期间动态变化，最终赋给分装包的长度length
    short int byte_count_;//xiao记录封装包数据区中的最后一个有效数据的位置
    unsigned char receiveFiniteStates(const unsigned char rx_data);//肖：对通信状态的一个检测
    unsigned char packageAnalysis(void);//肖：对从上位机和下位机间的交互命令进行分析，返回值有两种，1和0
    //肖：读取机器人相关部分信息
    unsigned char readCommandAnalysis(const Command command_state , unsigned char* p , const unsigned short int len);
    //肖：设置机器人相关部分信息
    unsigned char setCommandAnalysis(const Command command_state , unsigned char* p , const unsigned short int len);
    //肖：组装要发送的信息
    void sendStruct(const Command command_state , unsigned char* p , const unsigned short int len);
    void sendMessage(void);//肖：组装要发送的信息包
};


#endif  // #ifndef HF_LINK_H

