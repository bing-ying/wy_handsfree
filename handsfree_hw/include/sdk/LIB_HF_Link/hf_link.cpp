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

#include "hf_link.h"
#include "stdio.h"
#include <string.h>

unsigned char HFLink::byteAnalysisCall(const unsigned char rx_byte)
{//肖：分析包的完整性，完整时分析包并执行所属操作
    unsigned char package_update=0;
    unsigned char receive_message_update=0;
    //肖：对接收时各种状态的检查并且做出相应修改
    receive_message_update=receiveFiniteStates(rx_byte);//Jump communication status
    if( receive_message_update ==1 )//肖：假如收到了一个效验通过的包
    {
        receive_message_update = 0;
        package_update=packageAnalysis();
        /*肖：对数据包进行分析，只要包没问题返回都是1*/
        if(package_update==1) receive_package_count++;
        return package_update;
    }
    return 0;
}

/***********************************************************************************************************************
* Function:     void HFLink::Receive_Analysis(unsigned char rx_data)
*
* Scope:        Analysis the data from hf_link host
*
* Description:
*
* Arguments:
*
* Return:
*
* Cpu_Time:    stm32f4+fpu(1 us)
*
* History:
* by   mawenke   2015.12.1   creat
***********************************************************************************************************************/
unsigned char HFLink::receiveFiniteStates(const unsigned char rx_data)
{/*肖：下面就是去除协议帧的函数,它是通过一个状态机来解析协议的
该状态机呈瀑布状，一步步向下执行，前一个状态为后面所有状态的前置条件，如果出现问题就回到FF1级等待状态*/
    switch (receive_state_)//肖：除非收到的是RECEIVE_CHECK，且效验通过，否则返回都为0
    {
    case WAITING_FF1:
        if (rx_data == 0xff)
        {/*xiao先将receive_check_sum置0，再加上目前的收到的数据rx_data,作为日后效验的凭据
        将收到信息长度等都置为0,同时将接收状态置为FF2级等待*/
            receive_state_ = WAITING_FF2;
            receive_check_sum_ =0;
            receive_message_length_ = 0;
            byte_count_=0;
            receive_check_sum_ += rx_data;
        }
        break;

    case WAITING_FF2:
        if (rx_data == 0xff)
        {
            receive_state_ = SENDER_ID;
            receive_check_sum_ += rx_data;
        }
        else
            receive_state_ = WAITING_FF1;
        break;

    case SENDER_ID:
    /*xiao此时rx_data就是sender_id*/
        rx_message_.sender_id = rx_data ;
        if (rx_message_.sender_id == friend_id)  //id check
        {
            receive_check_sum_ += rx_data;
            receive_state_ = RECEIVER_ID;
        }
        else
        {
            printf("error , the sender_ID is not my friend \n");
            receive_state_ = WAITING_FF1;
        }
        break;

    case RECEIVER_ID:
    /*xiao此时rx_data就是receiver_id*/
        rx_message_.receiver_id = rx_data ;
        if (rx_message_.receiver_id == my_id)  //id check
        {
            receive_check_sum_ += rx_data;
            receive_state_ = RECEIVE_LEN_H;
        }
        else
        {
            printf("error , the reciver_ID is not my_ID \n");
            receive_state_ = WAITING_FF1;
        }
        break;

    case RECEIVE_LEN_H:
    //xiao此时rx_data是数据长度高8位的信息，因封装上传时将它右移了8位，现在恢复一下
        receive_check_sum_ += rx_data;
        receive_message_length_ |= rx_data<<8;
        receive_state_ = RECEIVE_LEN_L;
        break;

    case RECEIVE_LEN_L:
    //xiao现在rx_data是数据长度低8位的信息，与之前的数据相或可以恢复成正确的数据长度
        receive_check_sum_ += rx_data;
        receive_message_length_ |= rx_data;
        rx_message_.length = receive_message_length_;
        receive_state_ = RECEIVE_PACKAGE;
        break;

    case RECEIVE_PACKAGE:
    //肖，只有收到了包才会将信息记载进data
        receive_check_sum_ += rx_data;
        rx_message_.data[byte_count_++] = rx_data;
        if(byte_count_ >= receive_message_length_)
        {//xiao为确保整个报的数据被读入，只有当byte_count_>= receive_message_length_才会跳转向RECEIVE_CHECK
            receive_state_ = RECEIVE_CHECK;
            receive_check_sum_=receive_check_sum_%255;
        }
        break;

    case RECEIVE_CHECK:
    //肖：对比发送和接收的校验和，如果不一致直接放弃此数据帧
        if(rx_data == (unsigned char)receive_check_sum_)
        {
            receive_check_sum_=0;
            receive_state_ = WAITING_FF1;
           // printf("receive a package \n");
            return 1 ;
        }
        else
        {
            printf("check sum error \n");
            receive_state_ = WAITING_FF1;
        }
        break;
    default:
        receive_state_ = WAITING_FF1;
    }

    return 0;
}

/***********************************************************************************************************************
* Function:     void HFLink::packageAnalysis_3WD(void)
*
* Scope:
*
* Description:
*
* Arguments:
*
* Return:
*
* Cpu_Time:    stm32f4+fpu(1 us)
*
* History:
***********************************************************************************************************************/
unsigned char HFLink::packageAnalysis(void)
{/*肖：分析完协议后，对从上位机和下位机间的交互命令进行分析，返回值有两种，1和0*/
    unsigned char analysis_state=0;
    void* single_command;

    command_state_ = (Command)rx_message_.data[0];

    if (hf_link_node_model== 0)  //the slave need to check the SHAKING_HANDS"s state
    {                                //肖：此处握手指的应是数据传输之间的协议，表示开始数据传输前的交互
        if(shaking_hands_state==0 && command_state_ != SHAKING_HANDS) //if not  shaking hands
        {
            sendStruct(SHAKING_HANDS  , (unsigned char *)single_command, 0);
            return 1;
        }
    }

    switch (command_state_)
    {//肖：处理命令，大致可以将处理分为设定数据（setCommandAnalysis）和上传数据（readCommandAnalysis）两个函数。
    case SHAKING_HANDS :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&my_robot->measure_global_coordinate , sizeof(my_robot->measure_global_coordinate));
        break;
 
    case SET_ROBOT_SPEED :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&my_robot->expect_robot_speed , sizeof(my_robot->expect_robot_speed));
        break;
 
    case READ_ROBOT_SPEED :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&my_robot->measure_robot_speed , sizeof(my_robot->measure_robot_speed));
        break;

    case READ_GLOBAL_COORDINATE :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&my_robot->measure_global_coordinate , sizeof(my_robot->measure_global_coordinate));
        break;
 
    // case READ_ROBOT_COORDINATE :
    //     analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&my_robot->measure_global_coordinate , sizeof(my_robot->measure_global_coordinate));
    //     break;
 
    // case CLEAR_COORDINATE_DATA :
    //     if (hf_link_node_model==0)
    //     {
    //         sendStruct(command_state_ , (unsigned char *)single_command , 0);
    //         analysis_state=1;
    //         receive_package_renew[(unsigned char)command_state_] = 1 ;
    //     }
    //     break;
 
   
    /*case SET_HEAD_1 :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&my_robot->expect_head1_state , sizeof(my_robot->expect_head1_state ));
        break;
 
    case READ_HEAD_1 :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&my_robot->measure_head1_state , sizeof(my_robot->measure_head1_state));
        break;
 
    case SET_HEAD_2 :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&my_robot->expect_head2_state, sizeof(my_robot->expect_head2_state));
        break;
 
    case READ_HEAD_2 :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&my_robot->measure_head2_state , sizeof(my_robot->measure_head2_state));
        break;
    */
    case SET_CLAW :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&my_robot->claw_state_, sizeof(my_robot->claw_state_));
        break;

    case SET_ARM_TOTAL :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&my_robot->expect_arm_state, sizeof(my_robot->expect_arm_state));
        break;
                 
    case READ_ARM_TOTAL :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&my_robot->measure_arm_state , sizeof(my_robot->measure_arm_state));
        break;
                 
    case PLAT_MOVE :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&my_robot->plat_height, sizeof(my_robot->plat_height));
        break;

    case READ_PLAT:
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&my_robot->plat_height_back, sizeof(my_robot->plat_height_back));
        break; 

//xiao
	case SET_WRIST :
		analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&my_robot->wrist_angle_, sizeof(my_robot->wrist_angle_));
		break;		
	case READ_WRIST :
		analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&my_robot->wrist_angle_, sizeof(my_robot->wrist_angle_));
		break;
    default :
        analysis_state=0;
        break; 

    }

    rx_message_.sender_id=0;    //clear flag
    rx_message_.receiver_id=0;
    rx_message_.length=0;
    rx_message_.data[0]=0;

    return analysis_state;//肖：只要包没问题,且假如收到CLEAR_COORDINATE_DATA时本机是下位机，此处返回都是1
}
//肖：执行读取命令
unsigned char HFLink::readCommandAnalysis(const Command command_state , unsigned char* p , const unsigned short int len)
{    
    if (hf_link_node_model==1)//肖：上位机收到读取的命令后，自动更新机器人的相关部件信息
    { // master   , means the slave feedback a package to master , and the master save this package
        if((rx_message_.length-1) != len)//肖：确认下位机上传的信息长度与机器人相关部件信息长度是否匹配
        {
           // printf("I'm a master , can not read the message from slave , the length is not mathcing to struct \n");
            return 0;
        }
        memcpy(p, &rx_message_.data[1] , len);//肖：用接收的命令更新机器人相关部件信息
        receive_package_renew[(unsigned char)command_state] = 1 ;//肖：更新相关接收标志
    }
    else if(hf_link_node_model==0)//肖：下位机收到读取命令后，自动上传相应的机器人部件信息
    { // slave   , means the master pub a read command to slave ,and the slave feedback the a specific info to him
        sendStruct(command_state  , p , len);
        //肖：上位机要求读取机器人相关部件信息，下位机包装好后发送回去，此时不用再管接收到的数据（rx_message）,所以不用效验
        receive_package_renew[(unsigned char)command_state] = 1 ;
    }
    return 1;
}
//肖：执行设置命令
unsigned char HFLink::setCommandAnalysis(const Command command_state , unsigned char* p , const unsigned short int len)
{//肖：第一个参数是接收到的命令，第二个参数是命令描述的相关机器人部件，第三个参数是该部件数据结构大小
    void* ack;//xiao这个ack对于上、下位机通信和对于电子组在调试晓萌时都是挺有用的，可以看到晓萌是否收到了对应的指令
    if (hf_link_node_model==1)
    { // master  , the slave can set the master's data ,so this code means received the slave's ack
        if(command_state==SHAKING_HANDS) {
            shaking_hands_state=1;  //wait he master send SHAKING_HANDS肖：假如接收的命令是请求握手，那么握手成功
         //   printf("received a SHAKING_HANDS commmand and the slave is waiting master send SHAKING_HANDS data ");
        }
        else//肖：上位机收到设置命令后，明白下位机已正常执行设置命令，确认收到ack，对于上位机会把除了SHAKING_HANDS外的所有设置命令当做ack
        {
            //ack Analysis
          //  printf("received a ack ");
        }
        receive_package_renew[(unsigned char)command_state] = 1 ;//肖：更新接收标志
    }
    else if(hf_link_node_model==0)//肖：下位机收到设置命令后，根据设置命令更新相应的机器人描述数据，即通过调整参数来控制运动，以此响应命令
    { // slave  , means the master pub a set command to slave ,and the slave save this package then feed back a ack
        if((rx_message_.length-1) != len)//肖：确定机器部件数据类型大小是否与收到的数据大小匹配
        {
           // printf("I'm a slave , can not read the message from master , the length is not mathcing \n");
            return 0;
        }
        //肖：从rx_message_.data[1]复制len个字节到p，p即机器人相关部件，此即为更新机器人相关数据
        memcpy(p , &rx_message_.data[1] , len);//肖：就算是握手命令，这一步也会被执行，更新测量的全局坐标系
        if(command_state==SHAKING_HANDS) shaking_hands_state=1;   //SHAKING_HANDS not need ack to master
        else sendStruct(command_state  , (unsigned char *)ack , 0); //ack , tell the master , i receive your set package肖：发送ack
        receive_package_renew[(unsigned char)command_state] = 1 ;  //update receive flag , and wait the cpu to deal
    }
    return 1;
}

/***********************************************************************************************************************
* Function:    void HFLink::sendCommand(Command command)
*
* Scope:       public
*
* Description: send a command or data to the friend_id
*              this function is olny belongs to master
*
*
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/

unsigned char HFLink::masterSendCommand(const Command command_state)
{//肖：更新机器人相关部件数据  这一部分是上位机对下位机发送指令，电子组掌握的代码中这个该函数并没有什么用

    //else  means master send a command to slave
    unsigned char analysis_state =1;
    void* single_command;
    if(hf_link_node_model != 1){return 0;}//肖：假如是下位机就直接退出，防止误调
    
    switch (command_state)
    {
    case SHAKING_HANDS :
        receive_package_renew[(unsigned char)command_state] = 0 ;
        shaking_hands_state = 1;
        sendStruct(command_state , (unsigned char *)&my_robot->measure_global_coordinate , sizeof(my_robot->measure_global_coordinate));
        break;
 
    case SET_ROBOT_SPEED :
        receive_package_renew[(unsigned char)command_state] = 0 ;
        sendStruct(command_state , (unsigned char *)&my_robot->expect_robot_speed , sizeof(my_robot->expect_robot_speed));
        break;
 
    case READ_ROBOT_SPEED :
        receive_package_renew[(unsigned char)command_state] = 0 ;
        sendStruct(command_state , (unsigned char *)single_command , 0);
        break;
 
    case READ_GLOBAL_COORDINATE :
        receive_package_renew[(unsigned char)command_state] = 0 ;
        sendStruct(command_state , (unsigned char *)single_command , 0);
        break;
 
    // case READ_ROBOT_COORDINATE :
    //     receive_package_renew[(unsigned char)command_state] = 0 ;
    //     sendStruct(command_state , (unsigned char *)single_command , 0);
    //     break;
 
    // case CLEAR_COORDINATE_DATA :
    //     receive_package_renew[(unsigned char)command_state] = 0 ;
    //     sendStruct(command_state , (unsigned char *)single_command , 0);
    //     break;
 
    /*case SET_HEAD_1 :
        receive_package_renew[(unsigned char)command_state] = 0 ;
        sendStruct(command_state , (unsigned char *)&my_robot->expect_head1_state , sizeof(my_robot->expect_head1_state ));
        break;
 
    case READ_HEAD_1 :
        receive_package_renew[(unsigned char)command_state] = 0 ;
        sendStruct(command_state , (unsigned char *)single_command , 0);
        break;
 
    case SET_HEAD_2 :
        receive_package_renew[(unsigned char)command_state] = 0 ;
        sendStruct(command_state , (unsigned char *)&my_robot->expect_head2_state, sizeof(my_robot->expect_head2_state));
        break;
 
    case READ_HEAD_2 :
        receive_package_renew[(unsigned char)command_state] = 0 ;
        sendStruct(command_state , (unsigned char *)single_command , 0);
        break;
*/
    case SET_CLAW :
        receive_package_renew[(unsigned char)command_state] = 0 ;
        sendStruct(command_state , (unsigned char *)&my_robot->claw_state_, sizeof(my_robot->claw_state_));
        break;

    case SET_ARM_TOTAL :
        receive_package_renew[(unsigned char)command_state] = 0 ;
        sendStruct(command_state , (unsigned char *)&my_robot->expect_arm_state, sizeof(my_robot->expect_arm_state));
        break;
                 
    case READ_ARM_TOTAL :
        receive_package_renew[(unsigned char)command_state] = 0 ;
        sendStruct(command_state , (unsigned char *)&my_robot->measure_arm_state , sizeof(my_robot->measure_arm_state));
        break;
                 
    case PLAT_MOVE :
        receive_package_renew[(unsigned char)command_state] = 0 ;
        sendStruct(command_state ,  (unsigned char *)&my_robot->plat_height, sizeof(my_robot->plat_height));      
        break;

    case READ_PLAT :
        receive_package_renew[(unsigned char)command_state] = 0 ;
        sendStruct(command_state ,  (unsigned char *)&my_robot->plat_height_back, sizeof(my_robot->plat_height_back));      
        break;

//肖更新于1.16
	case SET_WRIST :
        receive_package_renew[(unsigned char)command_state] = 0 ;
        sendStruct(command_state ,  (unsigned char *)&my_robot->wrist_angle_, sizeof(my_robot->wrist_angle_));
		break;		
	case READ_WRIST :
        receive_package_renew[(unsigned char)command_state] = 0 ;
        sendStruct(command_state , (unsigned char *)single_command , 0);
		break;
    default :
        analysis_state=0;
        break;

    }

    return analysis_state;//肖：假如传回为0，则说明命令种类未知
}

/***********************************************************************************************************************
* Function:    void HFLink::sendStruct(const Command command_state , unsigned char* p , const unsigned short int len)
*
* Scope:       private
*
* Description:
* len =0       send a Single command to the friend
*              if i am slave , it can be  feed back a ack to master or request instructions  like SHAKING_HANDS
*              if i am master , it can be some request instructions like READ_ROBOT_SYSTEM_INFO READ_xxx
*
*
* len>0 :      send a Struct command to the friend hf_link nodeif
*              if i am slave , then means feed back  a  struc(valid data) to master
*              if i am master , then means set a a  struc(valid data)to slave
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/
void HFLink::sendStruct(const Command command_state , unsigned char* p , const unsigned short int len)//组装所要发送的tx_message_，字符串级别
{
    tx_message_.sender_id = my_id;
    tx_message_.receiver_id = friend_id;
    tx_message_.length=len+1;
    tx_message_.data[0] = (unsigned char)command_state;
    if (len == 0)
    {
        //xiao如果len是0，则说明这是下位机发出的请求握手，此时数据区里没数据
    }
    else if(len > 0)
    {
        memcpy(&tx_message_.data[1] , p , len);//肖：从p所指位置拷贝len个字节到tx_message_.data[1]
    }
    sendMessage();
}

/***********************************************************************************************************************
* Function:    void HF_Link::sendMessage(void)
*
* Scope:
*
* Description:  send a message to hf_link node
*
* Arguments:
*
* Return:
*
* Cpu_Time:    stm32f4+fpu(1 us)
*
* History:
***********************************************************************************************************************/
void HFLink::sendMessage(void)//组装所要发送的信息包，比特级别
{//xiao将封装包的东西导入上传区
    unsigned short int tx_i;
    unsigned int check_sum_=0;
    //肖：发送缓冲区一次含有一个包，每次组装包会更新发送缓冲区,前两个信息都是0xff
    tx_buffer[0]=0xff;
    check_sum_ += 0xff;

    tx_buffer[1]=0xff;
    check_sum_ += 0xff;

    tx_buffer[2]=tx_message_.sender_id;//肖：发送者id
    check_sum_ += tx_buffer[2];

    tx_buffer[3]=tx_message_.receiver_id;//肖：接受者id
    check_sum_ += tx_buffer[3];

    tx_buffer[4]=(unsigned char)( tx_message_.length >> 8); //LEN_H
    check_sum_ += tx_buffer[4];

    tx_buffer[5]=(unsigned char)tx_message_.length;   //LEN_L
    check_sum_ += tx_buffer[5];

    for(tx_i=0; tx_i < tx_message_.length ; tx_i++)   //package
    {
        tx_buffer[6+tx_i]=tx_message_.data[tx_i];
        check_sum_ += tx_buffer[6+tx_i];
    }
    check_sum_=check_sum_%255;
    tx_buffer[6+tx_i] = check_sum_;

    tx_buffer_length = 7 + tx_i;

#if HF_LINK_NODE_MODEL==0
    if(hf_link_node_model==0){
        hFLinkSendBuffer(port_num , tx_buffer , tx_buffer_length);
    }
#endif

    send_packag_count++;//肖：发送包的数量+1
}

