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
* Description: define handfree transport serial method
***********************************************************************************************************************/


#ifndef TRANSPORT_SERIAL_H_
#define TRANSPORT_SERIAL_H_

#include <handsfree_hw/transport.h>

namespace handsfree_hw {

class SerialParams {//：接口
public:
	std::string serialPort;//：串行接口
	unsigned int baudRate;//：波特率，表示每秒钟传送的符号的个数，是一个衡量符号传输速率的参数
	unsigned int flowControl;//：流量控制
	unsigned int parity;//：奇偶校验，是串口通信中一种简单的检错方式
	unsigned int stopBits;//：停止位数
	SerialParams() ://：默认构造函数
			serialPort(), baudRate(921600), flowControl(0), parity(0), stopBits(0)
	{
	}
	SerialParams(//：接收所有所需值为参数的构造函数
			std::string _serialPort,
			unsigned int _baudRate,
			unsigned int _flowControl,
			unsigned int _parity,
			unsigned int _stopBits
			) :
			serialPort(_serialPort),
			baudRate(_baudRate),
			flowControl(_flowControl),
			parity(_parity),
			stopBits(_stopBits)
	{
	}
};

class TransportSerial : public Transport {//：接口数据读写

public:
	TransportSerial ();
	TransportSerial (std::string url);//：url表示接口地址，该参数用来初始化基类

	virtual Buffer readBuffer();//：读缓冲,基类的实现	

	virtual void writeBuffer(Buffer &data);//：写缓冲，基类的实现	

private:
	//：一个指向boost::asio::serial_port类型的智能指针，串口通信由asio组件的serial_port类完成
	boost::shared_ptr<boost::asio::serial_port> port_;
	SerialParams params_;//：串口通信类的实例化
	// for async read
	//：数据类型为std::vector<uint8_t>的临时异步阅读缓冲区
	Buffer temp_read_buf_;
    //：控制一个线程执行mainrun()函数
	boost::thread thread_;
	// locks：互斥锁
	boost::mutex port_mutex_;//：端口锁
	boost::mutex write_mutex_;//：写入锁
	boost::mutex read_mutex_;//：读取锁

	bool initializeSerial();//：初始化串口
	void mainRun();//：从端口重复读取数据

	void start_a_read();//：用来从接口读取信息到读缓冲区
	void start_a_write();//：从写缓冲区提取信息写入到端口
	//：被start_a_read调用处理读取
	void readHandler(const boost::system::error_code &ec, size_t bytesTransferred);
	//：被start_a_write调用处理写入
	void writeHandler(const boost::system::error_code &ec);
};

}



#endif /* TRANSPORT_SERIAL_H_ */
