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
* luke liao       2016.3.29   V1.0           creat this file
*
* Description: define handfree transport base class
***********************************************************************************************************************/



#ifndef TRANSPORT_H_
#define TRANSPORT_H_

#include <iostream>
#include <inttypes.h>//：提供整数输入的各种转换宏
#include <vector>
#include <deque>
#include <queue>
#include <boost/asio.hpp>//：跟服务器有关，用于网络和底层I/O编程（串口通信）
#include <boost/function.hpp>//：实现了一个泛型的回调机制。它提供了函数指针、函数对象和成员函数指针的存储和后续的调用。
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>

//：C++的BOOST库中，通信库都在asio下

namespace handsfree_hw {

typedef std::vector<uint8_t> Buffer;

class Transport {
public:
	Transport(std::string url) :
		comm_url_(url),
		write_buffer_(),
		read_buffer_()
	{
		ios_ = boost::make_shared<boost::asio::io_service>();
	}

	virtual Buffer readBuffer() = 0;

	virtual void writeBuffer(Buffer &data) = 0;

	inline boost::shared_ptr<boost::asio::io_service> getIOinstace()
	{
		return ios_;
	}

	bool initialize_ok()//：初始化是否完成判断
	{
		return initialize_ok_;
	}
protected:
	// for communication location ：通讯地址
	std::string comm_url_;//串口地址
	std::queue<Buffer> write_buffer_;//：写缓冲区
	std::queue<Buffer> read_buffer_;//：读缓冲区

	

	bool initialize_ok_;

	// for boost asio service ：io_service表示程序到操作系统I/O服务的“连接”。该指针目的是防止该资源没有被正确的释放
	boost::shared_ptr<boost::asio::io_service> ios_;
	//：io_service对象是使用boost::asio库的必需要有的对象
};
//};

}



#endif /* TRANSPORT_BASE_H_ */
