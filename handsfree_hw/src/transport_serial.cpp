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


#include <handsfree_hw/transport_serial.h>

namespace handsfree_hw {

TransportSerial::TransportSerial() :
    Transport("serial:///dev/ttyUSB0")
{
    params_.serialPort = "/dev/ttyUSB0";
    if (!initializeSerial())//：假如没有完成初始化则发送错误报告
    {
        std::cerr << "serial Transport initialize failed ,please check your system" <<std::endl;
        initialize_ok_ = false;
    } else
    {//：初始化成功则发送初始化已准备好，将继承自父类的initialize_ok_置为true
     //   std::cout << "transport initialize ready" <<std::endl;
        initialize_ok_ = true;
    }
}

TransportSerial::TransportSerial(std::string url) :
    Transport(url)
{    //：comm_url基类成员string，substr（）方法返回一个子字符串
    if (comm_url_.substr(0, comm_url_.find("://")) != "serial")//：除了"serial"，还有"udp"、"tcp"协议
    {
        std::cerr << "url error, please correct your config" <<std::endl;
        return ;//：本代码块作用为一个对配置文件的检查
    }
    //：赋值为基类的串行接口                                                
    params_.serialPort = comm_url_.substr(comm_url_.find("://")+ 3, comm_url_.length() - comm_url_.find("://"));
    if (!initializeSerial())
    {
        std::cerr << "serial Transport initialize failed ,please check your system" <<std::endl;
        initialize_ok_ =  false;
    }
    else
    {
       // std::cout << "transport initialize ready" <<std::endl;
        initialize_ok_ =  true;
    }
}

void TransportSerial::mainRun()//重复从底层读取data型数据包
{
    //std::cout << "Transport main read/write started" <<std::endl;
    start_a_read();
    ios_->run();//：阻塞执行事件循环
}

//：用来从接口读取信息
void TransportSerial::start_a_read()
{
    boost::mutex::scoped_lock lock(port_mutex_);//：锁定端口

    //：serial_port类的成员函数，读取串口信息
    port_->async_read_some(boost::asio::buffer(temp_read_buf_),
        //：此处buffer为boost库的函数，一般参数都要buffer一下，该参数是一个临时异步阅读缓冲区
                           boost::bind(&TransportSerial::readHandler,
                                       this,
                                       boost::asio::placeholders::error,//：处理错误
                                       boost::asio::placeholders::bytes_transferred//：配合bind（）的一种占位符
                                       ));
}

//：用来处理读取
void TransportSerial::readHandler(const boost::system::error_code &ec, size_t bytesTransferred)
{
    if (ec)
    {
        std::cerr << "Transport Serial read Error "<< std::endl;
        return;
    }

    boost::mutex::scoped_lock lock(read_mutex_);//：锁定读取
    //：从临时异步阅读缓冲区读取数据
    Buffer data(temp_read_buf_.begin(), temp_read_buf_.begin() + bytesTransferred);
    read_buffer_.push(data);//read_buffer_用来存储读上来的data包 ：向读入缓冲区输入数据
    start_a_read();
}

void TransportSerial::start_a_write()
{
    boost::mutex::scoped_lock lock(port_mutex_);//：锁定端口

    if (!write_buffer_.empty())//：判读写缓冲区是否为空
    {//：如果不为空则从写缓冲区将数据写入端口，并弹出
        boost::asio::async_write(*port_, boost::asio::buffer((write_buffer_.front())),
                                 boost::bind(&TransportSerial::writeHandler, this, 
                                 boost::asio::placeholders::error));//：处理错误
        write_buffer_.pop();
    }


}

//：用来处理写入
void TransportSerial::writeHandler(const boost::system::error_code &ec)
{
    if (ec)
    {
        std::cerr << "Transport Serial write Error "<< std::endl;
        return;
    }

    boost::mutex::scoped_lock lock(write_mutex_);//：锁定写入
    //：只要写入缓冲区不为空就继续写入
    if (!write_buffer_.empty())	start_a_write();
}

//：从缓冲区读取数据并返回
Buffer TransportSerial::readBuffer()
{
    boost::mutex::scoped_lock lock(read_mutex_);//：锁定读取
    /*xiao:智能锁在该类对象构造时自动调用相应方法，锁住资源，该语句块终止时
    会自动调用其析构函数，释放资源*/
    //：假如读取缓冲区有数据就拷贝第一个元素，并且将其从缓冲区弹出
    if (!read_buffer_.empty())
    {
        Buffer data(read_buffer_.front());
        read_buffer_.pop();
        return data;//：返回读取的数据，假如缓冲区为空则返回也为空
    }
    Buffer data;
    return data;
}

void TransportSerial::writeBuffer(Buffer &data)
{
    boost::mutex::scoped_lock lock(write_mutex_);//：锁定写入

    write_buffer_.push(data);//：先向写入缓冲区写入数据，再往串口中写入
    start_a_write();
}

/*：关于上述写入和读取的总结，读取操作分为两种，一是从端口中读取数据到读取缓冲区，二是从读取缓冲区读取并返回
  写入操作也分为两种，一是向端口写入数据，二是接收一个数据并向写缓冲区写入*/
  
bool TransportSerial::initializeSerial()
{
    try//初始化串口的传输参数
    {
        std::cout<<params_.serialPort <<std::endl;
        //：第一个参数是io_service对象，第二个参数是串口名，ref函数的作用是返回对象的引用
        //：该步操作的目的是打开一个串口，打开后，该串口就被当做流来使用
        port_ = boost::make_shared<boost::asio::serial_port>(boost::ref(*ios_), params_.serialPort);
        //：利用串口类初始化串口
        port_->set_option(
                    boost::asio::serial_port::baud_rate(params_.baudRate));
        port_->set_option(
                    boost::asio::serial_port::flow_control((boost::asio::serial_port::flow_control::type)params_.flowControl));
        port_->set_option(
                    boost::asio::serial_port::parity((boost::asio::serial_port::parity::type)params_.parity));
        port_->set_option(
                    boost::asio::serial_port::stop_bits((boost::asio::serial_port::stop_bits::type)params_.stopBits));
        port_->set_option(boost::asio::serial_port::character_size(8));//：数据位

    }
    catch(std::exception &e)
    {
        std::cerr << "Failed to open the serial port " << std::endl;
        std::cerr << "Error info is "<< e.what() << std::endl;
        return false;
    }

    temp_read_buf_.resize(1024, 0);//：改变临时读缓冲区大小，增加的部分用0填充
    try
    {
        thread_ = boost::thread(boost::bind(&TransportSerial::mainRun, this));//用一个线程执行mainrun()函数
    }
    catch(std::exception &e)
    {
        std::cerr << "Transport Serial thread create failed " << std::endl;
        std::cerr << "Error Info: " << e.what() <<std::endl;
        return false;
    }

    return true;
}

}
