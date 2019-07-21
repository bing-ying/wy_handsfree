cmake_minimum_required(VERSION 2.8.3)
project(handsfree_hw)

find_package(catkin REQUIRED COMPONENTS #：将组件导出的include路径，库等也附加到catkin_变量，为之后的操作提供方便
  controller_manager
  geometry_msgs
  hardware_interface
  realtime_tools
  roscpp
  std_msgs
  tf

  xm_msgs #加于1.9
  ecl_threads
)
#find_package(Boost REQUIRED COMPONENTS #加于1.9
 #asio      ：roscpp似乎是依赖boost库的，所以安装ros时会附带安装boost库
 #function    这里添加了roscpp的依赖，就没必要再添加boost库的依赖了
 #smart_ptr
# thread
#)
#

set(config_path \"${CMAKE_CURRENT_SOURCE_DIR}\")#7.23日为解决绝对路径问题
add_definitions(-DCONFIG_PATH=${config_path})


catkin_package(   
  INCLUDE_DIRS include  #：包的导出路径

  LIBRARIES ${PROJECT_NAME} #取消注释于并改于1.9 handsfree_hw
  CATKIN_DEPENDS  controller_manager geometry_msgs hardware_interface realtime_tools roscpp std_msgs xm_msgs#加于1.9
#：该项目依赖的其他catkin项目，即用catkin构建的其他功能包
#  DEPENDS system_lib
#：该项目依赖的其他非catkin包
 #  DEPENDS Boost
)

include_directories(
  include
  
#注释于1.9 放弃注释 只有直接包含在include文件中的才可以不需要声明，handsfree命名空间的类的头文件在包含时都直接加了文件夹前缀，所以在此处可以不用特殊声明
  include/sdk/LIB_HF_Link/
  include/sdk/LIB_Robot_Abstract/

  ${catkin_INCLUDE_DIRS}
 # ${Boost_INCLUDE_DIRS}#加于1.9
)

 set(xiao src/main.cpp src/transport_serial.cpp include/sdk/LIB_HF_Link/hf_link.cpp src/hf_hw.cpp src/hf_hw_ros.cpp)#加于1.10
 add_executable(${PROJECT_NAME}_node ${xiao})
 add_library(${PROJECT_NAME} STATIC ${xiao})#加于1.10
 #add_dependencies(handsfree_hw_node gencpp)#_gencpp改于1.9  已直接注释掉

 add_executable(testvelocity src/testvelocity.cpp)

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
  # ${Boost_LIBRARIES}
 )
 target_link_libraries(testvelocity ${catkin_LIBRARIES})



