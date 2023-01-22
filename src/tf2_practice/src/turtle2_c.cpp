#include "ros/ros.h"
#include "turtlesim/Spawn.h"

/*
    需求:向服务端发送请求,生成一只新的小乌龟
    准备工作:
        1.服务话题 /spawn
        2.服务消息类型 turtlesim/Spawn
        3.运行前先启动 turtlesim_node 节点

    实现流程:
        1.包含头文件
          需要包含 turtlesim 包下资源，注意在 package.xml 配置
          在创建功能包时已经导入
        2.初始化 ROS 节点
        3.创建 ROS 句柄
        4.创建 service 客户端
        5.组织数据并发送
        6.处理响应

*/

int main(int argc, char *argv[])
{
    // 编码
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"service_turtle2_c");
    // 3.创建 ROS 句柄
    ros::NodeHandle nh;
    // 4.创建 service 客户端
    ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");
    // 5.组织数据并发送
    // 5.1等待服务启动
    // client.waitForExistence();
    ros::service::waitForService("/spawn");  
    // 5.2发送请求
    turtlesim::Spawn spawn;
    spawn.request.x = 1.0;
    spawn.request.y = 3.0;
    spawn.request.theta = 1.57;
    spawn.request.name = "turtle2";
    // 7.处理响应
    bool flag = client.call(spawn);// flag 接收响应状态,响应结果也会被设置进spawn对象
    if (flag)
    {
        ROS_INFO("新乌龟生成的名字为:%s",spawn.response.name.c_str());
    }
    else
    {
        ROS_INFO("乌龟生成失败！！！");
    }
    return 0;
}
