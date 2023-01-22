#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"

/*
    订阅方实现:
        需求1:换算出turtlel 相对于turtle2 的关系
        需求2:计算角速度和线速度并发布

    流程:
        1.包含头文件;
        2.编码、初始化、NodeHandle;
        3.创建订阅对象;
        4.编写解析逻辑;
        5.spinOnce();
*/

int main(int argc, char *argv[])
{
    // 2.编码、初始化、NodeHandle;
    
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"tfs_sub_c");
    ros::NodeHandle nh;
    // 3.创建订阅对象;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener sub(buffer);

    // A.创建发布对象
    ros::Publisher pub_v = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel",100);
    // 4.编写解析逻辑;

    ros::Rate rate(5);
    while (ros::ok())
    {
        
        // 核心代码
        try
        {
            // 1.计算turtle1与turtle2的相对关系
            /*
                A 相对于 B 的坐标系关系
                参数1:目标坐标系  B
                参数2:源坐标系    A
                参数3:ros::Time(0) 取间隔最短的两个坐标关系帧计算相对关系
                返回值:geometry msgs::TransformStamped 源相对于目标坐标系的相对关系
            */
            //--解析 turtle1 中的点相对于 turtle2 的坐标
            geometry_msgs::TransformStamped t1Tot2 = buffer.lookupTransform("turtle2","turtle1",ros::Time(0));
            // ROS_INFO("turtle1 相对于 turtle2 的坐标关系:父坐标系ID=%s",t1Tot2.header.frame_id.c_str());// turtle2
            // ROS_INFO("turtle1 相对于 turtle2 的坐标关系:子坐标系ID=%s",t1Tot2.child_frame_id.c_str());// turtle1
            // ROS_INFO("turtle1 相对于 turtle2 的坐标关系:x=%.2f,y=%.2f,z=%.2f",
            //             t1Tot2.transform.translation.x,
            //             t1Tot2.transform.translation.y,
            //             t1Tot2.transform.translation.z
            //         );
            
            // B.根据相对计算并组织速度消息
            geometry_msgs::Twist twist;
            /*
                组织速度,只需要设置线速度的x与角速度的z
                linear.x = 系数 * (y^2+x^2)^(0.5)
                angular.z = 系数 * arctan(对迅,邻边)
            */
            twist.linear.x = 0.5 * sqrt(pow(t1Tot2.transform.translation.x,2) + pow(t1Tot2.transform.translation.y,2));
            twist.angular.z = 4 * atan2(t1Tot2.transform.translation.y,t1Tot2.transform.translation.x);
            // C.发布
            pub_v.publish(twist);

        }
        catch(const std::exception& e)
        {
           // std::cerr << e.what() << '\n';
           ROS_INFO("异常信息:%s",e.what());
        }
        

        rate.sleep();
        ros::spinOnce();
    }
    
    // 5.spinOnce();
    
    return 0;
}
