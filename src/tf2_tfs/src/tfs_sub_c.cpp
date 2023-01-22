#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"

/*
    订阅方实现: 1.计算son1与son2的相对关系 2.计算son1的中某个坐标点在son2中的坐标值
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
    // 4.编写解析逻辑;

    // 创建坐标点
    geometry_msgs::PointStamped psAtSon1;

    psAtSon1.header.frame_id = "son1";
    psAtSon1.header.stamp = ros::Time::now();

    psAtSon1.point.x = 1.0;
    psAtSon1.point.y = 2.0;
    psAtSon1.point.z = 3.0;
    ros::Rate rate(5);
    while (ros::ok())
    {
        
        // 核心代码
        try
        {
            // 1.计算son1与son2的相对关系
            /*
                A 相对于 B 的坐标系关系
                参数1:目标坐标系  B
                参数2:源坐标系    A
                参数3:ros::Time(0) 取间隔最短的两个坐标关系帧计算相对关系
                返回值:geometry msgs::TransformStamped 源相对于目标坐标系的相对关系
            */
            //--解析 son1 中的点相对于 son2 的坐标
            geometry_msgs::TransformStamped son1Toson2 = buffer.lookupTransform("son2","son1",ros::Time(0));
            ROS_INFO("Son1 相对于 Son2 的坐标关系:父坐标系ID=%s",son1Toson2.header.frame_id.c_str());// son2
            ROS_INFO("Son1 相对于 Son2 的坐标关系:子坐标系ID=%s",son1Toson2.child_frame_id.c_str());// son1
            ROS_INFO("Son1 相对于 Son2 的坐标关系:x=%.2f,y=%.2f,z=%.2f",
                        son1Toson2.transform.translation.x,
                        son1Toson2.transform.translation.y,
                        son1Toson2.transform.translation.z
                    );
            // 2.计算son1的中某个坐标点在son2中的坐标值
            geometry_msgs::PointStamped psAtSon2 = buffer.transform(psAtSon1,"son2");
            ROS_INFO("坐标点在son2中的值(%.2f,%.2f,%.2f)",
                        psAtSon2.point.x,
                        psAtSon2.point.y,
                        psAtSon2.point.z);
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
