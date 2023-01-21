#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h" //创建发布者对象需要
#include "geometry_msgs/TransformStamped.h" //用于传输坐标系相关位置信息
#include "tf2/LinearMath/Quaternion.h" //用于欧拉角和四元数的转换

/*
    需求:发布两个坐标系的相对关系

    流程:
        1.包含头文件;
        2.设置编码 初始化 ROS 节点 设置节点句柄;
        3.创建发布对象;
        4.组织被发布的消息;
        5.发布数据;
        6.spin().

*/
int main(int argc, char *argv[])
{
    // 2.设置编码 初始化 ROS 节点 设置节点句柄;
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"static_pub_c");
    ros::NodeHandle nh;

    // 3.创建发布对象;
    tf2_ros::StaticTransformBroadcaster pub;// 创建静态坐标转换广播器
    // 4.组织被发布的消息;
    geometry_msgs::TransformStamped tfs;// 创建坐标系信息
    //----设置头信息
    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = "base_link";// 相对坐标系关系中被参考的那一个
    //----设置子级坐标系
    tfs.child_frame_id = "laser";
    //----设置子级相对于父级的偏移量
    tfs.transform.translation.x = 0.2;
    tfs.transform.translation.y = 0.0;
    tfs.transform.translation.z = 0.5;
    //----设置四元数:将"欧拉角"数据转换成四元数
    tf2::Quaternion qtn;// 创建"四元数"对象
    //向该对象设置欧拉角，这个对象可以将欧拉角转换成四元数
    qtn.setRPY(0,0,0);// 参数(roll,pitch,yaw),单位是 弧度
    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();
    // 5.发布数据;
    pub.sendTransform(tfs);
    // 6.spin().
    ros::spin();

    
    return 0;

}
