#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

/*
    发布方:需要订阅乌龟的位姿信息,转换成相对于窗体的坐标关系，并发布
    准备
        话题: /turtlel/pose
        消息: /turtlesim/Pose
    流程:
        1.包含头文件;
        2.设置编码、初始化、NodeHandle;
        3.创建订阅对象，订阅/turtlel/pose;
        4.回调函数处理订阅的消息:将位姿信息转换成坐标相对关系并发布(关注)
        5.spin()
*/

void doPose(const turtlesim::Pose::ConstPtr &pose){
    //获取位姿信息 ，转换成坐标系相对关系(核心) ,并发布
    //a.创建TF发布对象;
    static tf2_ros::TransformBroadcaster pub;
    //b.组织被发布的数据;
    geometry_msgs::TransformStamped tfs;
    tfs.header.frame_id = "world";
    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = "turtle1";
    //  |----坐标系相对信息设置(偏移量)
    tfs.transform.translation.x = pose->x;
    tfs.transform.translation.y = pose->y;
    tfs.transform.translation.z = 0;
    //  |--------- 四元数设置
    /*
        位姿信息中设有四元数,但是有个偏航角度,又已知乌龟是2D,没有翻滚与俯仰角度,所以可以得出乌龟
    的欧拉角: 0 0 theta.
    */
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,pose->theta);

    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();

    //C.发布.
    pub.sendTransform(tfs);

}
int main(int argc, char *argv[])
{
    // 2.设置编码、初始化、NodeHandle;
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"dynamic_pub_c");
    ros::NodeHandle nh;
    // 3.创建订阅对象，订阅/turtlel/pose;
    ros::Subscriber sub = nh.subscribe("/turtle1/pose",100,doPose);
    // 4.回调函数处理订阅的消息:将位姿信息转换成坐标相对关系并发布(关注)
    // 5.spin()
    ros::spin();
    return 0;
}
