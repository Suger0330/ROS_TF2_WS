#! /usr/bin/env python

import rospy
from turtlesim.msg import Pose
import tf2_ros
from geometry_msgs.msg import TransformStamped
# import tf
import tf_conversions 
# 替代 import tf,解决找不到transformations.quaternion_from_euler()的问题
import sys

"""
    发布方:订阅乌龟的位姿信息，转换成坐标系的相对关系，再发布
    准备
        话题: /turtle1/pose
        类型: /turtlesim/Pose 
    流程:
        1.导包;
        2.初始化R0S节点;
        3.创建订阅对象; 
        4.回调函数处理订阅到的消息(核心)
        5.spin();

"""

# 接收乌龟名称的变量
turtle_name_p = ""

def doPose(pose):
    # 1.创建发布坐标系相对关系的对象
    pub = tf2_ros.TransformBroadcaster()
    # 2.将pose转换成坐标系相对关系消息
    tfs = TransformStamped()
    tfs.header.frame_id = "world"
    tfs.header.stamp = rospy.Time.now()
    # 修改2,子集坐标系名称----------------
    tfs.child_frame_id = turtle_name_p
    # 子集子级坐标系相对
    tfs.transform.translation.x = pose.x
    tfs.transform.translation.y = pose.y
    tfs.transform.translation.z = 0

    # 四元数
    #从欧拉角转换四元数
    """
        乌龟是2D的,不存在X上的翻滚Y,上的俯仰,只有Z上的偏航
        0 0 pose.theta
    """
    qfn = tf_conversions.transformations.quaternion_from_euler(0,0,pose.theta)
    tfs.transform.rotation.x = qfn[0]
    tfs.transform.rotation.y = qfn[1]
    tfs.transform.rotation.z = qfn[2]
    tfs.transform.rotation.w = qfn[3]

    # 3.发布
    pub.sendTransform(tfs)


if __name__ == "__main__":
    
    # 2.初始化R0S节点;
    rospy.init_node("pub_turtle_p")

    # 解析传入的参数(现在传入几个参数?文件全路径+传入的参数+节点名称+日志文件路径)
    if len(sys.argv) != 4:
        rospy.loginfo("参数个数不对")
        sys.exit(1)
    else:
        turtle_name_p = sys.argv[1]


    # 3.创建订阅对象;
    # 修改1,话题名称------------------
    sub = rospy.Subscriber(turtle_name_p + "/pose",Pose,doPose,queue_size=100) 
    # 4.回调函数处理订阅到的消息(核心)

    # 5.spin();
    rospy.spin()