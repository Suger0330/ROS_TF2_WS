#!/usr/bin/env python
"""  
    需求:
        现有坐标系统,父级坐标系统 world,下有两子级系统 turtle1,turtle2_p,
        turtle1 相对于 world,以及 turtle2_p 相对于 world 的关系是已知的,
        求 turtle1 与 turtle2_p中的坐标关系,又已知在 turtle1中一点的坐标,要求求出该点在 turtle2_p 中的坐标
    实现流程:   
        1.导包
        2.初始化 ROS 节点
        3.创建 TF 订阅对象,创建一依赖于 turtle1 的坐标点
        4.调用 API 求出 turtle1 相对于 turtle2_p 的坐标关系
        5.调用 API 求出该点在 turtle2_p 中的坐标
        6.spin()

"""
# 1.导包
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped,Twist
from tf2_geometry_msgs import tf2_geometry_msgs
import math

if __name__ == "__main__":

    # 2.初始化 ROS 节点
    rospy.init_node("control_turtle2_p")
    # 3.创建 TF 订阅对象
    # 3-1.创建缓存对象
    buffer = tf2_ros.Buffer()
    # 3-2.创建订阅对象(将缓存转入)
    listener = tf2_ros.TransformListener(buffer)

    # 创建速度发布对象
    pub = rospy.Publisher("/turtle2_p/cmd_vel",Twist,queue_size=1000)
    # 5.转换逻辑实现,调用tf封装的算法
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():

        try:
        # 4.调用 API 求出 turtle1 相对于 turtle2_p 的坐标关系
        
            """
                参数1:目标坐标系
                参数2:源坐标系
                参数3:rospy.Time(0) ---- 取时间间隔最近的两个坐标系帧
                (turtle1相对world与turtle2_p相对world)计算关系
                返回值: turtle1 与 turtle2_p 的坐标关系
            """
            #lookup_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
            tfs = buffer.lookup_transform("turtle2_p","turtle1",rospy.Time(0))
            rospy.loginfo("turtle1 与 turtle2_p 相对关系:")
            rospy.loginfo("父级坐标系:%s,子级坐标系:%s",
                            tfs.header.frame_id,
                            tfs.child_frame_id)
            rospy.loginfo("相对坐标:x=%.2f, y=%.2f, z=%.2f",
                            tfs.transform.translation.x,
                            tfs.transform.translation.y,
                            tfs.transform.translation.z)

            # 组织 Twist 消息
            twist = Twist()
            # 组织速度,只需要设置线速度的x与角速度的z
            # linear.x = 系数 * (y^2+x^2)^(0.5)
            # angular.z = 系数 * atan2(对边,邻边)
            twist.linear.x = 0.5 * math.sqrt(math.pow(tfs.transform.translation.y,2) + math.pow(tfs.transform.translation.x,2))
            twist.angular.z = 4 * math.atan2(tfs.transform.translation.y,tfs.transform.translation.x)

            # 发布消息
            pub.publish(twist)
        except Exception as e:
            rospy.logerr("错误提示:%s",e)

        rate.sleep()
    # 6.spin()    
    # rospy.spin()
