#！ /usr/bin/env python

import rospy
import tf2_ros
# import tf
import tf_conversions 
# 替代 import tf,解决找不到transformations.quaternion_from_euler()的问题
from geometry_msgs.msg import TransformStamped
"""
    发布方:发布两个坐标系的相对关系(车辆底盘--- base_link 和雷达 --- laser)
    流程:
        1.导包;
        2.初始化节点;
        3.创建发布对象;
        4.组织被发布的数据;
        5.发布数据;
        6.spin().
"""

if __name__ == "__main__":
    # 2.初始化节点;
    rospy.init_node("static_pub_p")
    # 3.创建发布对象; --- 静态坐标广播器
    pub = tf2_ros.StaticTransformBroadcaster()
    # 4.组织被发布的数据;
    tfs = TransformStamped()
    # header --- 头信息
    tfs.header.stamp = rospy.Time.now()
    tfs.header.frame_id = "base_link"
    # child frame --- 子坐标系
    tfs.child_frame_id = "laser"
    # 相对关系(偏移量和四元数) --- 坐标系相对信息
    # ------ 偏移量
    tfs.transform.translation.x = 0.2
    tfs.transform.translation.y = 0.0
    tfs.transform.translation.z = 0.5
    # ------ 四元数
    # 4-1.先从欧拉角转换成四元数
    qtn = tf_conversions.transformations.quaternion_from_euler(0,0,0)
    # 4-2.再设置四元数
    tfs.transform.rotation.x = qtn[0]
    tfs.transform.rotation.y = qtn[1]
    tfs.transform.rotation.z = qtn[2]
    tfs.transform.rotation.w = qtn[3]
    # 5.发布数据;
    pub.sendTransform(tfs)
    # 6.spin().
    rospy.spin()
