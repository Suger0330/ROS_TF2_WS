# rosrun 实现静态坐标变换

## 当坐标系之间的相对位置固定时，那么所需参数也是固定的: 
* 父系坐标名称、子级坐标系名称、x偏移量、y偏移量、z偏移量、x 翻滚角度、y俯仰角度、z偏航角度。
## 实现逻辑相同，参数不同，那么 ROS 系统就已经封装好了专门的节点，使用方式如下:  
* `rosrun tf2_ros static_transform_publisher x偏移量 y偏移量 z偏移量 z偏航角度 y俯仰角度 x翻滚角度 父级坐标系 子级坐标系`
## 示例:
* `rosrun tf2_ros static_transform_publisher 0.2 0 0.5 0 0 0 /baselink /laser`
