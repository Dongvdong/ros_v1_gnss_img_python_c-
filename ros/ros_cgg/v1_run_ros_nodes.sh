#!/bin/bash

#外部给与执行权限
#sudo chmod +x run_ros_nodes.sh

# 定义 ROS 安装路径  #安装时候添加到系统路径了 不需要每次都source
ROS_SETUP="/opt/ros/noetic/setup.bash" 
# 定义工作目录路径  自己的工程没有加到系统路径，每次需要source
WORKSPACE_DIR="/home/dongdong/2project/1salm/2Mycode/ros/ros_cgg"


#指定目录

# 启动 ROS Master
echo "Starting ROS 总结点..."
gnome-terminal -- bash -c "\
cd $WORKSPACE_DIR; source devel/setup.bash; \
roscore; \
exec bash"

# 等待 ROS Master 启动
sleep 5

# 运行 C++ 发布节点
echo "Running C++ 发布节点..."
gnome-terminal -- bash -c "\
cd $WORKSPACE_DIR; source devel/setup.bash; \
rosrun image_gps_package image_gps_publisher; \
exec bash"

# 运行 C++ 接受节点
echo "Running C++ 订阅节点..."
gnome-terminal -- bash -c "\
cd $WORKSPACE_DIR; source devel/setup.bash; \
rosrun image_gps_package image_gps_subscriber; \
exec bash"
