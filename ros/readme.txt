
#=============文件结构
ros_cgg/
├── build
├── devel
├── src
│   ├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
│   ├── image_gaosi
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── src
│   │       ├── image_gps_subscriber.py
│   │       └── image_pose_publisher.cpp
│   └── image_gps_package
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── src
│           ├── image_gps_publisher.cpp
│           ├── image_gps_publisher（复件）.cpp
│           └── image_gps_subscriber.cpp
├── v1_run_ros_nodes.sh
└── v2_run_ros_nodes.sh


#==============编译
cd ~/ros_cgg
catkin_make
source devel/setup.bash


#==============手动运行
#0开启source确保找到
cd ~/ros_cgg
source devel/setup.bash
# 手动开启三个窗口
#1启动ROS Master：
roscore
#2运行你的C++发布节点：
rosrun image_gps_package image_gps_publisher
#3运行c++接受节点
rosrun image_gps_package image_gps_subscriber


#============ 脚本运行
sudo chmod +x run_ros_nodes.sh