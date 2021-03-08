# one_line_lidar_camera_fusion

This is a rospack which calculate the distance of the objects detected by darknet using single line lidar. It is an important part of the autonomous system which send position informations of the objects to the planners (e.g. motion, behavior), so that the planners can plan safe trajectories and behaviors. And to ensure the vehicle will not hitting on any object.

The node needs 2 main_sub_modules to run

- object detection module(in this case, the darknet is used as object detection module)
- LiDar-camera fusion module



## Design

The node intakes 2 rosmsgs, namely:

1. /image_view/output (from usbcam or a rosbag)
2. /scan (from WJ_718 single line liadr)

to run this node:
1. modify the "config_path_" to your current working path
2. catkin_make
3. roslaunch one_line_lidar_camera_fusion one_line_lidar_camera_fusion.launch

## How to build

### dependencies

darknet_ros, perception_msgs, darknet_ros_msgs, pcl, pcl_ros, Eigen, hdmap etc.

目标检测模块采用darknet_ros，因此需首先配置编译darknet_ros

### build

~~~bash
mkdir one_line_lidar_camera_fusion && cd perception
mkdir src && cd src
git clone ${this repo}
cd ..
~$ catkin make
~$ source devel/setup.bash
~$ roslaunch one_line_lidar_camera_fusion one_line_lidar_camera_fusion.launch
~~~

## config

the exterior parameters between the camera and the LiDar is is the directory as follows:

~~~
one_line_lidar_camera_fusion/src/one_line_lidar_camera_fusion/config
~~~

it could be adjusted according to the working conditions.





















