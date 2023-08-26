# YOLOv8 ROS package

## Installation

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:NCTU-AUV/yolov8_ros.git
```

## Getting started

### Build

```
catkin_make
source devel/setup.bash
```

### Run

```
rosrun yolov8_ros image_publisher.py
rosrun yolov8_ros yolov8_node.py
```

or run the `image_publisher` and `yolov8_node` with roslaunch:

```
roslaunch yolov8_ros yolov8_launch.launch
```
