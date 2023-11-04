# YOLOv8 ROS package

> [!NOTE]  
> This package is deprecated, we will use [object detection](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection) from Isaac ROS in the future.


## Nodes

There are 2 nodes in this package: `image_publisher` and `yolov8_node`.


### `image_publisher`

#### Function

- Publish a testing image to `/front_camera/image_raw`,  simulating the front ZED camera.

#### Type

- published data type: [sensor_msgs/Image](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)

### `yolov8_node`

#### Function

- Subscribe to `/front_camera/image_raw` and detect the door position with YOLO.
- Publish the detection messages to `/detections` topic.

#### Type

- subscribed data type: [sensor_msgs/Image](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)
- published data type: [Detection2DArray](https://docs.ros.org/en/lunar/api/vision_msgs/html/msg/Detection2DArray.html) (if no object detected the `detections` property will be an empty array)

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

```bash
rosrun yolov8_ros image_publisher.py # no need to run this line when using gazebo
rosrun yolov8_ros yolov8_node.py
```

or run the `image_publisher` and `yolov8_node` with roslaunch:

```
roslaunch yolov8_ros yolov8_launch.launch
```

### View

You can check the published result with:
```
rostopic echo /detections
```
