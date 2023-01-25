# Octomap Depth Mapping for ROS2

`octomap_depth_mapping` is a ROS2 package to create octomaps from depth images

## Dependencies

- ROS2 Foxy or above
- Octomap
- OpenCV
- cv_bridge
- message_filters
- CUDA Toolkit (optional, for [details](#Cuda))

## Usage

You can clone this repository into your `<your-ros2-workspace>/src` folder and build the workspace

```bash
git clone https://github.com/berkealgul/ros2-octomap-depth-mapping.git
```
To launch the package (dont forget to source the package)
```bash
ros2 launch octomap_depth_mapping mapping_launch.py
```

## Node: octodm_node

Main mapping and publishing node

### Published Topics

* `octomap_fullmap` (octomap_msgs/Octomap) : generated octomap (orginated at 0,0,0)

### Subscribed Topics 

* `depth/rect` (sensor_msgs/Image)   : rectified depth image see [here](#About-Image-Data) for details
* `pose` (geometry_msgs/PoseStamped) : pose of camera relative to the world origin

### Parameters
|Name|Default|Type|Description|
|---|---|---|---|
|`sensor_model/fx` | 524.0 | double | Camera fx calibration value |
|`sensor_model/fy` | 524.0 | double | Camera fy calibration value |
|`sensor_model/cx` | 316.8 | double | Camera cx calibration value |
|`sensor_model/cy` | 238.5 | double | Camera cy calibration value |
|`sensor_model/hit` | 0.7 | double | - |
|`sensor_model/miss`| 0.4 | double | - |
|`sensor_model/min` | 0.12 | double | - |
|`sensor_model/max` | 0.97 | double | -|

## About Image Data

This package supports 8 and 16 bit greyscale images

`CV_U8C1` and `CV_16UC1` in OpenCV [literature](http://ninghang.blogspot.com/2012/11/list-of-mat-type-in-opencv.html) 

`mono8` and `mono16` in cv_bridge [literature](http://docs.ros.org/en/diamondback/api/cv_bridge/html/c++/namespacecv__bridge.html#a49fedf7e642d505557b866f6e307a034)

In addition, images are assumed to be rectified beforehand; thus no distortion parameters are needed for this package

## Cuda

By default cuda is not supported. In order to compile with cuda, uncomment [line](https://github.com/berkealgul/ros2-octomap-depth-mapping/blob/1a8d29c2004f0891bf81fbf1937c6d8b9ced48cf/CMakeLists.txt#L18) at `CMakeLists.txt` file

```cmake
# uncomment this line to use cuda
#set(USE_CUDA TRUE)
```

This package developed with cuda toolkit 11.4 and supports with [gpu compute capabilities](https://developer.nvidia.com/cuda-gpus) 3.5 and above

To learn more about cuda device compatibility [look at this link](https://docs.nvidia.com/deploy/cuda-compatibility/index.html)