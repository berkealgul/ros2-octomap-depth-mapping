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


## About Image Data

This package supports 8 or 16 bit greyscale images

`CV_U8C1` and `CV_16UC1` in OpenCV [literature](http://ninghang.blogspot.com/2012/11/list-of-mat-type-in-opencv.html) 

`mono8` and `mono16` in cv_bridge [literature](http://docs.ros.org/en/diamondback/api/cv_bridge/html/c++/namespacecv__bridge.html#a49fedf7e642d505557b866f6e307a034)

In addition, images are assumed to be rectified beforehand; thus no distortion parameters are needed for this package

## Cuda

By default cuda is not supported. In order to compile with cuda, uncomment line at `CMakeLists.txt` file

```cmake
# uncomment this line to use cuda
#set(USE_CUDA TRUE)
```

I developed this package with cuda toolkit 11.4 and this package supports with gpu [compute capabilities](https://developer.nvidia.com/cuda-gpus) between 3.5-8.7

