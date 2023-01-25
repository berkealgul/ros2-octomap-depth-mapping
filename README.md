# Octomap Depth Mapping for ROS2

`octomap_depth_mapping` is a ROS2 package to create octomaps from depth images

## Dependencies

- ROS2 Foxy or above
- Octomap
- OpenCV
- cv_bridge
- message_filters
- CUDA Toolkit (optional, see details in usage section)

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

* `depth/rect` (sensor_msgs/Image)   : depth image [Markdown - Link](##Usage)
* `pose` (geometry_msgs/PoseStamped) : pose of camera relative to the world origin

























## About Image Data

