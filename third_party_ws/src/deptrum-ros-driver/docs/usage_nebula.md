# ROS Driver

The ROS Driver node exposes nebula sensor streams to ROS.

# Install and uninstall deptrum-ros-driver debian pakage
- `sudo dpkg -i ros-[ROS_DISTRO]-deptrum-ros-driver-nebula_[version]-0[Ubuntu DISTRIB_CODENAME]_amd64.deb` : Install deptrum-ros-driver
- `sudo dpkg --purge ros-[ROS_DISTRO]-deptrum-ros-driver-nebula`: Uninstall deptrum-ros-driver-nebula
## Launch Ros Driver Node

After the Ros2 Driver is installed, you can run this to launch the ros driver: `ros2 launch deptrum-ros-driver-nebula nebula_launch.py`

To set parameters, can modify launch file

 - `rgb_enable ` (default=`true`) : Enable or disable the rgb frame
 - `depth_enable ` (default=`true`) : Enable or disable the depth frame
 - `ir_enable ` (default=`true`) : Enable or disable the ir frame
 - `point_cloud_enable ` (default=`true`) : Generate a point cloud from depth data. Requires depth_enable
 - `flag_enable` (defalut=`true`): Enable or disable the flag frame
 - `usb_port_number` (default=``) : The device is created through USB port number,  you can set with usb port number;
 - `exposure_enable`(default=`false`):  Enable or disable to set camera exposure time
 - `exposure_time`(default=`1000`):  Set the exposure time for the camera(not set `exposure_enable` default AE)
 - `resolution_mode_index`(default=`0`):  Set Mode resolution index default `0`
 - `slam_mode`(default=`1`):  Set Mode stof or mtof default `1`,0-mtof and stof 1-mtof 2-stof
 - `rgbd_enable`(default=`false`):  get aligned rgb and depth
 - `mtof_filter_level`(default=`1`):  Set the mtof filter level 0:low  1:middle  2:high
 - `stof_filter_level`(default=`1`):  Set the stof filter level 0:low  1:middle  2:high
 - `boot_order`(default=`1`): Set multi-device boot order
 - `serial_number`(default=``): The serial_number of device
 - `update_file_path`(default=''): Set the path of update file
 - `log_dir`(default=`/tmp/`): Set the path of log file
 - `stream_sdk_log_enable`(default=`True`): Enable or disable stream-sdk log
 - `ir_fps`(default=25): Set the fps of ir
 - `stof_minimum_range`(default=10): Set the minimum_range of stof

## Topics

The node emits a variety of topics into its namespace.

- `mtof_rgb/image_raw` (`sensor_msgs::Image`) : The raw infrared image from the rgb camera sensor,when slam_mode is 0 or 1. 
- `stof_rgb/image_raw` (`sensor_msgs::Image`) : The raw infrared image from the rgb camera sensor,when slam_mode is 0 or 2. 
- `mtof_depth/image_raw` (`sensor_msgs::Image`) : The raw image from the depth camera, in 16UC1 format.when slam_mode is 0 or 1. 
- `stof_depth/image_raw` (`sensor_msgs::Image`) : The raw image from the depth camera, in 16UC1 format,when slam_mode is 0 or 2. 
- `ir/image_raw` (`sensor_msgs::Image`) : The raw infrared image from the ir camera sensor when slam_mode is 0 or 2.
- `flag/image_raw` (`sensor_msgs::Image`) : The flag data from the ir camera sensor,when slam_mode is 0 or 2.
- `stof_points2` (`sensor_msgs::PointCloud2`) : The point cloud generated by the Nebula SDK from the depth camera data，when slam_mode is 0 or 2. 
- `mtof_points2` (`sensor_msgs::PointCloud2`) : The point cloud generated by the Nebula SDK from the depth camera data，when slam_mode is 0 or 1.
- `ir/camera_info` (`sensor_msgs::CameraInfo`) : Calibration information for the ir camera, converted from Nebula SDK format.
- `log_dir` (default=`/tmp`) : The log path storage ;
- `serial_number`(`std_msgs::msg::String`): The serial number of device
## Visualization on rviz

Use rviz to visualize nebula topics mesages: `rosrun rviz rviz`

Set [Global Options] -> [Fixed Frame] = `depth_camera_link`

when open mtof:
Add [By topic] -> /mtof_depth/image_raw/image
               -> /mtof_rgb/image_raw/image
               -> /mtof_points2/PointCloud2
when open stof:
Add [By topic] -> /stof_depth/image_raw/image
               -> /stof_rgb/image_raw/image
               -> /ir/image_raw/image
               -> /stof_points2/PointCloud2
               -> /flag/image_raw/image

## Visualization on rviz config

Use rviz to visualize nebula topics mesages: `ros2 launch deptrum-ros-driver-nebula viewer_launch.py`

