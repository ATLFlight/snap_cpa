# snap_cpa
#### Camera Parameter Adjustment for Qualcomm Flight Board

This ROS package contains a node and nodelet implementation of MV CPA api.

### Getting Started

Clone this repo into your catkin_ws on your flight board:

```bash
cd ~/catkin_ws/src
git clone https://github.com/ATLFlight/snap_cpa.git
rosdep install . --from-paths
```

If you are using QFlight Pro, please follow build instructions here: [QFlight Pro Buildi Instructions](https://github.com/ATLFlight/QFlightProDocs/blob/master/RosSoftware.md).  You will also not need to run the rosdep install line.

This package is primarily target at use with [snap_vio](https://github.com/ATLFlight/snap_vio); however, it can be used with [snap_cam_ros](https://github.com/ATLFlight/snap_cam_ros) alone.  To test CPA, use the launch/downward_cpa.launch file.  You should now see gain and exposure adjusted based on total image luminance via the dynamic_reconfigure api of snap_cam_ros.
