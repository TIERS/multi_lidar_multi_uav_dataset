<div align="center">
    <h1>Towards Robust UAV Tracking: A Multi-LiDAR Dataset for Indoor/Outdoor Navigation in GNSS-Denied Environments</h1>
    <a href="https://github.com/TIERS/dynamic_scan_tracking/blob/main/LICENSE"><img src="https://img.shields.io/github/license/PRBonn/kiss-icp" /></a>
    <a href="https://github.com/TIERS/uav_lidar_tracking_dataset/blob/main"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://github.com/TIERS/uav_lidar_tracking_dataset/blob/main"><img src="https://img.shields.io/badge/Windows-0078D6?st&logo=windows&logoColor=white" /></a>
    <a href="https://github.com/TIERS/uav_lidar_tracking_dataset/blob/main"><img src="https://img.shields.io/badge/mac%20os-000000?&logo=apple&logoColor=white" /></a>
    <br />
    <br />
    <a href="https://tiers.github.io/uav_lidar_tracking_dataset/">Project Page</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://arxiv.org/pdf/2304.12125.pdf">Paper</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/TIERS/uav_lidar_tracking_dataset/issues">Contact Us</a>
  <br />
  <br />
  <p align="center">
    <img src="doc/setup.png" width=99% />
  </p>

</div>

We present a novel multi-LiDAR dataset specifically designed for UAV tracking. Our dataset includes data from a spinning LiDAR, two solid-state LiDARs with different Field of View (FoV) and scan patterns, and an RGB-D camera. This diverse sensor suite allows for research on new challenges in the field, including limited FoV adaptability and multi-modality data processing. For a comprehensive list of sequences refer to the paper [Towards Robust UAV Tracking: A Multi-LiDAR Dataset for Indoor/Outdoor Navigation in GNSS-Denied Environments](https://arxiv.org/abs/2304.12125) and the [project page](https://tiers.github.io/uav_lidar_tracking_dataset) 

<hr />

## Calibration 

We provide a ROS package to compute the extrinsic parameters between LiDARs and camera based on GICP. As the OS1 has the largest FOV, it is treated as base reference frame ("base_link") in which all the other point clouds are transformed. For the Avia, Mid-360 and Realsense D435, we integrated the first five frames to increase point cloud density.

To use this package, play teh Calibration rosbag in our dataset:
~~~
rosbag play Calibration.bag -l
~~~
Then run our calibration launch file:
~~~
roslaunch uav_lidar_dataset lidars_extrinsic_computation.launch
~~~

The computed extrinsic parameters will appear in the terminal:
~~~
OS -> base_link 0 0 0 0 0 0 /os_sensor /base_link 10
Avia -> base_link  0.157506 0.0196177 -0.186776  3.14043 -3.10279 -3.14086 /avia_frame /base_link 10
Mid360 -> base_link   0.137913 -0.0687341  -0.192956 0.00511776  0.0395961   0.049123 /mid360_frame /base_link 10
Camera -> base_link 0 0 0 0 0 -0.786475 /camera_depth_optical_frame /base_link 10
~~~

## Install
The code has been tested on Ubuntu 20.04 with ROS Noetic

### Dependencies
- PCL

- Eigen

- Livox_ros_driver, Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).

### Build
```
  cd ~/catkin_ws/src
  git clone https://github.com/TIERS/uav_lidar_tracking_dataset
  cd ..
  catkin build
  ```

## Citation
If you use this dataset for any academic work, please cite the following publication:

```
@misc{catalano2023uav,
    title={UAV Tracking with Solid-State Lidars:Dynamic Multi-Frequency Scan Integration}, 
    author={Iacopo Catalano and Ha Sier and Xianjia Yu and Jorge Pena Queralta and Tomi Westerlund},
    year={2023},
    eprint={2304.12125},
    archivePrefix={arXiv},
    primaryClass={cs.RO}
}
```