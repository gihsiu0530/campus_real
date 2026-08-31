# Hanxiang UAV OpenVINS Config

This folder is for the rosbag:

- `/image_raw`
- `/mavros/imu/data`
- `/mavros/global_position/global`
- `/mavros/local_position/velocity_local`

Run VIO as monocular:

```bash
source devel/setup.bash
roslaunch ov_msckf subscribe.launch config:=hanxiang_drone max_cameras:=1 use_stereo:=false dolivetraj:=false dosave:=true path_est:=/home/cyc/open_vins/vio_estimate_hanxiang_drone.csv
```

Play the bag separately:

```bash
rosbag play /home/cyc/open_vins/data/漢翔/2026_03_25-15_34_03_0.bag --clock
```

Required before trusting the result:

- Camera intrinsics and distortion for `/image_raw`.
- Image resolution and encoding.
- Camera optical frame to IMU frame transform, written as `T_imu_cam`.
- Camera to IMU time offset, `timeshift_cam_imu`.
- IMU noise densities and random walks.
- Confirmation whether `/mavros/imu/data` is raw IMU or already filtered/orientation-fused.
