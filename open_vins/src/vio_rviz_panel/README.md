# VIO RViz Panel

ROS Noetic RViz panel for displaying these image topics beside the OpenVINS 3D view:

- `/zed2i/zed_node/left/image_rect_color`
- `/senpai/seg_cls4_full`
- `/senpai/trajectory_plot`
- `/pedestrian_trajectory/image`

## Build

```bash
cd ~/open_vins_P
catkin_make --pkg vio_rviz_panel
source devel/setup.bash
```

## Run

The standard OpenVINS config already contains the panel:

```bash
rviz -d $(rospack find ov_msckf)/launch/display.rviz
```

Or use the convenience launch file:

```bash
roslaunch vio_rviz_panel dashboard.launch
```

The default RViz config creates four independent dock panels. Drag each panel by its title bar to arrange it, or double-click a title bar to float it. A useful dashboard layout is:

```text
+----------------------+----------------------+
| ZED2i camera         | Trajectory plot      |
|                      | (near-square)        |
+----------------------+----------------------+
| Semantic segmentation| RViz 3D view         |
| Pedestrian trajectory|                      |
+----------------------+----------------------+
```

Use `File > Save Config` after arranging the docks. Panel topics are configured in `display.rviz`.

Each input must be a base `sensor_msgs/Image` topic. The subscriber queue is one frame to keep the UI responsive.
