# MPL Calibration Toolbox

[VECtor Benchmark](https://star-datasets.github.io/vector/) is the first complete set of benchmark datasets captured with a multi-sensor setup containing an event-based stereo camera, a regular stereo camera, multiple depth sensors, and an inertial measurement unit. The setup is fully hardware-synchronized and underwent accurate extrinsic calibration. All sequences come with ground truth data captured by highly accurate external reference devices such as a motion capture system. Individual sequences include both small and large-scale environments, and cover the specific challenges targeted by dynamic vision sensors.

This toolbox is a ROS workspace integrating with a set of easy-to-use calibration functions, including:

- [Camera Intrinsic Calibration](https://github.com/mgaoling/mpl_calibration_toolbox#camera-intrinsic-calibration)
- [IMU Intrinsic Calibration](https://github.com/mgaoling/mpl_calibration_toolbox#imu-intrinsic-calibration)
- [Joint Camera Extrinsic Calibration](https://github.com/mgaoling/mpl_calibration_toolbox#joint-camera-extrinsic-calibration)
- [Camera-IMU Extrinsic Calibration](https://github.com/mgaoling/mpl_calibration_toolbox#camera-imu-extrinsic-calibration)
- [Camera-MoCap Hand-eye Calibration](https://github.com/mgaoling/mpl_calibration_toolbox#camera-mocap-hand-eye-calibration)

# License and Citation

This toolbox, together with the [MPL Dataset Toolbox](https://github.com/mgaoling/mpl_dataset_toolbox) and the [k4a Projector](https://github.com/greatoyster/k4a_projector), is available as open-source under the terms of the [BSD-3-Clause-Clear License](https://github.com/mgaoling/mpl_calibration_toolbox/blob/main/LICENSE.txt). If you use this toolbox in an academic context, please cite the [publication](https://star-datasets.github.io/vector/assets/pdf/VECtor.pdf) as follows:

```bibtex
@Article{gao2022vector,
  author  = {Gao, Ling and Liang, Yuxuan and Yang, Jiaqi and Wu, Shaoxun and Wang, Chenyu and Chen, Jiaben and Kneip, Laurent},
  title   = {{VECtor}: A Versatile Event-Centric Benchmark for Multi-Sensor SLAM},
  journal = {IEEE Robotics and Automation Letters},
  pages   = {8217--8224},
  volume  = {7},
  number  = {3},
  year    = {2022},
  doi     = {10.1109/LRA.2022.3186770}
}
```

# Getting Started

The following instructions are tested on [Ubuntu 20.04](https://ubuntu.com/download/desktop) with [ROS Noetic](http://wiki.ros.org/ROS/Installation), a ROS **desktop-full installation** is therefore required. On top of that, the following libraries ([Eigen 3](https://eigen.tuxfamily.org/index.php?title=Main_Page), [OpenCV 4.2](https://opencv.org/releases/), [Ceres Solver 1.14.0](http://ceres-solver.org/installation.html), [yaml-cpp](https://github.com/jbeder/yaml-cpp)) should be installed from `APT` or you can just use `rosdep` to manage package dependencies as below.

```
cd ~/catkin_ws/src
git clone https://github.com/mgaoling/mpl_calibration_toolbox.git
cd ..
rosdep install --from-paths src --ignore-src -y # install dependencies
catkin_make
source ~/catkin_ws/devel/setup.bash
```

# Camera Intrinsic Calibration

### How to reproduce the result?

- Download the intrinsic data bag from the [Calibration Page on VECtor Benchmark](https://star-datasets.github.io/vector/calibration/), and place it into the `data` folder. Here, we use the `right_event_camera_intrinsic_data.zip` file as an example. Decompress the file by:

```
roscd mpl_calibration_toolbox/data
unzip right_event_camera_intrinsic_data.zip
```

- Check and modify the parameters in the `config/intrinsic_calibration.yaml`, then launch the camera intrinsic calibration by:

```
roslaunch mpl_calibration_toolbox intrinsic_calibration.launch
```

- Once the playback is over, click the `calibrate` button, and wait for the results to be displayed on the terminal.

### How to calibrate by yourself?

- Launch the ROS driver to publish data from the camera to ROS topics. 

- (Optional) If this is an event camera, install the [MPL Dataset Toolbox](https://github.com/mgaoling/mpl_dataset_toolbox), then launch the event visualization by:

```
roslaunch mpl_dataset_toolbox event_visualization.launch
```

- Open the `data/metavision_calibration_pattern_chessboard.html` file on another screen. (Optional) Click the `start` button if this is an event camera.

- Launch the [ROS Camera Calibration toolbox](http://wiki.ros.org/camera_calibration) by:

```
rosrun camera_calibration cameracalibrator.py --size 9x6 --square [square_length] image:=[image_topic] --no-service-check
```

- Move the checkerboard around in the camera frame. Click the `calibrate` button whenever you have collected enough data, and wait for the results to be displayed on the terminal.

**Note:** It is recommended to double-check the corner extraction among all recorded images. Delete the unwanted images if necessary, and then reproduce the results for better accuracy.

# IMU Intrinsic Calibration

### How to reproduce the result?

- Download the config file and the data bag from the [Calibration Page on VECtor Benchmark](https://star-datasets.github.io/vector/calibration/).

- Install the [Allan Variance ROS toolbox](https://github.com/ori-drs/allan_variance_ros), then launch the IMU intrinsic calibration by:

```
rosrun allan_variance_ros allan_variance imu_intrinsic_data_5hrs_recordings.bag imu_intrinsic_config.yaml
rosrun allan_variance_ros analysis.py --data allan_variance.csv
```

### How to calibrate by yourself?

Please refer to the [Allan Variance ROS toolbox](https://github.com/ori-drs/allan_variance_ros) for more details.

# Joint Camera Extrinsic Calibration

### How to reproduce the result?

- Download the joint camera extrinsic data bag and all related intrinsic results from the [Calibration Page on VECtor Benchmark](https://star-datasets.github.io/vector/calibration/), and place them into the `data` folder. Here, we use the `small_scale_joint_camera_extrinsic_data.zip` file as an example. Decompress the file by:

```
roscd mpl_calibration_toolbox/data
unzip small_scale_joint_camera_extrinsic_data.zip
```

- Check and modify the parameters in the `config/joint_extrinsic_calibration.yaml`, then launch the joint camera extrinsic calibration by:

```
roslaunch mpl_calibration_toolbox joint_extrinsic_calibration.launch
```

### How to calibrate by yourself?

- Launch the ROS driver to publish data from the camera to ROS topics. 

- (Optional) If this is an event camera, install the [MPL Dataset Toolbox](https://github.com/mgaoling/mpl_dataset_toolbox), then launch the event visualization by:

```
roslaunch mpl_dataset_toolbox event_visualization.launch
```

- Open the `data/metavision_calibration_pattern_chessboard.html` file on another screen.

- Run `rqt` in another terminal to display all the regular image frames, along with accumulated event frames if exist.

- Hold the sensor suite still in one place, save all the frames as images and place them into a folder following the same format as in `small_scale_joint_camera_extrinsic_data`. Repeat this process ten to twenty times at various places.

- Check and modify the parameters in the `config/joint_extrinsic_calibration.yaml`, then launch the joint extrinsic camera calibration by:

```
roslaunch mpl_calibration_toolbox joint_extrinsic_calibration.launch
```

# Camera-IMU Extrinsic Calibration

### How to reproduce the result?

- Download and decompress the config file and the data bag from the [Calibration Page on VECtor Benchmark](https://star-datasets.github.io/vector/calibration/).

- Install the [Kalibr toolbox](https://github.com/ethz-asl/kalibr), then launch the Camera-IMU extrinsic calibration by:

```
kalibr_calibrate_imu_camera --bag [data.bag] --cam cam.yaml --imu imu.yaml --target apriltag.yaml --timeoffset-padding 0.1
```

### How to calibrate by yourself?

Please refer to the [WIKI page on Kalibr toolbox](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration) for more details.

# Camera-MoCap Hand-eye Calibration

### How to reproduce the result?

- Download the Camera-MoCap extrinsic data bag and all related intrinsic results from the [Calibration Page on VECtor Benchmark](https://star-datasets.github.io/vector/calibration/), and place them into the `data` folder.

- Check and modify the parameters in the `config/camera_mocap_calibration.yaml`, then launch the Camera-MoCap extrinsic calibration by:

```
roslaunch mpl_calibration_toolbox camera_mocap_calibration.launch
```

### How to calibrate by yourself?

Please refer to the statement from the [Calibration Page on VECtor Benchmark](https://star-datasets.github.io/vector/calibration/). **Note:** Synchronization between the camera system and the MoCap system is rather crucial in order to achieve better accuracy.
