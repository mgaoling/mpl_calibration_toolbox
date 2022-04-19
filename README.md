# MPL Calibration Toolbox

[VECtor Benchmark](https://star-datasets.github.io/vector/) is the first complete set of benchmark datasets captured with a multi-sensor setup containing an event-based stereo camera, a regular stereo camera, multiple depth sensors, and an inertial measurement unit. The setup is fully hardware-synchronized and underwent accurate extrinsic calibration. All sequences come with ground truth data captured by highly accurate external reference devices such as a motion capture system. Individual sequences include both small and large-scale environments, and cover the specific challenges targeted by dynamic vision sensors.

This toolbox is a ROS workspace integrating with a set of easy-to-use calibration functions, including:

- [Intrinsic Calibration](https://github.com/mgaoling/mpl_calibration_toolbox#intrinsic-calibration)

# Getting Started

### Requirement and Dependency

- [Ubuntu 20.04](https://ubuntu.com/download/desktop)
- [ROS Noetic](http://wiki.ros.org/ROS/Installation) with **Desktop-Full Installation**
- [OpenCV 4.2](https://opencv.org/releases/)
- [Ceres Solver](http://ceres-solver.org/installation.html)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)

### Compile

```
cd ~/catkin_ws/src
git clone https://github.com/mgaoling/mpl_calibration_toolbox.git
cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash
```

# Intrinsic Calibration

### How to reproduce the result?

- Download the intrinsic data bag from the [Calibration Page on VECtor Benchmark](https://star-datasets.github.io/vector/calibration/#Intrinsics) and place it into the `data` folder. Here, we use the `right_event_camera_intrinsic_data.zip` file as an example. Decompress the file by:

```
roscd mpl_calibration_toolbox/data
unzip right_event_camera_intrinsic_data.zip
```

- Check and modify the parameters in the `config/intrinsic_calibration.yaml`, then launch the intrinsic calibration by:

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
