# Gaze Correlation
![OS](https://img.shields.io/badge/OS-Ubuntu_18.04-orange.svg) ![ROS_2](https://img.shields.io/badge/ROS_2-Eloquent-brightgreen.svg)

This repository implements correlation of 3D gaze with geometric primitives in order to provide 3D point of gaze and information about the object of user's interest.
- [RGB-D Gaze](https://github.com/AndrejOrsula/rgbd_gaze)
- [Geometric Primitive Fitting](https://github.com/AndrejOrsula/gpf)


## Building

First, clone this repository into your favourite workspace. Then build all packages with [colcon](https://colcon.readthedocs.io/en/released/user/installation.html).
```bash
mkdir -p <awesome_ws>/src && cd <awesome_ws>/src
git clone https://github.com/AndrejOrsula/gaze_correlation
cd ..
colcon build --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

## Usage

First, source the ROS 2 global installation (if not done before).
```bash
source /opt/ros/eloquent/setup.bash
```

Then source the ROS 2 workspace overlay (if not done before).
```bash
source /path/to/awesome_ws/install/local_setup.bash
```

Finally, you can try it out.
```bash
ros2 launch rgbd_gaze rgbd_gaze.launch.py
ros2 launch gpf gpf.launch.py
ros2 launch gaze_correlation gaze_correlation.launch.py
```

## License
This project is licensed under [BSD 3-Clause License](LICENSE).
