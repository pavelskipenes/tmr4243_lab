# installation instructions using docker
These instructions were used to get a functional setup. These steps are documented just to reproduce a functional setup. They might be improved even further or automated into docker build process.

1. clone this reposetory with submodules to include cybership repo.
```bash
git clone get@github.com:/user/repo.git --recursive --init
```

1. allow local connections to x11 server [wiki](https://wiki.archlinux.org/title/Docker#Run_graphical_programs_inside_a_container).
```bash
xhost +local:
```

1. build the container.
```bash
systemctl start docker
docker compose build ros
docker compose run -it ros
```

1. Build a virtual environment.
```bash
cd ~/ros_ws
python3 -m venv venv --system-site-packages --symlinks
source venv/bin/activate
touch venv/COLCON_IGNORE
```

1. Install python dependencies to virtual environment.
```bash
find src/cybership_software_suite -name "requirements*txt" -exec pip install -r {} \;
```

1. Install ROS dependencies.
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

1. Build the workspace.
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

```bash
source ~/ros_ws/venv/bin/activate
source ~/ros_ws/install/setup.bash
sudo apt update
sudo apt install ros-jazzy-rviz2 mesa-utils iproute2 -y
source ~/ros_ws/venv/bin/activate
source ~/ros_ws/install/setup.bash
```

run "quick" start
```bash
ros2 launch tmr4243_utilities utilities.simulation.launch.py
```

# Troubleshooting
```
(venv) developer@c2f7f02ee0cb:~/ros_ws$ ros2 launch tmr4243_utilities utilities.simulation.launch.py
[INFO] [launch]: All log files can be found below /home/developer/.ros/log/2025-01-20-17-48-49-792173-c2f7f02ee0cb-3698
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [utility_node.py-1]: process started with pid [3702]
[INFO] [rviz2-2]: process started with pid [3703]
[INFO] [robot_state_publisher-3]: process started with pid [3704]
[INFO] [cybership_common.py-4]: process started with pid [3705]
[robot_state_publisher-3] [INFO] [1737395330.028069866] [enterprise.robot_state_publisher_c2f7f02ee0cb_be030aae705a]: Robot initialized
[rviz2-2] qt.qpa.xcb: could not connect to display :0
[rviz2-2] qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in "" even though it was found.
[rviz2-2] This application failed to start because no Qt platform plugin could be initialized. Reinstalling the application may fix this problem.
[rviz2-2]
[rviz2-2] Available platform plugins are: eglfs, linuxfb, minimal, minimalegl, offscreen, vnc, wayland-egl, wayland, wayland-xcomposite-egl, wayland-xcomposite-glx, xcb.
[rviz2-2]
[ERROR] [rviz2-2]: process has died [pid 3703, exit code -6, cmd '/opt/ros/jazzy/lib/rviz2/rviz2 -d /home/developer/ros_ws/install/cybership_viz/share/cybership_viz/config/enterprise.rviz --ros-args -r __node:=rviz_c2f7f02ee0cb_f19695adb7df -r __ns:=/enterprise'].
[utility_node.py-1] [INFO] [1737395330.857145947] [enterprise.utilities_c2f7f02ee0cb_d5346a6f282f]: Could not transform : "world" passed to lookupTransform argument target_frame does not exist.
```
Following error indicates that access to display server on host is fucked. Read the [wiki](https://wiki.archlinux.org/title/Docker#Run_graphical_programs_inside_a_container). default configuration for `firewalld` should not interfere.
