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

1. Copy this entire block and paste in the devcontainer.
```bash
cd ~/ros_ws
python3 -m venv venv --system-site-packages --symlinks
source venv/bin/activate
touch venv/COLCON_IGNORE
find src/cybership_software_suite -name "requirements*txt" -exec pip install -r {} \;
rosdep update
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
sudo apt update
sudo apt install ros-jazzy-rviz2 mesa-utils iproute2 joystick -y
source ~/ros_ws/venv/bin/activate
source ~/ros_ws/install/setup.bash
```

1. run "quick" start.
```bash
ros2 launch tmr4243_utilities utilities.simulation.launch.py
```
a gui application with the simulator should show up.

1. run joystick joystick node
```bash
ros2 run joy joy_node
```


# Joystick sharing inside docker
TLDR: share the usb device over the loopback connection and attach it inside the docker container.
Don't try any other workaround.

sources:
- https://wiki.archlinux.org/title/USB/IP
- https://bbs.archlinux.org/viewtopic.php?id=186112

## on host
```bash
sudo pacman -Syu usbip
systemctl start usbipd.service
sudo modprobe usbip_core
sudo modprobe usbip_host
sudo modprobe vhci-hcd # this is ment for the client, but since we share the kernel this needs to be run here as well.
```

```bash
usbip list -l
 - busid 1-2 (054c:09cc)
   Sony Corp. : DualShock 4 [CUH-ZCT2x] (054c:09cc)

 - busid 1-5 (8087:0a2b)
   Intel Corp. : Bluetooth wireless interface (8087:0a2b)

 - busid 1-7 (05c8:03c0)
   Cheng Uei Precision Industry Co., Ltd (Foxlink) : unknown product (05c8:03c0)
```
the first device is a playstation controller and it's bus id is `1-2`.

```bash
sudo usbip --debug bind --busid="1-2"
usbip: debug: usbip.c:129:[run_command] running command: `bind'
usbip: info: bind device on busid 1-2: complete
```
verify that you can connect to the device
```
usbip list -r 127.0.0.1
Exportable USB devices
======================
 - 127.0.0.1
        1-2: Sony Corp. : DualShock 4 [CUH-ZCT2x] (054c:09cc)
           : /sys/devices/pci0000:00/0000:00:14.0/usb1/1-2
           : (Defined at Interface level) (00/00/00)
```

## on client (inside the docker container)
if you're on Debian based host it should be enough with this:
```bash
sudo apt update
sudo apt install -y linux-tools-$(uname -r) # 🤡
```
However Ubuntu likes to ship 10 year old packages to `$(uname -r)` will never be up to date compared to Arch.

The plan is to mount the hosts binary `usbip` into a random directory inside the container. But since it's not a static binary we need to also mount it's dynamic libraries.
```
ldd /usr/bin/usbip # host
        linux-vdso.so.1 (0x0000763663c52000)
        /usr/lib/libinput-config.so (0x0000763663c38000)
        libusbip.so.0 => /usr/lib/libusbip.so.0 (0x0000763663c02000)
        libudev.so.1 => /usr/lib/libudev.so.1 (0x0000763663bbb000)
        libc.so.6 => /usr/lib/libc.so.6 (0x00007636639ca000)
        libcap.so.2 => /usr/lib/libcap.so.2 (0x00007636639be000)
        libgcc_s.so.1 => /usr/lib/libgcc_s.so.1 (0x0000763663990000)
        /lib64/ld-linux-x86-64.so.2 => /usr/lib64/ld-linux-x86-64.so.2 (0x0000763663c54000)
```
we'll create a fake root with directories `/usr/lib/` and `/usr/lib64` mount them from host to container and set `LD_LIBRARY_PATH` environment variable so that the binary looks for the libraries in the mounted directories not in the container.

```yaml
volumes:
      - /usr/bin/usbip:/archroot/usr/bin/usbip:ro
      - /usr/lib/libinput-config.so:/archroot/usr/lib/libinput-config.so:ro
      - /usr/lib/libusbip.so.0:/archroot/usr/lib/libusbip.so.0:ro
      - /usr/lib/libudev.so.1:/archroot/usr/lib/libudev.so.1:ro
      - /usr/lib/libcap.so.2:/archroot/usr/lib/libcap.so.2:ro
      - /usr/lib/libgcc_s.so.1:/archroot/usr/lib/libgcc_s.so.1:ro
      - /lib64/ld-linux-x86-64.so.2:/archroot/lib64/ld-linux-x86-64.so.2:ro
      - /usr/share/hwdata/:/archroot/usr/share/hwdata/ # needs to be writable
```
      
```bash
LD_LIBRARY_PATH=/archroot/usr/lib/:/archroot/lib/:/archroot/lib64/:/archroot/usr/lib64/ /archroot/usr/bin/usbip list -r 127.0.0.1
usbip: error: failed to open /usr/share/hwdata//usb.ids
Exportable USB devices
======================
 - 127.0.0.1
        1-2: unknown vendor : unknown product (054c:09cc)
           : /sys/devices/pci0000:00/0000:00:14.0/usb1/1-2
           : (Defined at Interface level) (00/00/00)
```

ended up here before going to bed:
```
LD_LIBRARY_PATH=/archroot/usr/lib/:/archroot/lib/:/archroot/lib64/:/archroot/usr/lib64/ sudo /archroot/usr/bin/usbip attach -r 127.0.0.1 -b "1-2"
/archroot/usr/bin/usbip: error while loading shared libraries: libusbip.so.0: cannot open shared object file: No such file or directory

LD_LIBRARY_PATH=/archroot/usr/lib/:/archroot/lib/:/archroot/lib64/:/archroot/usr/lib64/ ldd /archroot/usr/bin/usbip
        linux-vdso.so.1 (0x00007694d31e6000)
        libusbip.so.0 => /archroot/usr/lib/libusbip.so.0 (0x00007694d31c8000)
        libudev.so.1 => /archroot/usr/lib/libudev.so.1 (0x00007694d3181000)
        libc.so.6 => /lib/x86_64-linux-gnu/libc.so.6 (0x00007694d2f67000)
        libcap.so.2 => /archroot/usr/lib/libcap.so.2 (0x00007694d2f5b000)
        libgcc_s.so.1 => /archroot/usr/lib/libgcc_s.so.1 (0x00007694d2f2d000)
        /lib64/ld-linux-x86-64.so.2 (0x00007694d31e8000)

```

# Troubleshooting
## `could not connect to display :0`
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


