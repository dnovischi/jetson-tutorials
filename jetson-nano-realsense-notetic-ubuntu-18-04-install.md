# Realsense D435i Installation for ROS Noetic on Ubuntu 18.04 

In this tutorial I will describe the steps needed to fully install the Realsense D435i on the official Jetson Nano Ubuntu 18.04 distribution, with the latest Nvidia Jetpack installed based on a ROS Noetic deployment. If you don't have such a distribution installed, please complete the [Jetson Nano Ubuntu 18.04 Full Install](jetson-nano-ubuntu-18-04-install.md) and the [ROS Noetic Install on Jetson Nano with Ubuntu 18.04 (python3.6)](jetson-nano-ros-noetic-ubuntu-18-04-install.md) tutorials.

The tutorial assumes that the aforementioned installs are done and details the steps to properly install, [librealsense](https://github.com/IntelRealSense/librealsense), [realsense-ros warpper](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy) and the corresponding [realsense firmare](https://dev.intelrealsense.com/docs/firmware-releases).

Please note that the ROS Realsense warpper 2.3.2 that is compatible with ROS Noetic [2] requires librealsense version 2.50.0 [7]. While the recommaned Realsense D435i Ffirmware version is 5.13.0.50 [3].

## Librealsense Installation

1. Disconnect the Realsense USB from the Jetson Nano.
2. Make sure your Jetson is up to date

```bash
sudo apt-get update
sudo apt-get upgrade
```

3. Librealsense with CUDA (10.2) support requires us to build the library with version 8 for gcc and g++. If your followed [Jetson Nano Ubuntu 18.04 Full Install](ubuntu-18-04-install.md) you can simply select them by:

```bash
sudo update-alternatives --config gcc
```

```bash
[sudo] password for jetson:
There are 4 choices for the alternative gcc (providing /usr/bin/gcc).

  Selection    Path             Priority   Status
------------------------------------------------------------
* 0            /usr/bin/gcc-11   11        auto mode
  1            /usr/bin/gcc-10   10        manual mode
  2            /usr/bin/gcc-11   11        manual mode
  3            /usr/bin/gcc-7    7         manual mode
  4            /usr/bin/gcc-8    8         manual mode

Press <enter> to keep the current choice[*], or type selection number: 4
```

```bash
sudo update-alternatives --config g++
```

```bash
There are 4 choices for the alternative g++ (providing /usr/bin/g++).

  Selection    Path             Priority   Status
------------------------------------------------------------
* 0            /usr/bin/g++-11   11        auto mode
  1            /usr/bin/g++-10   10        manual mode
  2            /usr/bin/g++-11   11        manual mode
  3            /usr/bin/g++-7    7         manual mode
  4            /usr/bin/g++-8    8         manual mode

Press <enter> to keep the current choice[*], or type selection number: 4
```

4. Set the path for CUDA compiler by editing the `~/.bashrc` file:

```bash
nano ~/.bashrc
```

and append the following lines to the end of the file:

```bash
# cuda 10.2
export CUDA_HOME=/usr/local/cuda
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64
export PATH=$PATH:$CUDA_HOME/bin
```

then source the file:

```bash
source ~/.bashrc
```

5. Download the librealsense 2.50.0 realease and uzip the archive:

```bash
wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.50.0.zip

unzip v2.50.0.zip
```

6. Navigate to `librealsense-2.50.0`:

```bash
cd librealsense-2.50.0
```

7. You can try to run the L45 patch script, but it will fail since the latest jetpack is not supported. No worries, it will still work fine aside from a warining in ROS from the ROS realsense warpper:

```bash
./scripts/patch-realsense-ubuntu-L4T.sh
```

```bash
The script patches and applies in-tree kernel modules required for Librealsense SDK

Remove all RealSense cameras attached. Hit any key when ready

[sudo] password for jetson:
Reading package lists... Done
Building dependency tree
Reading state information... Done
build-essential is already the newest version (12.4ubuntu1).
build-essential set to manually installed.
git is already the newest version (1:2.17.1-1ubuntu0.13).
0 upgraded, 0 newly installed, 0 to remove and 13 not upgraded.
200
Jetson Board (proc/device-tree/model): NVIDIA Jetson Nano 2GB Developer Kit
Jetson L4T version: 32.7.3
Unsupported JetPack revision 32.7.3 aborting script

```

8. Install the librealsense build prerequisites:

```bash
sudo apt-get install git libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev -y
```

9. Install the udev Realsense access rules:

```bash
./scripts/setup_udev_rules.sh
```

10. Create a `build` directory and navigate to it:

```bash
mkdir build && cd build
```

11. Build and install librealsense:

```bash
cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=release -DFORCE_RSUSB_BACKEND=false -DBUILD_WITH_CUDA=true && make -j$(($(nproc)-1)) && sudo make install
```

## Realsense ROS Warpper Install

1. Make sure your Jetson is up to date

```bash
sudo apt-get update
sudo apt-get upgrade
```

2. Will be using gcc and g ++ version 10 to build the realsense-ros warpper. If your followed [Jetson Nano Ubuntu 18.04 Full Install](ubuntu-18-04-install.md) you can simply select them by:

```bash
sudo update-alternatives --config gcc
```

```bash
[sudo] password for jetson:
There are 4 choices for the alternative gcc (providing /usr/bin/gcc).

  Selection    Path             Priority   Status
------------------------------------------------------------
* 0            /usr/bin/gcc-11   11        auto mode
  1            /usr/bin/gcc-10   10        manual mode
  2            /usr/bin/gcc-11   11        manual mode
  3            /usr/bin/gcc-7    7         manual mode
  4            /usr/bin/gcc-8    8         manual mode

Press <enter> to keep the current choice[*], or type selection number: 1
```

```bash
sudo update-alternatives --config g++
```

```bash
There are 4 choices for the alternative g++ (providing /usr/bin/g++).

  Selection    Path             Priority   Status
------------------------------------------------------------
* 0            /usr/bin/g++-11   11        auto mode
  1            /usr/bin/g++-10   10        manual mode
  2            /usr/bin/g++-11   11        manual mode
  3            /usr/bin/g++-7    7         manual mode
  4            /usr/bin/g++-8    8         manual mode

Press <enter> to keep the current choice[*], or type selection number: 1
```

## Realsense Firmware Install
1. Make sure you have PC (x86_64) with Ubuntu 20.04 and Realsense properly installed.
2. Download the on the PC, the Realsense Firmware 5.13.0.50 [3] for librealsense SDK 2.50.0:
```bash
wget https://www.intelrealsense.com/download/19295/?_ga=2.152892921.1994987032.1673174169-2104036828.1672302692
```
3. Unzip the archive you just downloaded:
```bash
unzip D400_Series_FW_5_13_0_50.zip
```
4. Fire up `realsense-viewer`:
```bash
realsense-viewer
```
5. Connect the Realsense D435i to an USB3.2 of your PC
6. Close all the module streams (i.e Stero Module, RGB Camera and Motion Module)
7. Click on the "More" Icon for your Realsense, located in the menu on the lefthand side.
8. Select "Update Firmware" from the drop-down menu that appears.
9. Browse to the `D400_Series_FW_5_13_0_50` directory of the firmware file you unziped in step 3.
10. Select the `Signed_Image_UVC_5_13_0_50.bin` firmware and click "Ok".
11. Let the process finish, then your done here.

## Realsense ROS Stream Reolutions Configuration Options

To provide the configuration options when you launch the realsense warpper the argument format is as follows:

```bash
roslaunch realsense2_camera rs_camera.launch <stream-name>_width:=option <stream-name>_height:=option <stream-name>_fps:=option
```

For example to set the RGB camera color stream to run at 320x180 @ 30fps you can do the following:

```bash
roslaunch realsense2_camera rs_camera.launch color_width:=320 color_height:=180 color_fps:=30
```

### Infrared Stream

```bash
Infrared 2 1280x800 @ 30Hz Y8
Infrared 1 1280x800 @ 30Hz Y8
Infrared 1 1280x800 @ 25Hz Y16
Infrared 2 1280x800 @ 25Hz Y16
Infrared 2 1280x800 @ 15Hz Y16
Infrared 1 1280x800 @ 15Hz Y16
Infrared 1 1280x800 @ 15Hz Y8
Infrared 2 1280x800 @ 15Hz Y8
Infrared 1 1280x720 @ 30Hz Y8
Infrared 2 1280x720 @ 30Hz Y8
Infrared 2 1280x720 @ 15Hz Y8
Infrared 1 1280x720 @ 15Hz Y8
Infrared 1 1280x720 @ 6Hz Y8
Infrared 2 1280x720 @ 6Hz Y8
Infrared 2 848x480 @ 90Hz Y8
Infrared 1 848x480 @ 90Hz Y8
Infrared 2 848x480 @ 60Hz Y8
Infrared 1 848x480 @ 60Hz Y8
Infrared 2 848x480 @ 30Hz Y8
Infrared 1 848x480 @ 30Hz Y8
Infrared 2 848x480 @ 15Hz Y8
Infrared 1 848x480 @ 15Hz Y8
Infrared 2 848x480 @ 6Hz Y8
Infrared 1 848x480 @ 6Hz Y8
Infrared 2 640x480 @ 90Hz Y8
Infrared 1 640x480 @ 90Hz Y8
Infrared 2 640x480 @ 60Hz Y8
Infrared 1 640x480 @ 60Hz Y8
Infrared 1 640x480 @ 30Hz Y8
Infrared 2 640x480 @ 30Hz Y8
Infrared 1 640x480 @ 15Hz Y8
Infrared 2 640x480 @ 15Hz Y8
Infrared 2 640x480 @ 6Hz Y8
Infrared 1 640x480 @ 6Hz Y8
Infrared 1 640x400 @ 25Hz Y16
Infrared 2 640x400 @ 25Hz Y16
Infrared 1 640x400 @ 15Hz Y16
Infrared 2 640x400 @ 15Hz Y16
Infrared 2 640x360 @ 90Hz Y8
Infrared 1 640x360 @ 90Hz Y8
Infrared 1 640x360 @ 60Hz Y8
Infrared 2 640x360 @ 60Hz Y8
Infrared 1 640x360 @ 30Hz Y8
Infrared 2 640x360 @ 30Hz Y8
Infrared 2 640x360 @ 15Hz Y8
Infrared 1 640x360 @ 15Hz Y8
Infrared 1 640x360 @ 6Hz Y8
Infrared 2 640x360 @ 6Hz Y8
Infrared 2 480x270 @ 90Hz Y8
Infrared 1 480x270 @ 90Hz Y8
Infrared 1 480x270 @ 60Hz Y8
Infrared 2 480x270 @ 60Hz Y8
Infrared 1 480x270 @ 30Hz Y8
Infrared 2 480x270 @ 30Hz Y8
Infrared 2 480x270 @ 15Hz Y8
Infrared 1 480x270 @ 15Hz Y8
Infrared 1 480x270 @ 6Hz Y8
Infrared 2 480x270 @ 6Hz Y8
Infrared 1 424x240 @ 90Hz Y8
Infrared 2 424x240 @ 90Hz Y8
Infrared 2 424x240 @ 60Hz Y8
Infrared 1 424x240 @ 60Hz Y8
Infrared 2 424x240 @ 30Hz Y8
Infrared 1 424x240 @ 30Hz Y8
Infrared 1 424x240 @ 15Hz Y8
Infrared 2 424x240 @ 15Hz Y8
Infrared 2 424x240 @ 6Hz Y8
Infrared 1 424x240 @ 6Hz Y8
```

### Depth Stream

```bash
Depth 1280x720 @ 30Hz Z16
Depth 1280x720 @ 15Hz Z16
Depth 1280x720 @ 6Hz Z16
Depth 848x480 @ 90Hz Z16
Depth 848x480 @ 60Hz Z16
Depth 848x480 @ 30Hz Z16
Depth 848x480 @ 15Hz Z16
Depth 848x480 @ 6Hz Z16
Depth 640x480 @ 90Hz Z16
Depth 640x480 @ 60Hz Z16
Depth 640x480 @ 30Hz Z16
Depth 640x480 @ 15Hz Z16
Depth 640x480 @ 6Hz Z16
Depth 640x360 @ 90Hz Z16
Depth 640x360 @ 60Hz Z16
Depth 640x360 @ 30Hz Z16
Depth 640x360 @ 15Hz Z16
Depth 640x360 @ 6Hz Z16
Depth 480x270 @ 90Hz Z16
Depth 480x270 @ 60Hz Z16
Depth 480x270 @ 30Hz Z16
Depth 480x270 @ 15Hz Z16
Depth 480x270 @ 6Hz Z16
Depth 424x240 @ 90Hz Z16
Depth 424x240 @ 60Hz Z16
Depth 424x240 @ 30Hz Z16
Depth 424x240 @ 15Hz Z16
Depth 424x240 @ 6Hz Z16
```

### Color Stream

```bash
Color 1920x1080 @ 30Hz RGB8
Color 1920x1080 @ 30Hz RAW16
Color 1920x1080 @ 30Hz Y16
Color 1920x1080 @ 30Hz BGRA8
Color 1920x1080 @ 30Hz RGBA8
Color 1920x1080 @ 30Hz BGR8
Color 1920x1080 @ 30Hz YUYV
Color 1920x1080 @ 15Hz RGB8
Color 1920x1080 @ 15Hz Y16
Color 1920x1080 @ 15Hz BGRA8
Color 1920x1080 @ 15Hz RGBA8
Color 1920x1080 @ 15Hz BGR8
Color 1920x1080 @ 15Hz YUYV
Color 1920x1080 @ 6Hz RGB8
Color 1920x1080 @ 6Hz Y16
Color 1920x1080 @ 6Hz BGRA8
Color 1920x1080 @ 6Hz RGBA8
Color 1920x1080 @ 6Hz BGR8
Color 1920x1080 @ 6Hz YUYV
Color 1280x720 @ 30Hz RGB8
Color 1280x720 @ 30Hz Y16
Color 1280x720 @ 30Hz BGRA8
Color 1280x720 @ 30Hz RGBA8
Color 1280x720 @ 30Hz BGR8
Color 1280x720 @ 30Hz YUYV
Color 1280x720 @ 15Hz RGB8
Color 1280x720 @ 15Hz Y16
Color 1280x720 @ 15Hz BGRA8
Color 1280x720 @ 15Hz RGBA8
Color 1280x720 @ 15Hz BGR8
Color 1280x720 @ 15Hz YUYV
Color 1280x720 @ 6Hz RGB8
Color 1280x720 @ 6Hz Y16
Color 1280x720 @ 6Hz BGRA8
Color 1280x720 @ 6Hz RGBA8
Color 1280x720 @ 6Hz BGR8
Color 1280x720 @ 6Hz YUYV
Color 960x540 @ 60Hz RGB8
Color 960x540 @ 60Hz Y16
Color 960x540 @ 60Hz BGRA8
Color 960x540 @ 60Hz RGBA8
Color 960x540 @ 60Hz BGR8
Color 960x540 @ 60Hz YUYV
Color 960x540 @ 30Hz RGB8
Color 960x540 @ 30Hz Y16
Color 960x540 @ 30Hz BGRA8
Color 960x540 @ 30Hz RGBA8
Color 960x540 @ 30Hz BGR8
Color 960x540 @ 30Hz YUYV
Color 960x540 @ 15Hz RGB8
Color 960x540 @ 15Hz Y16
Color 960x540 @ 15Hz BGRA8
Color 960x540 @ 15Hz RGBA8
Color 960x540 @ 15Hz BGR8
Color 960x540 @ 15Hz YUYV
Color 960x540 @ 6Hz RGB8
Color 960x540 @ 6Hz Y16
Color 960x540 @ 6Hz BGRA8
Color 960x540 @ 6Hz RGBA8
Color 960x540 @ 6Hz BGR8
Color 960x540 @ 6Hz YUYV
Color 848x480 @ 60Hz RGB8
Color 848x480 @ 60Hz Y16
Color 848x480 @ 60Hz BGRA8
Color 848x480 @ 60Hz RGBA8
Color 848x480 @ 60Hz BGR8
Color 848x480 @ 60Hz YUYV
Color 848x480 @ 30Hz RGB8
Color 848x480 @ 30Hz Y16
Color 848x480 @ 30Hz BGRA8
Color 848x480 @ 30Hz RGBA8
Color 848x480 @ 30Hz BGR8
Color 848x480 @ 30Hz YUYV
Color 848x480 @ 15Hz RGB8
Color 848x480 @ 15Hz Y16
Color 848x480 @ 15Hz BGRA8
Color 848x480 @ 15Hz RGBA8
Color 848x480 @ 15Hz BGR8
Color 848x480 @ 15Hz YUYV
Color 848x480 @ 6Hz RGB8
Color 848x480 @ 6Hz Y16
Color 848x480 @ 6Hz BGRA8
Color 848x480 @ 6Hz RGBA8
Color 848x480 @ 6Hz BGR8
Color 848x480 @ 6Hz YUYV
Color 640x480 @ 60Hz RGB8
Color 640x480 @ 60Hz Y16
Color 640x480 @ 60Hz BGRA8
Color 640x480 @ 60Hz RGBA8
Color 640x480 @ 60Hz BGR8
Color 640x480 @ 60Hz YUYV
Color 640x480 @ 30Hz RGB8
Color 640x480 @ 30Hz Y16
Color 640x480 @ 30Hz BGRA8
Color 640x480 @ 30Hz RGBA8
Color 640x480 @ 30Hz BGR8
Color 640x480 @ 30Hz YUYV
Color 640x480 @ 15Hz RGB8
Color 640x480 @ 15Hz Y16
Color 640x480 @ 15Hz BGRA8
Color 640x480 @ 15Hz RGBA8
Color 640x480 @ 15Hz BGR8
Color 640x480 @ 15Hz YUYV
Color 640x480 @ 6Hz RGB8
Color 640x480 @ 6Hz Y16
Color 640x480 @ 6Hz BGRA8
Color 640x480 @ 6Hz RGBA8
Color 640x480 @ 6Hz BGR8
Color 640x480 @ 6Hz YUYV
Color 640x360 @ 60Hz RGB8
Color 640x360 @ 60Hz Y16
Color 640x360 @ 60Hz BGRA8
Color 640x360 @ 60Hz RGBA8
Color 640x360 @ 60Hz BGR8
Color 640x360 @ 60Hz YUYV
Color 640x360 @ 30Hz RGB8
Color 640x360 @ 30Hz Y16
Color 640x360 @ 30Hz BGRA8
Color 640x360 @ 30Hz RGBA8
Color 640x360 @ 30Hz BGR8
Color 640x360 @ 30Hz YUYV
Color 640x360 @ 15Hz RGB8
Color 640x360 @ 15Hz Y16
Color 640x360 @ 15Hz BGRA8
Color 640x360 @ 15Hz RGBA8
Color 640x360 @ 15Hz BGR8
Color 640x360 @ 15Hz YUYV
Color 640x360 @ 6Hz RGB8
Color 640x360 @ 6Hz Y16
Color 640x360 @ 6Hz BGRA8
Color 640x360 @ 6Hz RGBA8
Color 640x360 @ 6Hz BGR8
Color 640x360 @ 6Hz YUYV
Color 424x240 @ 60Hz RGB8
Color 424x240 @ 60Hz Y16
Color 424x240 @ 60Hz BGRA8
Color 424x240 @ 60Hz RGBA8
Color 424x240 @ 60Hz BGR8
Color 424x240 @ 60Hz YUYV
Color 424x240 @ 30Hz RGB8
Color 424x240 @ 30Hz Y16
Color 424x240 @ 30Hz BGRA8
Color 424x240 @ 30Hz RGBA8
Color 424x240 @ 30Hz BGR8
Color 424x240 @ 30Hz YUYV
Color 424x240 @ 15Hz RGB8
Color 424x240 @ 15Hz Y16
Color 424x240 @ 15Hz BGRA8
Color 424x240 @ 15Hz RGBA8
Color 424x240 @ 15Hz BGR8
Color 424x240 @ 15Hz YUYV
Color 424x240 @ 6Hz RGB8
Color 424x240 @ 6Hz Y16
Color 424x240 @ 6Hz BGRA8
Color 424x240 @ 6Hz RGBA8
Color 424x240 @ 6Hz BGR8
Color 424x240 @ 6Hz YUYV
Color 320x240 @ 60Hz RGB8
Color 320x240 @ 60Hz Y16
Color 320x240 @ 60Hz BGRA8
Color 320x240 @ 60Hz RGBA8
Color 320x240 @ 60Hz BGR8
Color 320x240 @ 60Hz YUYV
Color 320x240 @ 30Hz RGB8
Color 320x240 @ 30Hz Y16
Color 320x240 @ 30Hz BGRA8
Color 320x240 @ 30Hz RGBA8
Color 320x240 @ 30Hz BGR8
Color 320x240 @ 30Hz YUYV
Color 320x240 @ 6Hz RGB8
Color 320x240 @ 6Hz Y16
Color 320x240 @ 6Hz BGRA8
Color 320x240 @ 6Hz RGBA8
Color 320x240 @ 6Hz BGR8
Color 320x240 @ 6Hz YUYV
Color 320x180 @ 60Hz RGB8
Color 320x180 @ 60Hz Y16
Color 320x180 @ 60Hz BGRA8
Color 320x180 @ 60Hz RGBA8
Color 320x180 @ 60Hz BGR8
Color 320x180 @ 60Hz YUYV
Color 320x180 @ 30Hz RGB8
Color 320x180 @ 30Hz Y16
Color 320x180 @ 30Hz BGRA8
Color 320x180 @ 30Hz RGBA8
Color 320x180 @ 30Hz BGR8
Color 320x180 @ 30Hz YUYV
Color 320x180 @ 6Hz RGB8
Color 320x180 @ 6Hz Y16
Color 320x180 @ 6Hz BGRA8
Color 320x180 @ 6Hz RGBA8
Color 320x180 @ 6Hz BGR8
Color 320x180 @ 6Hz YUYV
```

## Notes
1. At this time for jetpacks above 4.4.1 there is no librealsense linux kernel patch. While the realsense will function as expected, you may receive some warnings when luanching the realsense ros warpper node.
2. gcc, g++, clang version 8 and 9 contain a bug regarnding aarch64 architectures, see [8-10] for more details.

## References

1. [Librealsense NVidia Jetson Devices](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md)
2. [ROS Wrapper for Intel® RealSense™ Devices](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy#ros-wrapper-for-intel-realsense-devices)
3. [Firmware releases D400](https://dev.intelrealsense.com/docs/firmware-releases)
4. [ddynamic_reconfigure package](https://github.com/pal-robotics/ddynamic_reconfigure/tree/kinetic-devel#ddynamic_reconfigure)
5. [image_transport_plugins](https://github.com/ros-perception/image_transport_plugins/tree/noetic-devel)
6. [Different Resolution for depth and color stream to make camera working](https://github.com/IntelRealSense/librealsense/issues/3376)
7. [Librealsense releases](https://github.com/IntelRealSense/librealsense/releases)
8. [gcc 9: aarch64 -ftree-loop-vectorize results in wrong code](https://gcc.gnu.org/bugzilla//show_bug.cgi?id=102435)
9. [Issue ROS crash on Raspberry Pi 4B with a very simple program](https://github.com/ros/ros_comm/issues/2197)
10. [Demo ROS crash on Raspberry Pi 4B with a very simple program](https://github.com/AutoxingTech/simple_publisher_crash)
