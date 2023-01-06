# ROS Noetic Install on Jetson Nano with Ubuntu 18.04 (python3.6)

The goal of this guide is to compile and install a fully working version of [ROS Noetic](http://wiki.ros.org/noetic) on the official Jetson Nano Ubuntu 18.04 distribution with native python 3.6 support.\
The guide assumes that a clean and up-to-date Ubuntu 18.04 installation
was done on the Jetson Nano. The steps for this are discussed in the [Jetson Nano Ubuntu 18.04 Full Install](ubuntu-18-04-install.md) guide.

1. First make sure you have gcc-10 and g++-10 activated:

```bash
sudo update-alternatives --config gcc
sudo update-alternatives --config g++
```

Select the number corresponding to version 10 for each of the compilers.

2. Install the necesarry package dependencies for the build:

```bash
sudo apt-get install python3-rosdep python3-rosinstall-generator python3-vcstools build-essential libgtest-dev liborocos-kdl-dev
sudo -H python3 -m pip install -U pip
sudo -H python3 -m pip install -U setuptools
sudo -H python3 -m pip install -U rosdep rosinstall_generator vcstool
```
3. Initialize ROS dependency manager tool:

```bash
sudo rosdep init
rosdep update
```
4. Create a catkin workspace. Since, this will be the place were we'll build ROS a suggest you give it a diffrent name than the usual `catkin_ws`:

```bash
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws
```
5. Depending on what you want to achive you can choose to download the packages you will need. That is, similar to an install through a packages manager, like  ROS-Base or ROS Desktop-Full Install, you can choose to download and build the variant to your specification as indicated in [4].\
For example, to download the packages contained within ROS-Base do the following:

```bash
rosinstall_generator robot perception --rosdistro noetic --deps --tar > noetic-robot-perception.rosinstall
mkdir ./src
vcs import --input noetic-robot-perception.rosinstall ./src
```

whereas to download the packages contained within ROS Desktop do the following:

```bash
rosinstall_generator desktop --rosdistro noetic --deps --tar > noetic-desktop.rosinstall
mkdir ./src
vcs import --input noetic-desktop.rosinstall ./src
```

6. In order to build we must first resolve dependencies with rosdep. Please note that some dependencies might be reported as missing (like tf), however the build and install will succeed properly. It is very important here to use the `-r` option to skip the ones that fail.

```bash
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y -r
```
7. Before building we need to modify the cv_bridge `CMakeLists.txt` to avoid a compilation error:

```bash
nano ./src/vision_opencv/cv_bridge/CMakeLists.txt
```

change the line `find_package(Boost REQUIRED python37)` to `find_package(Boost REQUIRED python3)`.

8. Build the distribution and install it as usual in the /opt/ros/noetic folder:

```bash
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 --install-space /opt/ros/noetic
```

9. Don't forget to setup your environment:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

10. To test the ROS Noetic installation launch the roscore before making other changes:

```bash
roscore
... logging to /home/jetson/.ros/log/43644112-8dab-11ed-ba50-a967e2d0f2c8/roslaunch-nano-23182.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://nano.local:43793/
ros_comm version 1.15.15


SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.15.15

NODES

auto-starting new master
process[master]: started with pid [23192]
ROS_MASTER_URI=http://nano.local:11311/

setting /run_id to 43644112-8dab-11ed-ba50-a967e2d0f2c8
process[rosout-1]: started with pid [23202]
started core service [/rosout]
^C[rosout-1] killing on exit
[master] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done
```

## References

1. [Installing from source](http://wiki.ros.org/Installation/Source)
2. [ROS Noetic Install on jetson nano (Ubuntu 18.04)](https://gist.github.com/Pyrestone/ef683aec160825eee5c252f22218ddb2)
3. [HOWTO install rospy ROS Noetic module to Ubuntu 18.04](https://vsbogd.github.io/coding/install-rospy-noetic-ubuntu-1804.html)
4. [REP:	150 - ROS Melodic and Newer Metapackages](https://www.ros.org/reps/rep-0150.html)
