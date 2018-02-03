# IGVC
Welcome to the UT RAS IGVC repo. This file will give installation/usage instructions. If you want to develop on this repo see our [contributing guide](other_file.md).

# Setup

First you need to download and install ros by following these two guides:

1. [Install Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) (If you're on 17.04 you'll need to [Install Lunar](http://wiki.ros.org/lunar/Installation/Ubuntu) instead)
2. [Configure Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

Then clone our repo into the src folder:

```
  cd ~/catkin_ws/src
  git clone https://github.com/ut-ras/IGVC2018.git
```

It isn't necessary to run the codebase but if you want to be able to edit the code on the microcontroller, you'll need to [install the Energia IDE](http://energia.nu/download/)

```
sudo apt install xboxdrv
cd catkin_ws/src
sudo apt install python-serial
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/rosserial.git
cd ..
catkin_make
catkin_make install
source install/setup.bash
source devel/setup.bash
```

If you get Energia you'll need to build the libraries for it:

```
cd <sketches_dir>/libraries
rosrun rosserial_tivac make_libraries_energia .
```

To check if it works, run the manual control launch file:

```
roslaunch motor manual.launch
```
