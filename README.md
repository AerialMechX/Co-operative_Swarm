# Guide for PX4 SITL Setup with MAVROS and Gazebo-Classic

This guide covers the setup for PX4 SITL with MAVROS and Gazebo-Classic, including handling Data Link Loss, cloning the PX4 repository, and installing required dependencies.

---

## Cloning the PX4 Repository

First, clone the PX4 repository and set up the environment.

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

```
```bash
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```


## add the source line in .bashrc file :
```bash
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
```

You may need to install the following additional dependencies:

```bash
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
```

reboot PC

Make the px4-sitl inside /px-4_autopilot

```bash
DONT_RUN=1 make px4_sitl_default gazebo-classic
```


## Data Link Loss

The Data Link Loss failsafe, which handles the unavailability of external data via MAVLink, is enabled by default. This setup requires the simulation to be connected to a Ground Control Station (GCS), SDK, or another MAVLink application. 

To modify this failsafe behavior, set the `NAV_DLL_ACT` parameter to the desired failsafe action. For instance, set it to `0` to disable the failsafe.

> **Note:** All parameters in SITL, including `NAV_DLL_ACT`, will reset when you run `make clean`.

---

## Inside mavros also install 

> install_geographiclib_datasets.sh

Also after cloning whole repo to install all dependencies run:

```
vcs import < dependencies.repos
```
### Now launch the SITL by command:
```
roslaunch px4 posix_sitl.launch
```
