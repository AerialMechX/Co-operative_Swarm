# Guide for PX-4 sitl Setup with mavros and gazebo-classic

## Data Link Loss

The Data Link Loss failsafe (unavailability of external data via MAVLink) is enabled by default. This makes the simulation only usable with a connected GCS, SDK, or other MAVLink application.
Set the parameter NAV_DLL_ACT to the desired failsafe action to change the behavior. For example, set to 0 to disable it.
All dev related to sarming multiple drone with connected payload

All parameters in SITL including this one get reset when you do make clean.

## clone px-4 git repo from:

git clone https://github.com/PX4/PX4-Autopilot.git --recursive

bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

You may need to install the following additional dependencies:

sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y

DONT_RUN=1 make px4_sitl_default gazebo-classic

## add the source line in .bashrc file :
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic

## to run mavros also install 

install_geographiclib_datasets.sh


Also after cloning whole repo to install all dependencies run:

vcs import < dependencies.repos


# Guide for PX4 SITL Setup with MAVROS and Gazebo-Classic

This guide covers the setup for PX4 SITL with MAVROS and Gazebo-Classic, including handling Data Link Loss, cloning the PX4 repository, and installing required dependencies.

---

## Data Link Loss

The Data Link Loss failsafe, which handles the unavailability of external data via MAVLink, is enabled by default. This setup requires the simulation to be connected to a Ground Control Station (GCS), SDK, or another MAVLink application. 

To modify this failsafe behavior, set the `NAV_DLL_ACT` parameter to the desired failsafe action. For instance, set it to `0` to disable the failsafe.

> **Note:** All parameters in SITL, including `NAV_DLL_ACT`, will reset when you run `make clean`.

---

## Cloning the PX4 Repository

First, clone the PX4 repository and set up the environment.

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

```bash
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh


