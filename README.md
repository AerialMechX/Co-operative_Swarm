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

