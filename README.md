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

## Installing Mavros and its dependencies

### Binary Installation (Debian / Ubuntu)
```
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
```
Then install GeographicLib datasets by running the install_geographiclib_datasets.sh script:
```
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```
### Source Installation
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init src
```
Also install other dependencies:
```bash
 sudo apt-get install python-catkin-tools python-rosinstall-generator -y
```

Install MAVROS from source using either released or latest version:
- Released/stable
```bash
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
```

Create workspace & deps:

```bash
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y
```
Install GeographicLib datasets:

```bash
./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```
> probably you will get error when when building mavros_extras:
- When inspecting the gps_status.cpp file, it seemed that GPS_RAW and GPS_RAW2 had the same fields.
So, I added the missing field to GPS2_RAW in the mavlink/message_definitions/v1.0/common.xml file, and the issue was resolved.
The code I added to GPS2_RAW with id=124 in the common.xml file is as follows: 
```bash
<message id="124" name="GPS2_RAW">
      <description>Second GPS data.</description>
      ...
      <extensions/>
      <field type="int32_t" name="alt_ellipsoid" units="mm">Altitude (above WGS84, EGM96 ellipsoid). Positive for up.</field>
      <field type="uint32_t" name="h_acc" units="mm">Position uncertainty.</field>
      <field type="uint32_t" name="v_acc" units="mm">Altitude uncertainty.</field>
      <field type="uint32_t" name="vel_acc" units="mm">Speed uncertainty.</field>
      <field type="uint32_t" name="hdg_acc" units="degE5">Heading / track uncertainty</field>
      <field type="uint16_t" name="yaw" units="cdeg">Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use 65535 if this GPS is configured to provide yaw and is currently unable to provide it. Use 36000 for north.</field>
</message>
...
```


#### Also after cloning whole repo to install all dependencies run:

```
vcs import < dependencies.repos
```
#### Now launch the SITL by command:
```
roslaunch px4 posix_sitl.launch
```
