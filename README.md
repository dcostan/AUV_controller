# AUV_controller 

Graphical application built with Qt used to control simulated Autonomous Underwater Vehicles with Gazebo and ROS. It was created to provide a realistic scenario emulating the real usage of ```rmw_desert```, in order to demonstrate the feasibility of controlling underwater robots using acoustic wireless signals.

*NOTE: Do not try to run a Gazebo simulation in a virtual machine, it will not work properly!*


## Installing the middleware

To properly install the middleware layer used to communicate with DESERT through ROS applications and all its components you have to read the wiki available in the [corresponding github repository](https://github.com/signetlabdei/rmw_desert/wiki).

Before proceeding make sure you have completed the instructions in the following sections:
*  DESERT Framework
*  Robot Operating System
*  Building the module

## Dependencies

Different packages are required to build this application.

*  First install Qt6

   ```
   sudo apt install qt6-base-dev qt6-tools-dev
   ```
   or alternatively Qt5

   ```
   sudo apt install qtbase5-dev qttools5-dev
   ```
*  Install Gazebo

   ```
   sudo apt install gz-ionic ros-rolling-ros-gz 
   ```
*  And finally install gz-transport version 14 using the [tutorial](https://gazebosim.org/api/transport/14/installation.html).

## Configuring the underwater simulation

Gazebo supports basic simulation of underwater vehicles, using the equations described in Fossen's "Guidance and Control of Ocean Vehicles". [This tutorial](https://gazebosim.org/api/sim/9/underwater_vehicles.html) will guide you through the steps you need to setup simulation of an underwater vehicle.

Note that the page refers to gazebo version 8, but since we are supposed to work with the ROS release codenamed rolling, the installation command reported above will download gazebo version 9.

To prevent incompatibilities just replace the word ```gz-sim8``` with ```gz-sim9``` in the tutorial.

## Building the controller

To build this application you just need to clone the repository and then run cmake.

```
git clone https://github.com/dcostan/AUV_controller.git
```
Move inside the ```AUV_controller``` directory.

```
mkdir build
cd build
cmake ../
make
```   

If no errors occured you should have a binary file compiled that you can run with the command below.

```
./AUV_controller
```   

## ROS-Gazebo bridge

Now you have the controller application that implements a ROS node and a simulated AUV vehicle in Gazebo representing the other node. The simulator has its own method to process topics, so you have to launch the ```ros_gz_bridge``` tool to convert Gazebo data streams in the ROS standard, enabling a bidirectional communication.

Open a terminal in the folder of this repository and type the following.

```
ns uwApplicationTCPwithPosition.tcl
```

On another terminal in the same folder run the bridge.

```
RMW_IMPLEMENTATION=rmw_desert DESERT_PORT=4000 ros2 launch ./ros_gz_bridge.launch.py name:=ros_gz_bridge config_file:=ros_gz_bridge.yaml bridge_name:=auv_bridge
```

In this way, when you open the Gazebo simulation, the application will automatically catch the signals coming from ROS and it will send them to the AUV to control its actuators.

## Deploying the simulation

In order to deploy a working underwater simulation you have to open four different terminal windows, each of which are on the directory of this repository.

   1. In the first one run the underwater network simulation
      
      ```
      ns uwApplicationTCPwithPosition.tcl
      ```

   2. In the second one start Gazebo
      
      ```
      export GZ_SIM_RESOURCE_PATH=:$HOME/gazebo_maritime/models
      gz sim ~/gazebo_maritime/worlds/buoyant_lrauv.sdf
      ```

   2. In the third one launch the ros-gz bridge

      ```
      RMW_IMPLEMENTATION=rmw_desert DESERT_PORT=4000 ros2 launch ./ros_gz_bridge.launch.py name:=ros_gz_bridge config_file:=ros_gz_bridge.yaml bridge_name:=auv_bridge
      ```

   2. In the fourth one execute the controller

      ```
      RMW_IMPLEMENTATION=rmw_desert DESERT_PORT=5000 ./build/AUV_controller
      ```

If you run them at the same time you will end up with a controller interface that can manage the AUV joints using acoustic undewater communications through the DESERT protocols stack. Note that the last two terminals contain the nodes involved in this scenario, and each of them is associated to a different TCP port as described in the TCL source.
