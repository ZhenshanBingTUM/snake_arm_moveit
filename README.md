# Snake_arm_moveit
We will see later

## How to Run it
### Step 1: Launch the Gazebo simulation
```
$ roslaunch IRB300_gazebo gazebo.launch
```
### Step 2: Launch the moveit plan package
```
$ roslaunch moveit_IRB300 demo_planning_execution.launch
```

## Instructions for simulating Gazebo and Moveit together
### 1. Export URDF from Solidworks
You have to install the [Solidworks-Urdf Exporter](http://wiki.ros.org/sw_urdf_exporter) first.

### 2. Verify the joints configuration (axis, rotating direction)

## Possible issues

* 'position_controllers/JointTrajectoryController' does not exist
```
$ sudo apt-get install ros-kinetic*controllers*
```
