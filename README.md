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
launch moveit setup assistant, you can check the configuration in step "Robot Pose"
```
roslaunch moveit_setup_assistant setup_assistant.launch
```
### 3. Verify your urdf in gazebo
#### 1). Create ROS Package "XXX_gazebo" for launching in gazebo
#### 2). Copy the "gazebo.launch" to your "XXX_gazebo" package
#### 3). Launch and examine it (You may check and edit the urdf path in the file gazebo.launch)

## Possible issues

* 'position_controllers/JointTrajectoryController' does not exist
```
$ sudo apt-get install ros-kinetic*controllers*
```
