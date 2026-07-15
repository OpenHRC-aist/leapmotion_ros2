# Leapmotion_ROS2
This packages offers a ROS2-wrapper node for Leap Motion Controller 2 by [Ultraleap](https://www.ultraleap.com/). The communication with the device depends on Ultraleap Gimini and LeapC Python Binding library.

## How to Use

### Test environment
- Ubuntu 22.04 & ROS2 Humble 


### Install
1. Install Ultraleap Gemini from [here](https://leap2.ultraleap.com/downloads/)
1. Install Gemini LeapC Python Bindings by following [here](https://github.com/ultraleap/leapc-python-bindings)
1. Clone this repository in the ROS2 workspace and build the packages.

### Get started
#### Run node for publishing hands information
```bash
$ ros2 run leap_node hands_publisher 
```
You can subscribe a topic named ``/leap_hands`` including both hands position, velocity and so.

#### Robot Teleoperation with OpenHRC 
If you have installed [OpenHRC](https://github.com/Automation-Research-Team/OpenHRC) package, you can teleoperate a robot via your hand postures.


Run the following commands in separate terminals:
```bash
# start simulator of UR5e
$ ros2 launch ur_simulation_gz ur_sim_control.launch.py initial_joint_controller:=forward_velocity_controller launch_rviz:=false

# start control via leap motion
$ ros2 launch ohrc_leap ohrc_leap_teleoperation.launch.py 
```
Your both hands need to be detected. While your left hand grabbed, the robot end-effector moves to follow the motion of your right hand (palm position).




## leap_node

In `leap_node`, the position, posture, and gesture results obtained from Leap Motion are published.

- `position_detector`: Displays position
- `posture_detector`: Displays position and posture
- `gesture_detector`: Displays pinch and grab states
- `all_detector`: Displays all information

- `hands_publisher`: Publish all information

--Under Development--
- `finger_detector`: Displays finger joint information
- `rviz_visualizer`: Displays finger joint information on RViz


## License
This package is subject to the MIT Licnese

## Author
- Masaki Saito (AIST / Keio University)
- Shunki Itadera (AIST, [https://itadera.github.io/](https://itadera.github.io/))