# Acciona ROS exercise

## Tested on
- Ubuntu 18.04
- ROS Melodic

## Dependecies
- turtlesim
- rosbridge-service
- xterm
- OpenCV
- Numpy
- Rospy
- Mediapipe
- Python >= 3.8
- PiP

***

## Installation:

1. Install ROS (Recommended use Ubuntu 18.04 and ROS-Melodic). [Installation instructions](http://wiki.ros.org/melodic/Installation/Ubuntu)

2. Install turtlesim
```
sudo apt-get install ros-$(rosversion -d)-turtlesim
```

3. Install rosbridge-server, xterm, pip, git
```
sudo apt update
sudo apt-get install ros-melodic-rosbridge-server xterm python3-pip git
```

4. Install Mediapipe
```
pip install mediapipe
```

5. Create ROS Workspace
```
source /opt/ros/melodic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```


6. Clone package repository and building
```
cd ~/catkin_ws/src/
git clone https://github.com/sergiomh499/practica_acciona.git
cd ~/catkin_ws/
catkin_make
```

***

## Execution:
1. Open a terminal and execute:
```
roscore
```

2. Open other terminal and execute:
```
cd /catkin_ws
source devel/setup.bash
rosrun practica_acciona start_turtlesim_snake
```

3. Open a new terminal and execute:
```
cd /catkin_ws
source devel/setup.bash
rosservice call /start_turtlesim_snake "x: 1.0
y: 1.0
theta: 0.0
camera_control: true"
```

``x``, ``y`` and ``theta`` indicates the initial position of the first turtle and ``camera_control`` is used to change the control between keyboard and web or control with webcam.

### Turtle web control

If you want to control the turtle via web with a joystick, you need to execute also in console, to launch the websocket:
```
roslaunch rosbridge_server rosbridge_websocket.launch
```

Later, open the ``controller.htlm`` in your web browser.

*NOTE: Check on the ROS service call that ``camera_control`` is ``false``.*