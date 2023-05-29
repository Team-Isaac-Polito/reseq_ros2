# Gazebo

## This is the base to test the robot on Gazebo.

---

---

## Prerequisites:

- install gazebo
- install other packages as shown below

```
sudo apt-get install ros-humble-gazebo-ros2-control
sudo apt-get install ros-humble-xacro
sudo apt-get install ros-humble-diff-drive-controller
sudo apt-get install ros-humble-joint-state-broadcaster
#sudo apt-get install ros-humble-position-controllers !!! NOT CURRENTLY USED !!!
sudo apt-get install ros-humble-teleop-twist-keyboard
sudo apt-get install ros-humble-rviz2
```

---

## To launch the simulation

open 2 or 3 terminal (remember one time at the opening of the terminal to do `source install/setup.bash` from `dev_ws` folder)

In terminal 1 (to launch many scripts)

```
ros2 launch reseq_gazebo launch_sim.launch.py
```

In terminal 2 (to launch the controller. You must select this terminal and type in keyboard to control the robot)

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont_17/cmd_vel_unstamped
```

remember that you have to select this terminal to give commands to the robot in Gazebo

> :warning: In this way you are only controlling the first module, the 3 module behing are "trying to stop" it. This because currently there are 4 differential drive controller (one for each module) that expect to read the linear and angular speed from

```
/cmd_vel:=/diff_cont_17/cmd_vel_unstamped # module 1
/cmd_vel:=/diff_cont_18/cmd_vel_unstamped # module 2
/cmd_vel:=/diff_cont_19/cmd_vel_unstamped # module 3
/cmd_vel:=/diff_cont_20/cmd_vel_unstamped # module 4
```

> The correct way would be to send the twist command to AGEVAR and AGEVAR should send the twist command for each module. :warning:

In terminal 3 (if you want to see the robot in Rviz too)

```
rviz2
```

- set "fixed frame" from "map" to "base link" (to refer to the robot body)
- add "TF" to visualize all tf from the robot parts (and see if everything is moving as it should)
- add "RobotModel" and change "Description Topic" to "/robot_description" to visualize the shape of the robot (You can actually decide to visualize "Visual" or "Collision" property of the robot)
- a final ctr-s wil save your configuration for next time you will use Rviz2

To terminate the process type ctrl-c (there are many running services, not only Gazebo client).
