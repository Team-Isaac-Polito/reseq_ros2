# Gazebo

## This is the base to test the robot on Gazebo.

---

---

## Prerequisites:

- keep your system updated with: `sudo apt-get update && sudo apt-get upgrade`
- <a href="https://docs.teamisaac.it/s/a9fc1d45-3830-400f-943f-88d75b56df82">instal ros2 humble</a> (internal link, just follow the <a href="https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html">official documentation</a> if not accessible)
- <a href="https://classic.gazebosim.org/tutorials?tut=install_ubuntu">install gazebo</a>
- install other packages as shown below

```
sudo apt-get install ros-humble-xacro
sudo apt-get install ros-humble-gazebo-ros2-control
sudo apt-get install ros-humble-controller-manager
sudo apt-get install ros-humble-diff-drive-controller
sudo apt-get install ros-humble-joint-state-broadcaster
sudo apt-get install ros-humble-teleop-twist-keyboard
sudo apt-get install ros-humble-teleop-twist-joy # only if yow want to use a controller :video_game:
sudo apt-get install ros-humble-rviz2
```

see <a href="https://index.ros.org/p/teleop_twist_joy/github-ros2-teleop_twist_joy/">ros2 wiki</a> for joystick settings

---

## To launch the simulation

> :heavy_exclamation_mark: remember one time at the opening of the terminal to do `source install/setup.bash` from `dev_ws` folder

In terminal 1 (to launch AGEVAR)

```
ros2 run reseq_ros2 agevar
```

In terminal 2 (to launch many scripts related to gazebo)

```
ros2 launch reseq_gazebo launch_sim.launch.py
```

In terminal 3 (to control the robot trough keyboard. You must select this terminal and type in keyboard to control the robot)

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

(alternately use `ros2 launch teleop_twist_joy teleop-launch.py` to control the robot trough the controller :video_game: )

In terminal 4 (if you want to see the robot in Rviz too)

```
rviz2
```

- set "fixed frame" from "map" to "base link" (to refer to the robot body)
- add "TF" to visualize all tf from the robot parts (and see if everything is moving as it should)
- add "RobotModel" and change "Description Topic" to "/robot_description" to visualize the shape of the robot (You can actually decide to visualize "Visual" or "Collision" property of the robot)
- a final ctr-s wil save your configuration for next time you will use Rviz2

To terminate the process type ctrl-c (there are many running services, not only Gazebo client).

---

## Known issues

- At this stage the robot trajectory is controlled in open-loop. The yaw angle from `/joint_states` (joint_b_1_joint,joint_b_2_joint,joint_b_3_joint) should be feed back in AGEVAR controller to close the loop.
- Another problem is probably related to friction, tests still need to be done to determinate best settings for the urdf of the robot.
