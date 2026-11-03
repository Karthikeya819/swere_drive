# Swerve Drive for ROS 2 and Gazebo Classic

A C++ Gazebo Classic model plugin for four-wheel steering and six-wheel drive robots. Send `geometry_msgs/msg/Twist` commands on `cmd_vel`; the plugin controls steering and wheel joints with PID controllers and publishes simulation ground-truth odometry on `odom`.

**Target stack:** Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11. This is a Gazebo Classic plugin, not a modern Gazebo (`ros_gz`) system plugin. The ROS package and repository retain the original `swere_drive` spelling.

## Supported Drive Systems

This plugin currently supports two main drive configurations:
1. **Four-Wheel Complete Swerve:** All four wheels can steer independently and drive.
2. **Six-Wheel Drive:** Only the four corner wheels steer (swerve), while the other two middle wheels are rigid and only drive.

<div style="display: inline-block;">
  <img src="assets/four_wheel.png" alt="Four-Wheel Drive">
  <img src="assets/six_wheel.png" alt="Six Wheel Drive">
</div>

## Prerequisites

Start with ROS 2 Humble installed and sourced. Install the build tools and simulator dependencies:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep \
  ros-humble-gazebo-ros-pkgs ros-humble-robot-state-publisher \
  ros-humble-teleop-twist-keyboard
```

## How to Install and Build

**1. Create a Workspace:**
```bash
mkdir -p ~/sim_ws/src
cd ~/sim_ws/src
```

**2. Clone the Repository:**
```bash
git clone https://github.com/Karthikeya819/swere_drive.git
```

**3. Build the Package:**
```bash
cd ~/sim_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select swere_drive
```

**4. Source the Workspace:**
```bash
source ~/sim_ws/install/setup.bash
```

## How to Launch

You can use the provided launch file to start the gazebo simulation. The launch file is parameterized so you can simply pass `six_wheel:=true` or `false`:

```bash
# For a Six-Wheel Robot
ros2 launch swere_drive robot_gazebo.launch.py six_wheel:=true

# For a Four-Wheel Robot
ros2 launch swere_drive robot_gazebo.launch.py six_wheel:=false
```

To drive the robot around, open a **new terminal**, source ROS2, and run the teleop node:
```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Behavior and limitations

- Four-wheel mode supports forward/reverse motion, turning, and pure sideways crab commands (`linear.y` with zero `linear.x` and `angular.z`). Mixed lateral commands are not general holonomic swerve kinematics.
- Six-wheel mode has four steering corner wheels and two fixed middle wheels. Sideways crab motion is unsupported.
- `sixWheel` accepts `true`/`false` (case-insensitive) and `1`/`0`; it defaults to false. Six-wheel mode requires both middle-wheel joint names.
- Entering crab mode holds wheel speeds at zero for 1.5 seconds of simulation time while steering aligns. Completion is checked on each Gazebo physics update, independently of the wall-time odometry rate, and takes effect on the first update at or after the deadline. This is nonblocking: odometry continues and a newer stop or non-crab command cancels the pending drive. Repeated crab commands update the speed without restarting the delay. This remains a fixed delay, not steering-feedback verification.
- Odometry uses Gazebo world pose, not wheel encoder integration. No odometry TF broadcaster or command watchdog is provided. Send an explicit zero command to stop.
- The existing turning equations are experimental; validate geometry and motion in simulation before adapting the controller to hardware.

### Quick smoke check

After launching either model, in another sourced terminal:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
ros2 topic echo /odom --once
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

For four-wheel mode, send a pure lateral command and then a stop during the steering delay. Verify that odometry keeps arriving and that no stale lateral command starts after the stop:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {y: 0.2}}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

### Tests

```bash
colcon test --packages-select swere_drive
colcon test-result --verbose
```

The command-state regression tests can also run without ROS or Gazebo:

```bash
g++ -std=c++14 -Wall -Wextra -Werror -Iinclude \
  test/test_drive_command_utils.cpp -o /tmp/test_drive_command_utils
/tmp/test_drive_command_utils
python3 -m unittest discover -s test -p 'test_*.py'
```

If the plugin cannot be found, source `install/setup.bash` in the launch terminal and confirm that `libackerman_drive_plugin.so` exists under `install/swere_drive/lib`. If dependency resolution fails on a fresh ROS setup, initialize rosdep once with `sudo rosdep init`, then run `rosdep update`. Keep the historical `wheelSeperation` parameter spelling when configuring the plugin.

## Plugin Configuration Examples

To use this plugin on your own robot, add the following blocks to your robot's SDF or URDF file. Make sure to replace the joint names with the actual names from your robot model.

### 1. Six Wheel Example

```xml
<gazebo>
  <plugin name="ackerman_drive_plugin" filename="libackerman_drive_plugin.so">
    <commandTopic>cmd_vel</commandTopic>
    <odometryTopic>odom</odometryTopic>
    <odometryFrame>odom</odometryFrame>
    <robotBaseFrame>base_link</robotBaseFrame>
    <updateRate>50</updateRate>
    <sixWheel>true</sixWheel>
    <wheelRadius>0.1</wheelRadius>
    <wheelSeperation>0.5</wheelSeperation>
    <wheelBase>0.6</wheelBase>
    <stepperP>1000.0</stepperP>
    <stepperI>0.0</stepperI>
    <stepperD>50.0</stepperD>
    <wheelP>0.5</wheelP>
    <wheelI>0.01</wheelI>
    <wheelD>0.02</wheelD>
    <front_left_stepper_joint>front_left_steer_joint</front_left_stepper_joint>
    <front_right_stepper_joint>front_right_steer_joint</front_right_stepper_joint>
    <back_left_stepper_joint>back_left_steer_joint</back_left_stepper_joint>
    <back_right_stepper_joint>back_right_steer_joint</back_right_stepper_joint>
    <front_left_wheel_joint>front_left_drive_joint</front_left_wheel_joint>
    <front_right_wheel_joint>front_right_drive_joint</front_right_wheel_joint>
    <middle_left_wheel_joint>middle_left_drive_joint</middle_left_wheel_joint>
    <middle_right_wheel_joint>middle_right_drive_joint</middle_right_wheel_joint>
    <back_left_wheel_joint>back_left_drive_joint</back_left_wheel_joint>
    <back_right_wheel_joint>back_right_drive_joint</back_right_wheel_joint>
  </plugin>
</gazebo>
```

### 2. Four Wheel Example

```xml
<gazebo>
  <plugin name="ackerman_drive_plugin" filename="libackerman_drive_plugin.so">
    <commandTopic>cmd_vel</commandTopic>
    <odometryTopic>odom</odometryTopic>
    <odometryFrame>odom</odometryFrame>
    <robotBaseFrame>base_link</robotBaseFrame>
    <updateRate>50</updateRate>
    <sixWheel>false</sixWheel>
    <wheelRadius>0.1</wheelRadius>
    <wheelSeperation>0.5</wheelSeperation>
    <wheelBase>0.6</wheelBase>
    <stepperP>1000.0</stepperP>
    <stepperI>0.0</stepperI>
    <stepperD>50.0</stepperD>
    <wheelP>0.5</wheelP>
    <wheelI>0.01</wheelI>
    <wheelD>0.02</wheelD>
    <front_left_stepper_joint>front_left_steer_joint</front_left_stepper_joint>
    <front_right_stepper_joint>front_right_steer_joint</front_right_stepper_joint>
    <back_left_stepper_joint>back_left_steer_joint</back_left_stepper_joint>
    <back_right_stepper_joint>back_right_steer_joint</back_right_stepper_joint>
    <front_left_wheel_joint>front_left_drive_joint</front_left_wheel_joint>
    <front_right_wheel_joint>front_right_drive_joint</front_right_wheel_joint>
    <back_left_wheel_joint>back_left_drive_joint</back_left_wheel_joint>
    <back_right_wheel_joint>back_right_drive_joint</back_right_wheel_joint>
  </plugin>
</gazebo>
```

## Demonstration

![Robot Demo](assets/demo.gif)

## Further Improvements

* **Reverse Kinematics:** Currently, the plugin uses Gazebo's exact `WorldPose()` to publish odometry. In the future, this can be updated to calculate real wheel-based odometry (Reverse Kinematics) for a more realistic simulation.
* **Velocity and Acceleration Limits:** Implement configurable maximum velocity, acceleration, and jerk limits within the plugin to simulate real-world hardware constraints and prevent sudden jumps.
