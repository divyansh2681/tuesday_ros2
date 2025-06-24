# Tuesday Lab ROS2 Pose Controller

ROS 2 stack for switching between clock-based and GUI-based pose inputs.

## Packages

- `clock_pose_issuer`: publishes poses based on system time
- `gui_pose_issuer`: GUI interface to click target pose
- `motion_controller`: selects active pose and publishes `cmd_vel`

## Setup

Clone the repository and run the following commands from the root of the cloned repo:

```bash
sudo apt update
pip3 install pygame
cd <cloned_repo>
colcon build
source install/setup.bash
```

## Launch

```bash
ros2 launch motion_controller full_stack.launch.py
```

## Launch Arguments

| Argument         | Default     | Description                           |
|------------------|-------------|---------------------------------------|
| `headless`       | `false`     | Skips the GUI (pygame window)         |
| `controller_exec`| `controller`| Custom motion controller executable   |
| `sim`            | `false`     | Placeholder for simulation code       |

## Behavior

- Follows GUI pose if one was issued in the last 30 seconds
- Falls back to clock-based pose otherwise
- Publishes velocity to `/cmd_vel`

## Plug-and-Play Motion Controller Integration

This stack is designed for easy testing of custom motion controllers. Your controller should:  

Subscribe to: `/target_pose` (`geometry_msgs/PoseStamped`)  
Publish to: `/cmd_vel` (`geometry_msgs/Twist`)  

## Examples

```bash
ros2 launch motion_controller full_stack.launch.py headless:=true
ros2 launch motion_controller full_stack.launch.py controller_exec:=my_controller
```
