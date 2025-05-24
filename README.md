# ROS2 Galactic Tests

## Build Instructions

### 1. Clone the repository
[Terminal 1]
```bash
cd ~/ros2_ws/src
git clone https://github.com/kailash197/cp23_ros2test_tortoisebot_waypoints.git
git checkout master

```
### 2. Use `colcon` to build the project
[Terminal 1]
```bash
source /opt/ros/galactic/setup.bash
cd ~/ros2_ws && colcon build --packages-select tortoisebot_waypoints && source install/setup.bash

```

## Test Instructions

### 1. Launch the ROS2 simulation for the TortoiseBot
[Terminal 1]
```bash
source /opt/ros/galactic/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=True

```

**Note**: Abort and re-launch Gazebo if there are any issues. Use `kill -9 <gazebo_pid>`

### 2. Launch the Waypoints Action Server for ROS2
[Terminal 2]
```bash
source /opt/ros/galactic/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run tortoisebot_waypoints tortoisebot_action_server

```

### 3. Verify passing Conditions

Execute the test file and verify if all the tests are passing properly with passing conditions.  
[Terminal 3]
```bash
source /opt/ros/galactic/setup.bash
cd ~/ros2_ws && colcon build && source install/setup.bash
colcon test --packages-select tortoisebot_waypoints --event-handler=console_direct+
colcon test-result --all

```

Expected output:
```bash

Summary: 2 tests, 0 errors, 0 failures, 0 skipped

```

### 4. Verify failing Conditions
The code provides the failing conditions which can be selected by passing ROS2 parameter `test_pass` with value `false`.  
[Terminal 3]
```bash
source /opt/ros/galactic/setup.bash
cd ~/ros2_ws && colcon build && source install/setup.bash
colcon test --packages-select tortoisebot_waypoints --event-handler=console_direct+
colcon test-result --all

```

Expected output:
```bash

Summary: 2 tests, 1 errors, 0 failures, 1 skipped

```
