# Bottle Recycling Classification - UR3 Pick & Place

Repository for Artifical Intelligence in Robotics final project code, mounted on top of the 
relevant RS2 project. 

## Workspace Setup

Navigate to the ROS2 workspace:

```bash
cd ~/ros2_ws
```

Source ROS2 and the workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Build the workspace:

```bash
colcon build --symlink-install
```

---

## Pre-Run Checklist

Before launching the system:

- Connect the UR3e robot to the laptop via Ethernet.
- Ensure the emergency stop is within reach.
- Verify the workspace is clear of people and obstructions.
- Power on the robot.
- Ensure all joints are unlocked.
- Return the robot to the home position.
- Connect the RGBD camera.
- Open `mission_control_sim.launch.py` in the `opencv_camera_cpp` package.
- Verify the configured robot IP matches the connected robot.

Example:

```python
'robot_ip': '192.168.0.194'
```

---

## Launch Procedure

### Terminal 1 - Mission Control

```bash
ros2 launch opencv_camera_cpp mission_control_sim.launch.py
```

### Terminal 2 - Robot Driver

Verify the robot IP before launching.

```bash
ros2 launch ur_onrobot_control start_robot.launch.py \
  ur_type:=ur3e \
  robot_ip:=192.168.0.191 \
  use_fake_hardware:=false \
  launch_rviz:=false
```

### Terminal 3 - Gripper Controllers and Camera Transform

Activate gripper controllers:

```bash
ros2 control set_controller_state finger_width_trajectory_controller inactive

ros2 control set_controller_state finger_width_controller inactive

ros2 control set_controller_state finger_width_controller active

ros2 control list_controllers | grep finger
```

Publish the camera transform:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.0 \
  --y 0.022 \
  --z 0.144 \
  --roll 0 \
  --pitch -1.5708 \
  --yaw 0 \
  --frame-id tool0 \
  --child-frame-id camera_link
```

### Terminal 4 - Motion Node

Verify the parameter file path before launching.

```bash
ros2 run ur3e_motion move_to_pose \
  --ros-args \
  --params-file ~/RS2_Repo/RS2_12_3_06/kinematics_onrobot.yaml
```

---

## GUI Operation

| Step | Action |
|------|--------|
| 1 | Press **Start** |
| 2 | Wait for robot to reach the first viewing position |
| 3 | Press **Locate Bottle** |
| 4 | Press **Move to 2nd View** |
| 5 | Wait for robot motion to complete |
| 6 | Press **Detect Bottle** |
| 7 | Press **Complete Mission** |
| 8 | Robot picks up bottle |
| 9 | Robot drops bottle at the designated location |

---

## Dependencies

### ROS 2 Packages

Replace `humble` below with your ROS 2 distribution (e.g. `iron`, `jazzy`, etc.)

```bash
sudo apt update
sudo apt install -y \
  ros-humble-tf2-ros \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-vision-msgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager
```

---

### Python / ML Dependencies (SVM + YOLOv8n ONNX)

#### Core ML + Vision Stack

```bash
pip3 install numpy opencv-python scikit-learn
```

---

#### YOLOv8n + ONNX Runtime

```bash
pip3 install ultralytics onnx onnxruntime
```

---

### Software

| Software | Version |
|-----------|----------|
| Ubuntu | 22.04 |
| ROS2 | Humble |
| OpenCV | 4.9+ |
| Python | 3.10+ |

### Hardware

- UR3e Robot
- OnRobot Gripper
- RGBD Camera

---

## Important Notes

- Verify all robot IP addresses before launching.
- Ensure the kinematics YAML path is correct.
- Keep the emergency stop accessible at all times.
- Do not enter the robot workspace while the system is operating.

