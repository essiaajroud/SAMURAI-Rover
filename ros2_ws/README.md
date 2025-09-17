# SAMURAI ROS2 Workspace

## Package Structure

### samurai_detection

- YOLO-based detection node
- Camera integration
- ROS2 messages for detections

### samurai_tracking

- Target tracking implementation
- Motion control
- State estimation

### samurai_simulation

- Gazebo simulation environment
- Virtual sensors
- Test scenarios

### samurai_msgs

- Custom message definitions
- Service definitions

## Building

```bash
# First time setup
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build

# Subsequent builds
source install/setup.bash
colcon build --packages-select samurai_detection  # Build specific package
```

## Testing

```bash
# Run tests
colcon test

# Run specific package tests
colcon test --packages-select samurai_detection
```

## Launch Files

### Full System

```bash
ros2 launch samurai_launch full_system.launch.py
```

### Simulation Only

```bash
ros2 launch samurai_simulation simulation.launch.py
```

### Detection Only

```bash
ros2 launch samurai_detection detection.launch.py
```

## Topics

- `/samurai/detections`: Detection results
- `/samurai/tracking`: Tracking status
- `/camera/color/image_raw`: RGB camera feed
- `/camera/depth/image_raw`: Depth camera feed
- `/cmd_vel`: Robot velocity commands

## Parameters

- `model_path`: Path to YOLO model
- `camera_topic`: Input camera topic
- `tracking_mode`: Tracking algorithm selection

## Notes

- Compatible with both ROS2 Foxy and Humble
- GPU acceleration supported when available
- Simulation provides full testing environment
