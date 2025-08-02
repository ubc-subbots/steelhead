# Steelhead Claw Simulation Guide

## Launch Options

### Full System (Recommended)
```bash
ros2 launch steelhead_claw steelhead_with_claw.launch.py
```

### Separate Components
```bash
# Terminal 1 - Base AUV
ros2 launch triton_gazebo barebones_teleop_launch.py

# Terminal 2 - Claw System
ros2 launch steelhead_claw claw_only.launch.py
```

### WSL/Display Issues
If Gazebo GUI won't open, add `gui:=false`:
```bash
ros2 launch steelhead_claw steelhead_with_claw.launch.py gui:=false
```

## Claw Control

### Service Commands
```bash
# Open claw
ros2 service call /steelhead/claw/open std_srvs/srv/SetBool "{data: true}"

# Close claw
ros2 service call /steelhead/claw/close std_srvs/srv/SetBool "{data: true}"
```

### Keyboard Control
```bash
ros2 run steelhead_claw claw_teleop
```

## Monitoring
```bash
# Check nodes are running
ros2 node list

# Monitor claw state
ros2 topic echo /steelhead/claw/state
```