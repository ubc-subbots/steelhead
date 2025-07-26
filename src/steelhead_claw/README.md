Launch the full system:

bash# Launch Gazebo + AUV + Controls + Claw
ros2 launch steelhead_claw steelhead_with_claw.launch.py

# Or use your existing barebones launch + claw separately:
# Terminal 1:
ros2 launch triton_gazebo barebones_teleop_launch.py

# Terminal 2:
ros2 launch steelhead_claw claw_only.launch.py

Test the claw:

bash# Service calls:
ros2 service call /steelhead/claw/open std_srvs/srv/SetBool "{data: true}"
ros2 service call /steelhead/claw/close std_srvs/srv/SetBool "{data: true}"

# Or use the teleop for keyboard control:
ros2 run steelhead_claw claw_teleop