```bash
cd /main_ws
colcon build --packages-select description
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch description docky_gz.launch.py
```

## To inspect the built description
```bash
ros2 topic echo /robot_description -f
ros2 param get /robot_state_publisher robot_description
```