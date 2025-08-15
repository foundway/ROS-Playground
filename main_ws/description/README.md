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

## URDF+XACRO Workflow
* Use Blender's Phobo add-on
* Store blender file in `description/.`
* Export with the ros_package option. It would export to `$MODEL_NAME/urdf` and `$MODEL_NAME/mesh`
* Set ros_package name to `description/$MODEL_NAME` to set the mesh path correctly
* Ignore cannot find CMakeList.txt error
* Use a xxx.xacro file to include the .urdf and plugins
* Use `xacro` to process the xacro file in the launch file