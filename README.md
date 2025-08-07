# ROS Playground

# Index

## Main Workspace [↗](./main_ws/) 

## ROS2 Control Demo [↗](./demo_ws/)

[Getting started](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)
[Build from package](https://control.ros.org/master/doc/ros2_control_demos/doc/index.html#build-from-debian-packages)

# Notes

## URDF Workflow

* Use Blender + Phobo Add-on to build and export

* Use URDF Visualizer VSCode plugin to preview. NOTE: assoicate .urdf to xml in Settings » File Associations

* Add Gazebo plugins
[Tutorial](https://gazebosim.org/api/sim/9/jointcontrollers.html)
```xml
  <!-- Gazebo joint position controller plugin -->
  <gazebo>
    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>Hip_joint</joint_name>
      <topic>/docky/hip_angle</topic>
    </plugin>
  </gazebo>
```

* Start Gazebo
```bash
gz sim
```
Use it in `http://localhost:8080/vnc.html`

* Load URDF
```bash
gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/main_ws/description/docky/urdf/docky.urdf", name: "docky_model"'
```

* Publish a topic
```bash
gz topic -t "/docky/hip_angle" -m gz.msgs.Double -p "data: 1"
```


## Resources

[Gazebo Official Tutorial](https://gazebosim.org/docs/harmonic/tutorials/)