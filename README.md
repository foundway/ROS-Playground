# ROS Playground

# Notes

## URDF Workflow

* Use Blender + Phobo Add-on to build and export

* Use URDF Visualizer VSCode plugin to preview. NOTE: assoicate .urdf to xml in Settings » File Associations

* Start Gazebo
```
gz sim
```

* Load URDF
```
gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/main_ws/description/docky/urdf/docky.urdf", nam
e: "docky_model"'
```

## Resources

[Gazebo Official Tutorial](https://gazebosim.org/docs/harmonic/tutorials/)