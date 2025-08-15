from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", " -r -v 3 empty.sdf")],
    )

    # Set Gazebo resource path
    gazebo_resource_path = Command(
        [
            "bash -c 'export GZ_SIM_RESOURCE_PATH=",
            PathJoinSubstitution([FindPackageShare("description")]),
            ":$GZ_SIM_RESOURCE_PATH && echo \"Resource path set to: $GZ_SIM_RESOURCE_PATH\"'",
        ]
    )

    # Gazebo bridge
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Robot description - use xacro to process the URDF file
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("description"), "docky/urdf", "docky.urdf.xacro"]
            ),
            " ",
            "use_gazebo:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot state publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Spawn robot in Gazebo with delay to ensure robot_description is ready
    gz_spawn_entity = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-topic",
                    "/robot_description",
                    "-name",
                    "docky",
                    "-allow_renaming",
                    "true",
                    "-x", "0",
                    "-y", "0", 
                    "-z", "0",
                ],
                parameters=[{"use_sim_time": True}],
            )
        ]
    )

    nodes = [
        gazebo,
        gazebo_bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
    ]

    return LaunchDescription(nodes)
