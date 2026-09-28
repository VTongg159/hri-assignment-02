from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")
    world = PathJoinSubstitution([FindPackageShare("ur3_llm_control"), "worlds", "hri_assignment.sdf"])
    controllers = PathJoinSubstitution(
        [FindPackageShare("ur3_llm_control"), "config", "ur_controllers.yaml"])
    description = PathJoinSubstitution(
        [FindPackageShare("ur3_llm_control"), "urdf", "ur3e_hri_gz.urdf.xacro"])
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ", description,
        " safety_limits:=false safety_pos_margin:=0.15 safety_k_position:=20",
        " name:=ur ur_type:=ur3e tf_prefix:='' simulation_controllers:=", controllers,
    ])
    robot_description = {"robot_description": robot_description_content}
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher", output="both",
        parameters=[{"use_sim_time": True}, robot_description])
    joint_state_spawner = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"])
    arm_controller_spawner = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"])
    spawn_robot = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-string", robot_description_content, "-name", "ur",
                   "-allow_renaming", "true"])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
        launch_arguments={"gz_args": IfElseSubstitution(
            gazebo_gui,
            if_value=[" -r -v 3 --physics-engine gz-physics-bullet-featherstone-plugin ", world],
            else_value=[" -s -r -v 3 --physics-engine gz-physics-bullet-featherstone-plugin ", world],
        )}.items())
    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"])
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(
            str(Path(__file__).parents[1] / "config" / "ur3e_robotiq.srdf.xacro"),
            {"name": "ur"})
        .joint_limits(file_path=str(
            Path(__file__).parents[1] / "config" / "moveit_joint_limits.yaml"))
        .to_moveit_configs()
    )
    # This small local override makes MoveIt use the same unscaled controller
    # that the Gazebo ros2_control launch activates.
    with open(Path(__file__).parents[1] / "config" / "moveit_controllers.yaml") as stream:
        moveit_controllers = yaml.safe_load(stream)
    wait_robot_description = Node(
        package="ur_robot_driver", executable="wait_for_robot_description", output="screen")
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[moveit_config.to_dict(), moveit_controllers,
                    {"use_sim_time": True,
                     "publish_robot_description_semantic": True}])
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2_moveit", output="log",
        condition=IfCondition(launch_rviz),
        # Use the assignment-specific RViz profile, including the HRI scene
        # markers and the interactive orange MoveIt goal-state overlay.
        arguments=["-d", PathJoinSubstitution(
            [FindPackageShare("ur3_llm_control"), "config", "moveit.rviz"])],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                    {"use_sim_time": True}])
    start_moveit = RegisterEventHandler(OnProcessExit(
        target_action=wait_robot_description, on_exit=[move_group, rviz]))
    gripper_controller = Node(
        package="controller_manager", executable="spawner",
        name="spawner_robotiq_gripper_controller", output="screen",
        arguments=["robotiq_gripper_controller", "-c", "/controller_manager",
                   "--controller-manager-timeout", "60"])
    # sdformat rewrites package:// URIs as model:// URIs. Gazebo therefore
    # needs the directory containing the package, not only AMENT_PREFIX_PATH.
    gazebo_resource_path = str(Path(get_package_share_directory("robotiq_description")).parent)
    return LaunchDescription([
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            [gazebo_resource_path, ":", PathJoinSubstitution(
                [FindPackageShare("ur_description"), ".."])]),
        robot_state_publisher, joint_state_spawner, arm_controller_spawner,
        spawn_robot, gazebo, clock_bridge,
        wait_robot_description, start_moveit, gripper_controller,
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             arguments=["/world/hri_assignment/set_pose@ros_gz_interfaces/srv/SetEntityPose"]),
        Node(package="ur3_llm_control", executable="skill_server", output="screen",
             parameters=[moveit_config.robot_description_kinematics,
                         moveit_config.joint_limits,
                         {"use_sim_time": True}]),
    ])
