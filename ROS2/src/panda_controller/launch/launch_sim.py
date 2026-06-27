import os
import tempfile
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('panda_controller')
    urdf_file        = os.path.join(pkg, 'config', 'fer_fake.urdf.xacro')
    controllers_yaml = os.path.join(pkg, 'config', 'ros2_controllers.yaml')
    rviz_config      = os.path.join(pkg, 'config', 'panda.rviz')

    # Expand xacro at launch time so all included files and math are resolved
    robot_description_xml = subprocess.run(
        ['xacro', urdf_file], capture_output=True, text=True, check=True
    ).stdout

    # controller_manager requires robot_description in its own parameter namespace.
    # We write it to a temporary YAML file and pass it alongside ros2_controllers.yaml.
    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    escaped = robot_description_xml.replace('\\', '\\\\').replace('"', '\\"')
    tmp.write(f'controller_manager:\n  ros__parameters:\n    robot_description: "{escaped}"\n')
    tmp.flush()

    # robot_state_publisher: broadcasts TF frames from the URDF for RViz
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_xml}],
    )

    # ros2_control_node: runs controller_manager + mock_components hardware
    # mock_components/GenericSystem with position interface mirrors commanded
    # positions directly to state interfaces — no physics, no integration needed.
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[tmp.name, controllers_yaml],
    )

    # Spawn joint_state_broadcaster after 2 s (controller_manager needs time to start)
    # It reads state interfaces and publishes /joint_states
    spawn_jsb = TimerAction(period=2.0, actions=[ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner',
             'joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )])

    # Spawn position_controller after 3 s (after joint_state_broadcaster is active)
    # It exposes /position_controller/commands (Float64MultiArray) → position interface
    spawn_pos = TimerAction(period=3.0, actions=[ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner',
             'position_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )])

    # RViz for visualisation
    rviz = Node(
        package='rviz2', executable='rviz2',
        output='screen', arguments=['-d', rviz_config],
    )

    # CLIK controller node: starts after 5 s to ensure both controllers are active
    panda_controller = TimerAction(period=5.0, actions=[Node(
        package='panda_controller', executable='panda_controller', output='screen',
    )])

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        spawn_jsb,
        spawn_pos,
        rviz,
        panda_controller,
    ])
