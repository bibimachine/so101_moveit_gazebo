import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('so101_moveit_gazebo')
    # Gazebo 固定使用这份静态 URDF（由 so101.urdf.xacro 生成，不含 XML 注释，
    # 规避 gazebo_ros2_control 转发 robot_description 时的 rcl 参数解析问题）
    default_model_path = os.path.join(pkg_share, 'urdf', 'so101_gazebo.urdf')
    robot_name_in_model = 'so101'

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='SO101 静态 URDF 文件的绝对路径')

    # 直接读取 URDF 文件内容（不使用 xacro 命令行）
    with open(default_model_path, 'r') as f:
        robot_description = f.read()

    # 发布 TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # 启动 Gazebo Classic（默认空世界）
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
                    'gazebo_ros'), '/launch', '/gazebo.launch.py']),
    )

    # 请求 Gazebo 加载机器人
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model,
                   '-z', '0.02'],
        output='screen'
    )

    # 机器人加载完成后启动控制器（controller_manager 由 gazebo_ros2_control 插件内部运行）
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['so101_joint_state_broadcaster',
                   '--controller-manager', '/controller_manager']
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['so101_arm_controller',
                   '--controller-manager', '/controller_manager']
    )
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['so101_gripper_controller',
                   '--controller-manager', '/controller_manager']
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[
                    joint_state_broadcaster_spawner,
                    arm_controller_spawner,
                    gripper_controller_spawner,
                ],
            )
        ),
    ])
