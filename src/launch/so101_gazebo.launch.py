import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('so101_moveit_gazebo')
    # Gazebo 版 xacro（gazebo 硬件 + gazebo_ros2_control 插件块）
    # 注意：xacro 注释中不要使用英文冒号，会触发 rcl 参数解析 bug
    default_model_path = os.path.join(pkg_share, 'urdf', 'so101_gazebo.urdf.xacro')
    robot_name_in_model = 'so101'

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='SO101 gazebo 版 xacro 文件的绝对路径')

    # 运行 xacro 生成 robot_description
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str)

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
                    'gazebo_ros'), '/launch/gazebo.launch.py']),
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

    # 机器人生成成功后，按顺序链式加载并激活控制器
    # （controller_manager 由 gazebo_ros2_control 插件内部运行，
    #   load_controller 命令会自行等待 /controller_manager 服务就绪）
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'so101_joint_state_broadcaster',
             '--set-state', 'active'],
        output='screen'
    )
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'so101_arm_controller',
             '--set-state', 'active'],
        output='screen'
    )
    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'so101_gripper_controller',
             '--set-state', 'active'],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity_node,
        # 事件动作，机器人生成结束后加载 joint_state_broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[load_joint_state_controller])
        ),
        # joint_state_broadcaster 加载完成后加载手臂控制器
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_arm_controller])
        ),
        # 手臂控制器加载完成后加载夹爪控制器
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_arm_controller,
                on_exit=[load_gripper_controller])
        ),
    ])
