"""SO101 MoveIt Servo 启动文件.

独立于 so101_gazebo.launch.py: 假设 Gazebo 仿真(控制器/joint_states/TF)已在运行,
本文件只负责把 servo_node 拉起来并调用 start_servo 服务.

前提:
  1. 已启动 Gazebo 仿真:  ros2 launch so101_moveit_gazebo so101_gazebo.launch.py
  2. 已启动手柄遥控链:    ros2 launch ee_teleop ee_teleop.launch.py
                          (输出 TwistStamped 到 /ee_teleop/twist, 见 servo_parameters.yaml)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_file(path):
    with open(path, 'r') as f:
        return f.read()


def load_yaml(path):
    import yaml
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg_share = get_package_share_directory('so101_moveit_gazebo')

    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='servo_node 日志级别, 排查问题时用 debug')

    log_level = LaunchConfiguration('log_level')

    # xacro 展开为 robot_description —— 必须是 gazebo 版 (与 so101.srdf 的 world_joint 对齐)
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg_share, 'urdf', 'so101_gazebo.urdf.xacro')]),
        value_type=str)
    srdf = load_file(os.path.join(pkg_share, 'moveit_config', 'so101.srdf'))
    kinematics = load_yaml(os.path.join(pkg_share, 'moveit_config', 'kinematics.yaml'))

    # 也可改用 MoveItConfigsBuilder (需保留 moveit_config/ 的 install 与 package.xml 依赖):
    # moveit_config = (
    #     MoveItConfigsBuilder('so101', package_name='so101_moveit_gazebo')
    #     .robot_description(file_path='urdf/so101_gazebo.urdf.xacro')
    #     .robot_description_semantic(file_path='moveit_config/so101.srdf')
    #     .robot_description_kinematics(file_path='moveit_config/kinematics.yaml')
    #     .joint_limits(file_path='moveit_config/joint_limits.yaml')
    #     .pilz_cartesian_limits(file_path='moveit_config/pilz_cartesian_limits.yaml')
    #     .to_moveit_configs()
    # )

    # ServoParameters 的命名空间是 "moveit_servo", 包一层后 flat key
    # 展开为 moveit_servo.publish_period 等 (与官方 demo 相同手法)
    servo_params = {'moveit_servo': load_yaml(os.path.join(pkg_share, 'moveit_config', 'servo_parameters.yaml'))}

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': srdf},
            {'robot_description_kinematics': kinematics},
            servo_params,
            {'use_sim_time': True},
        ],
    )

    # 2.5.9 的 servo_node 启动后处于暂停态, 必须调 start_servo 才开始输出
    start_servo = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/servo_node/start_servo', 'std_srvs/srv/Trigger'],
        output='screen'
    )

    return LaunchDescription([
        log_level_arg,
        servo_node,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=servo_node,
                on_start=[
                    # 等 Servo 内部 PlanningScene/参数初始化完成
                    TimerAction(period=3.0, actions=[start_servo]),
                ])
        ),
    ])
