import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('so101_moveit_gazebo')
    default_model_path = os.path.join(pkg_share, 'urdf', 'so101.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'so101_display.rviz')
    default_controllers_path = os.path.join(pkg_share, 'config', 'so101_ros2_control.yaml')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='SO101 URDF/xacro 文件的绝对路径')
    rviz_config_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value=str(default_rviz_config_path),
        description='RViz 配置文件的绝对路径')

    # 运行 xacro 生成 robot_description（默认 mock_components 硬件）
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str)

    # 发布 TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # ros2_control 控制管理器（加载 mock 硬件 + 控制器配置）
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            default_controllers_path
        ],
        output='screen'
    )

    # 启动控制器
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

    # RViz 显示
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    return LaunchDescription([
        model_arg,
        rviz_config_arg,
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        rviz_node
    ])
