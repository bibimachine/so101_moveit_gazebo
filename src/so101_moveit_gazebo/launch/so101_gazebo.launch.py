import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler,
                            SetEnvironmentVariable, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('so101_moveit_gazebo')
    # Gazebo 版 xacro（gazebo 硬件 + gazebo_ros2_control 插件块）
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

    # 启动 Gazebo Classic（so101.world：空地 + 太阳 + model_states/link_states 插件，
    # 后者用于夹取时读取物体位姿。注意 gazebo.launch.py 不透传参数，
    # 需直接 include gzserver/gzclient）
    # mesh 路径在 xacro 里用 $(find so101_moveit_gazebo) 展开为绝对路径，
    # 不再依赖 GAZEBO_MODEL_PATH 环境变量
    # GAZEBO_MODEL_DATABASE_URI 置空：gzclient 启动时会联网拉取在线模型列表
    # （models.gazebosim.org），网络不通时 GUI 会卡死/不刷新（见问题排查记录第 8 条）；
    # 本包模型全部本地，禁掉在线库无副作用
    gazebo_share = get_package_share_directory('gazebo_ros')
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_share, '/launch/gzserver.launch.py']),
        launch_arguments=[('verbose', 'true'),
                          ('world', os.path.join(
                              pkg_share, 'worlds', 'so101.world'))]
    )
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_share, '/launch/gzclient.launch.py'])
    )
    disable_model_db = SetEnvironmentVariable(
        'GAZEBO_MODEL_DATABASE_URI', '')

    # 请求 Gazebo 加载机器人
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model,
                   '-z', '0.02'],
        output='screen'
    )

    # 测试方块（2cm 立方体，effort 夹爪抓取用），延时等 Gazebo 就绪
    spawn_cube_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', os.path.join(pkg_share, 'objects', 'cube.urdf'),
                   '-entity', 'cube',
                   '-x', '0.303', '-y', '0.009', '-z', '0.21'],
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

    # RViz 显示（复用 display 的布局；use_sim_time 对齐 Gazebo 的 /clock）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'so101_display.rviz')],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        model_arg,
        # 必须在 gzclient 启动前设置（launch 按顺序执行动作）
        disable_model_db,
        robot_state_publisher_node,
        gzserver_launch,
        gzclient_launch,
        rviz_node,
        # spawn 延时 5s:WSLg 下 gzclient 连接服务器需要数秒,spawn 太早客户端
        # 没就绪会丢模型视觉(画面里隐形,见问题排查记录#1);太晚则拉长"控制器
        # 未激活"的重力下垂窗口。5s 两边都照顾到,且 grasp_cube.py 会先回标定位形
        TimerAction(
            period=5.0,
            actions=[spawn_entity_node]
        ),
        # 方块在机器人之后生成（不与机械臂初始位姿干涉）
        TimerAction(
            period=6.0,
            actions=[spawn_cube_node]
        ),
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
