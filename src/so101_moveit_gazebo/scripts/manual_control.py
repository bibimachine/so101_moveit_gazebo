#!/usr/bin/env python3
"""SO101 Gazebo 手动控制台。

用法:
    ros2 run so101_moveit_gazebo manual_control.py

前提:Gazebo 仿真已启动(so101_gazebo.launch.py),三个控制器 active。

主菜单:
    1) 生成小圆柱   —— 地面随机位置(x 0.10~0.30, y ±0.15),生成后确认
    2) 删除小圆柱   —— 删除完成后提示
    3) 移动机械臂   —— 子菜单:选关节输入目标角度,或开/关夹爪;
                       关闭夹爪用默认可夹起圆柱的力(-0.1 N·m),
                       每次关闭后按夹爪关节角度检测一次(没合到下限=夹到东西),会提示
    4) 查看状态     —— 打印各关节当前角、夹爪、圆柱位姿
    q) 退出

输入时用 Ctrl+C 可随时退出整个脚本。
"""
import os
import random
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
GRIP_CLOSE_EFFORT = -0.1  # N·m,负值端才是闭合(URDF 关节下限 -0.17=闭合,上限 1.74=张开)
GRIP_OPEN_EFFORT = 0.1
GRIP_CLOSED_LIMIT = -0.1745  # 关节下限:空夹闭合时钳口最终停的位置
# 地面随机生成区域(可达范围,单位 m)
SPAWN_AREA = {'x': (0.10, 0.30), 'y': (-0.15, 0.15)}


class ManualControl(Node):

    @staticmethod
    def read_line(prompt):
        """从 fd 0 无缓冲读一行(避免 Python stdin 缓冲把后续按键预读吞掉)。"""
        sys.stdout.write(prompt)
        sys.stdout.flush()
        buf = b''
        while True:
            ch = os.read(0, 1)
            if not ch:
                raise EOFError
            if ch == b'\n':
                break
            buf += ch
        return buf.decode(errors='replace').strip()

    def __init__(self):
        super().__init__('so101_manual_control')
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/so101_gripper_controller/commands', 1)
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/so101_arm_controller/joint_trajectory', 1)
        self._joint_state = None
        self._model_state = None
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        self.create_subscription(ModelStates, '/model_states', self._on_model_state, 10)

    # ---------- 订阅回调 ----------
    def _on_joint_state(self, msg):
        self._joint_state = msg

    def _on_model_state(self, msg):
        self._model_state = msg

    # ---------- 状态读取 ----------
    def refresh(self, seconds=0.3):
        """泵一会儿事件,让订阅数据更新。"""
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def joint_positions(self, refresh_s=0.3):
        """返回 {关节名: 位置},未收齐返回 None。"""
        self.refresh(refresh_s)
        if not self._joint_state:
            return None
        names = list(self._joint_state.name)
        if not all(j in names for j in ARM_JOINTS + ['gripper']):
            return None
        return {j: self._joint_state.position[names.index(j)]
                for j in ARM_JOINTS + ['gripper']}

    def cylinder_pose(self):
        """返回圆柱 (x, y, z),不存在返回 None。"""
        self.refresh(0.3)
        if self._model_state and 'cylinder' in self._model_state.name:
            i = self._model_state.name.index('cylinder')
            p = self._model_state.pose[i].position
            return (p.x, p.y, p.z)
        return None

    # ---------- 动作 ----------
    def send_joint(self, joint, target, duration=2.0):
        """单关节移动到目标角,其余关节保持在当前值。"""
        pos = self.joint_positions()
        if pos is None:
            print('!! 读不到关节状态')
            return False
        joints = ARM_JOINTS if joint in ARM_JOINTS else [joint]
        msg = JointTrajectory()
        msg.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = [target if j == joint else pos[j] for j in joints]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        msg.points = [point]
        self.traj_pub.publish(msg)
        time.sleep(duration + 0.5)
        return True

    def send_effort(self, value):
        msg = Float64MultiArray()
        msg.data = [value]
        self.gripper_pub.publish(msg)

    def spawn_cylinder(self):
        if self.cylinder_pose() is not None:
            print('!! 世界里已有圆柱,先删除再生成')
            return
        x = random.uniform(*SPAWN_AREA['x'])
        y = random.uniform(*SPAWN_AREA['y'])
        urdf = open(get_package_share_directory('so101_moveit_gazebo')
                    + '/objects/cylinder.urdf').read()
        cli = self.create_client(SpawnEntity, '/spawn_entity')
        if not cli.wait_for_service(timeout_sec=5.0):
            print('!! /spawn_entity 服务不可用')
            return
        req = SpawnEntity.Request()
        req.name = 'cylinder'
        req.xml = urdf
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = 0.031   # 圆柱高 6cm,半高 0.03
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        r = fut.result()
        if not r or not r.success:
            print(f'!! spawn 失败:{r.status_message if r else "无响应"}')
            return
        # 等服务端真正放进去
        if self.cylinder_pose() is None:
            time.sleep(1.0)
        print(f'OK 圆柱已生成在 ({x:.3f}, {y:.3f})')

    def delete_cylinder(self):
        if self.cylinder_pose() is None:
            print('!! 世界里没有圆柱')
            return
        cli = self.create_client(DeleteEntity, '/delete_entity')
        if not cli.wait_for_service(timeout_sec=3.0):
            print('!! /delete_entity 服务不可用')
            return
        rclpy.spin_until_future_complete(
            self, cli.call_async(DeleteEntity.Request(name='cylinder')), timeout_sec=3.0)
        deadline = time.time() + 3.0
        while time.time() < deadline and self.cylinder_pose() is not None:
            time.sleep(0.2)
        print('OK 圆柱已删除' if self.cylinder_pose() is None else '!! 圆柱还在,删除未生效')

    def check_grasped(self):
        """用夹爪关节角度判断夹到东西没有。

        空夹时钳口会一路闭合到关节下限(-0.1745);钳口之间有圆柱时,
        关节停在比下限明显更开的位置。比读 /model_states 位姿简单,
        不依赖 world 里的状态插件。
        """
        pos = self.joint_positions()
        if pos is None:
            return False
        return pos['gripper'] > GRIP_CLOSED_LIMIT + 0.05

    # ---------- 菜单 ----------
    def show_status(self):
        pos = self.joint_positions()
        if pos:
            for j in ARM_JOINTS + ['gripper']:
                print(f'  {j:14s} = {pos[j]:+.3f} rad')
        cylinder = self.cylinder_pose()
        print(f'  圆柱: {tuple(round(v, 3) for v in cylinder) if cylinder else "无"}')

    def move_arm_menu(self):
        while rclpy.ok():
            pos = self.joint_positions()
            if pos is None:
                print('!! 读不到关节状态,回主菜单')
                return
            print('\n-- 移动机械臂 --')
            for i, j in enumerate(ARM_JOINTS, 1):
                print(f'  {i}) {j:14s} 当前 {pos[j]:+.3f} rad')
            grip_state = '闭合' if pos['gripper'] < 0.5 else '张开'
            print(f'  o) 打开夹爪    c) 关闭夹爪(默认 {GRIP_CLOSE_EFFORT} N·m,夹到会提示)   当前:{grip_state}')
            print('  b) 返回上一级')
            choice = self.read_line('选择: ').lower()
            try:
                if choice == 'b':
                    return
                if choice == 'o':
                    self.send_effort(GRIP_OPEN_EFFORT)
                    print('OK 夹爪已打开')
                    continue
                if choice == 'c':
                    self.send_effort(GRIP_CLOSE_EFFORT)
                    print(f'OK 夹爪闭合指令已发({GRIP_CLOSE_EFFORT} N·m),等待夹稳...')
                    time.sleep(2.0)
                    if self.check_grasped():
                        print('*** 夹到圆柱了! ***')
                    else:
                        print('.. 没夹到圆柱')
                    continue
                idx = int(choice) - 1
                if not (0 <= idx < len(ARM_JOINTS)):
                    print('!! 无效选择')
                    continue
                self.adjust_joint(ARM_JOINTS[idx])
            except (ValueError, EOFError):
                print('!! 无效输入')

    # ---------- 关节调整 ----------
    def adjust_joint(self, joint):
        """输入目标角度(相对当前增减),发给 arm_controller 执行。"""
        pos = self.joint_positions()
        raw = self.read_line(f'目标角度 rad(当前 {pos[joint]:+.3f},回车取消): ')
        if raw:
            try:
                self.send_joint(joint, float(raw), 2.0)
            except ValueError:
                print('!! 不是数字')


def main():
    rclpy.init()
    node = ManualControl()
    try:
        print('SO101 手动控制台(前提:Gazebo 仿真已启动,控制器 active)')
        while rclpy.ok():
            print('\n== 主菜单 ==')
            print('  1) 生成小圆柱(地面随机)')
            print('  2) 删除小圆柱')
            print('  3) 移动机械臂')
            print('  4) 查看状态')
            print('  q) 退出')
            choice = node.read_line('选择: ').lower()
            if choice == 'q':
                break
            if choice == '1':
                node.spawn_cylinder()
            elif choice == '2':
                node.delete_cylinder()
            elif choice == '3':
                node.move_arm_menu()
            elif choice == '4':
                node.show_status()
            else:
                print('!! 无效选择')
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
