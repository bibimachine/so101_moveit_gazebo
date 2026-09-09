#!/usr/bin/env python3
"""夹取方块演示:effort 闭合夹爪 -> 抬手臂 -> 验证方块跟随。

用法:
    ros2 run so101_moveit_gazebo grasp_cube.py [选项]

前提:Gazebo 仿真已启动(so101_gazebo.launch.py),三个控制器 active。
流程与问题排查记录第 12 条的手动验证一致,封装成脚本方便反复试验。

选项:
    --respawn          试验前重置方块(默认开启;先回标定位形再删除重新 spawn)
    --no-respawn       跳过重置(只有方块已在钳口之间时才用)
    --effort 0.1       闭合 effort(N·m),正值闭合
    --open-effort -0.1 张开 effort,--release 时用到
    --release          验证完张开放开方块
    --wrist-flex -0.95 抬升目标(腕关节角度,rad;标定位形约 -0.48)
    --duration 3.0     抬升轨迹时长(s)
    --close-time 2.0   闭合后等待夹稳的时间(s)
    --cube-x/y/z       方块 spawn 位置(默认标定值,对应标定位形)

退出码:0=夹取成功(方块跟随上升),1=失败。
"""
import argparse
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import LinkStates, ModelStates
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
# 方块 spawn 坐标对应的标定位形(spawn 前先把手臂回到这里)
CALIB_POSE = {'shoulder_pan': 0.0, 'shoulder_lift': 0.077,
              'elbow_flex': 0.334, 'wrist_flex': -0.483, 'wrist_roll': 0.0}


class GraspDemo(Node):

    def __init__(self, args):
        super().__init__('so101_grasp_cube')
        self.args = args
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/so101_gripper_controller/commands', 1)
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/so101_arm_controller/joint_trajectory', 1)
        self._joint_state = None
        self._model_state = None
        self._link_state = None
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        self.create_subscription(ModelStates, '/model_states', self._on_model_state, 10)
        self.create_subscription(LinkStates, '/link_states', self._on_link_state, 10)

    def _on_link_state(self, msg):
        self._link_state = msg

    def jaw_midpoint(self, timeout=5.0):
        """读 gripper_link 与 moving_jaw 的世界坐标,返回中点(x, y, z)。

        spawn 方块用这个中点而不是写死坐标——钳口张角/臂姿变化都会反映进去。
        """
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if not self._link_state:
                continue
            names = self._link_state.name
            if 'so101::gripper_link' in names and 'so101::moving_jaw_so101_v1_link' in names:
                p1 = self._link_state.pose[names.index('so101::gripper_link')].position
                p2 = self._link_state.pose[names.index('so101::moving_jaw_so101_v1_link')].position
                return ((p1.x + p2.x) / 2, (p1.y + p2.y) / 2, (p1.z + p2.z) / 2)
        return None

    def _on_joint_state(self, msg):
        self._joint_state = msg

    def _on_model_state(self, msg):
        self._model_state = msg

    def cube_z(self, timeout=5.0):
        """读方块世界系 z,不存在返回 None。"""
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._model_state and 'cube' in self._model_state.name:
                i = self._model_state.name.index('cube')
                return self._model_state.pose[i].position.z
        return None

    def arm_positions(self, timeout=5.0):
        """读当前手臂 5 关节位置,超时返回 None。"""
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._joint_state and all(
                    j in self._joint_state.name for j in ARM_JOINTS):
                return {j: self._joint_state.position[self._joint_state.name.index(j)]
                        for j in ARM_JOINTS}
        return None

    def send_effort(self, value):
        msg = Float64MultiArray()
        msg.data = [value]
        self.gripper_pub.publish(msg)

    def send_trajectory(self, positions, duration):
        """发一条单点轨迹(positions 为 ARM_JOINTS 顺序的位置列表)。"""
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        msg.points = [point]
        self.traj_pub.publish(msg)

    def wait_arm_reached(self, target, tolerance=0.03, timeout=15.0):
        """轮询关节位置,全部进入容差返回 True。"""
        end = time.time() + timeout
        while time.time() < end:
            pos = self.arm_positions(timeout=2.0)
            if pos and all(abs(pos[j] - target[j]) < tolerance for j in ARM_JOINTS):
                return True
        return False

    def respawn_cube(self):
        # 1) 回标定位形:方块 spawn 坐标按此位形标定;爪/contact 受力会让腕关节漂移,
        #    不在标定位形 spawn,方块可能砸在爪背甚至被挤出世界
        self.get_logger().info('手臂回到标定位形...')
        self.send_trajectory([CALIB_POSE[j] for j in ARM_JOINTS], duration=4.0)
        if not self.wait_arm_reached(CALIB_POSE):
            self.get_logger().warn('回标定位形超时,仍继续(可能影响落点)')
        time.sleep(1.0)

        # 1.5) 张开夹爪:spawn 坐标按"全张"(关节下限 -0.1745)标定,
        #     钳口半闭时方块会落在爪背/被弹飞。回位期间 effort=0 爪会自由下落,
        #     这里主动给负 effort 确保全张到位再 spawn
        self.send_effort(-0.15)
        time.sleep(1.5)

        # 2) 删除旧方块(没有就跳过)
        del_cli = self.create_client(DeleteEntity, '/delete_entity')
        if del_cli.wait_for_service(timeout_sec=3.0):
            rclpy.spin_until_future_complete(
                self, del_cli.call_async(DeleteEntity.Request(name='cube')), timeout_sec=3.0)
            self.get_logger().info('已请求删除旧方块')
            # 等删除真正生效(同名 delete+spawn 背靠背会竞态:删除若延迟生效,
            # 可能把刚 spawn 的新方块一起删掉),最多等 3s
            deadline = time.time() + 3.0
            while time.time() < deadline:
                if self.cube_z(timeout=0.5) is None:
                    break
            else:
                self.get_logger().warn('旧方块 3s 后仍在,继续(可能撞上同名冲突)')

        # 3) spawn 新方块到钳口之间(带验证,失败重试一次)
        # spawn 点:未显式指定时取两颚世界坐标中点上方 3cm,方块自由落进钳口 V 型槽,
        # 不受钳口张角/臂姿漂移影响
        if self.args.cube_x is None:
            mid = self.jaw_midpoint()
            if mid is None:
                self.get_logger().error('读不到夹爪 link 位姿,/link_states 不可用?')
                sys.exit(2)
            spawn_pos = (mid[0], mid[1], mid[2] - 0.01)
            self.get_logger().info(f'钳口中点 {tuple(round(v,3) for v in mid)},低于中点 1cm 放下方块(落进钳口 V 型槽)')
        else:
            spawn_pos = (self.args.cube_x, self.args.cube_y, self.args.cube_z)
        urdf_path = get_package_share_directory('so101_moveit_gazebo') + '/objects/cube.urdf'
        with open(urdf_path) as f:
            urdf = f.read()
        spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        if not spawn_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/spawn_entity 服务不可用')
            sys.exit(2)
        for attempt in (1, 2):
            req = SpawnEntity.Request()
            req.name = 'cube'
            req.xml = urdf
            req.initial_pose.position.x = spawn_pos[0]
            req.initial_pose.position.y = spawn_pos[1]
            req.initial_pose.position.z = spawn_pos[2]
            fut = spawn_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if not fut.result() or not fut.result().success:
                self.get_logger().error(f'spawn 方块失败:{fut.result().status_message if fut.result() else "服务无响应"}')
                sys.exit(2)
            # 服务返回 success 不等于模型真的在了——等 /model_states 确认
            if self.cube_z(timeout=3.0) is not None:
                break
            self.get_logger().warn(f'spawn 第 {attempt} 次后方块未出现,重试')
        else:
            self.get_logger().error('方块两次 spawn 均未在 /model_states 出现')
            sys.exit(2)
        self.get_logger().info(f'方块已 spawn 到 {tuple(round(v,3) for v in spawn_pos)}')
        time.sleep(2.0)  # 等方块落到钳口之间稳定

    def lift(self):
        pos = self.arm_positions()
        if pos is None:
            self.get_logger().error('读不到关节状态,手臂控制器未激活?')
            sys.exit(2)
        pos['wrist_flex'] = self.args.wrist_flex
        self.send_trajectory([pos[j] for j in ARM_JOINTS], self.args.duration)
        self.get_logger().info(
            f'抬升:wrist_flex -> {self.args.wrist_flex} rad,耗时 {self.args.duration}s')
        time.sleep(self.args.duration + 2.0)


def main():
    p = argparse.ArgumentParser(description='SO101 Gazebo 夹取方块演示')
    p.add_argument('--respawn', dest='respawn', action='store_true', default=True,
                   help='试验前重置方块(默认)')
    p.add_argument('--no-respawn', dest='respawn', action='store_false',
                   help='跳过重置(方块已在钳口之间时)')
    p.add_argument('--effort', type=float, default=0.1, help='闭合 effort (N·m)')
    p.add_argument('--open-effort', type=float, default=-0.1, help='张开 effort (N·m)')
    p.add_argument('--release', action='store_true', help='验证完张开放开')
    p.add_argument("--wrist-flex", type=float, default=-0.95, help='抬升目标腕角 (rad)')
    p.add_argument('--duration', type=float, default=3.0, help='抬升轨迹时长 (s)')
    p.add_argument('--close-time', type=float, default=2.0, help='闭合后等待夹稳 (s)')
    p.add_argument('--cube-x', type=float, default=None,
                   help='方块 spawn x(缺省取钳口中点自动计算)')
    p.add_argument('--cube-y', type=float, default=None, help='方块 spawn y')
    p.add_argument('--cube-z', type=float, default=None, help='方块 spawn z')
    args = p.parse_args()

    rclpy.init()
    node = GraspDemo(args)
    try:
        if args.respawn:
            node.respawn_cube()

        z0 = node.cube_z()
        if z0 is None:
            node.get_logger().error('世界里找不到 cube(用 --respawn 或检查 launch)')
            sys.exit(2)
        node.get_logger().info(f'夹前方块 z = {z0:.3f} m')

        node.send_effort(args.effort)
        node.get_logger().info(f'闭合夹爪:effort = {args.effort} N·m')
        time.sleep(args.close_time)

        node.lift()

        z1 = node.cube_z()
        node.get_logger().info(f'抬升后方块 z = {z1:.3f} m')
        if z1 - z0 >= 0.02:
            node.get_logger().info('✅ 夹取成功:方块跟随手臂上升')
            ok = True
        else:
            node.get_logger().error('❌ 夹取失败:方块没有跟随上升')
            ok = False

        if args.release:
            node.send_effort(args.open_effort)
            node.get_logger().info(f'张开夹爪:effort = {args.open_effort} N·m')
            time.sleep(2.0)
            z2 = node.cube_z()
            node.get_logger().info(f'张开后方块 z = {z2:.3f} m')

        sys.exit(0 if ok else 1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
