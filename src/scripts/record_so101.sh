#!/bin/bash
# SO101 Gazebo 数据采集脚本(ros2 bag)
#
# 用法:
#   ./record_so101.sh [bag名前缀]        # 从源码目录直接跑
#   ros2 run so101_moveit_gazebo record_so101.sh [bag名前缀]   # 安装后跑
#
# 依赖环境:
#   1. 已启动 so101_gazebo.launch.py(相机/控制器在跑)
#   2. mcap 格式需要: sudo apt install ros-humble-rosbag2-storage-mcap
#      没装会自动回退 sqlite3 并提示
set -e

# ============ 常用选项(按需要改) ============
BAG_DIR="${BAG_DIR:-$HOME/ros_study/bags}"   # bag 输出目录
STORAGE="${STORAGE:-mcap}"                   # mcap | sqlite3
DURATION="${DURATION:-60}"                   # 单个文件最长时长(秒),到点自动切分;0 表示不切
NAME_PREFIX="${1:-so101}"                    # bag 名前缀,默认 so101_时间戳
# 录制话题(可用环境变量 TOPICS 整体覆盖: TOPICS="/a /b" ./record_so101.sh)
TOPICS="${TOPICS:-/so101_camera/image_raw /so101_camera/camera_info /joint_states /tf /tf_static /robot_description /clock /so101_arm_controller/joint_trajectory /so101_gripper_controller/commands}"

source /opt/ros/humble/setup.bash
# 兼容"ros2 run"和源码目录两种调用方式
if [ -f "$(dirname "$0")/../../install/setup.bash" ]; then
  source "$(dirname "$0")/../../install/setup.bash"
elif [ -f "$HOME/ros_study/so101_moveit_gazebo/install/setup.bash" ]; then
  source "$HOME/ros_study/so101_moveit_gazebo/install/setup.bash"
fi

# ---- mcap 插件检查,缺失则回退 sqlite3 ----
if [ "$STORAGE" = "mcap" ]; then
  if ! ros2 bag record --help 2>&1 | grep -q mcap; then
    echo "[提示] 未安装 mcap 存储插件,回退 sqlite3。"
    echo "       安装: sudo apt install ros-humble-rosbag2-storage-mcap"
    STORAGE=sqlite3
  fi
fi

# ---- 检查话题都在发布,缺了列出来确认 ----
echo "[检查] 等待话题上线(最多 60s)..."
for t in $TOPICS; do
  for i in $(seq 1 60); do
    ros2 topic list 2>/dev/null | grep -q "^$t$" && break
    [ "$i" = "60" ] && echo "[警告] 话题 $t 未发布,确认 gazebo launch 已启动" && exit 1
    sleep 1
  done
done
echo "[检查] 全部话题已上线"

mkdir -p "$BAG_DIR"
BAG_NAME="${NAME_PREFIX}_$(date +%Y%m%d_%H%M%S)"
SPLIT_ARGS=""
[ "$DURATION" != "0" ] && SPLIT_ARGS="--max-bag-duration $DURATION"

echo "[开始] 存储: $STORAGE | 切分: ${DURATION}s | 输出: $BAG_DIR/$BAG_NAME"
echo "[开始] Ctrl+C 停止录制"
# shellcheck disable=SC2086
exec ros2 bag record -o "$BAG_DIR/$BAG_NAME" -s "$STORAGE" $SPLIT_ARGS $TOPICS
