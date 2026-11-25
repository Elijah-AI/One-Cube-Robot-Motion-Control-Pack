#!/usr/bin/env bash
# =============================================================================
# 自动编译脚本 (ROS 2 / colcon)
#
#  用法:
#    ./build.sh                  # 全量: 先编译 com_interfaces, 成功后再编译其余节点
#    ./build.sh <pkg> [pkg...]   # 定向: 先确保 com_interfaces, 再编译指定包
#
#  说明:
#    - com_interfaces 承载 msg/srv/action 接口定义, 是其它节点的编译依赖,
#      因此必须先编译且成功, 才能继续编译其它节点 (com_interfaces 失败即中止)。
#    - 编译产物输出到仓库根目录下的 build/ install/ log/,
#      这些目录及编辑器临时文件均已写入 .gitignore, 不会被 git 提交上传。
# =============================================================================
set -euo pipefail

# ---- 定位仓库根目录(脚本所在目录), 支持从任意路径调用 ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

# ROS 环境脚本会读取未定义变量(如 AMENT_TRACE_SETUP_FILES),
# 因此 source 期间需临时关闭 nounset(-u), 加载完成后再恢复。
set +u

# ---- 1. 加载 ROS 2 环境(未 source 时自动探测发行版) ----
if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  _found=0
  for _f in /opt/ros/*/setup.bash; do
    [ -f "${_f}" ] || continue
    # shellcheck disable=SC1090
    source "${_f}"
    _found=1
    break
  done
  if [ "${_found}" -eq 0 ]; then
    echo "[ERROR] 未在 /opt/ros 下找到 ROS 2 环境。" >&2
    exit 1
  fi
fi

if ! command -v colcon >/dev/null 2>&1; then
  echo "[ERROR] 未找到 colcon, 请确认已安装并 source ROS 2 环境。" >&2
  exit 1
fi

# 加载本地已安装产物环境(首次构建时 install 尚不存在则跳过)
if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/install/setup.bash"
fi

set -u

# ---- 2. 收集目标包(未传参则为全量) ----
TARGET_PKGS=("$@")

# ---- 3. 步骤一: 先编译接口包 com_interfaces ----
echo "======================================================================"
echo " [步骤 1/2] 编译 com_interfaces (msg/srv/action 接口包)"
echo "======================================================================"
colcon build --packages-select com_interfaces
# set -e: 若上一步失败将立即退出, 不会继续编译其它节点

# 重新加载环境, 使刚生成的 com_interfaces 接口对后续编译可见
set +u
if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/install/setup.bash"
fi
set -u

# ---- 4. 步骤二: 编译其余节点 ----
if [ "${#TARGET_PKGS[@]}" -gt 0 ]; then
  echo "======================================================================"
  echo " [步骤 2/2] 编译指定节点: ${TARGET_PKGS[*]}"
  echo "======================================================================"
  colcon build --packages-up-to "${TARGET_PKGS[@]}"
else
  echo "======================================================================"
  echo " [步骤 2/2] 编译其余全部节点 (跳过 com_interfaces)"
  echo "======================================================================"
  colcon build --packages-ignore com_interfaces
fi

echo ""
echo ">>> [完成] 全部编译成功。产物位于 build/ install/ log/ (均已 git 忽略)。"
