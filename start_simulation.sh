#!/usr/bin/env bash
# 一键启动 Daedalus 仿真器 + DGE 自瞄闭环
# 用法: ./start_simulation.sh [--sim|--dge]     （不带参数=先仿真器，10 秒后起 DGE）
# 环境变量（可选）:
#   SIM_DIR       仿真器仓库（缺省: 本脚本同级的 bevy_robomaster_simulator-master）
#   DGE_DIR       DGE 仓库（缺省: 系统桌面目录/DGE_vision_26 或本脚本同级）
#   OPENVINO_DIR  OpenVINO 安装目录（缺省自动探测 /opt/intel/openvino*）
# 文档: 《DGE_部署Daedalus仿真指南.md》《Foxglove使用说明.md》《环境配置与使用指南.md》
#
# ⚠️ 仿真器必须从其仓库根目录以 cargo run 启动（本脚本已处理）：
#    直接执行 target/release/daedalus 会因 bevy assets 相对路径找不到模型而黑屏。

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="${SIM_DIR:-$SCRIPT_DIR/bevy_robomaster_simulator-master}"

# DGE 目录：环境变量 → 系统桌面目录 → 本脚本同级/上级
if [ -z "${DGE_DIR:-}" ]; then
    desktop="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
    for d in "$desktop/DGE_vision_26" "$HOME/Desktop/DGE_vision_26" "$SCRIPT_DIR/../DGE_vision_26" "$SCRIPT_DIR/DGE_vision_26"; do
        [ -d "$d/build" ] && { DGE_DIR="$d"; break; }
    done
fi

# OpenVINO 运行库路径（探测不到则不设置，不报错）
if [ -z "${OPENVINO_DIR:-}" ]; then
    for c in /opt/intel/openvino_2024.6.0 /opt/intel/openvino; do
        [ -d "$c/runtime/lib/intel64" ] && { OPENVINO_DIR="$c"; break; }
    done
fi
[ -n "${OPENVINO_DIR:-}" ] && [ -d "$OPENVINO_DIR/runtime/lib/intel64" ] \
    && export LD_LIBRARY_PATH="$OPENVINO_DIR/runtime/lib/intel64:${LD_LIBRARY_PATH:-}"

export PATH="$HOME/.local/bin:$PATH"
source ~/.cargo/env 2>/dev/null || true

start_sim() {
    cd "$SIM_DIR"
    echo "[i] 启动 Daedalus 仿真器（cargo run，必须从仓库根目录启动）..."
    exec cargo run --release
}

start_dge() {
    [ -n "${DGE_DIR:-}" ] && [ -x "$DGE_DIR/build/standard" ] \
        || { echo "[x] 找不到 DGE（${DGE_DIR:-未检测到}）；可用 DGE_DIR=... 指定，或先跑 setup_dge_daedalus.sh"; exit 1; }
    cd "$DGE_DIR"
    echo "[i] 启动 DGE 自瞄；Foxglove 地址 ws://localhost:8765 ..."
    exec "$DGE_DIR/build/standard" "$DGE_DIR/configs/daedalus.yaml"
}

case "${1:-all}" in
    --sim)   start_sim ;;
    --dge)   start_dge ;;
    all)
        start_sim &
        echo "[i] 等 10 秒让仿真器建好共享内存并渲染出画面..."
        sleep 10
        start_dge
        ;;
    *) echo "用法: $0 [--sim|--dge]（不带参数=全启）"; exit 1 ;;
esac
