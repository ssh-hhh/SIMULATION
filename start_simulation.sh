#!/usr/bin/env bash
# 一键启动 Daedalus + Talos 闭环仿真
# 用法:
#   ./start_simulation.sh          # 启动完整闭环（仿真器 + 自瞄）
#   ./start_simulation.sh --sim    # 只启动仿真器
#   ./start_simulation.sh --talos  # 只启动 Talos
# 前提: 两个项目已按《环境配置与使用指南.md》构建完成
#
# ⚠️ 仿真器必须从其仓库根目录以 cargo run 启动（或本脚本）：
#    直接执行 target/release/daedalus 会因 bevy assets 相对路径找不到模型而黑屏。

set -e
SIM_DIR="$HOME/桌面/simulation/bevy_robomaster_simulator-master"
TALOS_DIR="$HOME/桌面/simulation/talos26-master"
export PATH="$HOME/.local/bin:$PATH"
source ~/.cargo/env 2>/dev/null || true

start_sim() {
    cd "$SIM_DIR"
    echo "[i] 启动 Daedalus 仿真器（cargo run，必须保持本进程从仓库根目录启动）..."
    exec cargo run --release
}

start_talos() {
    cd "$TALOS_DIR"
    echo "[i] 启动 Talos 自瞄（需要仿真器先启动，否则报共享内存打开失败）..."
    exec "$TALOS_DIR/build/bin/talos"
}

case "${1:-all}" in
    --sim)   start_sim ;;
    --talos) start_talos ;;
    all)
        start_sim &
        SIM_PID=$!
        echo "[i] 等 10 秒让仿真器建好共享内存并渲染出画面..."
        sleep 10
        start_talos
        ;;
    *) echo "用法: $0 [--sim|--talos]"; exit 1 ;;
esac
