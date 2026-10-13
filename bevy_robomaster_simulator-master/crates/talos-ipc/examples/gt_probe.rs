//! 地面真值探针：打印机器人 odom 位置、云台姿态、最新云台指令，以及所有 GT 目标
//! 并计算每个目标相对机器人的真实方位角/俯仰角（DGE 坐标系：X前, Y左, Z上）。
//! 非消费式（只读槽位）。用法: cargo run --release -p talos-ipc --example gt_probe [次数]
use std::sync::atomic::Ordering;
use talos_ipc::{PoseIndex, ShmMetaRegion, ShmRegion, SHM_NAME_META};

fn main() {
    let meta_region = match ShmRegion::open(SHM_NAME_META, std::mem::size_of::<ShmMetaRegion>()) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("连接失败: {e:?} (仿真器没在运行?)");
            return;
        }
    };
    let n: usize = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(1);
    for i in 0..n {
        unsafe {
            let meta = meta_region.as_ref::<ShmMetaRegion>();
            let odom = &meta.poses[PoseIndex::Odom as usize].slots[0];
            let gimbal = &meta.poses[PoseIndex::Gimbal as usize].slots[0];
            let cmd_state = meta.gimbal_cmd.state.load(Ordering::Acquire);
            let cmd = &meta.gimbal_cmd.slots[(cmd_state & 0x03) as usize % 3];

            println!(
                "[{i:02}] robot dge pos=({:.3},{:.3},{:.3}) | link quat w={:.4} x={:.4} y={:.4} z={:.4}",
                odom.position[0], odom.position[1], odom.position[2],
                gimbal.quaternion[0], gimbal.quaternion[1], gimbal.quaternion[2], gimbal.quaternion[3],
            );
            println!(
                "     cmd yaw={:+.2}deg pitch={:+.2}deg dist={:.2} fire={} (new={})",
                cmd.yaw_deg, cmd.pitch_deg, cmd.distance_m, cmd.fire_advice,
                cmd_state & 0x80 != 0
            );
            let gt = &meta.ground_truth;
            for k in 0..gt.target_count as usize {
                let t = &gt.targets[k];
                let dx = t.position[0] - odom.position[0];
                let dy = t.position[1] - odom.position[1];
                let dz = t.position[2] - odom.position[2];
                let az = (dy.atan2(dx)).to_degrees();
                let el = (dz.atan2((dx * dx + dy * dy).sqrt())).to_degrees();
                let dist = (dx * dx + dy * dy + dz * dz).sqrt();
                println!(
                    "     GT[{}] team={} label={} outpost={} pos=({:.3},{:.3},{:.3}) yaw={:+.1}deg | rel az={:+.1}deg el={:+.1}deg dist={:.2}m",
                    k, if t.team == 0 { "red" } else { "blue" }, t.armor_label, t.is_outpost,
                    t.position[0], t.position[1], t.position[2], (t.yaw as f64).to_degrees(),
                    az, el, dist
                );
            }
        }
        if n > 1 {
            std::thread::sleep(std::time::Duration::from_millis(500));
        }
    }
}
