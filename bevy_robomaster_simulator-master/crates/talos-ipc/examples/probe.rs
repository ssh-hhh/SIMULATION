//! 非消费式监视探针：只读原始槽位，不清 FLAG_NEW，可以和 DGE/Talos 同时跑。
//! 用法: cargo run --release -p talos-ipc --example probe [采样次数]
use std::sync::atomic::Ordering;
use std::time::{SystemTime, UNIX_EPOCH};
use talos_ipc::{PoseIndex, ShmMetaRegion, ShmRegion, SHM_NAME_META};

fn main() {
    let meta_region =
        match ShmRegion::open(SHM_NAME_META, std::mem::size_of::<ShmMetaRegion>()) {
            Ok(r) => r,
            Err(e) => {
                eprintln!("连接失败: {e:?} (仿真器没在运行?)");
                return;
            }
        };
    let n: usize = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(15);
    for i in 0..n {
        unsafe {
            let meta = meta_region.as_ref::<ShmMetaRegion>();
            let img_state = meta.image.state.load(Ordering::Acquire);
            let img = &meta.image.slots[(img_state & 0x03) as usize % 3];
            let gimbal = &meta.poses[PoseIndex::Gimbal as usize].slots[0];
            let yaw_deg = 2.0 * gimbal.quaternion[3].atan2(gimbal.quaternion[0]) * 57.29578;
            let pitch_deg = 2.0 * gimbal.quaternion[2].atan2(gimbal.quaternion[0]) * 57.29578;
            let cmd_state = meta.gimbal_cmd.state.load(Ordering::Acquire);
            let cmd = &meta.gimbal_cmd.slots[(cmd_state & 0x03) as usize % 3];
            let now = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64;
            let hb_age_ms = now.saturating_sub(meta.header.heartbeat_ns) / 1_000_000;
            println!(
                "[{:02}] img.seq={} new={} | 云台 yaw={:+.1} pitch={:+.1} seq={} | cmd.new={} cmd.yaw={:+.1} fire={} | 心跳 {}ms",
                i,
                img.seq,
                img_state & 0x80 != 0,
                yaw_deg,
                pitch_deg,
                gimbal.frame_seq,
                cmd_state & 0x80 != 0,
                cmd.yaw_deg,
                cmd.fire_advice,
                hb_age_ms,
            );
        }
        std::thread::sleep(std::time::Duration::from_millis(400));
    }
}
