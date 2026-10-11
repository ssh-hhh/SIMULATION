//! 诊断探针2：打印仿真器的相机内参与云台/里程计位姿（用于填写 DGE 的 daedalus.yaml）
//! 用法: cargo run --release -p talos-ipc --example probe2
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
    unsafe {
        let meta = meta_region.as_ref::<ShmMetaRegion>();
        let ci = &meta.camera_info;
        println!("== CameraInfo ==");
        println!("fx={} fy={} cx={} cy={}", ci.fx, ci.fy, ci.cx, ci.cy);
        println!("distortion={:?} ({}x{})", ci.distortion, ci.width, ci.height);

        for name_idx in [
            ("gimbal", PoseIndex::Gimbal),
            ("odom", PoseIndex::Odom),
            ("camera", PoseIndex::Camera),
        ] {
            let p = &meta.poses[name_idx.1 as usize].slots[0];
            let now = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64;
            let age_ms = now.saturating_sub(p.timestamp_ns) / 1_000_000;
            println!(
                "== pose {} (age {}ms) == pos=({:.3},{:.3},{:.3}) quat=(w{:.4} x{:.4} y{:.4} z{:.4})",
                name_idx.0, age_ms, p.position[0], p.position[1], p.position[2],
                p.quaternion[0], p.quaternion[1], p.quaternion[2], p.quaternion[3],
            );
        }
        let gt = &meta.ground_truth;
        println!("== ground_truth 时间戳 =={} ", gt.timestamp_ns);
    }
}
