//! 诊断探针：连接 Daedalus 仿真器的共享内存，打印图像/姿态通道的实时状态。
//! 用法: cargo run --release -p talos-ipc --example probe
use std::sync::atomic::Ordering;
use std::time::{SystemTime, UNIX_EPOCH};
use talos_ipc::{PoseIndex, ShmMetaRegion, ShmRegion, SHM_NAME_META};

fn main() {
    let meta_region = match ShmRegion::open(SHM_NAME_META, std::mem::size_of::<ShmMetaRegion>()) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("连接失败: {e:?} (仿真器没在运行?)");
            return;
        }
    };
    println!("已连接 {SHM_NAME_META}");
    for i in 0..15 {
        unsafe {
            let meta = meta_region.as_ref::<ShmMetaRegion>();
            let img_state = meta.image.state.load(Ordering::Acquire);
            let ready_slot = (img_state & 0x03) as usize % 3;
            let img = &meta.image.slots[ready_slot];
            let gimbal = &meta.poses[PoseIndex::Gimbal as usize];
            let now = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64;
            let hb_age_ms = now.saturating_sub(meta.header.heartbeat_ns) / 1_000_000;
            println!(
                "[{i:02}] img.state=0x{img_state:02x} img.seq={} w={} h={} buf={} | gimbal.seq={} | 心跳龄={hb_age_ms}ms",
                img.seq, img.width, img.height, img.buffer_id, gimbal.slots[0].frame_seq,
            );
        }
        std::thread::sleep(std::time::Duration::from_millis(300));
    }
}
// 追加: 打印相机内参与云台四元数（编译同一份 example 会覆盖 main，见 probe2）
