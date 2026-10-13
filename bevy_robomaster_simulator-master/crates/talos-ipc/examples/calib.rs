//! 坐标系标定工具
//! watch 模式: 消费图像保持数据流，打印云台姿态在三种轴变换下的 yaw/pitch
//!             V0 = 当前配置（imu_left/right=单位四元数，直接用原始四元数）
//!             V1/V2 = 历史/备用变换，排查轴系问题时对照
//! send 模式:  发一条云台指令（度），用于标定指令符号
use std::sync::atomic::{AtomicU8, Ordering};
use talos_ipc::{PoseIndex, ShmMetaRegion, ShmRegion, SHM_NAME_META};

#[derive(Clone, Copy, Default)]
struct Quat { w: f64, x: f64, y: f64, z: f64 }
impl Quat {
    fn mul(self, o: Quat) -> Quat {
        Quat {
            w: self.w*o.w - self.x*o.x - self.y*o.y - self.z*o.z,
            x: self.w*o.x + self.x*o.w + self.y*o.z - self.z*o.y,
            y: self.w*o.y - self.x*o.z + self.y*o.w + self.z*o.x,
            z: self.w*o.z + self.x*o.y - self.y*o.x + self.z*o.w,
        }
    }
    fn conj(self) -> Quat { Quat { w: self.w, x: -self.x, y: -self.y, z: -self.z } }
    fn yaw_deg(self) -> f64 { 2.0 * self.z.atan2(self.w) * 57.29578 }
    fn pitch_deg(self) -> f64 { 2.0 * self.y.atan2(self.w) * 57.29578 }
}
const P: Quat = Quat { w: 0.5, x: 0.5, y: 0.5, z: 0.5 }; // 【历史】早期误用的“再变换一次”（当前配置不用它）

fn main() {
    let mut meta = ShmRegion::open(SHM_NAME_META, std::mem::size_of::<ShmMetaRegion>()).unwrap();
    let mode = std::env::args().nth(1).unwrap_or_else(|| "watch".into());

    if mode == "send" {
        let yaw: f32 = std::env::args().nth(2).unwrap().parse().unwrap();
        let pitch: f32 = std::env::args().nth(3).unwrap().parse().unwrap();
        unsafe {
            let m = meta.as_mut::<ShmMetaRegion>();
            let buf = &mut m.gimbal_cmd;
            let slot = buf.write_idx as usize % 3;
            let cmd = &mut buf.slots[slot];
            cmd.timestamp_ns = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos() as u64;
            cmd.yaw_deg = yaw;
            cmd.pitch_deg = pitch;
            cmd.distance_m = 2.0;
            cmd.fire_advice = 0;
            let old = AtomicU8::swap(&buf.state, buf.write_idx | 0x80, Ordering::AcqRel);
            buf.write_idx = old & 0x03;
        }
        println!("已发送 cmd yaw={yaw} pitch={pitch}");
        return;
    }

    // watch 模式
    loop {
        unsafe {
            let m = meta.as_mut::<ShmMetaRegion>();
            // 消费全部位姿通道（维持图像握手）
            for i in 0..=4 {
                let pose_buf = &mut m.poses[i as usize];
                let state = pose_buf.state.load(Ordering::Acquire);
                if state & 0x80 != 0 {
                    let ready = (state & 0x03) as usize % 3;
                    let _ = &pose_buf.slots[ready];
                    let desired = pose_buf.read_idx;
                    let _ = AtomicU8::compare_exchange(
                        &pose_buf.state, state, desired, Ordering::AcqRel, Ordering::Acquire);
                    pose_buf.read_idx = ready as u8;
                    if i == PoseIndex::Gimbal as usize {
                        let q = &pose_buf.slots[ready].quaternion;
                        let q = Quat { w: q[0] as f64, x: q[1] as f64, y: q[2] as f64, z: q[3] as f64 };
                        let v0 = q;                        // 当前配置（单位变换）：直接用发布值
                        let v1 = P.mul(q).mul(P.conj());   // 【历史】0.5 轴变换（25°抬头会被当成 yaw）
                        let v2 = P.conj().mul(q).mul(P);   // 反向候选（备用）
                        println!(
                            "V0(当前) yaw={:+7.1} pitch={:+6.1} | V1(旧0.5变换) yaw={:+7.1} pitch={:+6.1} | V2(反向) yaw={:+7.1} pitch={:+6.1}",
                            v0.yaw_deg(), v0.pitch_deg(),
                            v1.yaw_deg(), v1.pitch_deg(),
                            v2.yaw_deg(), v2.pitch_deg());
                    }
                }
            }
            // 消费图像（维持发布节奏）
            let img = &mut m.image;
            let state = img.state.load(Ordering::Acquire);
            if state & 0x80 != 0 {
                let desired = img.read_idx;
                let _ = AtomicU8::compare_exchange(
                    &img.state, state, desired, Ordering::AcqRel, Ordering::Acquire);
                img.read_idx = (state & 0x03) as u8;
            }
        }
        std::thread::sleep(std::time::Duration::from_millis(4));
    }
}
