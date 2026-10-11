//! 导出仿真器当前图像帧为 PPM（不改共享内存状态）
use std::sync::atomic::Ordering;
use talos_ipc::{ShmMetaRegion, ShmRegion, IMAGE_POOL_SIZE, IMAGE_SIZE, SHM_NAME_IMAGE_POOL, SHM_NAME_META};

fn main() {
    let meta = ShmRegion::open(SHM_NAME_META, std::mem::size_of::<ShmMetaRegion>()).unwrap();
    let pool = ShmRegion::open(SHM_NAME_IMAGE_POOL, IMAGE_POOL_SIZE).unwrap();
    unsafe {
        let m = meta.as_ref::<ShmMetaRegion>();
        let state = m.image.state.load(Ordering::Acquire);
        let ready = (state & 0x03) as usize % 3;
        let img = &m.image.slots[ready];
        if img.seq == 0 {
            eprintln!("还没有图像帧");
            return;
        }
        let data = std::slice::from_raw_parts(
            { let base = pool.as_ref::<u8>() as *const u8; base.add(img.buffer_id as usize * IMAGE_SIZE) },
            IMAGE_SIZE,
        );
        // 写 P6 PPM（RGB）
        let header = format!("P6\n{} {}\n255\n", img.width, img.height);
        let mut out = header.into_bytes();
        out.extend_from_slice(data);
        std::fs::write("/tmp/daedalus_frame.ppm", &out).unwrap();
        println!("已导出 seq={} → /tmp/daedalus_frame.ppm", img.seq);
    }
}
