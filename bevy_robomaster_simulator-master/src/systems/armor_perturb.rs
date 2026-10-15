//! 装甲板布局扰动（测试用）。
//!
//! 只挂在**蓝色假人**（SlapperInfantry）车辆的装甲板根节点上，由 `scene.rs`
//! 在场景加载后调用一次 `setup_armor_perturbation` 完成标记。
//! 运行时每帧由 `apply_armor_perturbation` 按 `config.toml` 的 `[armor_perturb]`
//! 叠加扰动；关闭时恢复 GLB 原始布局。
//!
//! 板编号：以**车头方向（炮口朝向在底盘水平面的投影）**为基准顺时针编号——
//! 1 = 前、2 = 右、3 = 后、4 = 左；步兵与英雄模型通用。启动日志会打印每块板的
//! 编号与方位角，可据此核对。
//!
//! 说明：仿真真值（GroundTruthBatch）只含整车位姿，不含装甲板级真值；
//! 扰动量即为"设定值"，用它对比 EKF 估计出的 r1 / l / dz 是否正确。

use bevy::prelude::*;

use crate::config::{ArmorPlateOverride, SimulationConfig};

/// 挂在每块装甲板根节点（名字含 `ARMOR_ROOT`）上的扰动标记。
#[derive(Component, Clone, Copy, Debug)]
pub struct ArmorPerturb {
    /// 用户可见的板编号：1 前 / 2 右 / 3 后 / 4 左
    plate_index: u8,
    /// GLB 原始局部平移（底盘坐标系）
    base_translation: Vec3,
    /// GLB 原始局部旋转
    base_rotation: Quat,
    /// GLB 原始局部缩放
    base_scale: Vec3,
    /// 板子在水平面内的方位角（相对车头，顺时针为正，rad）
    azimuth: f32,
}

/// 场景加载后调用一次：识别 4 块板、计算车头方向并编号，插入 `ArmorPerturb`。
pub fn setup_armor_perturbation(
    In(root): In<Entity>,
    names: Query<&Name>,
    children: Query<&Children>,
    child_of: Query<&ChildOf>,
    transforms: Query<&Transform>,
    mut commands: Commands,
) {
    let mut base = None;
    let mut gimbal = None;
    let mut shot = None;
    let mut armor_roots = Vec::new();

    for entity in children.iter_descendants(root) {
        let Ok(name) = names.get(entity) else {
            continue;
        };
        let name = name.to_string();
        match name.as_str() {
            "BASE" => base = Some(entity),
            "GIMBAL" => gimbal = Some(entity),
            "SHOT_DIRECTION" => shot = Some(entity),
            _ => {
                if name.contains("ARMOR_ROOT") {
                    armor_roots.push((entity, name));
                }
            }
        }
    }
    if armor_roots.is_empty() {
        return;
    }

    // 车头方向 = 炮口方向在底盘水平面的投影；算不出来时退化为 -Z
    let front = muzzle_forward(base, gimbal, shot, root, &child_of, &transforms)
        .unwrap_or(Vec3::NEG_Z);
    let right = front.cross(Vec3::Y);

    let robot = names
        .get(root)
        .ok()
        .map(|n| n.to_string())
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| format!("#{}", root.index()));

    for (entity, name) in armor_roots {
        let Ok(transform) = transforms.get(entity) else {
            continue;
        };
        let offset = Vec3::new(transform.translation.x, 0.0, transform.translation.z);
        let radius = offset.length();
        let direction = offset.normalize_or_zero();

        // 相对车头的方位角：顺时针为正（右=+90°、后=180°、左=-90°）
        let azimuth = direction.dot(right).atan2(direction.dot(front));
        let degrees = azimuth.to_degrees();
        let plate_index = if (-45.0..45.0).contains(&degrees) {
            1 // 前
        } else if (45.0..135.0).contains(&degrees) {
            2 // 右
        } else if degrees >= 135.0 || degrees < -135.0 {
            3 // 后
        } else {
            4 // 左
        };

        commands.entity(entity).insert(ArmorPerturb {
            plate_index,
            base_translation: transform.translation,
            base_rotation: transform.rotation,
            base_scale: transform.scale,
            azimuth,
        });

        info!(
            "armor perturbation: [{}] plate {} <- '{}' (az {:+.1}°, r={:.3}m, y={:.3}m)",
            robot, plate_index, name, degrees, radius, transform.translation.y
        );
    }
}

/// 每帧应用扰动。读 `SimulationConfig`，config.toml 热重载后立即生效。
pub fn apply_armor_perturbation(
    config: Res<SimulationConfig>,
    mut armors: Query<(&ArmorPerturb, &mut Transform)>,
) {
    let cfg = &config.armor_perturb;

    // 关闭时恢复原始布局（幂等，可随时热切换）
    if !cfg.enabled {
        for (perturb, mut transform) in &mut armors {
            transform.translation = perturb.base_translation;
            transform.rotation = perturb.base_rotation;
            transform.scale = perturb.base_scale;
        }
        return;
    }

    // 解析单板覆盖（键：1/前、2/右、3/后、4/左）
    let mut plate_overrides: [Option<&ArmorPlateOverride>; 5] = [None; 5];
    for (key, value) in &cfg.plate {
        if let Some(index) = parse_plate_key(key) {
            plate_overrides[index as usize] = Some(value);
        }
    }

    let yaw_rotation = Quat::from_rotation_y(cfg.yaw_offset_deg.to_radians());

    for (perturb, mut transform) in &mut armors {
        // 1) 全局：水平半径缩放
        let mut position = Vec3::new(
            perturb.base_translation.x * cfg.radius_scale,
            perturb.base_translation.y,
            perturb.base_translation.z * cfg.radius_scale,
        );

        // 2) 全局：前后板对 / 左右板对的高度错位（|cos| 更大 => 前/后那一对）
        let front_back_pair = perturb.azimuth.cos().abs() > perturb.azimuth.sin().abs();
        let stagger = if front_back_pair {
            cfg.height_stagger_m * 0.5
        } else {
            -cfg.height_stagger_m * 0.5
        };
        position.y += cfg.height_offset_m + stagger;

        // 3) 全局：整体绕底盘轴旋转（板位与板朝向一起转）
        position = yaw_rotation * position;
        let mut rotation = yaw_rotation * perturb.base_rotation;
        let mut size_scale = 1.0f32;

        // 4) 单板 / 对向板覆盖
        if let Some(ov) = plate_overrides[perturb.plate_index as usize] {
            if ov.angle_deg != 0.0 {
                let plate_rotation = Quat::from_rotation_y(ov.angle_deg.to_radians());
                position = plate_rotation * position;
                rotation = plate_rotation * rotation;
            }
            let radial = Vec3::new(position.x, 0.0, position.z).normalize_or_zero();
            position += radial * ov.radial_m;
            position.y += ov.height_m;
            size_scale *= ov.size_scale;
        }

        transform.translation = position;
        transform.rotation = rotation;
        transform.scale = perturb.base_scale * size_scale;
    }
}

/// 把配置键解析为板编号：1/前、2/右、3/后、4/左。
fn parse_plate_key(key: &str) -> Option<u8> {
    let key = key.trim().to_ascii_lowercase();
    match key.as_str() {
        "front" | "前" => Some(1),
        "right" | "右" => Some(2),
        "back" | "后" => Some(3),
        "left" | "左" => Some(4),
        other => other
            .parse::<u8>()
            .ok()
            .filter(|index| (1..=4).contains(index)),
    }
}

/// 沿 ChildOf 链把局部旋转累积到 root 帧：返回 root -> entity 的旋转。
fn rotation_to_root(
    entity: Entity,
    root: Entity,
    child_of: &Query<&ChildOf>,
    transforms: &Query<&Transform>,
) -> Option<Quat> {
    let mut rotation = Quat::IDENTITY;
    let mut current = entity;
    for _ in 0..64 {
        if current == root {
            return Some(rotation);
        }
        let transform = transforms.get(current).ok()?;
        rotation = transform.rotation * rotation;
        current = child_of.get(current).ok()?.0;
    }
    None
}

/// 炮口方向在底盘（BASE）水平面的投影，即"车头方向"。
fn muzzle_forward(
    base: Option<Entity>,
    gimbal: Option<Entity>,
    shot: Option<Entity>,
    root: Entity,
    child_of: &Query<&ChildOf>,
    transforms: &Query<&Transform>,
) -> Option<Vec3> {
    let base_rotation = rotation_to_root(base?, root, child_of, transforms)?;
    let gimbal_rotation = rotation_to_root(gimbal?, root, child_of, transforms)?;
    let shot_local = transforms.get(shot?).ok()?.rotation;

    let muzzle = gimbal_rotation * shot_local * Vec3::Y;
    let muzzle_in_base = base_rotation.inverse() * muzzle;
    let forward = Vec3::new(muzzle_in_base.x, 0.0, muzzle_in_base.z).normalize_or_zero();
    (forward.length_squared() > 0.5).then_some(forward)
}
