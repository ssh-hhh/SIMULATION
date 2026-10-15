//! 装甲板"判定盒"：把模型自带的装甲网格碰撞体（Trimesh）替换成**可调尺寸的长方体**，
//! 用于控制"打在哪里才算命中"，并提供绿色半透明盒可视化。
//!
//! - 尺寸 = 原网格的局部包围盒 × `[armor_hitbox]` 的三个缩放系数；
//! - 系数按"最大边=宽、中间边=高、最小边=厚"自动对应到模型的局部轴（步兵/英雄通用）；
//! - `debug_draw = true` 时绘制判定盒（在仿真器窗口/DGE 画面里都能看到）；
//! - config.toml 热重载后立即生效。

use avian3d::prelude::*;
use bevy::prelude::*;

use super::construct::Armor;
use crate::config::SimulationConfig;

/// 缩放系数合法范围（防止误填巨大值把整个场地变成判定盒）。
const MIN_SCALE: f32 = 0.05;
const MAX_SCALE: f32 = 5.0;
/// 可视化盒每边外扩的尺寸（米），便于在装甲板本体外面看到判定盒。
const DEBUG_INFLATE_M: f32 = 0.015;

#[derive(Component, Clone, Copy)]
pub struct ArmorHitbox {
    /// 原网格局部包围盒尺寸（未缩放）
    raw_dims: Vec3,
    /// 包围盒中心（局部）
    raw_center: Vec3,
    /// 已应用的缩放（宽/高/厚），NaN = 尚未应用
    applied: [f32; 3],
    /// 当前缩放后的尺寸（可视化用）
    current_dims: Vec3,
    /// 可视化子实体
    debug_child: Option<Entity>,
}

fn is_armor_entity(
    entity: Entity,
    armors: &Query<(), With<Armor>>,
    child_of: &Query<&ChildOf>,
) -> bool {
    armors.contains(entity) || child_of.iter_ancestors(entity).any(|a| armors.contains(a))
}

impl ArmorHitbox {
    /// 薄轴（板面法向所在的局部轴，最小边）。
    pub fn thin_axis(&self) -> usize {
        let dims = self.raw_dims;
        if dims.x <= dims.y && dims.x <= dims.z {
            0
        } else if dims.y <= dims.z {
            1
        } else {
            2
        }
    }
}

/// 给每个"装甲网格碰撞体"挂上判定盒组件（只做一次）。
fn mark_armor_hitboxes(
    mut commands: Commands,
    colliders: Query<(Entity, &Collider), Without<ArmorHitbox>>,
    armors: Query<(), With<Armor>>,
    child_of: Query<&ChildOf>,
) {
    let mut marked = 0;
    for (entity, collider) in &colliders {
        if !is_armor_entity(entity, &armors, &child_of) {
            continue;
        }
        // 原始（未缩放）局部包围盒：aabb 会带碰撞体自身的缩放，这里除回去
        let aabb = collider.aabb(Vec3::ZERO, Rotation::IDENTITY);
        let scale = collider.scale().max(Vec3::splat(1e-6));
        let dims = (aabb.max - aabb.min) / scale;
        let center = ((aabb.max + aabb.min) * 0.5) / scale;
        let raw_dims = Vec3::new(dims.x, dims.y, dims.z);
        if raw_dims.max_element() <= 0.0 {
            continue;
        }
        commands.entity(entity).insert(ArmorHitbox {
            raw_dims,
            raw_center: Vec3::new(center.x, center.y, center.z),
            applied: [f32::NAN; 3],
            current_dims: raw_dims,
            debug_child: None,
        });
        marked += 1;
    }
    if marked > 0 {
        info!("armor hitbox: marked {} armor colliders", marked);
    }
}

/// 按配置重建判定盒 / 更新可视化（config.toml 热重载即时生效）。
#[allow(clippy::type_complexity)]
fn apply_armor_hitbox(
    config: Res<SimulationConfig>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut assets: Local<Option<(Handle<Mesh>, Handle<StandardMaterial>)>>,
    mut reported: Local<Option<[f32; 3]>>,
    mut hitboxes: Query<(Entity, &mut ArmorHitbox)>,
) {
    let cfg = &config.armor_hitbox;
    let sanitize = |value: f32| {
        if value.is_finite() {
            value.clamp(MIN_SCALE, MAX_SCALE)
        } else {
            1.0
        }
    };
    let factors = [
        sanitize(cfg.width_scale),
        sanitize(cfg.height_scale),
        sanitize(cfg.thickness_scale),
    ];

    // 配置变化时打一条日志，方便确认热重载是否生效
    if reported.as_ref() != Some(&factors) {
        *reported = Some(factors);
        if (cfg.width_scale - factors[0]).abs() > 1e-6
            || (cfg.height_scale - factors[1]).abs() > 1e-6
            || (cfg.thickness_scale - factors[2]).abs() > 1e-6
        {
            warn!(
                "armor hitbox: 缩放超出 [{MIN_SCALE}, {MAX_SCALE}]，已截断为 ({:.2}, {:.2}, {:.2})",
                factors[0], factors[1], factors[2]
            );
        }
        info!(
            "armor hitbox: scales=({:.2},{:.2},{:.2}) debug_draw={}",
            factors[0], factors[1], factors[2], cfg.debug_draw
        );
    }

    for (entity, mut hitbox) in &mut hitboxes {
        // 1) 缩放变化时重建碰撞体（按边长排序映射 宽/高/厚）
        if hitbox.applied != factors {
            let dims = hitbox.raw_dims;
            let mut order = [0usize, 1, 2];
            order.sort_by(|&a, &b| {
                dims[b]
                    .partial_cmp(&dims[a])
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            let mut scaled = dims;
            for (rank, &axis) in order.iter().enumerate() {
                scaled[axis] = dims[axis] * factors[rank];
            }

            let shape = if hitbox.raw_center.length_squared() > 1e-12 {
                Collider::compound(vec![(
                    hitbox.raw_center,
                    Quat::IDENTITY,
                    Collider::cuboid(scaled.x, scaled.y, scaled.z),
                )])
            } else {
                Collider::cuboid(scaled.x, scaled.y, scaled.z)
            };
            commands.entity(entity).insert(shape);

            hitbox.applied = factors;
            hitbox.current_dims = scaled;
        }

        // 2) 可视化：绿色半透明盒（双面渲染 + 每边外扩 1.5cm，便于在板子外面看到）
        if cfg.debug_draw {
            let (mesh, material) = assets.get_or_insert_with(|| {
                (
                    meshes.add(Cuboid::new(1.0, 1.0, 1.0)),
                    materials.add(StandardMaterial {
                        base_color: Color::srgba(0.1, 1.0, 0.3, 0.35),
                        alpha_mode: AlphaMode::Blend,
                        unlit: true,
                        cull_mode: None,
                        ..default()
                    }),
                )
            });
            let debug_dims = hitbox.current_dims + Vec3::splat(DEBUG_INFLATE_M);
            let transform = Transform::from_translation(hitbox.raw_center).with_scale(debug_dims);
            match hitbox.debug_child {
                Some(child) => {
                    commands.entity(child).insert(transform);
                }
                None => {
                    let child = commands
                        .spawn((
                            Mesh3d(mesh.clone()),
                            MeshMaterial3d(material.clone()),
                            transform,
                        ))
                        .id();
                    commands.entity(entity).add_child(child);
                    hitbox.debug_child = Some(child);
                    debug!("armor hitbox: debug box spawned for #{}", entity.index());
                }
            }
        } else if let Some(child) = hitbox.debug_child.take() {
            commands.entity(child).despawn();
        }
    }
}

#[derive(Default)]
pub(super) struct ArmorHitboxPlugin;

impl Plugin for ArmorHitboxPlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.add_systems(Update, (mark_armor_hitboxes, apply_armor_hitbox));
    }
}
