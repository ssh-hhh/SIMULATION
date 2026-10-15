use avian3d::prelude::{CollisionEventsEnabled, CollisionStart};
use bevy::prelude::{ChildOf, Commands, Entity, On, Plugin, Query, ResMut, With};

use super::construct::Armor;
use crate::components::FirstContactArmed;
use crate::robomaster::power_rune::prelude::Projectile;
use crate::statistic::ProjectileStatistics;

/// 直击命中判定：弹丸的**第一次碰撞**就是装甲板才算命中。
/// - 先撞到环境/其他物体、反弹后再打中装甲板的不算；
/// - "穿模后从车体内部打到装甲板背面"的情况由车体的挡弹体（BODY_SHIELD，
///   见 `setup.rs`）在物理上挡住，弹丸根本到不了装甲板背面。
fn handle_armor_collision(
    event: On<CollisionStart>,
    mut commands: Commands,
    mut stats: ResMut<ProjectileStatistics>,
    projectiles: Query<Entity, With<Projectile>>,
    mut armed: Query<&mut FirstContactArmed>,
    armors: Query<(), With<Armor>>,
    child_of: Query<&ChildOf>,
) {
    let projectile_body1 = event.body1.and_then(|body| projectiles.get(body).ok());
    let projectile_body2 = event.body2.and_then(|body| projectiles.get(body).ok());

    let projectile_entity = match (projectile_body1, projectile_body2) {
        (Some(e), _) => e,
        (_, Some(e)) => e,
        _ => return,
    };

    // 第一次碰撞已经用掉，后续碰撞不再判定
    let Ok(mut flag) = armed.get_mut(projectile_entity) else {
        return;
    };
    if !flag.armed {
        return;
    }
    flag.armed = false;

    let other_collider = if projectile_body1.is_some() {
        event.collider2
    } else {
        event.collider1
    };

    if !(armors.contains(other_collider)
        || child_of
            .iter_ancestors(other_collider)
            .any(|ancestor| armors.contains(ancestor)))
    {
        return;
    }

    // 命中装甲板：记一次，并关闭该弹丸的后续碰撞事件
    commands
        .entity(projectile_entity)
        .remove::<CollisionEventsEnabled>();
    stats.increase_accurate();
}

#[derive(Default)]
pub(super) struct ArmorCollisionPlugin;

impl Plugin for ArmorCollisionPlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.add_observer(handle_armor_collision);
    }
}
