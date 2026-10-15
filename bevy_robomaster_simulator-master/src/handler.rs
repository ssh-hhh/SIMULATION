use bevy::{
    asset::AssetServer,
    audio::AudioPlayer,
    ecs::{
        observer::On,
        system::{Commands, Query, Res},
    },
};

use crate::robomaster::prelude::{PowerRune, RuneActivated};

pub fn on_activate(
    ev: On<RuneActivated>,
    mut commands: Commands,
    query: Query<&PowerRune>,
    asset_server: Res<AssetServer>,
) {
    let Ok(_rune) = query.get(ev.rune) else {
        return;
    };
    commands.spawn(AudioPlayer::new(asset_server.load("rune_activated.ogg")));
}

