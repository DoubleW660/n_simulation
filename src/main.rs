mod body;
mod camera;
mod octree;
mod simulation;

use bevy::prelude::*;
use simulation::SimulationPlugin;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "N-Body Simulation".to_string(),
                resolution: (1280.0, 720.0).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(SimulationPlugin)
        .run();
}