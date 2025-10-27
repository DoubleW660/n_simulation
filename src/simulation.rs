use bevy::prelude::*;
use crate::body::{Body, BodyMarker};
use crate::octree::{update_simulation_step, create_two_body_orbit, create_particle_cloud};
use crate::camera::{CameraController, CameraPlugin};

// Resources for simulation parameters
#[derive(Resource)]
pub struct SimulationParams {
    pub dt: f32,
    pub theta: f32,
    pub softening: f32,
    pub g: f32,
    pub paused: bool,
}

impl Default for SimulationParams {
    fn default() -> Self {
        SimulationParams {
            dt: 0.01,
            theta: 0.5,
            softening: 0.1,
            g: 1.0,
            paused: false,
        }
    }
}

// Resource to hold all bodies
#[derive(Resource)]
pub struct Bodies {
    pub data: Vec<Body>,
}

pub struct SimulationPlugin;

impl Plugin for SimulationPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(SimulationParams::default())
            .add_plugins(CameraPlugin)
            .add_systems(Startup, setup)
            .add_systems(Update, (
                handle_input,
                update_physics,
                update_body_transforms,
            ).chain());
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Create initial bodies
    let bodies = create_two_body_orbit();
    // Or use particle cloud:
    // let bodies = create_particle_cloud(50, 20.0);
    
    println!("Created {} bodies", bodies.len());

    // Spawn visual entities for each body
    for body in bodies.iter() {
        let radius = (body.mass / 10.0).cbrt().max(0.2);
        
        commands.spawn((
            PbrBundle {
                mesh: meshes.add(Sphere::new(radius)),
                material: materials.add(StandardMaterial {
                    base_color: Color::srgb(0.8, 0.6, 0.2),
                    emissive: LinearRgba::new(0.5, 0.3, 0.1, 1.0),
                    ..default()
                }),
                transform: Transform::from_translation(Vec3::new(
                    body.position.x,
                    body.position.y,
                    body.position.z,
                )),
                ..default()
            },
            body.clone(),
            BodyMarker,
        ));
    }

    // Store bodies in resource
    commands.insert_resource(Bodies { data: bodies });

    // Camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0.0, 30.0, 50.0)
                .looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        CameraController::default(),
    ));

    // Light
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 2000000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(40.0, 80.0, 40.0),
        ..default()
    });

    // Ambient light
    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 100.0,
    });

    println!("Simulation setup complete!");
    println!("Controls:");
    println!("  SPACE - Pause/Resume");
    println!("  R - Reset simulation");
    println!("  Right Mouse + Drag - Rotate camera");
    println!("  Mouse Wheel - Zoom in/out");
}

fn handle_input(
    keys: Res<ButtonInput<KeyCode>>,
    mut params: ResMut<SimulationParams>,
    mut commands: Commands,
    mut bodies_res: ResMut<Bodies>,
    body_entities: Query<Entity, With<BodyMarker>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Toggle pause
    if keys.just_pressed(KeyCode::Space) {
        params.paused = !params.paused;
        println!("Simulation {}", if params.paused { "paused" } else { "resumed" });
    }

    // Reset simulation
    if keys.just_pressed(KeyCode::KeyR) {
        println!("Resetting simulation...");
        
        // Despawn all existing bodies
        for entity in body_entities.iter() {
            commands.entity(entity).despawn();
        }

        // Create new bodies
        let bodies = create_two_body_orbit();
        // Or use: let bodies = create_particle_cloud(50, 20.0);
        
        // Spawn new visual entities
        for body in bodies.iter() {
            let radius = (body.mass / 10.0).cbrt().max(0.2);
            
            commands.spawn((
                PbrBundle {
                    mesh: meshes.add(Sphere::new(radius)),
                    material: materials.add(StandardMaterial {
                        base_color: Color::srgb(0.8, 0.6, 0.2),
                        emissive: LinearRgba::new(0.5, 0.3, 0.1, 1.0),
                        ..default()
                    }),
                    transform: Transform::from_translation(Vec3::new(
                        body.position.x,
                        body.position.y,
                        body.position.z,
                    )),
                    ..default()
                },
                body.clone(),
                BodyMarker,
            ));
        }

        bodies_res.data = bodies;
        println!("Reset complete!");
    }
}

fn update_physics(
    mut bodies_res: ResMut<Bodies>,
    params: Res<SimulationParams>,
) {
    if params.paused {
        return;
    }

    update_simulation_step(
        &mut bodies_res.data,
        params.dt,
        params.theta,
        params.softening,
        params.g,
    );
}

fn update_body_transforms(
    bodies_res: Res<Bodies>,
    mut query: Query<(&mut Transform, &Body), With<BodyMarker>>,
) {
    for (mut transform, body) in query.iter_mut() {
        // Find the corresponding body in the resource
        if let Some(updated_body) = bodies_res.data.iter().find(|b| b.index == body.index) {
            transform.translation = Vec3::new(
                updated_body.position.x,
                updated_body.position.y,
                updated_body.position.z,
            );
        }
    }
}