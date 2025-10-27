use bevy::prelude::*;
use bevy::input::mouse::{MouseMotion, MouseWheel};

#[derive(Component)]
pub struct CameraController {
    pub enabled: bool,
    pub sensitivity: f32,
    pub zoom_speed: f32,
}

impl Default for CameraController {
    fn default() -> Self {
        CameraController {
            enabled: true,
            sensitivity: 0.003,
            zoom_speed: 5.0,
        }
    }
}

pub struct CameraPlugin;

impl Plugin for CameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, (
            camera_rotation,
            camera_zoom,
        ));
    }
}

fn camera_rotation(
    mut mouse_motion: EventReader<MouseMotion>,
    mouse_button: Res<ButtonInput<MouseButton>>,
    mut camera_query: Query<(&mut Transform, &CameraController)>,
) {
    // Only rotate when right mouse button is held
    if !mouse_button.pressed(MouseButton::Right) {
        return;
    }

    let mut delta = Vec2::ZERO;
    for motion in mouse_motion.read() {
        delta += motion.delta;
    }

    for (mut transform, controller) in camera_query.iter_mut() {
        if !controller.enabled {
            continue;
        }

        // Rotate around the origin
        let yaw = -delta.x * controller.sensitivity;
        let pitch = -delta.y * controller.sensitivity;

        // Get the current position and distance from origin
        let distance = transform.translation.length();
        
        // Convert to spherical coordinates
        let current_pitch = (transform.translation.y / distance).asin();
        let current_yaw = transform.translation.z.atan2(transform.translation.x);

        // Update angles with limits
        let new_pitch = (current_pitch + pitch).clamp(-1.5, 1.5);
        let new_yaw = current_yaw + yaw;

        // Convert back to Cartesian coordinates
        let y = distance * new_pitch.sin();
        let xz_length = distance * new_pitch.cos();
        let x = xz_length * new_yaw.cos();
        let z = xz_length * new_yaw.sin();

        transform.translation = Vec3::new(x, y, z);
        transform.look_at(Vec3::ZERO, Vec3::Y);
    }
}

fn camera_zoom(
    mut scroll: EventReader<MouseWheel>,
    mut camera_query: Query<(&mut Transform, &CameraController)>,
    time: Res<Time>,
) {
    let mut scroll_amount = 0.0;
    for event in scroll.read() {
        scroll_amount += event.y;
    }

    if scroll_amount == 0.0 {
        return;
    }

    for (mut transform, controller) in camera_query.iter_mut() {
        if !controller.enabled {
            continue;
        }

        let distance = transform.translation.length();
        let zoom_delta = -scroll_amount * controller.zoom_speed * time.delta_seconds() * distance * 0.1;
        
        // Prevent getting too close or too far
        let new_distance = (distance + zoom_delta).clamp(5.0, 500.0);
        
        transform.translation = transform.translation.normalize() * new_distance;
    }
}