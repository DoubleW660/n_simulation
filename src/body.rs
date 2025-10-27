use bevy::prelude::*;
use glam::Vec3;

#[derive(Component, Clone)]
pub struct Body {
    pub position: Vec3,
    pub velocity: Vec3,
    pub acceleration: Vec3,
    pub mass: f32,
    pub index: usize,
}

impl Body {
    pub fn new(position: Vec3, velocity: Vec3, mass: f32, index: usize) -> Self {
        Body {
            position,
            velocity,
            acceleration: Vec3::ZERO,
            mass,
            index,
        }
    }

    pub fn update_velocity_half(&mut self, dt: f32) {
        self.velocity += self.acceleration * (dt / 2.0);
    }

    pub fn update_position(&mut self, dt: f32) {
        self.position += self.velocity * dt;
    }

    pub fn apply_force(&mut self, force: Vec3) {
        self.acceleration = force / self.mass;
    }
}

// Marker component for body entities
#[derive(Component)]
pub struct BodyMarker;