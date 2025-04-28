use bevy::prelude::*;

#[derive(Component, Clone, Default)]
pub struct DebugWheelValues {}

#[derive(Component, Clone)]
pub struct Wheel {
    pub suspension_rest_length: f32,
    pub suspension_max_length: f32,

    pub suspension_damping: f32,
    pub suspension_stiffness: f32,

    pub tire_grip: f32,
    pub tire_mass: f32,
}

#[derive(Component, Default, Clone)]
pub struct DrivenWheel {}
#[derive(Component, Default, Clone)]
pub struct SteeredWheel {}

impl Wheel {
    pub fn new(
        rest_length: f32,
        max_length: f32,
        damping: f32,
        stiffness: f32,
        grip: f32,
        mass: f32,
    ) -> Self {
        Wheel {
            suspension_rest_length: rest_length,
            suspension_max_length: max_length,
            suspension_damping: damping,
            suspension_stiffness: stiffness,

            tire_grip: grip,
            tire_mass: mass,
        }
    }
}

impl Default for Wheel {
    fn default() -> Self {
        Self {
            suspension_rest_length: Default::default(),
            suspension_max_length: Default::default(),
            suspension_damping: Default::default(),
            suspension_stiffness: Default::default(),

            tire_grip: Default::default(),
            tire_mass: Default::default(),
        }
    }
}
