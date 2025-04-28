use bevy::{math::vec2, prelude::*};

#[derive(Component, Debug)]
pub struct Car {
    pub acceleration: f32,
    pub top_speed: f32,

    // Acceleration model 
    pub acceleration_curve : CubicCurve<Vec2>,
}

impl Default for Car {
    fn default() -> Self {
        let points = [[
            vec2(0.0, 0.5),
            vec2(0.2, 1.0),
            vec2(0.7, 1.0),
            vec2(1.0, 0.4),
        ]];

        Self {
            acceleration: 25.0,
            top_speed: 140.0,
            acceleration_curve : CubicBezier::new(points).to_curve().unwrap(),
        }
    }
}
