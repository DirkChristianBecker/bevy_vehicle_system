use avian3d::prelude::*;
use bevy::prelude::*;
use bevy_panorbit_camera::*;
use bevy_vehicle_system::prelude::*;
use rand::Rng;

// https://idanarye.github.io/bevy-tnua/avian3d/collision/collider/struct.Collider.html

fn main() {
    App::new()
        .add_plugins(BevyVehiclesPlugin::default())
        .add_plugins(PanOrbitCameraPlugin)
        .add_systems(Startup, (setup_environment, spawn_bouncy_ball))
        .add_systems(Update, (push_item, change_force))
        .run();
}

#[derive(Debug, Default, Component)]
pub struct PushForce {
    pub force: f32,
}

#[derive(Debug, Default, Component)]
pub struct AngularForce {
    pub force: f32,
}

fn change_force(keys: Res<ButtonInput<KeyCode>>, mut query: Query<&mut PushForce>) {
    if !(keys.just_pressed(KeyCode::NumpadAdd) || keys.just_pressed(KeyCode::NumpadSubtract)) {
        return;
    }

    for mut force in &mut query {
        if keys.just_pressed(KeyCode::NumpadAdd) {
            force.force += 1.0;
        }

        if keys.just_pressed(KeyCode::NumpadSubtract) {
            force.force -= 1.0;
        }

        info!("Force set to! {}", force.force);
    }
}

fn push_item(
    mut commands: Commands,
    keys: Res<ButtonInput<KeyCode>>,
    query: Query<(&RigidBody, &Transform, &PushForce, Entity)>,
) {
    if keys.just_pressed(KeyCode::Space) {
        let mut rng = rand::thread_rng();

        for (_body, transform, force, entity) in &query {
            let mut impulse = ExternalImpulse::default();

            impulse.apply_impulse_at_point(
                Vec3::Y * force.force,
                transform.translation
                    + Vec3::new(rng.gen::<f32>(), rng.gen::<f32>(), rng.gen::<f32>()),
                transform.translation,
            );

            commands.entity(entity).insert(impulse);
        }
    }
}

fn spawn_bouncy_ball(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let size = 1.0;
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(size, size, size))),
        MeshMaterial3d(materials.add(Color::WHITE)),
        RigidBody::Dynamic,
        Collider::cuboid(size, size, size),
        Transform::from_xyz(0.0, 2.0, 0.0),
        GravityScale(1.0),
        Mass(5.0),
        PushForce { force: 25.0 },
        // Bounciness
        Restitution::new(0.8),
    ));
}

fn setup_environment(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let size = 120.0;
    let half_height = 0.02;

    // plane
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(size, half_height, size))),
        MeshMaterial3d(materials.add(Color::WHITE)),
        Collider::cuboid(size, half_height, size),
        RigidBody::Static,
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));

    // light - does what the name suggests
    commands.spawn((
        PointLight {
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0),
    ));

    // camera
    commands.spawn((
        PanOrbitCamera::default(),
        Transform::from_xyz(2.5, 5.5, 13.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}
