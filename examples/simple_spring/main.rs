use avian3d::prelude::*;
use bevy::prelude::*;
use bevy_panorbit_camera::*;
use bevy_vehicle_system::prelude::*;
use rand::Rng;

fn main() {
    App::new()
        .add_plugins(BevyVehiclesPlugin::default())
        .add_plugins(PanOrbitCameraPlugin)
        .add_systems(Startup, (setup_environment, spawn_cubes_with_spring))
        .add_systems(Update, (push_item, change_force))
        .run();
}

#[derive(Debug, Default, Component)]
pub struct PushForce {
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

            let mut angular_force = ExternalAngularImpulse::new(Vec3::Y);
            let _ = angular_force
                .apply_impulse(Vec3::Y)
                .with_persistence(false)
                .with_y(force.force);
            commands.entity(entity).insert(angular_force);
        }
    }
}

fn spawn_cubes_with_spring(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let size = 1.0;

    // A spring should not allow rotations. Imagine this is a wheel mount, then we should be able to rotate around the y axis.
    let upper_axes_locks = LockedAxes::new().lock_rotation_x().lock_rotation_z();
    let lower_axes_locks = LockedAxes::new()
        .lock_rotation_x()
        .lock_rotation_y()
        .lock_rotation_z();

    let cube_id = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(size, size, size))),
            MeshMaterial3d(materials.add(Color::WHITE)),
            RigidBody::Dynamic,
            upper_axes_locks.clone(),
            // Add some angular damping, so the cube stops spinning after some time.
            AngularDamping(0.5),
            Collider::cuboid(size, size, size),
            Transform::from_xyz(0.0, size * 3.0, 0.0),
            PushForce { force: 25.0 },
            Mass(1.0),
        ))
        .id();

    let spring_end = commands
        .spawn((
            Transform::from_xyz(0.0, size, 0.0),
            Collider::cuboid(size, size, size),
            RigidBody::Dynamic,
            lower_axes_locks.clone(),
            // We are using rubber...
            Restitution::new(0.8),
            Mesh3d(meshes.add(Cuboid::new(size, size, size))),
            MeshMaterial3d(materials.add(Color::WHITE)),
            // Give it a big mass so the base does not move as much.
            Mass(5.0),
        ))
        .id();

    commands.spawn(
        DistanceJoint::new(cube_id, spring_end)
            .with_local_anchor_1(Vec3::new(0.0, -size * 0.5, 0.0))
            .with_local_anchor_2(Vec3::ZERO)
            // Spring damper
            .with_linear_velocity_damping(10.0)
            // Compliance is the opposite of stiffness. So a value of 0.0 means inifite stiffness.
            .with_compliance(1.0 / 200.0)
            // The length of the spring when no more forces are applied
            .with_rest_length(2.0)
            // Min and max values for the spring length
            .with_limits(1.0, 3.0),
    );
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
