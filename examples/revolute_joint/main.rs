use avian3d::prelude::*;
use bevy::prelude::*;
use bevy_panorbit_camera::*;
use bevy_vehicle_system::prelude::*;

// https://idanarye.github.io/bevy-tnua/avian3d/collision/collider/struct.Collider.html

fn main() {
    App::new()
        .add_plugins(BevyVehiclesPlugin::default())
        .add_plugins(PanOrbitCameraPlugin)
        .add_systems(Startup, (setup_environment, spawn_vehicle))
        .add_systems(
            Update,
           (suspension_system, friction_system, steering_system, acceleration_system),
        )
        .run();
}

fn suspension_system(
    mut cars: Query<(
        &Children,
        RigidBodyQuery,
        &Car,
        &GlobalTransform,
        &mut ExternalForce,
    )>,
    wheels: Query<(&RayCaster, &RayHits, &Wheel, &Transform, &GlobalTransform)>,
) {
    for (children, body_query, &_, gl_transform_car, mut ex_force) in cars.iter_mut() {
        for wheel in children {
            if let Ok((ray, hits, wheel, transform, gl_transform)) = wheels.get(*wheel) {
                for hit in hits.iter_sorted() {
                    // Ignore hits that are too far away.
                    if hit.distance > wheel.suspension_max_length {
                        continue;
                    }

                    // world-space direction of the spring force
                    let spring_dir = gl_transform.up();

                    // The hitpoint
                    let hit_point = ray.origin + *ray.direction * hit.distance;
                    // Where is the suspension rest position?
                    let rest_position = ray.origin + *ray.direction * wheel.suspension_rest_length;
                    // Calc the offset between rest position and ray hit.
                    let offset = hit_point.distance(rest_position);
                    // Get the velocity 
                    let tire_velocity = body_query.velocity_at_point(gl_transform.translation());

                    // calculate velocity along the spring direction
                    let vel = spring_dir.dot(tire_velocity);
                    let force =
                        (offset * wheel.suspension_stiffness) - (vel * wheel.suspension_damping);

                    ex_force.apply_force_at_point(
                        spring_dir * force,
                        gl_transform.translation(),
                        gl_transform_car.translation(),
                    );
                }
            }
        }
    }
}

fn friction_system(
    time: Res<Time>,
    mut cars: Query<(
        &Children,
        RigidBodyQuery,
        &Car,
        &GlobalTransform,
        &mut ExternalForce,
    )>,
    wheels: Query<(&Wheel, &Transform, &GlobalTransform)>,
) {
    for (children, body_query, &_, gl_transform_car, mut ex_force) in cars.iter_mut() {
        for wheel in children {
            if let Ok((wheel, transform, gl_transform)) = wheels.get(*wheel) {
                // world-space direction of the spring force
                let streering_dir = gl_transform.right();

                // world-space velocity of the suspension
                let tire_velocity = body_query.velocity_at_point(transform.translation);

                // what is the tire´s velocity in the steering direction?
                let steering_velocity = streering_dir.dot(tire_velocity);

                // the change in velocity that we´re locking for is -steering_velocity * grip.
                // grip is in the range of 0.0 - 1.0. 0.0 means no grip. 1.0 means full grip
                let desired_change = -steering_velocity * wheel.tire_grip;

                // turn change in velocity into an acceleration.
                // this will produce the acceleration necessary to change the velocity by desired_vel_change in 1 physics step.
                let desired_acceleration = desired_change / time.delta_secs();

                ex_force.apply_force_at_point(
                    streering_dir * wheel.tire_mass * desired_acceleration,
                    gl_transform.translation(),
                    gl_transform_car.translation(),
                );
            }
        }
    }
}

fn steering_system(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut wheels: Query<&mut Transform, (Without<DrivenWheel>, With<SteeredWheel>)>,
) {
    for mut transform in &mut wheels {
        let mut rot = transform.rotation.to_euler(EulerRot::XYZ);

        if keys.pressed(KeyCode::KeyA) {
            rot.1 += -10.0 * time.delta_secs();
        } else if keys.pressed(KeyCode::KeyD) {
            rot.1 = 10.0 * time.delta_secs();
        } else {
            rot.1 = 0.0;
        }

        rot.1 = rot.1.clamp(-30.0, 30.0);
        transform.rotate(Quat::from_axis_angle(Vec3::Y, f32::to_radians(rot.1)));
    }
}

fn acceleration_system(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut cars: Query<(
        &Children,
        RigidBodyQuery,
        &Car,
        &GlobalTransform,
        &mut ExternalForce,
        &Transform,
    )>,
    wheels: Query<(&Wheel, &Transform, &GlobalTransform), With<DrivenWheel>>,
) {
    if !(keys.pressed(KeyCode::KeyW) || keys.pressed(KeyCode::KeyS)) {
        return;
    }

    for (children, body_query, car, gl_transform_car, mut ex_force, car_transform) in
        cars.iter_mut()
    {
        for wheel in children {
            if let Ok((wheel, transform, gl_transform)) = wheels.get(*wheel) {
                // Calculate input.
                let mut acceleration_input = -5.0; // use a little bit of friction
                // Accelerate...
                if keys.pressed(KeyCode::KeyW) {
                    acceleration_input = car.acceleration;
                }

                // ...break
                if keys.pressed(KeyCode::KeyS) {
                    acceleration_input = car.acceleration * -1.5;
                }

                // world-space direction of the acceleration / braking force.
                let acceleration_direction = transform.forward();
                // forward speed of the car (in the direction of driving)
                let car_speed = car_transform
                    .forward()
                    .dot(body_query.velocity_at_point(gl_transform_car.translation()));

                // normalized car speed
                let normalized_speed = (car_speed.abs() / car.top_speed).clamp(0.0, 1.0);

                // Sample the actual acceleration based on the normalized speed.
                let acceleration = car.acceleration_curve.position(normalized_speed) * acceleration_input;

                // apply force
                ex_force.apply_force_at_point(
                    acceleration_direction * acceleration.y,
                    gl_transform.translation(),
                    gl_transform_car.translation(),
                );
            }
        }
    }
}

fn spawn_vehicle(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let car_length = 4.0;
    let car_width = 2.2;
    let car_height = 1.5;
    let spawn_height = 3.0;

    let car_id = commands
        .spawn((
            Car::default(),
            Mesh3d(meshes.add(Cuboid::new(car_width, car_height, car_length))),
            MeshMaterial3d(materials.add(Color::WHITE)),
            RigidBody::Dynamic,
            ExternalForce::default().with_persistence(false),
            Collider::cuboid(car_width, car_height, car_length),
            Transform::from_xyz(0.0, car_height + spawn_height, 0.0).with_rotation(Quat::from_axis_angle(Vec3::Y, f32::to_radians(90.0))),
        ))
        .id();

    let wheel_radius = 0.5;
    let wheel_depth = 0.2;
    let wheel_rotation = Quat::from_axis_angle(Vec3::X, f32::to_radians(90.0));
    let wheel_chassis_spacing = 0.1;

    let wheel_height = car_height * -0.5;

    let wheel_transforms = vec![
        Transform::from_xyz(
            car_width * 0.5 + wheel_chassis_spacing + wheel_depth,
            wheel_height,
            car_length * 0.5 - wheel_radius,
        ),
        Transform::from_xyz(
            car_width * -0.5 - wheel_chassis_spacing - wheel_depth,
            wheel_height,
            car_length * 0.5 - wheel_radius,
        ),
        Transform::from_xyz(
            car_width * 0.5 + wheel_chassis_spacing + wheel_depth,
            wheel_height,
            car_length * -0.5 + wheel_radius,
        ),
        Transform::from_xyz(
            car_width * -0.5 - wheel_chassis_spacing - wheel_depth,
            wheel_height,
            car_length * -0.5 + wheel_radius,
        ),
    ];

    for (_i, wheel_transform) in wheel_transforms.iter().enumerate() {
        let wheel_id = commands
            .spawn((
                *wheel_transform,
                RayCaster::new(Vec3::ZERO, Dir3::from_xyz(0.0, -1.0, 0.0).unwrap()),
                Wheel::new(
                    wheel_radius * 1.5,
                    wheel_radius * 2.0,
                    15.0,
                    100.0,
                    0.80,
                    2.0,
                ),
            ))
            .id();

        if wheel_transform.translation.z > 0.0 {
            commands.entity(wheel_id).insert(SteeredWheel::default());
        } else {
            commands.entity(wheel_id).insert(DrivenWheel::default());
        }

        commands.entity(car_id).add_child(wheel_id);
    }
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
        Friction::new(0.2),
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
