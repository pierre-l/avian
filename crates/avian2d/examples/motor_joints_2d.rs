//! Demonstrates motor joints in 2D.
//!
//! - Left side: Revolute joint with angular motor (spinning wheel)
//! - Right side: Prismatic joint with linear motor (piston)
//!
//! Controls:
//! - Arrow keys: Adjust revolute motor target velocity
//! - W/S: Adjust prismatic motor target position
//! - Space: Toggle motors on/off

use avian2d::{math::*, prelude::*};
use bevy::prelude::*;
use examples_common_2d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            PhysicsPlugins::default(),
        ))
        .insert_resource(ClearColor(Color::srgb(0.05, 0.05, 0.1)))
        .insert_resource(SubstepCount(50))
        .insert_resource(Gravity(Vector::ZERO)) // No gravity for clearer motor demo
        .add_systems(Startup, setup)
        .add_systems(Update, (control_motors, update_ui))
        .run();
}

#[derive(Component)]
struct RevoluteMotorJoint;

#[derive(Component)]
struct PrismaticMotorJoint;

#[derive(Component)]
struct UiText;

fn setup(mut commands: Commands) {
    commands.spawn(Camera2d);

    // === Revolute Joint with Angular Motor (left side) ===
    let wheel_sprite = Sprite {
        color: Color::srgb(0.9, 0.3, 0.3),
        custom_size: Some(Vec2::splat(80.0)),
        ..default()
    };

    // Static anchor for the wheel
    let wheel_anchor = commands
        .spawn((
            Sprite {
                color: Color::srgb(0.5, 0.5, 0.5),
                custom_size: Some(Vec2::splat(20.0)),
                ..default()
            },
            Transform::from_xyz(-200.0, 0.0, 0.0),
            RigidBody::Static,
        ))
        .id();

    // Spinning wheel
    let wheel = commands
        .spawn((
            wheel_sprite,
            Transform::from_xyz(-200.0, 0.0, 0.0),
            RigidBody::Dynamic,
            Mass(1.0),
            AngularInertia(1.0),
            SleepingDisabled, // Prevent sleeping so motor can always control it
        ))
        .id();

    // Revolute joint with angular motor
    // Note: Local anchors must be set for the joint to work properly.
    commands.spawn((
        RevoluteJoint::new(wheel_anchor, wheel)
            .with_local_anchor1(Vector::ZERO)
            .with_local_anchor2(Vector::ZERO),
        AngularJointMotor {
            target_velocity: 5.0, // 5 rad/s
            damping: 1.0,         // Lower gain for stable approach (1.0 ≈ reach target in ~1 second)
            max_torque: 1000.0,
            motor_model: MotorModel::AccelerationBased,
            ..default()
        },
        RevoluteMotorJoint,
    ));

    // === Prismatic Joint with Linear Motor (right side) ===
    let piston_base_sprite = Sprite {
        color: Color::srgb(0.5, 0.5, 0.5),
        custom_size: Some(Vec2::new(40.0, 200.0)),
        ..default()
    };

    let piston_sprite = Sprite {
        color: Color::srgb(0.3, 0.9, 0.3),
        custom_size: Some(Vec2::new(60.0, 40.0)),
        ..default()
    };

    // Static base for the piston
    let piston_base = commands
        .spawn((
            piston_base_sprite,
            Transform::from_xyz(200.0, 0.0, 0.0),
            RigidBody::Static,
        ))
        .id();

    // Moving piston
    let piston = commands
        .spawn((
            piston_sprite,
            Transform::from_xyz(200.0, 0.0, 0.0),
            RigidBody::Dynamic,
            Mass(1.0),
            SleepingDisabled, // Prevent sleeping so motor can always control it
        ))
        .id();

    // Prismatic joint with linear motor
    // Note: Local anchors must be set for the joint to work properly.
    commands.spawn((
        PrismaticJoint::new(piston_base, piston)
            .with_local_anchor1(Vector::ZERO)
            .with_local_anchor2(Vector::ZERO)
            .with_slider_axis(Vector::Y),
        LinearJointMotor {
            target_position: 50.0, // Target 50 units up
            stiffness: 20.0,
            damping: 10.0,
            max_force: 5000.0,
            motor_model: MotorModel::AccelerationBased,
            ..default()
        },
        PrismaticMotorJoint,
    ));

    // UI Text
    commands.spawn((
        Text::new("Motor Joints Demo\n\nArrow Up/Down: Revolute motor speed\nW/S: Prismatic motor position\nSpace: Toggle motors\n\nRevolute: 5.0 rad/s\nPrismatic: 50.0 units"),
        TextFont {
            font_size: 18.0,
            ..default()
        },
        TextColor(Color::WHITE),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        },
        UiText,
    ));
}

fn control_motors(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut revolute_motors: Query<&mut AngularJointMotor, With<RevoluteMotorJoint>>,
    mut prismatic_motors: Query<&mut LinearJointMotor, With<PrismaticMotorJoint>>,
) {
    // Control revolute motor with arrow keys
    for mut motor in revolute_motors.iter_mut() {
        if keyboard.just_pressed(KeyCode::ArrowUp) {
            motor.target_velocity += 1.0;
        }
        if keyboard.just_pressed(KeyCode::ArrowDown) {
            motor.target_velocity -= 1.0;
        }
        if keyboard.just_pressed(KeyCode::Space) {
            // Toggle motor: when "off", keep damping but set target to 0 (braking)
            if motor.target_velocity != 0.0 {
                motor.target_velocity = 0.0; // Brake to a stop
            } else {
                motor.target_velocity = 5.0; // Resume spinning
            }
        }
    }

    // Control prismatic motor with W/S keys
    for mut motor in prismatic_motors.iter_mut() {
        if keyboard.just_pressed(KeyCode::KeyW) {
            motor.target_position += 20.0;
        }
        if keyboard.just_pressed(KeyCode::KeyS) {
            motor.target_position -= 20.0;
        }
        if keyboard.just_pressed(KeyCode::Space) {
            // Toggle motor: when "off", set target to 0 (return to center)
            if motor.target_position != 0.0 {
                motor.target_position = 0.0; // Return to center
            } else {
                motor.target_position = 50.0; // Move to target
            }
        }
    }
}

fn update_ui(
    revolute_motors: Query<&AngularJointMotor, With<RevoluteMotorJoint>>,
    prismatic_motors: Query<&LinearJointMotor, With<PrismaticMotorJoint>>,
    mut ui_text: Query<&mut Text, With<UiText>>,
) {
    let revolute_vel = revolute_motors
        .iter()
        .next()
        .map(|m| m.target_velocity)
        .unwrap_or(0.0);
    let prismatic_pos = prismatic_motors
        .iter()
        .next()
        .map(|m| m.target_position)
        .unwrap_or(0.0);

    for mut text in ui_text.iter_mut() {
        text.0 = format!(
            "Motor Joints Demo\n\n\
             Arrow Up/Down: Revolute motor speed\n\
             W/S: Prismatic motor position\n\
             Space: Toggle motors\n\n\
             Revolute: {:.1} rad/s\n\
             Prismatic: {:.1} units",
            revolute_vel, prismatic_pos
        );
    }
}
