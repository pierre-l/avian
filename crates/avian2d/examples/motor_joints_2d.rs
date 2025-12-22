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
            MassPropertiesBundle::from_shape(&Circle::new(40.0), 1.0),
        ))
        .id();

    // Revolute joint with angular motor
    commands.spawn((
        RevoluteJoint::new(wheel_anchor, wheel),
        AngularJointMotor {
            target_velocity: 5.0, // 5 rad/s
            damping: 10.0,
            max_torque: 1000.0,
            motor_model: MotorModel::AccelerationBased,
            ..default()
        },
        RevoluteMotorJoint,
    ));

    // Add spokes to the wheel for visual rotation feedback
    for i in 0..4 {
        let angle = i as f32 * std::f32::consts::FRAC_PI_2;
        commands.spawn((
            Sprite {
                color: Color::srgb(0.7, 0.2, 0.2),
                custom_size: Some(Vec2::new(60.0, 8.0)),
                ..default()
            },
            Transform::from_xyz(-200.0, 0.0, 1.0).with_rotation(Quat::from_rotation_z(angle)),
            RigidBody::Dynamic,
            MassPropertiesBundle::from_shape(&Rectangle::new(60.0, 8.0), 0.1),
        ));
    }

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
            MassPropertiesBundle::from_shape(&Rectangle::new(60.0, 40.0), 1.0),
        ))
        .id();

    // Prismatic joint with linear motor
    commands.spawn((
        PrismaticJoint::new(piston_base, piston).with_slider_axis(Vector::Y),
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
            // Toggle motor by setting damping to 0 or restoring it
            if motor.damping > 0.0 {
                motor.damping = 0.0;
                motor.target_velocity = 0.0;
            } else {
                motor.damping = 10.0;
                motor.target_velocity = 5.0;
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
            // Toggle motor
            if motor.stiffness > 0.0 {
                motor.stiffness = 0.0;
                motor.damping = 0.0;
            } else {
                motor.stiffness = 20.0;
                motor.damping = 10.0;
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
