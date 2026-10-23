use avian3d::prelude::*;
use bevy::prelude::*;

use super::Benchmark3dPlugins;

/// Capsule radius, roughly a person's shoulders.
const RADIUS: f32 = 0.25;
/// Length of the capsule's segment, for a character 1.8 m tall.
const SEGMENT: f32 = 1.8 - 2.0 * RADIUS;
/// Gap left between the capsules and the floor, so that they start clear of it.
const FLOOR_GAP: f32 = 0.05;
/// Walking speed, in m/s.
const SPEED: f32 = 4.8;

#[derive(Component)]
struct Character;

#[derive(PhysicsLayer, Default)]
enum Layer {
    #[default]
    Default,
    Character,
}

/// Kinematic characters moved with `MoveAndSlide`, all walking towards the centre of the
/// arena, where they jam into a dense crowd.
///
/// The characters start on a grid with `spacing` metres between them, which is how dense
/// the crowd starts. The characters move one at a time, but the spatial queries only see
/// where each of them stood at the start of the step, as Avian only updates the collider
/// tree in the physics step.
pub fn create_bench(count: usize, spacing: f32) -> App {
    let mut app = App::new();
    app.add_plugins((Benchmark3dPlugins, PhysicsPlugins::default()));
    app.add_systems(Startup, move |commands: Commands| {
        setup(commands, count, spacing)
    });
    app.add_systems(FixedUpdate, move_characters);
    app
}

fn setup(mut commands: Commands, count: usize, spacing: f32) {
    let side = (count as f32).sqrt().ceil() as usize;
    let half_extent = side as f32 * spacing / 2.0;
    let arena = 2.0 * half_extent + 2.0;

    // Ground
    commands.spawn((
        RigidBody::Static,
        Collider::cuboid(arena, 1.0, arena),
        Transform::from_xyz(0.0, -0.5, 0.0),
    ));

    // Walls around the arena
    for (x, z, width, depth) in [
        (0.0, arena / 2.0, arena, 0.5),
        (0.0, -arena / 2.0, arena, 0.5),
        (arena / 2.0, 0.0, 0.5, arena),
        (-arena / 2.0, 0.0, 0.5, arena),
    ] {
        commands.spawn((
            RigidBody::Static,
            Collider::cuboid(width, 3.0, depth),
            Transform::from_xyz(x, 1.5, z),
        ));
    }

    let collider = Collider::capsule(RADIUS, SEGMENT);
    let y = SEGMENT / 2.0 + RADIUS + FLOOR_GAP;

    for i in 0..count {
        let (row, column) = (i / side, i % side);
        // Offset every other row so that the characters don't all line up with the centre.
        let shift = if row % 2 == 0 { 0.0 } else { spacing / 2.0 };
        let x = column as f32 * spacing + shift - half_extent;
        let z = row as f32 * spacing - half_extent;

        commands.spawn((
            Character,
            RigidBody::Kinematic,
            // Movement is handled by `MoveAndSlide`.
            CustomPositionIntegration,
            // Characters only see the world and each other through `MoveAndSlide`'s
            // spatial queries, so the solver doesn't need any contacts for them.
            // This keeps the benchmark focused on the character movement.
            CollisionLayers::new(Layer::Character, LayerMask::NONE),
            collider.clone(),
            Transform::from_xyz(x, y, z),
        ));
    }
}

fn move_characters(
    mut query: Query<(Entity, &mut Transform, &mut LinearVelocity, &Collider), With<Character>>,
    move_and_slide: MoveAndSlide,
    time: Res<Time>,
) {
    let config = MoveAndSlideConfig::default();

    for (entity, mut transform, mut lin_vel, collider) in &mut query {
        // Walk towards the centre, along the ground.
        let position = transform.translation;
        let towards_centre = Vec3::new(-position.x, 0.0, -position.z).normalize_or_zero();

        let MoveAndSlideOutput {
            position: new_position,
            projected_velocity,
        } = move_and_slide.move_and_slide(
            collider,
            position,
            transform.rotation,
            towards_centre * SPEED,
            time.delta(),
            &config,
            &SpatialQueryFilter::from_excluded_entities([entity]),
            |_| MoveAndSlideHitResponse::Accept,
        );

        transform.translation = new_position;
        lin_vel.0 = projected_velocity;
    }
}
