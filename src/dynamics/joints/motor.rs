//! Joint motors for driving revolute and prismatic joints.
//!
//! Motors can be added to [`RevoluteJoint`] and [`PrismaticJoint`] entities
//! to actively drive the joint towards a target velocity or position.
//!
//! # Motor Types
//!
//! - [`AngularJointMotor`]: For revolute joints, drives rotation around the hinge axis.
//! - [`LinearJointMotor`]: For prismatic joints, drives translation along the slider axis.
//!
//! # Motor Modes
//!
//! Motors can operate in different modes depending on the parameters you set:
//!
//! - **Velocity mode**: Set a non-zero `target_velocity` and zero `stiffness` to drive
//!   the joint at a constant velocity.
//! - **Position mode**: Set a non-zero `stiffness` and `target_position` to drive
//!   the joint towards a specific position with spring-like behavior.
//! - **Combined mode**: Set both velocity and stiffness for a combination of both behaviors.
//!
//! # Example
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! # fn setup(mut commands: Commands) {
//! #     let body1 = commands.spawn(RigidBody::Dynamic).id();
//! #     let body2 = commands.spawn(RigidBody::Dynamic).id();
//! #
//! // Create a revolute joint with a motor that drives at 2 rad/s
//! commands.spawn((
//!     RevoluteJoint::new(body1, body2),
//!     AngularJointMotor {
//!         target_velocity: 2.0,
//!         max_torque: 100.0,
//!         ..default()
//!     },
//! ));
//! # }
//! ```

use crate::prelude::*;
use bevy::prelude::*;

/// The motor model determines how the motor force/torque is computed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq, Hash)]
pub enum MotorModel {
    /// The motor force/torque is computed based on the acceleration required to reach the target.
    ///
    /// This model is more stable and easier to tune, but it ignores the mass of the bodies.
    #[default]
    AccelerationBased,
    /// The motor force/torque is computed directly from the stiffness and damping parameters.
    ///
    /// This model takes the mass of the bodies into account, resulting in more physically
    /// accurate behavior, but it may be harder to tune.
    ForceBased,
}

/// A motor for driving the angular motion of a [`RevoluteJoint`].
///
/// When attached to a revolute joint entity, this component will apply torque
/// to drive the joint towards the target velocity and/or position.
///
/// # Motor Modes
///
/// - **Velocity mode**: Set `target_velocity` and keep `stiffness` at zero.
///   The motor will apply torque to reach and maintain the target angular velocity.
///
/// - **Position mode**: Set `target_position` and a non-zero `stiffness`.
///   The motor will act like a spring, applying torque to reach the target angle.
///
/// - **Combined mode**: Set both `target_velocity` and `stiffness`.
///   The motor will try to reach the target position while also considering velocity.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// #     let body1 = commands.spawn(RigidBody::Dynamic).id();
/// #     let body2 = commands.spawn(RigidBody::Dynamic).id();
/// #
/// // Create a revolute joint with a motor
/// commands.spawn((
///     RevoluteJoint::new(body1, body2),
///     AngularJointMotor {
///         target_velocity: 3.0,  // 3 rad/s
///         max_torque: 50.0,
///         ..default()
///     },
/// ));
/// # }
/// ```
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq)]
pub struct AngularJointMotor {
    /// The target angular velocity in radians per second.
    ///
    /// A positive value rotates from body1 towards body2 in the direction
    /// defined by the joint's axis.
    pub target_velocity: Scalar,
    /// The target angle in radians for position-based motor control.
    ///
    /// This is only used when `stiffness` is non-zero.
    /// The motor will act like a spring, applying torque to reach this angle.
    pub target_position: Scalar,
    /// The stiffness coefficient for position-based motor control (N·m/rad).
    ///
    /// Higher values make the motor respond more strongly to position errors.
    /// Set to zero for pure velocity control.
    pub stiffness: Scalar,
    /// The damping coefficient (N·m·s/rad).
    ///
    /// Controls how much the motor resists velocity changes.
    /// Higher values result in smoother, less oscillatory motion.
    pub damping: Scalar,
    /// The maximum torque the motor can apply (N·m).
    ///
    /// The actual applied torque will be clamped to this value.
    /// Set to a high value (or [`Scalar::MAX`]) for no limit.
    pub max_torque: Scalar,
    /// The motor model used for computing the motor force.
    pub motor_model: MotorModel,
}

impl Default for AngularJointMotor {
    fn default() -> Self {
        Self {
            target_velocity: 0.0,
            target_position: 0.0,
            stiffness: 0.0,
            damping: 0.0,
            max_torque: Scalar::MAX,
            motor_model: MotorModel::default(),
        }
    }
}

impl AngularJointMotor {
    /// Creates a new angular motor with the given target velocity.
    ///
    /// This creates a velocity-mode motor with no position control.
    #[inline]
    pub const fn new(target_velocity: Scalar) -> Self {
        Self {
            target_velocity,
            target_position: 0.0,
            stiffness: 0.0,
            damping: 0.0,
            max_torque: Scalar::MAX,
            motor_model: MotorModel::AccelerationBased,
        }
    }

    /// Creates a new angular motor targeting a specific position.
    ///
    /// This creates a position-mode motor that acts like a spring.
    #[inline]
    pub const fn with_target_position(target_position: Scalar, stiffness: Scalar) -> Self {
        Self {
            target_velocity: 0.0,
            target_position,
            stiffness,
            damping: 0.0,
            max_torque: Scalar::MAX,
            motor_model: MotorModel::AccelerationBased,
        }
    }

    /// Sets the target angular velocity in radians per second.
    #[inline]
    pub const fn with_target_velocity(mut self, velocity: Scalar) -> Self {
        self.target_velocity = velocity;
        self
    }

    /// Sets the target position and stiffness for position control.
    #[inline]
    pub const fn with_position_target(
        mut self,
        target_position: Scalar,
        stiffness: Scalar,
    ) -> Self {
        self.target_position = target_position;
        self.stiffness = stiffness;
        self
    }

    /// Sets the damping coefficient.
    #[inline]
    pub const fn with_damping(mut self, damping: Scalar) -> Self {
        self.damping = damping;
        self
    }

    /// Sets the maximum torque the motor can apply.
    #[inline]
    pub const fn with_max_torque(mut self, max_torque: Scalar) -> Self {
        self.max_torque = max_torque;
        self
    }

    /// Sets the motor model.
    #[inline]
    pub const fn with_motor_model(mut self, model: MotorModel) -> Self {
        self.motor_model = model;
        self
    }
}

/// A motor for driving the linear motion of a [`PrismaticJoint`].
///
/// When attached to a prismatic joint entity, this component will apply force
/// to drive the joint towards the target velocity and/or position.
///
/// # Motor Modes
///
/// - **Velocity mode**: Set `target_velocity` and keep `stiffness` at zero.
///   The motor will apply force to reach and maintain the target linear velocity.
///
/// - **Position mode**: Set `target_position` and a non-zero `stiffness`.
///   The motor will act like a spring, applying force to reach the target position.
///
/// - **Combined mode**: Set both `target_velocity` and `stiffness`.
///   The motor will try to reach the target position while also considering velocity.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// #     let body1 = commands.spawn(RigidBody::Dynamic).id();
/// #     let body2 = commands.spawn(RigidBody::Dynamic).id();
/// #
/// // Create a prismatic joint with a linear motor
/// commands.spawn((
///     PrismaticJoint::new(body1, body2),
///     LinearJointMotor {
///         target_velocity: 1.0,  // 1 m/s
///         max_force: 100.0,
///         ..default()
///     },
/// ));
/// # }
/// ```
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq)]
pub struct LinearJointMotor {
    /// The target linear velocity in meters per second.
    ///
    /// A positive value moves body2 away from body1 along the slider axis.
    pub target_velocity: Scalar,
    /// The target position in meters for position-based motor control.
    ///
    /// This is only used when `stiffness` is non-zero.
    /// The motor will act like a spring, applying force to reach this position.
    pub target_position: Scalar,
    /// The stiffness coefficient for position-based motor control (N/m).
    ///
    /// Higher values make the motor respond more strongly to position errors.
    /// Set to zero for pure velocity control.
    pub stiffness: Scalar,
    /// The damping coefficient (N·s/m).
    ///
    /// Controls how much the motor resists velocity changes.
    /// Higher values result in smoother, less oscillatory motion.
    pub damping: Scalar,
    /// The maximum force the motor can apply (N).
    ///
    /// The actual applied force will be clamped to this value.
    /// Set to a high value (or [`Scalar::MAX`]) for no limit.
    pub max_force: Scalar,
    /// The motor model used for computing the motor force.
    pub motor_model: MotorModel,
}

impl Default for LinearJointMotor {
    fn default() -> Self {
        Self {
            target_velocity: 0.0,
            target_position: 0.0,
            stiffness: 0.0,
            damping: 0.0,
            max_force: Scalar::MAX,
            motor_model: MotorModel::default(),
        }
    }
}

impl LinearJointMotor {
    /// Creates a new linear motor with the given target velocity.
    ///
    /// This creates a velocity-mode motor with no position control.
    #[inline]
    pub const fn new(target_velocity: Scalar) -> Self {
        Self {
            target_velocity,
            target_position: 0.0,
            stiffness: 0.0,
            damping: 0.0,
            max_force: Scalar::MAX,
            motor_model: MotorModel::AccelerationBased,
        }
    }

    /// Creates a new linear motor targeting a specific position.
    ///
    /// This creates a position-mode motor that acts like a spring.
    #[inline]
    pub const fn with_target_position(target_position: Scalar, stiffness: Scalar) -> Self {
        Self {
            target_velocity: 0.0,
            target_position,
            stiffness,
            damping: 0.0,
            max_force: Scalar::MAX,
            motor_model: MotorModel::AccelerationBased,
        }
    }

    /// Sets the target linear velocity in meters per second.
    #[inline]
    pub const fn with_target_velocity(mut self, velocity: Scalar) -> Self {
        self.target_velocity = velocity;
        self
    }

    /// Sets the target position and stiffness for position control.
    #[inline]
    pub const fn with_position_target(
        mut self,
        target_position: Scalar,
        stiffness: Scalar,
    ) -> Self {
        self.target_position = target_position;
        self.stiffness = stiffness;
        self
    }

    /// Sets the damping coefficient.
    #[inline]
    pub const fn with_damping(mut self, damping: Scalar) -> Self {
        self.damping = damping;
        self
    }

    /// Sets the maximum force the motor can apply.
    #[inline]
    pub const fn with_max_force(mut self, max_force: Scalar) -> Self {
        self.max_force = max_force;
        self
    }

    /// Sets the motor model.
    #[inline]
    pub const fn with_motor_model(mut self, model: MotorModel) -> Self {
        self.motor_model = model;
        self
    }
}
