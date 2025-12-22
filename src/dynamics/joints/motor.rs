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
/// When attached to a revolute joint entity, this component applies torque
/// to drive the joint towards a target velocity and/or position.
///
/// For velocity control, set [`target_velocity`](Self::target_velocity) with zero [`stiffness`](Self::stiffness).
/// For position control, set [`target_position`](Self::target_position) with non-zero [`stiffness`](Self::stiffness).
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq)]
pub struct AngularJointMotor {
    /// The target angular velocity (rad/s).
    pub target_velocity: Scalar,
    /// The target angle (rad) for position control. Only used when `stiffness` is non-zero.
    pub target_position: Scalar,
    /// The stiffness coefficient for position control (N·m/rad). Set to zero for pure velocity control.
    pub stiffness: Scalar,
    /// The damping coefficient (N·m·s/rad).
    pub damping: Scalar,
    /// The maximum torque the motor can apply (N·m).
    pub max_torque: Scalar,
    /// The motor model used for computing the motor torque.
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
/// When attached to a prismatic joint entity, this component applies force
/// to drive the joint towards a target velocity and/or position.
///
/// For velocity control, set [`target_velocity`](Self::target_velocity) with zero [`stiffness`](Self::stiffness).
/// For position control, set [`target_position`](Self::target_position) with non-zero [`stiffness`](Self::stiffness).
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq)]
pub struct LinearJointMotor {
    /// The target linear velocity (m/s).
    pub target_velocity: Scalar,
    /// The target position (m) for position control. Only used when `stiffness` is non-zero.
    pub target_position: Scalar,
    /// The stiffness coefficient for position control (N/m). Set to zero for pure velocity control.
    pub stiffness: Scalar,
    /// The damping coefficient (N·s/m).
    pub damping: Scalar,
    /// The maximum force the motor can apply (N).
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
