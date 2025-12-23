use super::FixedAngleConstraintShared;
use crate::{
    dynamics::{
        joints::{LinearJointMotor, MotorModel},
        solver::{
            solver_body::{SolverBody, SolverBodyInertia},
            xpbd::{XpbdMotorConstraint, *},
        },
    },
    prelude::*,
};
use bevy::prelude::*;

/// Constraint data required by the XPBD constraint solver for a [`PrismaticJoint`].
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq)]
pub struct PrismaticJointSolverData {
    pub(super) world_r1: Vector,
    pub(super) world_r2: Vector,
    pub(super) center_difference: Vector,
    pub(super) free_axis1: Vector,
    pub(super) total_position_lagrange: Vector,
    pub(super) angle_constraint: FixedAngleConstraintShared,
    /// Accumulated motor Lagrange multiplier for this frame.
    pub(super) total_motor_lagrange: Scalar,
    /// Motor Lagrange multiplier from the previous frame, used for warm starting.
    /// This is zeroed after being applied in the first substep.
    pub(super) warm_start_motor_lagrange: Scalar,
}

impl XpbdConstraintSolverData for PrismaticJointSolverData {
    fn clear_lagrange_multipliers(&mut self) {
        self.total_position_lagrange = Vector::ZERO;
        self.angle_constraint.clear_lagrange_multipliers();
        // Save motor lagrange for warm starting before clearing.
        self.warm_start_motor_lagrange = self.total_motor_lagrange;
        self.total_motor_lagrange = 0.0;
    }

    fn total_position_lagrange(&self) -> Vector {
        self.total_position_lagrange
    }

    fn total_rotation_lagrange(&self) -> AngularVector {
        self.angle_constraint.total_rotation_lagrange()
    }

    fn total_motor_lagrange(&self) -> Scalar {
        self.total_motor_lagrange
    }
}

impl XpbdConstraint<2> for PrismaticJoint {
    type SolverData = PrismaticJointSolverData;

    fn prepare(
        &mut self,
        bodies: [&RigidBodyQueryReadOnlyItem; 2],
        solver_data: &mut PrismaticJointSolverData,
    ) {
        let [body1, body2] = bodies;

        let Some(local_anchor1) = self.local_anchor1() else {
            return;
        };
        let Some(local_anchor2) = self.local_anchor2() else {
            return;
        };
        let Some(local_basis1) = self.local_basis1() else {
            return;
        };
        let Some(local_basis2) = self.local_basis2() else {
            return;
        };

        // Prepare the point-to-point constraint.
        solver_data.angle_constraint.prepare(
            body1.rotation,
            body2.rotation,
            local_basis1,
            local_basis2,
        );

        // Prepare the prismatic joint.
        solver_data.world_r1 = body1.rotation * (local_anchor1 - body1.center_of_mass.0);
        solver_data.world_r2 = body2.rotation * (local_anchor2 - body2.center_of_mass.0);
        solver_data.center_difference = (body2.position.0 - body1.position.0)
            + (body2.rotation * body2.center_of_mass.0 - body1.rotation * body1.center_of_mass.0);
        solver_data.free_axis1 = *body1.rotation * local_basis1 * self.slider_axis;
    }

    fn solve(
        &mut self,
        bodies: [&mut SolverBody; 2],
        inertias: [&SolverBodyInertia; 2],
        solver_data: &mut PrismaticJointSolverData,
        dt: Scalar,
    ) {
        let [body1, body2] = bodies;

        // Solve the angular constraint.
        solver_data
            .angle_constraint
            .solve([body1, body2], inertias, self.angle_compliance, dt);

        // Constrain the relative positions of the bodies, only allowing translation along one free axis.
        self.constrain_positions(body1, body2, inertias[0], inertias[1], solver_data, dt);
    }
}

impl XpbdMotorConstraint<2> for PrismaticJoint {
    type Motor = LinearJointMotor;

    fn solve_motor(
        &self,
        bodies: [&mut SolverBody; 2],
        inertias: [&SolverBodyInertia; 2],
        solver_data: &mut PrismaticJointSolverData,
        motor: &LinearJointMotor,
        dt: Scalar,
    ) {
        let [body1, body2] = bodies;
        let [inertia1, inertia2] = inertias;

        self.apply_motor(body1, body2, inertia1, inertia2, solver_data, motor, dt);
    }

    fn warm_start_motor(
        &self,
        bodies: [&mut SolverBody; 2],
        inertias: [&SolverBodyInertia; 2],
        solver_data: &mut PrismaticJointSolverData,
        _dt: Scalar,
        warm_start_coefficient: Scalar,
    ) {
        let [body1, body2] = bodies;
        let [inertia1, inertia2] = inertias;

        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

        // Get the slider axis in world space.
        let axis = body1.delta_rotation * solver_data.free_axis1;

        // Get anchor points for angular velocity contribution.
        let world_r1 = body1.delta_rotation * solver_data.world_r1;
        let world_r2 = body2.delta_rotation * solver_data.world_r2;

        // Apply the previous frame's motor impulse as an initial velocity change.
        // The motor lagrange is a scalar (magnitude along the slider axis).
        let impulse = warm_start_coefficient * solver_data.warm_start_motor_lagrange * axis;

        // Apply linear impulse to both bodies.
        // Motor impulse is positive when body2 should move in the positive axis direction.
        body1.linear_velocity -= impulse * inv_mass1;
        body2.linear_velocity += impulse * inv_mass2;

        // Apply angular impulse from the moment arm.
        body1.angular_velocity -= inv_angular_inertia1 * cross(world_r1, impulse);
        body2.angular_velocity += inv_angular_inertia2 * cross(world_r2, impulse);

        // Clear the warm start lagrange after applying it.
        solver_data.warm_start_motor_lagrange = 0.0;
    }
}

impl PrismaticJoint {
    /// Constrains the relative positions of the bodies, only allowing translation along one free axis.
    ///
    /// Returns the force exerted by this constraint.
    fn constrain_positions(
        &self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        inertia2: &SolverBodyInertia,
        solver_data: &mut PrismaticJointSolverData,
        dt: Scalar,
    ) {
        // Compute the effective inverse masses and angular inertias of the bodies.
        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

        let world_r1 = body1.delta_rotation * solver_data.world_r1;
        let world_r2 = body2.delta_rotation * solver_data.world_r2;

        let mut delta_x = Vector::ZERO;

        let axis1 = body1.delta_rotation * solver_data.free_axis1;
        if let Some(limits) = self.limits {
            let separation = (body2.delta_position - body1.delta_position)
                + (world_r2 - world_r1)
                + solver_data.center_difference;
            delta_x += limits.compute_correction_along_axis(separation, axis1);
        }

        let zero_distance_limit = DistanceLimit::ZERO;

        #[cfg(feature = "2d")]
        {
            let axis2 = Vector::new(axis1.y, -axis1.x);

            let separation = (body2.delta_position - body1.delta_position)
                + (world_r2 - world_r1)
                + solver_data.center_difference;
            delta_x += zero_distance_limit.compute_correction_along_axis(separation, axis2);
        }
        #[cfg(feature = "3d")]
        {
            let axis2 = axis1.any_orthogonal_vector();
            let axis3 = axis1.cross(axis2);

            let separation = (body2.delta_position - body1.delta_position)
                + (world_r2 - world_r1)
                + solver_data.center_difference;
            delta_x += zero_distance_limit.compute_correction_along_axis(separation, axis2);

            let separation = (body2.delta_position - body1.delta_position)
                + (world_r2 - world_r1)
                + solver_data.center_difference;
            delta_x += zero_distance_limit.compute_correction_along_axis(separation, axis3);
        }

        let magnitude = delta_x.length();

        if magnitude <= Scalar::EPSILON {
            return;
        }

        let dir = delta_x / magnitude;

        // Compute generalized inverse masses
        let w1 = PositionConstraint::compute_generalized_inverse_mass(
            self,
            inv_mass1.max_element(),
            inv_angular_inertia1,
            world_r1,
            dir,
        );
        let w2 = PositionConstraint::compute_generalized_inverse_mass(
            self,
            inv_mass2.max_element(),
            inv_angular_inertia2,
            world_r2,
            dir,
        );

        // Compute Lagrange multiplier update
        let delta_lagrange =
            compute_lagrange_update(0.0, magnitude, &[w1, w2], self.align_compliance, dt);
        let impulse = delta_lagrange * dir;
        solver_data.total_position_lagrange += impulse;

        // Apply positional correction to align the positions of the bodies
        self.apply_positional_impulse(
            body1, body2, inertia1, inertia2, impulse, world_r1, world_r2,
        );
    }
}

impl PrismaticJoint {
    /// Applies motor forces to drive the joint towards the target velocity and/or position.
    pub fn apply_motor(
        &self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        inertia2: &SolverBodyInertia,
        solver_data: &mut PrismaticJointSolverData,
        motor: &LinearJointMotor,
        dt: Scalar,
    ) {
        // Get the slider axis in world space.
        let axis1 = body1.delta_rotation * solver_data.free_axis1;

        // Compute the current position along the slider axis.
        let world_r1 = body1.delta_rotation * solver_data.world_r1;
        let world_r2 = body2.delta_rotation * solver_data.world_r2;
        let separation = (body2.delta_position - body1.delta_position)
            + (world_r2 - world_r1)
            + solver_data.center_difference;
        let current_position = separation.dot(axis1);

        // Compute the linear velocity projected onto the slider axis.
        let relative_velocity = body2.linear_velocity - body1.linear_velocity;
        let current_velocity = relative_velocity.dot(axis1);

        // Get effective inverse masses.
        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

        // Compute generalized inverse masses.
        let w1 = PositionConstraint::compute_generalized_inverse_mass(
            self,
            inv_mass1.max_element(),
            inv_angular_inertia1,
            world_r1,
            axis1,
        );
        let w2 = PositionConstraint::compute_generalized_inverse_mass(
            self,
            inv_mass2.max_element(),
            inv_angular_inertia2,
            world_r2,
            axis1,
        );

        let w_sum = w1 + w2;
        if w_sum <= Scalar::EPSILON {
            return;
        }

        // Compute the motor impulse using a PD controller approach.
        // The motor tries to drive the joint to:
        // - target_velocity with damping strength
        // - target_position with stiffness strength
        let velocity_error = motor.target_velocity - current_velocity;
        let position_error = motor.target_position - current_position;

        // Compute the desired velocity change based on motor parameters.
        let target_velocity_change = if let (Some(frequency), Some(damping_ratio)) =
            (motor.frequency, motor.damping_ratio)
        {
            // Use timestep-independent implicit Euler formulation.
            // This provides stable spring-damper behavior regardless of substep count.
            let omega = TAU * frequency;
            let omega_sq = omega * omega;
            let two_zeta_omega = 2.0 * damping_ratio * omega;

            // Implicit Euler denominator for stability.
            let inv_denominator = 1.0 / (1.0 + two_zeta_omega * dt + omega_sq * dt * dt);

            // Compute velocity change with implicit Euler integration.
            (omega_sq * position_error + two_zeta_omega * velocity_error) * dt * inv_denominator
        } else {
            // Use the legacy stiffness/damping formulation.
            match motor.motor_model {
                MotorModel::AccelerationBased => {
                    // Directly compute velocity change needed.
                    motor.damping * velocity_error + motor.stiffness * position_error * dt
                }
                MotorModel::ForceBased => {
                    // Force = stiffness * position_error + damping * velocity_error
                    // Velocity change = Force * inv_mass = Force * w_sum
                    // The dt scaling happens in the correction computation below.
                    (motor.stiffness * position_error + motor.damping * velocity_error) * w_sum
                }
            }
        };

        // Convert to position correction (impulse = m * dv, but in XPBD we work with positions).
        // The correction is the position change needed: dv * dt.
        let correction = target_velocity_change * dt;

        // Skip if correction is too small.
        if correction.abs() <= Scalar::EPSILON {
            return;
        }

        // Compute Lagrange multiplier update.
        // The constraint is: we want to apply a correction of `correction` along the axis.
        // Using XPBD: delta_lagrange = correction / w_sum (simplified for motors).
        let delta_lagrange = correction / w_sum;

        // Clamp the delta lagrange based on max force.
        // max_force * dt^2 gives the max lagrange per substep.
        let delta_lagrange = if motor.max_force < Scalar::MAX && motor.max_force > 0.0 {
            let max_delta = motor.max_force * dt * dt;
            let new_lagrange = solver_data.total_motor_lagrange + delta_lagrange;
            if new_lagrange.abs() > max_delta {
                let clamped = new_lagrange.clamp(-max_delta, max_delta);
                clamped - solver_data.total_motor_lagrange
            } else {
                delta_lagrange
            }
        } else {
            delta_lagrange
        };

        solver_data.total_motor_lagrange += delta_lagrange;

        // Compute the impulse along the axis.
        // Positive delta_lagrange should move body2 in positive axis direction relative to body1.
        let impulse = delta_lagrange * axis1;
        solver_data.total_position_lagrange += impulse;

        // Apply positional correction.
        // Note: apply_positional_impulse adds to body1 and subtracts from body2.
        // For motors, we want body2 to move in the direction of the impulse,
        // so we negate the impulse to get the correct direction.
        self.apply_positional_impulse(
            body1, body2, inertia1, inertia2, -impulse, world_r1, world_r2,
        );
    }
}

impl PositionConstraint for PrismaticJoint {}

impl AngularConstraint for PrismaticJoint {}
