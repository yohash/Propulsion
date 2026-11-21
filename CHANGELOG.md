# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.6.0] - Unreleased

This release converges the PD controllers into a single, physically exact controller.
It contains breaking API changes: see **Migrating from 0.5** below.

### Migrating from 0.5

| 0.5 | 0.6 |
|---|---|
| `BackwardsPdController` | `StablePdController` |
| `StablePdController2D` | `StablePdController` (the single-axis overloads) |
| `backwardsPd.ComputeForce(dt, position, targetPosition, velocity)` | `stablePd.ComputeAcceleration(dt, position, velocity, targetPosition, Vector3.zero)`, or `ComputeForce(dt, mass, ...)` with the same arguments, applied with `AddForce` |
| `backwardsPd.ComputeTorque(dt, targetRotation, rotation, angularVelocity, inertiaTensorRotation, inertiaTensor)` | `stablePd.ComputeTorque(dt, inertiaTensorRotation, inertiaTensor, rotation, angularVelocity, targetRotation, Vector3.zero)` |
| `stablePd.ComputeForce(dt, p, v, pTarget, vTarget, mass)` | `stablePd.ComputeForce(dt, mass, p, v, pTarget, vTarget)` |
| `stablePd.ComputeTorque(dt, r, w, rTarget, wTarget, inertiaTensorRotation, inertiaTensor)` | `stablePd.ComputeTorque(dt, inertiaTensorRotation, inertiaTensor, r, w, rTarget, wTarget)` |
| `stablePd2D.ComputeForce1D(dt, p, v, pTarget, vTarget, mass)` | `stablePd.ComputeForce(dt, mass, p, v, pTarget, vTarget)` |
| `stablePd2D.ComputeTorque(dt, angle, w, targetAngle, wTarget, inertia)` | `stablePd.ComputeTorque(dt, inertia, angle, w, targetAngle, wTarget)` |
| `pid.ComputeForce1D(dt, current, target)` | `pid.Compute(dt, current, target)` |
| `pid.ComputeTorque1D(dt, currentAngle, targetAngle)` | `pid.ComputeAngle(dt, currentAngle, targetAngle)` |
| `PidController.AngleDifference(a, b)` | `Mathf.DeltaAngle(b, a)` |

- **Retune `frequency`.** It is now in Hz: divide old values by about 2 for a similar stiffness (old 4 is about new 2). `damping = 1` is now truly critically damped (in 0.5 its effective damping ratio was about 0.75).
- **`BackwardsPdController.ComputeForce` returned an acceleration** despite its name. If you applied it with `AddForce` on a body whose mass isn't 1, switch to `ComputeForce(dt, mass, ...)`; the response will then no longer depend on the mass.
- **The `BackwardsPdController` integral term is gone.** To hold position against gravity, pass `-Physics.gravity` as the feedforward acceleration.

### Changed

- `StablePdController` is now the single PD controller for every case: linear 1-axis and 3-axis, angular 1-axis (radians) and 3-axis (quaternion)
- `StablePdController` gains are now solved exactly for Unity's semi-implicit Euler integrator, so the closed-loop response has exactly the requested `frequency` (Hz) and `damping` (ratio) at any timestep, and is unconditionally stable for any gains. Previously `damping = 1` was under damped (true ratio 0.75), `frequency` was not in Hz, and the controller diverged above `frequency` ~12.6 at 50 Hz physics
- `StablePdController` now accepts a feedforward acceleration, used to track an accelerating target exactly or to cancel gravity (`targetAcceleration - Physics.gravity`)
- `StablePdController` Force / Torque methods now take the mass / inertia directly after `dt`: `ComputeForce(dt, mass, ...)`, `ComputeTorque(dt, momentOfInertia, ...)` and `ComputeTorque(dt, inertiaTensorRotation, inertiaTensor, ...)`
- `StablePdController` default `frequency` is now 2 (Hz), which is about as stiff as the previous default of 4. Sample scenes were retuned to match
- `PidController.ComputeForce1D` is renamed `Compute` and `ComputeTorque1D` is renamed `ComputeAngle`, since both return a throttle rather than a force or torque
- `PidController` `integralSaturation` now limits the integral term's contribution to the output (previously it limited the raw integral, so it only matched `outputMax` when `integralGain` = 1). The integral also no longer winds up while the output is saturated
- `PidController.ComputeAngle` uses wrap-safe angle differences for the error and both derivative terms, for any input angle range
- The package now requires Unity 6 (6000.3; previously declared as 2019.1), and declares its dependencies on the Physics and Physics 2D modules
- Sample display names now state what each sample compares
- Sample scripts read state from the Rigidbody rather than the Transform, no longer produce a target-velocity spike on their first physics step, and the 2D / 3D rotation samples compute the target angular velocity correctly (wrap-safe in 2D; in rad/s about the correct axis in 3D)

### Added

- `PropulsionMath`: shared helpers for wrapping angles, robust quaternion to rotation-vector conversion, angular velocity from successive rotations, and converting angular acceleration to world-space torque with the inertia tensor
- `StablePdController.GetDiscreteGains(dt, ...)` exposes the gains actually applied at a timestep
- `StablePdController` and `PidController` constructors, and `Frequency` / `Damping` properties
- EditMode tests for the controllers, which drive them with Unity's integrator to verify stability, convergence, tracking, wrap handling and inertia handling
- Package documentation (`README.md`, including the samples' input-handling and render-pipeline requirements), third-party license notices, and real documentation and license URLs in `package.json` (previously placeholders)

### Fixed

- `PidController.Reset` now clears the stored integral and derivative history, not only the derivative initialization flag
- 3D torque conversion now composes the inertia frame as `rotation * inertiaTensorRotation`. The removed `BackwardsPdController` used the reverse order, which gave wrong torques for bodies whose principal axes are not aligned with the body
- Quaternion error no longer relies on `Quaternion.ToAngleAxis`, which can return NaN for slightly non-unit quaternions
- The 3D rotation sample's `StableTurret` now runs `StablePdRotationController`. Its prefab was wired to the same script as `BackwardsTurret`, so the target angular-velocity tracking never ran and the two turrets behaved identically
- The "Vehicle AI follower" sample can now be imported: `package.json` pointed at `Samples~/VehiclePdExample` instead of `Samples~/VehiclePdFollower`

### Removed

- `BackwardsPdController`: its control law (with a target velocity) is algebraically identical to the implicit stable PD, now `StablePdController`. Its unscaled integral term has been removed; use the feedforward acceleration for known forces such as gravity, or `PidController` for unknown ones
- `StablePdController2D`: its single-axis methods are now overloads on `StablePdController`
- `PidController.AngleDifference`: use `Mathf.DeltaAngle`
- The bundled *Stable Proportional-Derivative Controllers* paper PDF no longer ships in the package. It moved to the repository's `Documentation/` folder, and the package docs link to the paper instead

## [0.5.0] - 2024-10-18

### Added

- Added the new control algorithm `StablePdController` and `StablePdController2D`, with cited source material
- Added a `PidExample1d` Sample scene to compare and showcase 1-dimensional capabilities in 1 scene
- Added a `RotationController2d` Sample scene to compare and showcase the 2-dimensional rotational capabilities

### Changed

- Changed the names of several algorithm call methods for accuracy
- Placed all samples into their own namespace with associated assembly definition files

### Removed

- Removed the `PidRotationController` due to inherent vulnerabilities to instability and gimbal lock

## [0.3.2] - 2023-03-07

### Changed

- Modified the Pid Example scene to compare the PID controllers to the backwards PD controller

## [0.3.1] - 2023-03-06

### Changed

- Inputs for Backwards PD rotation controller reduced to physics values to remove dependence on Rigidbody. This allows the algorithm to be used with an ArticulationBody, or a custom solution

## [0.3.0] - 2023-03-03

### Added

- Backwards PD Controller for quaternion based rotation control from [digitalopus.ca/site/pd-controllers/](http://digitalopus.ca/site/pd-controllers/)