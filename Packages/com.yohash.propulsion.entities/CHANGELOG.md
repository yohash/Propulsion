# Changelog

All notable changes to this package will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - Unreleased

The first release: stable PD control for Unity ECS physics bodies, built on `com.yohash.propulsion.core` 0.1.0.

### Added

- `PdPosition2D` and `PdRotation2D`: drive a 2D body's centre of mass towards a target position, and its angle towards a target angle (the short way round), with exactly the requested frequency and damping.
  - Outputs are applied as forces and torques, added to the engine's accumulator, so they combine with every other force on the body.
  - `TargetVelocity` / `TargetAngularVelocity` for moving targets, and `Feedforward` for known extra accelerations.
  - `CompensateGravity` cancels the body's gravity (world gravity × gravity scale) exactly.
- `PdPositionHistory2D` and `PdRotationHistory2D` (opt-in, enableable): derive the target velocities from the targets' movement each step, with a reset for teleports and snaps.
- `PdGainsCache`, embedded in each controller: gains are re-solved only when the settings, timestep or sub-step count change.
- `PdControl2D` and `PdTargetDerivation2D`: the engine-neutral, Burst-compatible functions the backends call.
- `PdBody2DAuthoring` and its baker: frequency and damping per controller, "Compensate gravity" and "Derive target velocity", with targets starting at the GameObject's pose.
- **ECSPhysics2D backend** (`yohash.propulsion.entities.physics2d`), compiled only when `com.ecsphysics2d.core` 0.3.0+ is installed:
  - `Propulsion2DSystemGroup`, scheduled after body and shape creation and before the simulation;
  - `PdTargetVelocity2DSystem`, `PdPosition2DSystem` and `PdRotation2DSystem`;
  - gains solved for ECSPhysics2D's `FixedDeltaTime` and each world's sub-step count;
  - angular velocity converted from Unity's degrees per second.
- Sample: **PD bodies 2D (ECSPhysics2D)**.
- EditMode tests, plus engine tests that measure frequency and damping in Box2D v3.
