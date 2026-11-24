# Changelog

All notable changes to this package will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - Unreleased

The first release: the Propulsion Toolkit's controllers as a Burst-compatible core over Unity.Mathematics, with no dependency on UnityEngine physics, Entities or any physics package.

### Added

- `StablePd`, with `StablePdSettings` (frequency in Hz, damping ratio) and `StablePdGains`: the stable PD controller of `com.yohash.propulsion` 0.6.0, as static functions.
  - `StablePd.Gains(settings, dt, subSteps)` solves the gains exactly for semi-implicit Euler split into `subSteps` sub-steps, with the force held constant across them. That covers sub-stepping engines such as Box2D v3 / `UnityEngine.LowLevelPhysics2D` (4 sub-steps by default) as well as single-step engines. The closed loop keeps exactly the requested frequency and damping, and stays unconditionally stable.
  - `Acceleration` for `float`, `float2` and `float3`; `AngularAcceleration` for a single-axis angle (wrap-safe) and for quaternions (world space).
  - Feedforward acceleration on every function, e.g. to cancel gravity exactly.
- `Pid`, with `PidSettings` and `PidState`: the PID controller of `com.yohash.propulsion` 0.6.0, with the memory between steps in its own struct. `ComputeAngle` works in radians.
- `MotionMath`: angle wrapping, quaternion ↔ rotation vector, angular velocity from successive rotations (3D and wrap-safe 1D), and angular acceleration → world torque with an inertia tensor.
- No exceptions anywhere. An invalid timestep or sub-step count gives zero gains (feedforward only), and `Pid` returns 0 with its state untouched.
- EditMode tests, including sub-step pole matching and stability sweeps, and a Burst test assembly (compiled only when `com.unity.burst` is installed) that runs every public function in a Burst-compiled job and checks it wasn't run as managed code.
