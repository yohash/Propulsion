# Propulsion Core

The Burst-compatible math core of the Propulsion Toolkit: physically exact controllers for moving and rotating bodies, as plain static functions over [Unity.Mathematics](https://docs.unity3d.com/Packages/com.unity.mathematics@latest).

| Type | Use it for |
|---|---|
| `StablePd`, `StablePdSettings`, `StablePdGains` | Matching a target position / velocity (1, 2 or 3 axes) or rotation / angular velocity (1 axis or quaternion). Parameterized physically (frequency in Hz, damping ratio), solved exactly for the physics engine's integrator, including engines that sub-step, and unconditionally stable. |
| `Pid`, `PidSettings`, `PidState` | A bounded throttle with an integral term, for actuators with limited strength or unknown persistent disturbances. Hand tuned. |
| `MotionMath` | Angle wrapping, quaternion ↔ rotation vector, angular velocity from successive rotations, angular acceleration → torque with an inertia tensor. |

- **Burst-compatible:** everything is static functions over value types, with no managed allocations and no exceptions. Call it from Burst jobs and `ISystem`s.
- **No engine dependencies:** the assembly has no UnityEngine references (`noEngineReferences`), and the package depends only on `com.unity.mathematics`. It knows nothing about PhysX, Unity Physics, Box2D or Entities. You read your bodies' state, call the core, and apply the result.
- Everything is in the `Yohash.Propulsion.Core` namespace.

Requires Unity 6 (6000.3) or newer.

## Installation

In the Package Manager, choose **+ > Add package from git URL...** and enter:

```
https://github.com/yohash/Propulsion.git#upm-core
```

Pin a release with its tag instead, e.g. `#core-v0.1.0`. Unity installs the `com.unity.mathematics` dependency automatically.

## StablePd

Solve the gains **once per physics step**, since they depend only on the settings, the timestep and the sub-step count. Then reuse them for every body that shares those settings:

```csharp
using Unity.Mathematics;
using Yohash.Propulsion.Core;

var settings = new StablePdSettings(frequency: 2f, damping: 1f);
StablePdGains gains = StablePd.Gains(settings, dt, subSteps: 1);

// position: an acceleration; multiply by mass for a force.
// Pass -gravity as the feedforward to hold position against it exactly.
float3 acceleration = StablePd.Acceleration(gains, position, velocity, targetPosition, targetVelocity, -gravity);
float3 force = mass * acceleration;

// rotation: a world-space angular acceleration; convert to torque with the inertia tensor
float3 angularAcceleration = StablePd.AngularAcceleration(gains, rotation, angularVelocity, targetRotation, targetAngularVelocity);
float3 torque = MotionMath.AngularAccelerationToTorque(angularAcceleration, rotation, inertiaTensorRotation, inertiaTensor);
```

The 1-axis and 2D overloads work the same way: `float` or `float2` positions, and a `float` angle (radians) for single-axis rotation. Multiply single-axis angular accelerations by the moment of inertia for a torque.

### Choosing `subSteps`

`StablePd.Gains` solves K and D so that the closed loop has exactly the requested frequency and damping **under the engine's integrator**: semi-implicit Euler, optionally split into sub-steps, with your force held constant across them. Pass the number of sub-steps the engine runs per step:

| How the output reaches the body | `subSteps` |
|---|---|
| Force or torque, on an engine that integrates once per step: PhysX `Rigidbody`, or classic `Rigidbody2D` (its measured response in `com.yohash.propulsion`'s 2D sample matches a single step) | `1` |
| Force or torque on a sub-stepping engine, e.g. **ECSPhysics2D** / `UnityEngine.LowLevelPhysics2D` (Box2D v3) | its sub-step count, e.g. `PhysicsWorldConfig.SimulationSubSteps` (**4** by default) |
| A velocity change or impulse at the start of the step (`v += a * dt`, or `m * a * dt`) | `1`, whatever the engine |

Gains solved for the wrong sub-step count still settle at most settings, but lose the exact tuning. At low damping they can go unstable: for example, 1-step gains at 10 Hz and damping 0.1, run under 4 sub-steps.

### Choosing the settings

- **Frequency (Hz)** is the true natural frequency. With damping = 1, the error to a stationary target falls to about 5% after `0.75 / frequency` seconds (about 0.4 s at 2 Hz).
- **Damping** is the true damping ratio: `1` is critically damped (fastest without overshoot), below `1` overshoots, above `1` is sluggish. `0` oscillates forever.
- **These figures are for a stationary target.** When you pass a target velocity, the controller follows it. So a target that moves fast and stops abruptly is overshot; pass its deceleration as feedforward if that matters.
- **Any value is stable,** given the right sub-step count and a mass/inertia that matches the body. As frequency rises, the response tends towards reaching the target in a single step. For damping < 1, the oscillation is capped at a quarter cycle per step.
- **When mass, inertia or forces are uncertain,** keep frequency below about `1 / (2π · dt)` (8 Hz at 50 Hz physics). The stability margin shrinks as frequency rises.

### Units and conventions

- Angles are in **radians**, angular velocities in **rad/s**, and 3D angular quantities are **world space**.
- **Unity's 2D physics APIs report angular velocity in degrees per second:** classic `Rigidbody2D.angularVelocity` and `UnityEngine.LowLevelPhysics2D.PhysicsBody.angularVelocity` (and so ECSPhysics2D's bodies). Convert with `math.radians(...)` before passing it in. Their angles (`rotation.angle`) are radians and their torques act in radians, so leave those unconverted. Unconverted, the derivative term is about 57× too strong, and the body won't settle.
- Quaternions should be unit length, as Unity.Mathematics' `rotate` / `mul` assume.
- To track a moving target rotation, derive its angular velocity with `MotionMath.AngularVelocity(previousTargetRotation, targetRotation, dt)`. For a single-axis angle, use the `float` overload, which is wrap-safe.

### Feedforward

The last parameter of each `StablePd` function is a feedforward acceleration, added on top of the feedback:

```
feedforwardAcceleration = targetAcceleration - externalAcceleration
```

Pass `-gravity` to cancel gravity exactly (without it, the body settles `g / K` below its target), or a target's acceleration to track it with no lag. Feedforward sits outside the feedback loop, so it never affects stability. It can only cancel forces you know about; for unknown ones, use `Pid`.

### Invalid input

There are no exceptions. `StablePd.Gains` returns zero gains (`IsValid == false`) for `dt <= 0` or `subSteps < 1`, and zero gains apply no feedback: the functions return only their feedforward term.

## Pid

A classic PID with output clamping and anti-windup, adapted from [vazgriz](https://vazgriz.com/621/pid-controllers/). The tuning (`PidSettings`) and the memory between steps (`PidState`, one per controlled axis) are separate structs, so the state can live wherever your data does, such as an ECS component:

```csharp
var pidSettings = new PidSettings(proportionalGain: 2f, integralGain: 0.5f, derivativeGain: 1f);
var pidState = new PidState();

float throttle = Pid.Compute(pidSettings, ref pidState, dt, current, target);              // in [OutputMin, OutputMax]
float turn = Pid.ComputeAngle(pidSettings, ref pidState, dt, currentAngle, targetAngle);   // radians, shortest way around
pidState.Reset();                                                                          // after a teleport or re-target
```

- `IntegralSaturation` caps the integral term's contribution to the output.
- An invalid `dt` returns 0 and leaves the state untouched.

## Tests

The package's EditMode tests drive the controllers with the integrator the engines use, including sub-stepping, to check pole placement, stability, convergence, tracking, gravity feedforward, angle wrapping, inertia handling and PID anti-windup. A separate test assembly, which compiles only when `com.unity.burst` is installed, runs every public function in a Burst-compiled job. It checks that the job really was Burst-compiled, and that its results match managed code.

To run them from the package, add it to your project's `testables` in `Packages/manifest.json`:

```json
"testables": ["com.yohash.propulsion.core"]
```

## Relationship to `com.yohash.propulsion`

`com.yohash.propulsion` is the MonoBehaviour package: components, vehicles and samples. It doesn't depend on this package yet, and for now it carries its own copy of the same controllers. A later release will rebuild it on top of this core, so there is one implementation.

## Background

- J. Tan, K. Liu, G. Turk, [*Stable Proportional-Derivative Controllers*](http://www.jie-tan.net/project/spd.pdf).
- digitalopus, [PD controllers](http://digitalopus.ca/site/pd-controllers/) and [UsefulUtils](https://github.com/Phong13/UsefulUtils).
- vazgriz, [PID controllers](https://vazgriz.com/621/pid-controllers/) and [PID_Controller](https://github.com/vazgriz/PID_Controller).

## License

MIT. See [LICENSE.md](LICENSE.md) and [Third Party Notices.md](Third%20Party%20Notices.md).
