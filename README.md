# Propulsion Toolkit

Physics-based controllers for moving and rotating Rigidbodies with forces and torques.

| Type | Use it for |
|---|---|
| `StablePdController` | Matching a target position / velocity, or rotation / angular velocity, in 1 or 3 axes. Parameterized physically (frequency in Hz, damping ratio), scaled by mass or inertia, and unconditionally stable at any timestep. Known forces such as gravity are cancelled exactly through its feedforward term. |
| `PidController` | A bounded, normalized throttle with an integral term: for actuators with limited strength, or unknown persistent disturbances. Hand tuned. |
| `PropulsionMath` | Helpers: angle wrapping, quaternion to rotation vector, angular velocity from successive rotations, angular acceleration to torque. |
| `VehicleMotor`, `VehiclePdController` | A simple physics car and an AI driver that follows another body's trajectory. |

Requires Unity 6 (6000.3) or newer. Everything lives in the `Yohash.Propulsion` namespace, except the vehicle types, which are in the global namespace.

## Installation

In the Package Manager, choose **+ > Add package from git URL...** and enter:

```
https://github.com/yohash/Propulsion.git#upm
```

The samples can then be imported from the package's **Samples** tab.

## StablePdController

Add a `StablePdController` field to a component, set its **Frequency** and **Damping** in the inspector, and call it from `FixedUpdate`. The controller is stateless, so one instance can drive any number of bodies.

### Follow a target position

```csharp
using UnityEngine;
using Yohash.Propulsion;

public class Follow : MonoBehaviour
{
  public Transform Target;
  public StablePdController Controller = new StablePdController(frequency: 2f, damping: 1f);

  private Rigidbody rb;
  private Vector3 lastTargetPosition;

  private void Awake()
  {
    rb = GetComponent<Rigidbody>();
    lastTargetPosition = Target.position;
  }

  private void FixedUpdate()
  {
    float dt = Time.fixedDeltaTime;
    var targetVelocity = (Target.position - lastTargetPosition) / dt;
    lastTargetPosition = Target.position;

    // cancel gravity exactly, so the body holds the target instead of hanging below it
    var feedforward = rb.useGravity ? -Physics.gravity : Vector3.zero;

    var force = Controller.ComputeForce(
      dt, rb.mass,
      rb.position, rb.linearVelocity,
      Target.position, targetVelocity,
      feedforward);

    rb.AddForce(force);
  }
}
```

### Point at a target rotation

```csharp
// in FixedUpdate
var targetRotation = Quaternion.LookRotation(Target.position - rb.position, Vector3.up);
var targetAngularVelocity = PropulsionMath.AngularVelocity(lastTargetRotation, targetRotation, dt);
lastTargetRotation = targetRotation;

var torque = Controller.ComputeTorque(
  dt, rb.inertiaTensorRotation, rb.inertiaTensor,
  rb.rotation, rb.angularVelocity,
  targetRotation, targetAngularVelocity);

rb.AddTorque(torque);  // world space: not AddRelativeTorque
```

### Choosing the gains

- **Frequency (Hz)** is the true natural frequency. With damping = 1, the error falls to about 5% after `0.75 / frequency` seconds (about 0.4 s at 2 Hz).
- **Damping** is the true damping ratio: `1` is critically damped (fastest without overshoot), below `1` overshoots, above `1` is sluggish. `0` oscillates forever.
- **Any value is stable.** As frequency rises, the response tends towards reaching the target in a single physics step. For under-damped settings, the oscillation is capped at a quarter cycle per physics step (12.5 Hz at 50 Hz physics).
- **When the mass, inertia or forces are uncertain**, keep frequency below about `1 / (2π · fixedDeltaTime)` (8 Hz at 50 Hz physics). The stability guarantee assumes the mass and inertia you pass in are correct, and the tolerance shrinks as frequency rises. The class documentation has the figures.

### Units and conventions

- Methods named `…Acceleration` return accelerations: apply them with `ForceMode.Acceleration`, or use the `…Force` / `…Torque` variants, which scale by mass or inertia, with the default `ForceMode.Force`.
- Angles are in **radians** and angular velocities in **rad/s**. `Rigidbody2D` reports `rotation` and `angularVelocity` in degrees: multiply by `Mathf.Deg2Rad`.
- 3D angular quantities are world space, as `Rigidbody.angularVelocity` reports them.
- Read state from the Rigidbody (`rb.position`, `rb.rotation`, `rb.linearVelocity`, `rb.angularVelocity`) rather than the Transform, which may be interpolated.

### Feedforward

The last parameter of every method is a feedforward acceleration, added on top of the feedback:

```
feedforwardAcceleration = targetAcceleration - externalAcceleration
```

- Pass `-Physics.gravity` (or `-Physics2D.gravity * rb.gravityScale`) to cancel gravity exactly. Without it, the body settles below its target by `g / K`, about 8 cm at 2 Hz.
- Pass a target's acceleration to track it with no lag.
- Feedforward sits outside the feedback loop, so it never affects stability.
- It can only cancel forces you know about. For unknown disturbances, such as an uncertain mass, drag or contact forces, use `PidController`.

### Limitations

- `Rigidbody.maxAngularVelocity` defaults to 7 rad/s and silently caps fast rotation. Raise it if a stiff rotation controller seems slow.
- Drag and contacts are unmodelled disturbances: drag leaves a small lag when tracking a moving target.

## PidController

A classic PID with output clamping and anti-windup, adapted from [vazgriz](https://vazgriz.com/621/pid-controllers/). It returns a throttle in `[outputMin, outputMax]` for you to scale:

```csharp
float throttle = pid.Compute(Time.fixedDeltaTime, rb.position.y, target.position.y);
rb.AddForce(Vector3.up * throttle * power);

// angles are in degrees, and the error takes the shortest way around
float turn = pid.ComputeAngle(Time.fixedDeltaTime, currentAngleDegrees, targetAngleDegrees);
```

- `integralSaturation` caps the integral term's contribution to the output.
- Call `Reset()` when re-targeting or re-enabling, to clear the integral and derivative history.

## Samples

| Sample | Shows |
|---|---|
| 1-axis translation: PID vs Stable PD | PID and Stable PD side by side; Stable PD with and without target-velocity tracking |
| 3-axis translation: PID vs Stable PD | The same in 3D, under gravity (the PD samples use no feedforward, so they sag) |
| 2D rotation: PID vs Stable PD | Rotating a `Rigidbody2D` turret to face a target |
| 3D rotation: Stable PD | Rotating a turret to face a target while staying upright |
| Ground vehicle | `VehicleMotor` driven with WASD |
| Vehicle AI follower | `VehiclePdController` following a WASD-driven car |

## Tests

EditMode tests live in `Tests/Editor`. They step the controllers with Unity's integrator to check pole placement, stability, convergence, tracking, gravity feedforward, angle wrapping, inertia handling and PID anti-windup.

To run them from the package, add it to your project's `testables` in `Packages/manifest.json`:

```json
"testables": ["com.yohash.propulsion"]
```

Then open **Window > General > Test Runner > EditMode**.

## Background

- J. Tan, K. Liu, G. Turk, [*Stable Proportional-Derivative Controllers*](http://www.jie-tan.net/project/spd.pdf).
- digitalopus, [PD controllers](http://digitalopus.ca/site/pd-controllers/) and [UsefulUtils](https://github.com/Phong13/UsefulUtils).
- vazgriz, [PID controllers](https://vazgriz.com/621/pid-controllers/) and [PID_Controller](https://github.com/vazgriz/PID_Controller).

`StablePdController` belongs to the same family as Tan's SPD and the digitalopus "backwards" PD. Instead of discretizing the gains, it solves them so that the discrete closed-loop poles equal those of the requested continuous-time spring-damper, under Unity's semi-implicit Euler integrator.

## License

MIT. See [LICENSE.md](LICENSE.md) and [Third Party Notices.md](Third%20Party%20Notices.md).
