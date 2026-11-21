using System;
using UnityEngine;

namespace Yohash.Propulsion
{
  /// <summary>
  /// *******************************************************************
  /// Implicit ("stable") proportional-derivative controller.
  ///
  /// Based on:
  ///   Jie Tan, Karen Liu, Greg Turk, "Stable Proportional-Derivative Controllers"
  ///   http://www.jie-tan.net/project/spd.pdf
  /// and the "backwards" PD controller from
  ///   http://digitalopus.ca/site/pd-controllers/
  ///   https://github.com/Phong13/UsefulUtils
  /// *******************************************************************
  ///
  /// THE CONTROL LAW
  ///
  /// Unity's physics (PhysX and Box2D) integrates with semi-implicit Euler:
  ///     v1 = v0 + a * dt
  ///     p1 = p0 + v1 * dt
  ///
  /// The controller works on the error e = p - pTarget and its rate de = v - vTarget, and
  /// returns
  ///
  ///     a = aff - K * (e + dt * de) - (D - K * dt) * de        ( = aff - K * e - D * de )
  ///
  /// where (e + dt * de) is the error predicted one step ahead with no control input, and aff
  /// is a feedforward acceleration: the target's own acceleration, minus any known external
  /// acceleration such as gravity.
  ///
  /// This is the same family as Tan's SPD and the digitalopus "backwards" PD: PD feedback on the
  /// state one step ahead, solved so that it is consistent with the integrator. Those choose
  ///     K = kp / (1 + kd*dt + kp*dt^2),   D = (kd + kp*dt) / (1 + kd*dt + kp*dt^2)
  /// (a backward-Euler discretisation), which is stable but responds noticeably slower than the
  /// requested gains (about 20-30% at 2 Hz, 50 Hz physics).
  ///
  /// Here K and D are instead solved exactly, so the closed-loop poles of the *discrete*
  /// system are exactly exp(s * dt), where s are the poles of the continuous-time
  /// mass-spring-damper the frequency and damping describe:
  ///
  ///     D * dt   = 1 - (z1 * z2)
  ///     K * dt^2 = (1 - z1) * (1 - z2)
  ///     z1, z2   = exp(s1 * dt), exp(s2 * dt)
  ///
  /// The response therefore oscillates and decays at exactly the requested rate, at any
  /// timestep. (Its initial transient differs from continuous time only by O(omega * dt),
  /// because the acceleration is held constant across each physics step.)
  ///
  /// Since |z| = exp(-damping * omega * dt) < 1, the loop is unconditionally stable for any
  /// frequency, any damping > 0 and any timestep. (Damping = 0 is an undamped oscillator,
  /// exactly as requested.) With damping >= 1, raising the frequency tends to "deadbeat"
  /// control: the body reaches the target in a single physics step, without overshoot.
  ///
  /// GAINS
  ///
  ///     omega = 2 * PI * frequency
  ///     kp    = omega^2              (continuous-time equivalent, see Kp)
  ///     kd    = 2 * damping * omega  (continuous-time equivalent, see Kd)
  ///
  /// So `frequency` is the true undamped natural frequency in Hz, and `damping` is the true
  /// damping ratio: 1 = critically damped, < 1 under damped (overshoots), > 1 over damped.
  /// With damping = 1, the error falls to ~5% after about 0.75 / frequency seconds.
  ///
  /// A physics step cannot represent an oscillation faster than a few steps per cycle, so for
  /// damping < 1 the damped oscillation frequency is capped at a quarter cycle per step
  /// (1 / (4 * dt): 12.5 Hz at the default 50 Hz physics rate).
  ///
  /// ROBUSTNESS
  ///
  /// Stability assumes the mass / inertia passed in matches the body. The tolerance shrinks
  /// as frequency approaches the physics rate. At 50 Hz physics with damping = 1, the loop
  /// stays stable if the applied force is too strong by up to about 4.8x at 2 Hz, 2.8x at
  /// 4 Hz, 1.9x at 8 Hz and 1.3x in the deadbeat limit. (Too weak is always stable.)
  /// Keep frequency comfortably below 1 / (2 * PI * dt) (8 Hz at 50 Hz physics) when the mass,
  /// inertia or applied forces are uncertain.
  ///
  /// PRACTICAL NOTES
  ///
  ///  - All outputs are accelerations unless the method says Force / Torque. Force and Torque
  ///    methods scale by mass / inertia so they can be applied with ForceMode.Force.
  ///    Accelerations can be applied directly with ForceMode.Acceleration.
  ///  - The controller is stateless: one instance can drive any number of bodies.
  ///  - Gravity, drag and contacts are disturbances the controller does not know about. Cancel
  ///    gravity exactly by passing it in the feedforward term:
  ///        feedforwardAcceleration = targetAcceleration - Physics.gravity
  ///    Drag (rigidbody.linearDamping / angularDamping) leaves a small tracking lag on moving targets.
  ///  - Rigidbody.maxAngularVelocity defaults to 7 rad/s, and it silently limits fast rotation.
  ///  - Read state from the Rigidbody (rb.position, rb.rotation, rb.linearVelocity,
  ///    rb.angularVelocity) rather than the Transform, which may be interpolated.
  /// </summary>
  [Serializable]
  public class StablePdController
  {
    [Tooltip("Undamped natural frequency, in Hz. Higher values converge faster.\n" +
      "With damping = 1 the error falls to ~5% after about 0.75 / frequency seconds.")]
    [SerializeField, Min(0f)] private float frequency = 2f;

    [Tooltip("Damping ratio.\n" +
      "  = 1  critically damped (fastest response without overshoot)\n" +
      "  < 1  under damped (overshoots / oscillates)\n" +
      "  > 1  over damped (sluggish)")]
    [SerializeField, Min(0f)] private float damping = 1f;

    public StablePdController() { }

    public StablePdController(float frequency, float damping)
    {
      Frequency = frequency;
      Damping = damping;
    }

    /// <summary>Undamped natural frequency, in Hz.</summary>
    public float Frequency {
      get { return frequency; }
      set { frequency = Mathf.Max(0f, value); }
    }

    /// <summary>Damping ratio (1 = critically damped).</summary>
    public float Damping {
      get { return damping; }
      set { damping = Mathf.Max(0f, value); }
    }

    /// <summary>Continuous-time proportional gain, per unit mass (1/s^2). See GetDiscreteGains.</summary>
    public float Kp {
      get {
        float omega = AngularFrequency;
        return omega * omega;
      }
    }

    /// <summary>Continuous-time derivative gain, per unit mass (1/s). See GetDiscreteGains.</summary>
    public float Kd {
      get { return 2f * Mathf.Max(0f, damping) * AngularFrequency; }
    }

    private float AngularFrequency {
      get { return 2f * Mathf.PI * Mathf.Max(0f, frequency); }
    }

    // ******************************************************************
    //    LINEAR, 1 AXIS
    // ******************************************************************

    /// <summary>
    /// Acceleration needed to bring a body onto the target position / velocity along one axis.
    /// </summary>
    public float ComputeAcceleration(
      float dt,
      float currentPosition,
      float currentVelocity,
      float targetPosition,
      float targetVelocity,
      float feedforwardAcceleration = 0f
    )
    {
      float error = currentPosition - targetPosition;
      float errorRate = currentVelocity - targetVelocity;
      return feedforwardAcceleration + Solve(dt, error + dt * errorRate, errorRate);
    }

    /// <summary>
    /// Force needed to bring a body of the given mass onto the target position / velocity
    /// along one axis.
    /// </summary>
    public float ComputeForce(
      float dt,
      float mass,
      float currentPosition,
      float currentVelocity,
      float targetPosition,
      float targetVelocity,
      float feedforwardAcceleration = 0f
    )
    {
      return mass * ComputeAcceleration(
        dt,
        currentPosition,
        currentVelocity,
        targetPosition,
        targetVelocity,
        feedforwardAcceleration
      );
    }

    // ******************************************************************
    //    LINEAR, 3 AXIS
    // ******************************************************************

    /// <summary>
    /// Acceleration needed to bring a body onto the target position / velocity.
    /// </summary>
    public Vector3 ComputeAcceleration(
      float dt,
      Vector3 currentPosition,
      Vector3 currentVelocity,
      Vector3 targetPosition,
      Vector3 targetVelocity,
      Vector3 feedforwardAcceleration = default
    )
    {
      var error = currentPosition - targetPosition;
      var errorRate = currentVelocity - targetVelocity;
      return feedforwardAcceleration + Solve(dt, error + dt * errorRate, errorRate);
    }

    /// <summary>
    /// Force needed to bring a body of the given mass onto the target position / velocity.
    /// Apply with rigidbody.AddForce(force).
    /// </summary>
    public Vector3 ComputeForce(
      float dt,
      float mass,
      Vector3 currentPosition,
      Vector3 currentVelocity,
      Vector3 targetPosition,
      Vector3 targetVelocity,
      Vector3 feedforwardAcceleration = default
    )
    {
      return mass * ComputeAcceleration(
        dt,
        currentPosition,
        currentVelocity,
        targetPosition,
        targetVelocity,
        feedforwardAcceleration
      );
    }

    // ******************************************************************
    //    ANGULAR, 1 AXIS (e.g. Rigidbody2D, or a hinge)
    // ******************************************************************

    /// <summary>
    /// Angular acceleration (rad/s^2) needed to bring a body onto the target angle / angular
    /// velocity around a single axis. Angles are in radians, angular velocities in rad/s.
    /// The angle error always takes the shortest way around.
    ///
    /// Note: Rigidbody2D reports rotation and angularVelocity in degrees; convert with
    /// Mathf.Deg2Rad before calling.
    /// </summary>
    public float ComputeAngularAcceleration(
      float dt,
      float currentAngle,
      float currentAngularVelocity,
      float targetAngle,
      float targetAngularVelocity,
      float feedforwardAngularAcceleration = 0f
    )
    {
      float errorRate = currentAngularVelocity - targetAngularVelocity;
      float predictedError = PropulsionMath.WrapAngle(
        (currentAngle + dt * currentAngularVelocity) - (targetAngle + dt * targetAngularVelocity)
      );
      return feedforwardAngularAcceleration + Solve(dt, predictedError, errorRate);
    }

    /// <summary>
    /// Torque needed to bring a body with the given moment of inertia onto the target angle /
    /// angular velocity around a single axis. See ComputeAngularAcceleration for units.
    /// For a Rigidbody2D, momentOfInertia = rigidbody2d.inertia.
    /// </summary>
    public float ComputeTorque(
      float dt,
      float momentOfInertia,
      float currentAngle,
      float currentAngularVelocity,
      float targetAngle,
      float targetAngularVelocity,
      float feedforwardAngularAcceleration = 0f
    )
    {
      return momentOfInertia * ComputeAngularAcceleration(
        dt,
        currentAngle,
        currentAngularVelocity,
        targetAngle,
        targetAngularVelocity,
        feedforwardAngularAcceleration
      );
    }

    // ******************************************************************
    //    ANGULAR, 3 AXIS (quaternion)
    // ******************************************************************

    /// <summary>
    /// World-space angular acceleration (rad/s^2) needed to bring a body onto the target
    /// rotation / angular velocity. All inputs are world space; angular velocities in rad/s,
    /// as given by rigidbody.angularVelocity. Always takes the shortest way around.
    ///
    /// To track a moving target rotation, derive its angular velocity with
    /// PropulsionMath.AngularVelocity(previousTargetRotation, targetRotation, dt).
    /// </summary>
    public Vector3 ComputeAngularAcceleration(
      float dt,
      Quaternion currentRotation,
      Vector3 currentAngularVelocity,
      Quaternion targetRotation,
      Vector3 targetAngularVelocity,
      Vector3 feedforwardAngularAcceleration = default
    )
    {
      ValidateTimestep(dt);

      // Predict both rotations one step ahead (with no control input), on the rotation
      // manifold rather than linearly, so large errors and fast spins stay accurate.
      var current1 = PropulsionMath.Integrate(currentRotation, currentAngularVelocity, dt);
      var target1 = PropulsionMath.Integrate(targetRotation, targetAngularVelocity, dt);

      var predictedError = PropulsionMath.ToRotationVector(current1 * Quaternion.Inverse(target1));
      var errorRate = currentAngularVelocity - targetAngularVelocity;

      return feedforwardAngularAcceleration + Solve(dt, predictedError, errorRate);
    }

    /// <summary>
    /// World-space torque needed to bring a rigid body onto the target rotation / angular
    /// velocity. Apply with rigidbody.AddTorque(torque) (world space).
    ///
    /// The inertia inputs come straight from the Rigidbody / ArticulationBody:
    ///     inertiaTensorRotation = rigidbody.inertiaTensorRotation
    ///     inertiaTensor         = rigidbody.inertiaTensor
    /// </summary>
    public Vector3 ComputeTorque(
      float dt,
      Quaternion inertiaTensorRotation,
      Vector3 inertiaTensor,
      Quaternion currentRotation,
      Vector3 currentAngularVelocity,
      Quaternion targetRotation,
      Vector3 targetAngularVelocity,
      Vector3 feedforwardAngularAcceleration = default
    )
    {
      var angularAcceleration = ComputeAngularAcceleration(
        dt,
        currentRotation,
        currentAngularVelocity,
        targetRotation,
        targetAngularVelocity,
        feedforwardAngularAcceleration
      );

      return PropulsionMath.AngularAccelerationToTorque(
        angularAcceleration,
        currentRotation,
        inertiaTensorRotation,
        inertiaTensor
      );
    }

    // ******************************************************************
    //    IMPLICIT SOLVE
    // ******************************************************************

    /// <summary>
    /// The per-unit-mass gains actually applied at this timestep, a = -stiffness * e - damper * de.
    /// They tend to (Kp, Kd) as dt tends to zero. `stiffness` also sets the static stiffness:
    /// an unmodelled constant acceleration g leaves a steady offset of g / stiffness.
    /// </summary>
    public void GetDiscreteGains(float dt, out float stiffness, out float damper)
    {
      ValidateTimestep(dt);

      // Double precision: (1 - z1)(1 - z2) ~ (omega * dt)^2 cancels badly in float at low gains
      double h = dt;
      double x = 2.0 * Math.PI * Math.Max(0f, frequency) * h;
      double zeta = Math.Max(0f, damping);
      double decay = zeta * x;

      // z1 + z2 and z1 * z2 for the poles exp(s * dt)
      double sum;
      if (zeta <= 1.0) {
        // complex pair (or double real root at zeta = 1)
        double theta = Math.Min(x * Math.Sqrt(1.0 - zeta * zeta), 0.5 * Math.PI);
        sum = 2.0 * Math.Exp(-decay) * Math.Cos(theta);
      } else {
        // two real roots
        double r = x * Math.Sqrt(zeta * zeta - 1.0);
        sum = Math.Exp(r - decay) + Math.Exp(-r - decay);
      }
      double product = Math.Exp(-2.0 * decay);

      stiffness = (float)((1.0 - sum + product) / (h * h));
      damper = (float)((1.0 - product) / h);
    }

    // Given the error one step ahead with no control input (e + dt * de) and the current error
    // rate (de), returns the corrective acceleration.
    private float Solve(float dt, float predictedError, float errorRate)
    {
      GetDiscreteGains(dt, out float k, out float d);
      return -(k * predictedError + (d - k * dt) * errorRate);
    }

    private Vector3 Solve(float dt, Vector3 predictedError, Vector3 errorRate)
    {
      GetDiscreteGains(dt, out float k, out float d);
      return -(k * predictedError + (d - k * dt) * errorRate);
    }

    private static void ValidateTimestep(float dt)
    {
      if (!(dt > 0f)) {
        throw new ArgumentOutOfRangeException(nameof(dt), dt, "Timestep must be positive.");
      }
    }
  }
}
