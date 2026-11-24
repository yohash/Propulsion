using Unity.Mathematics;

namespace Yohash.Propulsion.Core
{
  /// <summary>
  /// *******************************************************************
  /// Stable ("implicit") PD control, as Burst-compatible static functions over Unity.Mathematics.
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
  ///     a = a_ff - K * e - D * de,     e = p - p_target,   de = v - v_target
  ///
  /// a_ff is a feedforward acceleration: the target's own acceleration, minus any known external
  /// acceleration such as gravity (pass -gravity to cancel it exactly). Feedforward sits outside
  /// the feedback loop, so it never affects stability.
  ///
  /// THE GAINS
  ///
  /// K and D are solved so that the closed loop, under the physics engine's integrator, has
  /// discrete poles exactly exp(s * dt), where s are the poles of the continuous spring-damper the
  /// settings describe. The response therefore has exactly the requested frequency and damping at
  /// any timestep, and is unconditionally stable for any frequency, any damping &gt; 0 and any
  /// timestep, as long as the mass passed in matches the body.
  ///
  /// The integrator is semi-implicit Euler (v += a dt; p += v dt), the scheme PhysX, Box2D,
  /// Unity Physics and Havok all use, optionally split into n equal sub-steps per physics step,
  /// with the acceleration (a force set before the step) held constant across them:
  ///
  ///     v1 = v0 + a dt,     p1 = p0 + v0 dt + c a dt^2,     c = (n + 1) / (2 n)
  ///
  /// c = 1 for a single step. Box2D v3, and so UnityEngine.LowLevelPhysics2D, sub-steps by
  /// default (4 sub-steps: c = 5/8). Pass the engine's sub-step count to Gains: gains solved for
  /// the wrong count still settle at most settings, but lose the exact tuning and, at low damping,
  /// can go unstable.
  ///
  /// If the output is instead applied as a velocity change at the start of the step (an
  /// impulse of m a dt), the sub-steps don't matter: use subSteps = 1.
  ///
  /// BURST
  ///
  /// Everything here is a static function over value types, with no managed allocations and no
  /// exceptions, so it can be called from Burst-compiled jobs and systems.
  /// </summary>
  public static class StablePd
  {
    /// <summary>
    /// Solves the per-unit-mass gains for one physics step of length dt, split into subSteps
    /// equal sub-steps. Returns zero gains (no feedback) if dt &lt;= 0 or subSteps &lt; 1.
    ///
    /// For damping &lt; 1, the damped oscillation is capped at a quarter cycle per step
    /// (1 / (4 dt): 12.5 Hz at 50 Hz physics), the fastest a physics step can represent.
    /// </summary>
    public static StablePdGains Gains(in StablePdSettings settings, float dt, int subSteps = 1)
    {
      if (!(dt > 0f) || subSteps < 1) { return default; }

      // Double precision: (1 - z1)(1 - z2) ~ (omega * dt)^2 cancels badly in float at low gains.
      double h = dt;
      double x = 2.0 * math.PI_DBL * math.max(0.0, settings.Frequency) * h;
      double zeta = math.max(0.0, settings.Damping);
      double decay = zeta * x;

      // sum = z1 + z2 and product = z1 * z2, for the target poles z = exp(s * dt)
      double sum;
      if (zeta <= 1.0) {
        // a complex pair, or a double real root at zeta = 1
        double theta = math.min(x * math.sqrt(1.0 - zeta * zeta), 0.5 * math.PI_DBL);
        sum = 2.0 * math.exp(-decay) * math.cos(theta);
      } else {
        // two real roots
        double r = x * math.sqrt(zeta * zeta - 1.0);
        sum = math.exp(r - decay) + math.exp(-r - decay);
      }
      double product = math.exp(-2.0 * decay);

      // With X = K dt^2 and Y = D dt, the step above has trace 2 - c X - Y and determinant
      // 1 - Y + (1 - c) X. Matching them to sum and product:
      double c = (subSteps + 1.0) / (2.0 * subSteps);
      double stiffnessDt2 = 1.0 - sum + product;
      double damperDt = (1.0 - product) + (1.0 - c) * stiffnessDt2;

      return new StablePdGains {
        Stiffness = (float)(stiffnessDt2 / (h * h)),
        Damper = (float)(damperDt / h),
        Dt = dt,
      };
    }

    // ******************************************************************
    //    LINEAR
    // ******************************************************************

    /// <summary>
    /// Acceleration bringing a body onto the target position / velocity along one axis.
    /// Multiply by mass for a force.
    /// </summary>
    public static float Acceleration(
      in StablePdGains gains,
      float position,
      float velocity,
      float targetPosition,
      float targetVelocity,
      float feedforwardAcceleration = 0f
    )
    {
      return feedforwardAcceleration
        - gains.Stiffness * (position - targetPosition)
        - gains.Damper * (velocity - targetVelocity);
    }

    /// <summary>
    /// Acceleration bringing a body onto the target position / velocity in 2D.
    /// Multiply by mass for a force.
    /// </summary>
    public static float2 Acceleration(
      in StablePdGains gains,
      float2 position,
      float2 velocity,
      float2 targetPosition,
      float2 targetVelocity,
      float2 feedforwardAcceleration = default
    )
    {
      return feedforwardAcceleration
        - gains.Stiffness * (position - targetPosition)
        - gains.Damper * (velocity - targetVelocity);
    }

    /// <summary>
    /// Acceleration bringing a body onto the target position / velocity in 3D.
    /// Multiply by mass for a force.
    /// </summary>
    public static float3 Acceleration(
      in StablePdGains gains,
      float3 position,
      float3 velocity,
      float3 targetPosition,
      float3 targetVelocity,
      float3 feedforwardAcceleration = default
    )
    {
      return feedforwardAcceleration
        - gains.Stiffness * (position - targetPosition)
        - gains.Damper * (velocity - targetVelocity);
    }

    // ******************************************************************
    //    ANGULAR
    // ******************************************************************

    /// <summary>
    /// Angular acceleration (rad/s^2) bringing a body onto the target angle / angular velocity
    /// around a single axis (a 2D body, or a hinge). Angles in radians, angular velocities in
    /// rad/s. The error always takes the shortest way around. Multiply by the moment of inertia
    /// for a torque.
    /// </summary>
    public static float AngularAcceleration(
      in StablePdGains gains,
      float angle,
      float angularVelocity,
      float targetAngle,
      float targetAngularVelocity,
      float feedforwardAngularAcceleration = 0f
    )
    {
      float dt = gains.Dt;
      float errorRate = angularVelocity - targetAngularVelocity;
      float predictedError = MotionMath.WrapAngle(
        (angle + dt * angularVelocity) - (targetAngle + dt * targetAngularVelocity));

      // the law -K e - D de, written with the wrapped error one step ahead: e + dt de
      return feedforwardAngularAcceleration
        - gains.Stiffness * predictedError
        - (gains.Damper - gains.Stiffness * dt) * errorRate;
    }

    /// <summary>
    /// World-space angular acceleration (rad/s^2) bringing a body onto the target rotation /
    /// angular velocity. Inputs are world space; angular velocities in rad/s. Always takes the
    /// shortest way around. Convert to a torque with MotionMath.AngularAccelerationToTorque.
    ///
    /// To track a moving target rotation, derive its angular velocity with
    /// MotionMath.AngularVelocity(previousTargetRotation, targetRotation, dt).
    /// </summary>
    public static float3 AngularAcceleration(
      in StablePdGains gains,
      quaternion rotation,
      float3 angularVelocity,
      quaternion targetRotation,
      float3 targetAngularVelocity,
      float3 feedforwardAngularAcceleration = default
    )
    {
      float dt = gains.Dt;

      // Predict both rotations one step ahead (with no control input) on the rotation manifold,
      // rather than linearly, so large errors and fast spins stay accurate.
      quaternion current1 = MotionMath.Integrate(rotation, angularVelocity, dt);
      quaternion target1 = MotionMath.Integrate(targetRotation, targetAngularVelocity, dt);

      float3 predictedError = MotionMath.ToRotationVector(math.mul(current1, math.inverse(target1)));
      float3 errorRate = angularVelocity - targetAngularVelocity;

      return feedforwardAngularAcceleration
        - gains.Stiffness * predictedError
        - (gains.Damper - gains.Stiffness * dt) * errorRate;
    }
  }
}
