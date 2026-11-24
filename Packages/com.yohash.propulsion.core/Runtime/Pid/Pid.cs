using Unity.Mathematics;

namespace Yohash.Propulsion.Core
{
  /// <summary>
  /// *******************************************************************
  /// Adapted from https://vazgriz.com/621/pid-controllers/
  /// *******************************************************************
  ///
  /// A single-axis PID controller returning a throttle in [OutputMin, OutputMax], for the caller
  /// to scale into a force or torque. It has two anti-windup measures: the integral's
  /// contribution is limited to +/- IntegralSaturation, and while the output is saturated the
  /// integral may not grow further into saturation.
  ///
  /// Use it where StablePd can't help: an integral term for unknown persistent disturbances, and
  /// a bounded output for actuators with limited strength.
  ///
  /// Burst-compatible: static functions over value types, no exceptions. An invalid timestep
  /// (dt &lt;= 0) returns 0 and leaves the state untouched.
  /// </summary>
  public static class Pid
  {
    /// <summary>Update for a linear value along one axis.</summary>
    public static float Compute(in PidSettings settings, ref PidState state, float dt, float current, float target)
    {
      if (!(dt > 0f)) { return 0f; }

      float error = target - current;
      float errorRate = (error - state.ErrorLast) / dt;
      float valueRate = (current - state.ValueLast) / dt;

      state.ErrorLast = error;
      state.ValueLast = current;

      return Evaluate(settings, ref state, dt, error, errorRate, valueRate);
    }

    /// <summary>
    /// Update for an angle around one axis, in radians. The error and both derivative terms take
    /// the shortest way around, for any input angle range.
    /// </summary>
    public static float ComputeAngle(in PidSettings settings, ref PidState state, float dt, float currentAngle, float targetAngle)
    {
      if (!(dt > 0f)) { return 0f; }

      float error = MotionMath.WrapAngle(targetAngle - currentAngle);
      float errorRate = MotionMath.WrapAngle(error - state.ErrorLast) / dt;
      float valueRate = MotionMath.WrapAngle(currentAngle - state.ValueLast) / dt;

      state.ErrorLast = error;
      state.ValueLast = currentAngle;

      return Evaluate(settings, ref state, dt, error, errorRate, valueRate);
    }

    private static float Evaluate(in PidSettings s, ref PidState state, float dt, float error, float errorRate, float valueRate)
    {
      float p = s.ProportionalGain * error;

      // the first update has no history, so it contributes no derivative
      float d = 0f;
      if (state.Initialized) {
        d = s.DerivativeGain * (s.DerivativeMeasurement == PidDerivativeMeasurement.Velocity ? -valueRate : errorRate);
      } else {
        state.Initialized = true;
      }

      // integral, with its contribution limited, and no further wind-up while saturated
      float saturation = math.max(0f, s.IntegralSaturation);
      float limit = s.IntegralGain != 0f ? saturation / math.abs(s.IntegralGain) : 0f;
      float candidate = math.clamp(state.Integral + error * dt, -limit, limit);

      float unclamped = p + s.IntegralGain * candidate + d;
      float growth = s.IntegralGain * (candidate - state.Integral);
      bool windingUp = (unclamped > s.OutputMax && growth > 0f) || (unclamped < s.OutputMin && growth < 0f);

      if (!windingUp) {
        state.Integral = candidate;
      }
      state.Integral = math.clamp(state.Integral, -limit, limit);

      return math.clamp(p + s.IntegralGain * state.Integral + d, s.OutputMin, s.OutputMax);
    }
  }
}
