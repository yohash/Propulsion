using System;

namespace Yohash.Propulsion.Core
{
  /// <summary>
  /// The tuning of a PID controller. The gains are hand tuned: they aren't normalized by mass or
  /// timestep, so large values can make the loop oscillate or diverge.
  /// </summary>
  [Serializable]
  public struct PidSettings
  {
    public float ProportionalGain;
    public float IntegralGain;
    public float DerivativeGain;

    /// <summary>The output is clamped to [OutputMin, OutputMax].</summary>
    public float OutputMin;
    public float OutputMax;

    /// <summary>
    /// The largest contribution the integral term may make to the output. Setting it equal to
    /// OutputMax lets the integral alone saturate the output.
    /// </summary>
    public float IntegralSaturation;

    public PidDerivativeMeasurement DerivativeMeasurement;

    public PidSettings(
      float proportionalGain,
      float integralGain,
      float derivativeGain,
      float outputMin = -1f,
      float outputMax = 1f,
      float integralSaturation = 1f,
      PidDerivativeMeasurement derivativeMeasurement = PidDerivativeMeasurement.Velocity
    )
    {
      ProportionalGain = proportionalGain;
      IntegralGain = integralGain;
      DerivativeGain = derivativeGain;
      OutputMin = outputMin;
      OutputMax = outputMax;
      IntegralSaturation = integralSaturation;
      DerivativeMeasurement = derivativeMeasurement;
    }

    /// <summary>Unit gains, output in [-1, 1], integral saturation 1, derivative on measurement.</summary>
    public static PidSettings Default => new PidSettings(1f, 1f, 1f);
  }
}
