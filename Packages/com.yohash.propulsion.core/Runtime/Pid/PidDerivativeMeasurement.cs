namespace Yohash.Propulsion.Core
{
  /// <summary>Which signal the derivative term differentiates.</summary>
  public enum PidDerivativeMeasurement : byte
  {
    /// <summary>The measured value: no "kick" when the target jumps.</summary>
    Velocity,

    /// <summary>The error: also responds to target motion.</summary>
    ErrorRateOfChange,
  }
}
