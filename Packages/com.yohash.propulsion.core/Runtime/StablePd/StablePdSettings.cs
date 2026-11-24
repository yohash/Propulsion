using System;
using Unity.Mathematics;

namespace Yohash.Propulsion.Core
{
  /// <summary>
  /// The tuning of a stable PD controller: a natural frequency and a damping ratio.
  ///
  ///     omega = 2 * PI * Frequency,   kp = omega^2,   kd = 2 * Damping * omega
  ///
  /// Frequency is the true undamped natural frequency in Hz, and Damping the true damping ratio
  /// (1 = critically damped, &lt; 1 overshoots, &gt; 1 sluggish). With Damping = 1, the error to a
  /// stationary target falls to ~5% after about 0.75 / Frequency seconds.
  /// </summary>
  [Serializable]
  public struct StablePdSettings
  {
    /// <summary>Undamped natural frequency, in Hz. Negative values are treated as 0.</summary>
    public float Frequency;

    /// <summary>Damping ratio (1 = critically damped). Negative values are treated as 0.</summary>
    public float Damping;

    public StablePdSettings(float frequency, float damping)
    {
      Frequency = frequency;
      Damping = damping;
    }

    /// <summary>2 Hz, critically damped: the package's default tuning.</summary>
    public static StablePdSettings Default => new StablePdSettings(2f, 1f);

    /// <summary>Continuous-time proportional gain, per unit mass (1/s^2).</summary>
    public float Kp {
      get {
        float omega = 2f * math.PI * math.max(0f, Frequency);
        return omega * omega;
      }
    }

    /// <summary>Continuous-time derivative gain, per unit mass (1/s).</summary>
    public float Kd => 2f * math.max(0f, Damping) * 2f * math.PI * math.max(0f, Frequency);
  }
}
