using System;

namespace Yohash.Propulsion.Core
{
  /// <summary>
  /// The memory a PID controller carries between steps: the stored integral and the derivative
  /// history. Keep one per controlled axis. The default value is a freshly reset controller.
  /// </summary>
  [Serializable]
  public struct PidState
  {
    public float ValueLast;
    public float ErrorLast;
    public float Integral;

    /// <summary>False until the first update, which has no history, so no derivative term.</summary>
    public bool Initialized;

    /// <summary>Clears the integral and derivative history.</summary>
    public void Reset()
    {
      this = default;
    }
  }
}
