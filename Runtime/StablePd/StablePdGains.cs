using System;

namespace Yohash.Propulsion.Core
{
  /// <summary>
  /// The per-unit-mass gains a stable PD controller applies over one physics step, solved by
  /// StablePd.Gains. Compute them once per step (they depend only on the settings, the timestep
  /// and the sub-step count), then reuse them for every body that shares those settings.
  ///
  /// A default (all-zero) value applies no feedback: the controller then returns only its
  /// feedforward term. StablePd.Gains returns that for an invalid timestep or sub-step count.
  /// </summary>
  [Serializable]
  public struct StablePdGains
  {
    /// <summary>Stiffness K (1/s^2): acceleration per unit of position error.</summary>
    public float Stiffness;

    /// <summary>Damper D (1/s): acceleration per unit of velocity error.</summary>
    public float Damper;

    /// <summary>The timestep (s) the gains were solved for.</summary>
    public float Dt;

    /// <summary>False for the zero gains StablePd.Gains returns for invalid input.</summary>
    public bool IsValid => Dt > 0f;
  }
}
