using System;
using Yohash.Propulsion.Core;

namespace Yohash.Propulsion.Entities
{
  /// <summary>
  /// The gains a controller last solved, and the inputs it solved them for.
  ///
  /// Every controller component embeds one. It is internal bookkeeping, never authored: the
  /// control systems call <see cref="Resolve"/> each step, which re-solves the gains only when
  /// the settings, the timestep or the sub-step count differ from last time. In practice that is
  /// once, on the body's first step, and again only if the controller is retuned at runtime or
  /// the physics configuration changes.
  ///
  /// A default (all-zero) cache has <c>Gains.Dt == 0</c>, so it always solves on first use.
  /// </summary>
  [Serializable]
  public struct PdGainsCache
  {
    /// <summary>The settings <see cref="Gains"/> was solved for.</summary>
    public StablePdSettings SolvedSettings;

    /// <summary>The sub-step count <see cref="Gains"/> was solved for.</summary>
    public int SolvedSubSteps;

    /// <summary>The solved gains. They carry the timestep they were solved for.</summary>
    public StablePdGains Gains;

    /// <summary>
    /// Returns gains for these inputs, re-solving them only if an input changed since the last
    /// call. Invalid input (dt &lt;= 0, subSteps &lt; 1) gives zero gains, which apply no
    /// feedback, exactly as <see cref="StablePd.Gains"/> does.
    /// </summary>
    public StablePdGains Resolve(in StablePdSettings settings, float dt, int subSteps)
    {
      if (Gains.Dt != dt
          || SolvedSubSteps != subSteps
          || SolvedSettings.Frequency != settings.Frequency
          || SolvedSettings.Damping != settings.Damping) {
        Gains = StablePd.Gains(settings, dt, subSteps);
        SolvedSettings = settings;
        SolvedSubSteps = subSteps;
      }
      return Gains;
    }
  }
}
