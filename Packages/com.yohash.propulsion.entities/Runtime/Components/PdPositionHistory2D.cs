using Unity.Entities;
using Unity.Mathematics;

namespace Yohash.Propulsion.Entities
{
  /// <summary>
  /// <b>Opt-in: derives <see cref="PdPosition2D.TargetVelocity"/> for you, from how far the
  /// target moved since the last physics step.</b>
  ///
  /// WHAT IT DOES
  ///
  /// Add this component next to a <see cref="PdPosition2D"/>. Every physics step, before the
  /// controller runs, the backend's derivation system sets
  ///
  ///     TargetVelocity = (TargetPosition - PreviousTarget) / dt
  ///
  /// and then remembers the current target in <see cref="PreviousTarget"/>. dt is the physics
  /// engine's own timestep. The controller then tracks the moving target with no lag, even
  /// though nothing ever wrote its velocity.
  ///
  /// WHEN TO ADD IT
  ///
  /// When your code (or a designer, or an animation) only ever moves the target <i>position</i>:
  /// a waypoint, a cursor, a point another system drags around. Without it, the controller
  /// assumes the target is standing still, and a moving target is trailed by a steady lag of
  /// about TargetSpeed x D / K (roughly 0.16 m per 1 m/s of target speed at 2 Hz, critically
  /// damped).
  ///
  /// WHEN NOT TO ADD IT
  ///
  /// When you know the target's velocity, e.g. because the target is another physics body
  /// (read its velocity) or a path follower that knows its speed. Supplying the velocity is
  /// exact; deriving it is one step late on sudden changes. <b>While this component is present
  /// it overwrites TargetVelocity every step, so any value you write there is lost.</b> Remove
  /// the component (or disable it, see below) to supply the velocity yourself.
  ///
  /// TELEPORTS AND RE-TARGETING
  ///
  /// The derived velocity is the distance the target moved in one step, divided by a very short
  /// time. If you <i>jump</i> the target (teleport it, or switch to a different target far
  /// away), that one step reads as an enormous velocity, and the body gets a violent kick for
  /// one step. To avoid it, set <see cref="Initialized"/> to false whenever you jump the target.
  /// The next step then derives zero, as on the very first step, and derivation resumes
  /// normally from there.
  ///
  /// ACCURACY
  ///
  /// Exact for a target moving at constant velocity. When the target changes speed or
  /// direction, the derived velocity catches up one step later.
  ///
  /// ORDERING
  ///
  /// Move the target in a system that runs before the backend's system group (for ECSPhysics2D:
  /// <c>[UpdateBefore(typeof(Propulsion2DSystemGroup))]</c>, inside
  /// FixedStepSimulationSystemGroup). Derivation runs first inside that group, then control.
  ///
  /// This is an enableable component: disabling it (<c>SetComponentEnabled</c>) pauses
  /// derivation and leaves TargetVelocity alone, without a structural change. Re-enabling it
  /// resumes from <see cref="PreviousTarget"/>, so set <see cref="Initialized"/> to false at
  /// the same time if the target moved in between.
  /// </summary>
  public struct PdPositionHistory2D : IComponentData, IEnableableComponent
  {
    /// <summary>The target position on the previous physics step.</summary>
    public float2 PreviousTarget;

    /// <summary>
    /// False until the first derivation, and whenever you set it to false: the next step then
    /// derives a velocity of zero instead of a jump. Set it to false after teleporting or
    /// re-targeting.
    /// </summary>
    public bool Initialized;
  }
}
