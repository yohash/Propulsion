using Unity.Entities;

namespace Yohash.Propulsion.Entities
{
  /// <summary>
  /// <b>Opt-in: derives <see cref="PdRotation2D.TargetAngularVelocity"/> for you, from how far
  /// the target angle turned since the last physics step.</b>
  ///
  /// WHAT IT DOES
  ///
  /// Add this component next to a <see cref="PdRotation2D"/>. Every physics step, before the
  /// controller runs, the backend's derivation system sets
  ///
  ///     TargetAngularVelocity = WrapAngle(TargetAngle - PreviousTarget) / dt
  ///
  /// and then remembers the current target in <see cref="PreviousTarget"/>. The change is taken
  /// the short way round, so a target crossing +/-180 degrees (e.g. from 179 to -179) reads as
  /// a 2 degree turn, not a 358 degree spin. dt is the physics engine's own timestep. The
  /// controller then tracks the turning target with no lag, even though nothing ever wrote its
  /// angular velocity.
  ///
  /// WHEN TO ADD IT
  ///
  /// When your code only ever sets the target <i>angle</i>: aiming at a moving point, facing
  /// the direction of travel, following a heading another system sets. Without it, the
  /// controller assumes the heading is fixed, and a turning target is trailed by a steady lag.
  ///
  /// WHEN NOT TO ADD IT
  ///
  /// When you know the target's angular velocity, e.g. because you are matching another body's
  /// spin. Supplying it is exact; deriving it is one step late on sudden changes. <b>While this
  /// component is present it overwrites TargetAngularVelocity every step, so any value you
  /// write there is lost.</b> Remove the component (or disable it) to supply it yourself.
  ///
  /// SNAPPING TO A NEW HEADING
  ///
  /// If you <i>jump</i> the target angle (snap to a new heading, or switch targets), that one
  /// step reads as a very fast turn, and the body gets a kick for one step. To avoid it, set
  /// <see cref="Initialized"/> to false whenever you jump the target. The next step then
  /// derives zero, and derivation resumes normally from there. (A jump of more than 180 degrees
  /// can't be told apart from a turn the other way, which is one more reason to reset.)
  ///
  /// ACCURACY
  ///
  /// Exact for a target turning at a constant rate below half a turn per step. When the turning
  /// rate changes, the derived value catches up one step later.
  ///
  /// ORDERING
  ///
  /// Set the target angle in a system that runs before the backend's system group (for
  /// ECSPhysics2D: <c>[UpdateBefore(typeof(Propulsion2DSystemGroup))]</c>, inside
  /// FixedStepSimulationSystemGroup). Derivation runs first inside that group, then control.
  ///
  /// This is an enableable component: disabling it pauses derivation and leaves
  /// TargetAngularVelocity alone. Re-enabling it resumes from <see cref="PreviousTarget"/>, so
  /// set <see cref="Initialized"/> to false at the same time if the target turned in between.
  /// </summary>
  public struct PdRotationHistory2D : IComponentData, IEnableableComponent
  {
    /// <summary>The target angle (radians) on the previous physics step.</summary>
    public float PreviousTarget;

    /// <summary>
    /// False until the first derivation, and whenever you set it to false: the next step then
    /// derives an angular velocity of zero instead of a jump. Set it to false after snapping to
    /// a new heading.
    /// </summary>
    public bool Initialized;
  }
}
