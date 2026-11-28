using Unity.Entities;
using Unity.Mathematics;
using Yohash.Propulsion.Core;

namespace Yohash.Propulsion.Entities
{
  /// <summary>
  /// Drives a 2D body's centre of mass towards a target position with a stable PD controller.
  ///
  /// Each physics step, the backend's control system reads the body's state from the physics
  /// engine, computes the acceleration that moves it towards the target with exactly the
  /// requested frequency and damping, and applies it as a <b>force</b> (mass x acceleration).
  /// The force is added to the engine's own force accumulator, so it combines with every other
  /// force on the body: gravity, contacts, explosions, force fields and your own systems.
  ///
  /// Write <see cref="TargetPosition"/> (and <see cref="TargetVelocity"/>, unless the entity
  /// also has a <see cref="PdPositionHistory2D"/>) from a system that runs before the backend's
  /// system group, e.g. <c>[UpdateBefore(typeof(Propulsion2DSystemGroup))]</c>.
  ///
  /// Positions are world-space metres, velocities metres per second.
  /// </summary>
  public struct PdPosition2D : IComponentData
  {
    /// <summary>Frequency (Hz) and damping ratio of the response.</summary>
    public StablePdSettings Settings;

    /// <summary>Where the body's centre of mass should be.</summary>
    public float2 TargetPosition;

    /// <summary>
    /// How fast the target is moving. The controller matches it, so a moving target is tracked
    /// without lag. Leave it zero for a stationary target.
    ///
    /// Overwritten every step when the entity has a <see cref="PdPositionHistory2D"/>.
    /// </summary>
    public float2 TargetVelocity;

    /// <summary>
    /// An extra acceleration you know about, added to the output: for example the target's own
    /// acceleration, so a speeding-up target is tracked without lag. Zero by default.
    /// </summary>
    public float2 Feedforward;

    /// <summary>
    /// Cancels this body's gravity exactly (the physics world's gravity times the body's
    /// gravity scale), so the body holds its target with no sag. Without it, the body settles
    /// g / K below the target (about 7.6 cm at 2 Hz with 60 Hz physics).
    ///
    /// Only gravity is cancelled. Every other force still acts on the body, and the controller
    /// pushes back against it at its tuned stiffness.
    /// </summary>
    public bool CompensateGravity;

    /// <summary>Internal: the gains solved for the current settings and timestep.</summary>
    public PdGainsCache Cache;
  }
}
