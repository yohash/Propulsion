using Unity.Entities;
using Yohash.Propulsion.Core;

namespace Yohash.Propulsion.Entities
{
  /// <summary>
  /// Turns a 2D body towards a target angle with a stable PD controller.
  ///
  /// Each physics step, the backend's control system reads the body's angle and angular
  /// velocity from the physics engine, computes the angular acceleration that turns it towards
  /// the target with exactly the requested frequency and damping, and applies it as a
  /// <b>torque</b> (rotational inertia x angular acceleration). The torque is added to the
  /// engine's own accumulator, so it combines with every other torque on the body.
  ///
  /// The body always turns the short way round: an error of 350 degrees is a turn of -10.
  ///
  /// Angles are in <b>radians</b> (counter-clockwise), angular velocities in <b>radians per
  /// second</b>. (Unity's 2D physics reports angular velocity in degrees per second; the
  /// backend converts it, so everything on this component is radians.)
  /// </summary>
  public struct PdRotation2D : IComponentData
  {
    /// <summary>Frequency (Hz) and damping ratio of the response.</summary>
    public StablePdSettings Settings;

    /// <summary>The angle the body should face, in radians. Any value; it is wrapped.</summary>
    public float TargetAngle;

    /// <summary>
    /// How fast the target angle is turning, in radians per second. Leave it zero for a fixed
    /// heading.
    ///
    /// Overwritten every step when the entity has a <see cref="PdRotationHistory2D"/>.
    /// </summary>
    public float TargetAngularVelocity;

    /// <summary>
    /// An extra angular acceleration (rad/s^2) you know about, added to the output: for example
    /// the target's own angular acceleration. Zero by default.
    /// </summary>
    public float Feedforward;

    /// <summary>Internal: the gains solved for the current settings and timestep.</summary>
    public PdGainsCache Cache;
  }
}
