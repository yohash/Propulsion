using UnityEngine;

namespace Yohash.Propulsion.Entities
{
  /// <summary>
  /// Bakes stable PD control onto a 2D physics body: <see cref="PdPosition2D"/> and/or
  /// <see cref="PdRotation2D"/>, each optionally with its target-velocity history.
  ///
  /// Add it to a GameObject whose entity also gets a physics body: for ECSPhysics2D, the
  /// body's components (<c>PhysicsBodyComponent</c>, <c>PhysicsDynamicTag</c>, a shape) come
  /// from your own baker or spawning code, as in ECSPhysics2D's samples. The targets start
  /// at the GameObject's position and Z rotation, so by default the body holds where it was
  /// placed. Your systems then move the targets at runtime.
  ///
  /// This component does nothing at runtime on its own: it needs a physics backend, e.g. the
  /// ECSPhysics2D backend, which compiles when <c>com.ecsphysics2d.core</c> is installed.
  /// </summary>
  [DisallowMultipleComponent]
  [AddComponentMenu("Propulsion/PD Body 2D")]
  public class PdBody2DAuthoring : MonoBehaviour
  {
    [Header("Position")]
    [Tooltip("Drive the body's centre of mass towards a target position with a force.")]
    public bool ControlPosition = true;

    [Tooltip("Natural frequency of the position response, in Hz. With damping 1, the error falls to ~5% after about 0.75 / frequency seconds.")]
    [Min(0f)]
    public float PositionFrequency = 2f;

    [Tooltip("Damping ratio of the position response: 1 is critically damped (fastest without overshoot), below 1 overshoots, above 1 is sluggish.")]
    [Min(0f)]
    public float PositionDamping = 1f;

    [Tooltip("Cancel this body's gravity exactly (world gravity x gravity scale), so it holds its target with no sag. Only gravity is cancelled; every other force still acts.")]
    public bool CompensateGravity = true;

    [Tooltip("Derive the target's velocity from how far the target moves each step (adds PdPositionHistory2D). Turn off if your code sets TargetVelocity itself: while on, it OVERWRITES TargetVelocity every step.")]
    public bool DerivePositionTargetVelocity = true;

    [Header("Rotation")]
    [Tooltip("Turn the body towards a target angle with a torque, the short way round.")]
    public bool ControlRotation = true;

    [Tooltip("Natural frequency of the rotation response, in Hz.")]
    [Min(0f)]
    public float RotationFrequency = 2f;

    [Tooltip("Damping ratio of the rotation response: 1 is critically damped.")]
    [Min(0f)]
    public float RotationDamping = 1f;

    [Tooltip("Derive the target's angular velocity from how far the target angle turns each step (adds PdRotationHistory2D). Turn off if your code sets TargetAngularVelocity itself: while on, it OVERWRITES it every step.")]
    public bool DeriveRotationTargetVelocity = true;
  }
}
