using Unity.Mathematics;
using Yohash.Propulsion.Core;

namespace Yohash.Propulsion.Entities
{
  /// <summary>
  /// The maths behind <see cref="PdPositionHistory2D"/> and <see cref="PdRotationHistory2D"/>:
  /// derives a target's velocity from its change since the previous step. The backend systems
  /// call these; they are public so you can derive velocities the same way in your own systems.
  ///
  /// Burst-compatible, no exceptions. An invalid timestep (dt &lt;= 0) changes nothing.
  /// </summary>
  public static class PdTargetDerivation2D
  {
    /// <summary>
    /// Sets <c>controller.TargetVelocity</c> from the target's movement since the previous
    /// step, and records the current target. Derives zero when <c>history.Initialized</c> is
    /// false (the first step, or after a reset).
    /// </summary>
    public static void DeriveVelocity(ref PdPosition2D controller, ref PdPositionHistory2D history, float dt)
    {
      if (!(dt > 0f)) { return; }

      controller.TargetVelocity = history.Initialized
        ? (controller.TargetPosition - history.PreviousTarget) / dt
        : float2.zero;

      history.PreviousTarget = controller.TargetPosition;
      history.Initialized = true;
    }

    /// <summary>
    /// Sets <c>controller.TargetAngularVelocity</c> from the target angle's change since the
    /// previous step, the short way round. Derives zero when <c>history.Initialized</c> is
    /// false (the first step, or after a reset).
    /// </summary>
    public static void DeriveAngularVelocity(ref PdRotation2D controller, ref PdRotationHistory2D history, float dt)
    {
      if (!(dt > 0f)) { return; }

      controller.TargetAngularVelocity = history.Initialized
        ? MotionMath.AngularVelocity(history.PreviousTarget, controller.TargetAngle, dt)
        : 0f;

      history.PreviousTarget = controller.TargetAngle;
      history.Initialized = true;
    }
  }
}
