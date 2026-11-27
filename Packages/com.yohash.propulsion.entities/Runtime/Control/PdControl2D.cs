using Unity.Mathematics;
using Yohash.Propulsion.Core;

namespace Yohash.Propulsion.Entities
{
  /// <summary>
  /// The engine-neutral part of 2D control: given a body's state, returns the acceleration a
  /// controller component asks for. Each physics backend reads the state from its engine, calls
  /// these, and applies the result as a force or torque. All the maths is in
  /// <c>com.yohash.propulsion.core</c>; this only wires a component's fields to it.
  ///
  /// Burst-compatible, no exceptions. Both functions update the controller's gains cache, which
  /// re-solves only when the settings, the timestep or the sub-step count change.
  /// </summary>
  public static class PdControl2D
  {
    /// <summary>
    /// The linear acceleration (m/s^2) for a <see cref="PdPosition2D"/>: multiply by the body's
    /// mass for the force.
    /// </summary>
    /// <param name="controller">The controller; its gains cache is updated.</param>
    /// <param name="position">The body's centre of mass, world space.</param>
    /// <param name="velocity">The body's centre-of-mass velocity, world space.</param>
    /// <param name="gravity">
    /// The gravitational acceleration acting on this body: the world's gravity times the
    /// body's gravity scale. Used only when <c>CompensateGravity</c> is set.
    /// </param>
    /// <param name="dt">The physics engine's timestep (s).</param>
    /// <param name="subSteps">The sub-steps the engine runs per step, for a force held across them.</param>
    public static float2 Acceleration(
      ref PdPosition2D controller,
      float2 position,
      float2 velocity,
      float2 gravity,
      float dt,
      int subSteps
    )
    {
      var gains = controller.Cache.Resolve(controller.Settings, dt, subSteps);
      return StablePd.Acceleration(
        gains,
        position,
        velocity,
        controller.TargetPosition,
        controller.TargetVelocity,
        Feedforward(controller, gravity)
      );
    }

    /// <summary>
    /// The angular acceleration (rad/s^2) for a <see cref="PdRotation2D"/>: multiply by the
    /// body's rotational inertia for the torque.
    /// </summary>
    /// <param name="controller">The controller; its gains cache is updated.</param>
    /// <param name="angle">The body's angle, in radians.</param>
    /// <param name="angularVelocity">The body's angular velocity, in <b>radians</b> per second.</param>
    /// <param name="dt">The physics engine's timestep (s).</param>
    /// <param name="subSteps">The sub-steps the engine runs per step, for a torque held across them.</param>
    public static float AngularAcceleration(
      ref PdRotation2D controller,
      float angle,
      float angularVelocity,
      float dt,
      int subSteps
    )
    {
      var gains = controller.Cache.Resolve(controller.Settings, dt, subSteps);
      return StablePd.AngularAcceleration(
        gains,
        angle,
        angularVelocity,
        controller.TargetAngle,
        controller.TargetAngularVelocity,
        controller.Feedforward
      );
    }

    /// <summary>
    /// The feedforward acceleration a <see cref="PdPosition2D"/> asks for: its own
    /// <c>Feedforward</c>, minus gravity when <c>CompensateGravity</c> is set.
    /// </summary>
    public static float2 Feedforward(in PdPosition2D controller, float2 gravity)
    {
      return controller.CompensateGravity
        ? controller.Feedforward - gravity
        : controller.Feedforward;
    }
  }
}
