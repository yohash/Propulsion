using Unity.Mathematics;
using UnityEngine.LowLevelPhysics2D;

namespace Yohash.Propulsion.Entities.Physics2D
{
  /// <summary>
  /// Every read the ECSPhysics2D backend makes from <c>UnityEngine.LowLevelPhysics2D</c>, in one
  /// place, already converted to the units the controllers use: metres, radians, radians per
  /// second, world space.
  /// </summary>
  internal static class EcsPhysics2DState
  {
    /// <summary>
    /// The sub-steps the world runs per step (Box2D v3 defaults to 4). Read from the live world,
    /// so it is always what the world actually simulates with.
    /// </summary>
    public static int SubSteps(PhysicsWorld world)
    {
      return math.max(1, world.simulationSubSteps);
    }

    /// <summary>The gravitational acceleration on this body: world gravity x its gravity scale.</summary>
    public static float2 Gravity(PhysicsWorld world, PhysicsBody body)
    {
      return (float2)world.gravity * body.gravityScale;
    }

    /// <summary>The body's centre of mass, world space. The controllers drive this point.</summary>
    public static float2 CenterOfMass(PhysicsBody body)
    {
      return (float2)body.worldCenterOfMass;
    }

    /// <summary>The velocity of the body's centre of mass, world space.</summary>
    public static float2 LinearVelocity(PhysicsBody body)
    {
      return (float2)body.linearVelocity;
    }

    /// <summary>The body's angle, in radians.</summary>
    public static float Angle(PhysicsBody body)
    {
      return body.rotation.angle;
    }

    /// <summary>
    /// The body's angular velocity, in <b>radians</b> per second. Unity's 2D physics reports
    /// degrees per second; unconverted, the controller's damping would be ~57x too strong.
    /// </summary>
    public static float AngularVelocity(PhysicsBody body)
    {
      return math.radians(body.angularVelocity);
    }

    /// <summary>The body's rotational inertia about its centre of mass (kg m^2).</summary>
    public static float RotationalInertia(PhysicsBody body)
    {
      return body.rotationalInertia;
    }
  }
}
