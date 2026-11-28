using Unity.Mathematics;

namespace Yohash.Propulsion.Entities.Tests
{
  /// <summary>
  /// One physics step of semi-implicit Euler split into equal sub-steps, with the acceleration
  /// held constant across them: what Box2D v3 does with a force applied before the step.
  /// </summary>
  internal static class SubSteppedIntegrator
  {
    public static void Step(ref float2 position, ref float2 velocity, float2 acceleration, float dt, int subSteps)
    {
      float h = dt / subSteps;
      for (int i = 0; i < subSteps; i++) {
        velocity += acceleration * h;
        position += velocity * h;
      }
    }

    public static void Step(ref float angle, ref float angularVelocity, float angularAcceleration, float dt, int subSteps)
    {
      float h = dt / subSteps;
      for (int i = 0; i < subSteps; i++) {
        angularVelocity += angularAcceleration * h;
        angle += angularVelocity * h;
      }
    }
  }
}
