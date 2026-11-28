using ECSPhysics2D;
using Unity.Burst;
using Unity.Entities;

namespace Yohash.Propulsion.Entities.Physics2D
{
  /// <summary>
  /// Derives target velocities for controllers that opted in with a
  /// <see cref="PdPositionHistory2D"/> or <see cref="PdRotationHistory2D"/>, before the
  /// controllers run. See those components for exactly what this overwrites, and how to reset
  /// it after a teleport.
  ///
  /// Uses ECSPhysics2D's own timestep (<c>PhysicsWorldSingleton.FixedDeltaTime</c>), the dt its
  /// worlds are simulated with, so derived velocities are in the same time units as the bodies'.
  /// Disabled history components are skipped.
  /// </summary>
  [UpdateInGroup(typeof(Propulsion2DSystemGroup), OrderFirst = true)]
  [BurstCompile]
  public partial struct PdTargetVelocity2DSystem : ISystem
  {
    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
      state.RequireForUpdate<PhysicsWorldSingleton>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
      float dt = SystemAPI.GetSingleton<PhysicsWorldSingleton>().FixedDeltaTime;

      foreach (var (controller, history) in SystemAPI
                 .Query<RefRW<PdPosition2D>, RefRW<PdPositionHistory2D>>()) {
        PdTargetDerivation2D.DeriveVelocity(ref controller.ValueRW, ref history.ValueRW, dt);
      }

      foreach (var (controller, history) in SystemAPI
                 .Query<RefRW<PdRotation2D>, RefRW<PdRotationHistory2D>>()) {
        PdTargetDerivation2D.DeriveAngularVelocity(ref controller.ValueRW, ref history.ValueRW, dt);
      }
    }
  }
}
