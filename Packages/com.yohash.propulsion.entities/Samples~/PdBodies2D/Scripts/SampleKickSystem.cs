using ECSPhysics2D;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Transforms;

namespace Yohash.Propulsion.Entities.Samples
{
  /// <summary>
  /// Sample only: periodically hits bodies with a <see cref="SampleKick"/> using ECSPhysics2D's
  /// own one-shot <c>PhysicsImpulse</c>. The impulse lands off-centre, so the body is knocked
  /// away and spun; its controllers then pull it back, at their tuned frequency and damping.
  /// </summary>
  [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
  [UpdateAfter(typeof(BuildPhysicsWorldSystem))]
  [UpdateBefore(typeof(BatchForceApplicationSystem))]
  [BurstCompile]
  public partial struct SampleKickSystem : ISystem
  {
    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
      state.RequireForUpdate<PdBodies2DSampleTag>();
      state.RequireForUpdate<PhysicsWorldSingleton>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
      float dt = SystemAPI.GetSingleton<PhysicsWorldSingleton>().FixedDeltaTime;
      var ecb = new EntityCommandBuffer(Allocator.Temp);

      foreach (var (kick, transform, entity) in SystemAPI
                 .Query<RefRW<SampleKick>, RefRO<LocalTransform>>()
                 .WithEntityAccess()) {
        kick.ValueRW.Timer += dt;
        if (kick.ValueRO.Timer < kick.ValueRO.Interval) { continue; }
        kick.ValueRW.Timer -= kick.ValueRO.Interval;

        var point = transform.ValueRO.Position.xy + kick.ValueRO.Offset;
        ecb.AddComponent(entity, PhysicsImpulse.CreatePointImpulse(kick.ValueRO.Impulse, point));
      }

      ecb.Playback(state.EntityManager);
      ecb.Dispose();
    }
  }
}
