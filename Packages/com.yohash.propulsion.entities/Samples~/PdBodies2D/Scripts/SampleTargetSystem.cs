using ECSPhysics2D;
using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Yohash.Propulsion.Core;
using Yohash.Propulsion.Entities.Physics2D;

namespace Yohash.Propulsion.Entities.Samples
{
  /// <summary>
  /// Sample only: moves the targets of bodies with a <see cref="SampleTarget"/>.
  ///
  /// This is the pattern for your own target writers: run in FixedStepSimulationSystemGroup,
  /// before <see cref="Propulsion2DSystemGroup"/>, and write only the target <i>positions and
  /// angles</i>. The bodies' history components derive the target velocities.
  /// </summary>
  [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
  [UpdateBefore(typeof(Propulsion2DSystemGroup))]
  [BurstCompile]
  public partial struct SampleTargetSystem : ISystem
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

      foreach (var (pattern, controller) in SystemAPI
                 .Query<RefRO<SampleTarget>, RefRW<PdPosition2D>>()) {
        var p = pattern.ValueRO;
        if (p.Mode != SampleTargetMode.Circle) { continue; }
        float theta = 2f * math.PI * p.Time / p.Period;
        controller.ValueRW.TargetPosition = p.Center + p.Radius * new float2(math.cos(theta), math.sin(theta));
      }

      foreach (var (pattern, controller, entity) in SystemAPI
                 .Query<RefRO<SampleTarget>, RefRW<PdRotation2D>>()
                 .WithEntityAccess()) {
        var p = pattern.ValueRO;
        if (p.Mode == SampleTargetMode.Circle) {
          // face the direction of travel
          float theta = 2f * math.PI * p.Time / p.Period;
          controller.ValueRW.TargetAngle = MotionMath.WrapAngle(theta + 0.5f * math.PI);
        } else if (p.Mode == SampleTargetMode.SnapHeading) {
          float half = 0.5f * p.Period;
          int phase = (int)math.floor(p.Time / half) & 1;
          int previousPhase = (int)math.floor((p.Time - dt) / half) & 1;
          if (phase != previousPhase) {
            controller.ValueRW.TargetAngle = phase == 0 ? 0f : 0.5f * math.PI;
            // a snap is a jump, not a turn: reset the history so it derives zero, not a kick
            if (SystemAPI.HasComponent<PdRotationHistory2D>(entity)) {
              SystemAPI.GetComponentRW<PdRotationHistory2D>(entity).ValueRW.Initialized = false;
            }
          }
        }
      }

      foreach (var pattern in SystemAPI.Query<RefRW<SampleTarget>>()) {
        pattern.ValueRW.Time += dt;
      }
    }
  }
}
