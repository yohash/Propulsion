using ECSPhysics2D;
using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;

namespace Yohash.Propulsion.Entities.Physics2D
{
  /// <summary>
  /// Applies each <see cref="PdRotation2D"/> to its ECSPhysics2D body as a torque.
  ///
  /// For each dynamic body, every physics step:
  ///   1. read the angle (radians) and angular velocity, converted from Unity's degrees per
  ///      second to radians per second, and the world's timestep and sub-step count;
  ///   2. ask <see cref="PdControl2D"/> for the angular acceleration (gains solved for that
  ///      timestep and sub-step count, and cached);
  ///   3. <c>ApplyTorque(rotational inertia x angular acceleration)</c>.
  ///
  /// The torque is <b>added</b> to Box2D's accumulator, so it combines with every other torque
  /// on the body. A sleeping body is left asleep while its torque would be negligible.
  /// </summary>
  [UpdateInGroup(typeof(Propulsion2DSystemGroup))]
  [UpdateAfter(typeof(PdTargetVelocity2DSystem))]
  [BurstCompile]
  public partial struct PdRotation2DSystem : ISystem
  {
    // Below this (rad/s^2), a sleeping body isn't woken.
    private const float SleepingAccelerationThreshold = 1e-3f;

    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
      state.RequireForUpdate<PhysicsWorldSingleton>();
      state.RequireForUpdate<PdRotation2D>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
      var physics = SystemAPI.GetSingleton<PhysicsWorldSingleton>();
      float dt = physics.FixedDeltaTime;

      foreach (var (controller, bodyComponent) in SystemAPI
                 .Query<RefRW<PdRotation2D>, RefRO<PhysicsBodyComponent>>()
                 .WithAll<PhysicsDynamicTag>()) {
        if (!bodyComponent.ValueRO.IsValid) { continue; }
        int worldIndex = bodyComponent.ValueRO.WorldIndex;
        if (!physics.IsValidWorldIndex(worldIndex)) { continue; }

        var world = physics.GetWorld(worldIndex);
        var body = bodyComponent.ValueRO.Body;

        float angularAcceleration = PdControl2D.AngularAcceleration(
          ref controller.ValueRW,
          EcsPhysics2DState.Angle(body),
          EcsPhysics2DState.AngularVelocity(body),
          dt,
          EcsPhysics2DState.SubSteps(world)
        );

        if (!body.awake && math.abs(angularAcceleration) < SleepingAccelerationThreshold) {
          continue;
        }
        body.ApplyTorque(EcsPhysics2DState.RotationalInertia(body) * angularAcceleration);
      }
    }
  }
}
