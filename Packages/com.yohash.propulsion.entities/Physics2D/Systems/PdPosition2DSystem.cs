using ECSPhysics2D;
using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;

namespace Yohash.Propulsion.Entities.Physics2D
{
  /// <summary>
  /// Applies each <see cref="PdPosition2D"/> to its ECSPhysics2D body as a force at the centre of
  /// mass.
  ///
  /// For each dynamic body, every physics step:
  ///   1. read the centre of mass and its velocity, the body's gravity, and the world's timestep
  ///      and sub-step count from the live physics world;
  ///   2. ask <see cref="PdControl2D"/> for the acceleration (the gains are solved for that
  ///      timestep and sub-step count, and cached);
  ///   3. <c>ApplyForceToCenter(mass x acceleration)</c>.
  ///
  /// The force is <b>added</b> to Box2D's force accumulator, which the world clears after each
  /// step. Nothing on the body is ever overwritten, so the controller combines with every other
  /// force on it.
  ///
  /// A sleeping body is left asleep while either its <b>feedback</b> (the response to its
  /// position and velocity error, without feedforward) or its <b>net</b> acceleration (the
  /// controller's output plus gravity) is negligible. So a body at rest on its target stays
  /// asleep in every case:
  ///   - gravity-compensated, hovering: both are zero (its output alone is -gravity);
  ///   - uncompensated, resting on the ground: its feedback is zero (its net is gravity);
  ///   - uncompensated, hanging at its sag below the target: its net is zero.
  /// Moving its target makes both non-zero, and <c>ApplyForceToCenter</c> wakes the body.
  /// </summary>
  [UpdateInGroup(typeof(Propulsion2DSystemGroup))]
  [UpdateAfter(typeof(PdTargetVelocity2DSystem))]
  [BurstCompile]
  public partial struct PdPosition2DSystem : ISystem
  {
    // Below this feedback or net acceleration (m/s^2), a sleeping body isn't woken.
    private const float SleepingAccelerationThreshold = 1e-3f;

    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
      state.RequireForUpdate<PhysicsWorldSingleton>();
      state.RequireForUpdate<PdPosition2D>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
      var physics = SystemAPI.GetSingleton<PhysicsWorldSingleton>();
      float dt = physics.FixedDeltaTime;

      foreach (var (controller, bodyComponent) in SystemAPI
                 .Query<RefRW<PdPosition2D>, RefRO<PhysicsBodyComponent>>()
                 .WithAll<PhysicsDynamicTag>()) {
        if (!bodyComponent.ValueRO.IsValid) { continue; }
        int worldIndex = bodyComponent.ValueRO.WorldIndex;
        if (!physics.IsValidWorldIndex(worldIndex)) { continue; }

        var world = physics.GetWorld(worldIndex);
        var body = bodyComponent.ValueRO.Body;
        float2 gravity = EcsPhysics2DState.Gravity(world, body);

        float2 acceleration = PdControl2D.Acceleration(
          ref controller.ValueRW,
          EcsPhysics2DState.CenterOfMass(body),
          EcsPhysics2DState.LinearVelocity(body),
          gravity,
          dt,
          EcsPhysics2DState.SubSteps(world)
        );

        // not the raw output: with CompensateGravity it includes -gravity even for a body at
        // rest on its target. Feedback covers bodies resting on the ground; net covers
        // compensated bodies hovering, and uncompensated ones hanging at their sag.
        if (!body.awake) {
          float2 feedback = acceleration - PdControl2D.Feedforward(controller.ValueRO, gravity);
          float2 net = acceleration + gravity;
          const float threshold2 = SleepingAccelerationThreshold * SleepingAccelerationThreshold;
          if (math.lengthsq(feedback) < threshold2 || math.lengthsq(net) < threshold2) { continue; }
        }
        body.ApplyForceToCenter(body.mass * acceleration);
      }
    }
  }
}
