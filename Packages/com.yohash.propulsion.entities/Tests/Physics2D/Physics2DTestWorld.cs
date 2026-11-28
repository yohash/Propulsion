using System;
using ECSPhysics2D;
using Unity.Core;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Yohash.Propulsion.Entities.Physics2D;

namespace Yohash.Propulsion.Entities.Tests.Physics2D
{
  /// <summary>
  /// A throwaway Entities world running every default system, including ECSPhysics2D's and the
  /// Propulsion backend's, stepped one physics step at a time. ECSPhysics2D creates its default
  /// physics world on creation: gravity (0, -9.81), 4 sub-steps, 1/60 s.
  /// </summary>
  internal sealed class Physics2DTestWorld : IDisposable
  {
    public readonly World World;
    public EntityManager EntityManager => World.EntityManager;

    private readonly FixedStepSimulationSystemGroup fixedStep;
    private double elapsed;

    public Physics2DTestWorld()
    {
      World = new World("Propulsion Physics2D test world");
      var systems = DefaultWorldInitialization.GetAllSystems(WorldSystemFilterFlags.Default);
      DefaultWorldInitialization.AddSystemsToRootLevelSystemGroups(World, systems);

      fixedStep = World.GetExistingSystemManaged<FixedStepSimulationSystemGroup>();
      // no rate manager: each Step() runs the fixed-step group exactly once
      fixedStep.RateManager = null;
    }

    public PhysicsWorldSingleton Physics
    {
      get {
        using var query = EntityManager.CreateEntityQuery(typeof(PhysicsWorldSingleton));
        return query.GetSingleton<PhysicsWorldSingleton>();
      }
    }

    public float Dt => Physics.FixedDeltaTime;

    /// <summary>One physics step: body creation, Propulsion's control, simulate, export.</summary>
    public void Step()
    {
      elapsed += Dt;
      World.SetTime(new TimeData(elapsed, Dt));
      fixedStep.Update();
    }

    /// <summary>
    /// A dynamic circle body. Its centre of mass is its position, unless the circle is offset
    /// from the body's origin by <paramref name="shapeOffset"/>.
    /// </summary>
    public Entity CreateBall(float2 position, float gravityScale = 1f, float radius = 0.5f, float2 shapeOffset = default)
    {
      var entity = EntityManager.CreateEntity();
      EntityManager.AddComponentData(entity, LocalTransform.FromPosition(position.x, position.y, 0f));
      EntityManager.AddComponentData(entity, new PhysicsBodyComponent { WorldIndex = 0, GravityScale = gravityScale });
      EntityManager.AddComponent<PhysicsDynamicTag>(entity);
      EntityManager.AddComponentData(entity, new PhysicsMaterial { Friction = 0.4f, Bounciness = 0f, Density = 1f });
      EntityManager.AddComponentData(entity, CollisionFilter.Default);
      EntityManager.AddComponentData(entity, new PhysicsShapeCircle { Radius = radius, Center = shapeOffset });
      return entity;
    }

    public UnityEngine.LowLevelPhysics2D.PhysicsBody Body(Entity entity)
    {
      return EntityManager.GetComponentData<PhysicsBodyComponent>(entity).Body;
    }

    public float2 CenterOfMass(Entity entity) => EcsPhysics2DState.CenterOfMass(Body(entity));
    public float2 Origin(Entity entity) => (float2)Body(entity).position;
    public bool Awake(Entity entity) => Body(entity).awake;
    public float Angle(Entity entity) => EcsPhysics2DState.Angle(Body(entity));

    public void Dispose()
    {
      if (World.IsCreated) { World.Dispose(); }
    }
  }
}
