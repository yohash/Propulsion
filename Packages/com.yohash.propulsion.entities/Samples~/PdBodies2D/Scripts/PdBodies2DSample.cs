using ECSPhysics2D;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using Yohash.Propulsion.Core;

namespace Yohash.Propulsion.Entities.Samples
{
  /// <summary>
  /// Sample: three ECSPhysics2D boxes under gravity, each driven by forces from Propulsion's PD
  /// controllers while other forces act on them too.
  ///
  ///   Left:   holds its position and heading, and is kicked off-centre every 3 seconds by an
  ///           ECSPhysics2D impulse. It recovers at its tuned frequency and damping.
  ///   Middle: follows a target circling at 2 m radius, facing its direction of travel. Only the
  ///           target position and angle are written; the history components derive the
  ///           target velocities, so it tracks with no lag.
  ///   Right:  holds its position against a constant ECSPhysics2D "wind" force, so it settles a
  ///           little downwind, and snaps its heading between 0 and 90 degrees every 2 seconds,
  ///           resetting its rotation history on each snap.
  ///
  /// The bodies are created in code, like ECSPhysics2D's spawners, so the scene only needs a
  /// camera and this component. Turn on <b>ECS Physics 2D > Debug Draw Enabled</b> to see them.
  /// </summary>
  public class PdBodies2DSample : MonoBehaviour
  {
    private void Start()
    {
      var world = World.DefaultGameObjectInjectionWorld;
      if (world == null) {
        Debug.LogError("PdBodies2DSample: no default Entities world.");
        return;
      }
      var em = world.EntityManager;
      em.CreateSingleton<PdBodies2DSampleTag>();

      // left: holds, and is kicked
      var holder = CreateBox(em, new float2(-5f, 1f));
      AddControllers(em, holder, new float2(-5f, 1f), 0f, position: new StablePdSettings(1.5f, 0.5f), rotation: new StablePdSettings(1.5f, 0.4f));
      em.AddComponentData(holder, new SampleKick { Interval = 3f, Impulse = new float2(3f, 4f), Offset = new float2(0f, 0.5f) });

      // middle: follows a circling target, facing its direction of travel
      var follower = CreateBox(em, new float2(2f, 1f));
      AddControllers(em, follower, new float2(2f, 1f), 0.5f * math.PI, position: new StablePdSettings(2f, 1f), rotation: new StablePdSettings(3f, 1f));
      em.AddComponentData(follower, new SampleTarget { Mode = SampleTargetMode.Circle, Center = new float2(0f, 1f), Radius = 2f, Period = 5f });

      // right: holds against the wind, and snaps its heading
      var vane = CreateBox(em, new float2(5f, 1f));
      AddControllers(em, vane, new float2(5f, 1f), 0f, position: new StablePdSettings(1f, 1f), rotation: new StablePdSettings(2f, 0.7f));
      em.AddComponentData(vane, PhysicsForce.CreateCentralForce(new float2(4f, 0f)));
      em.AddComponentData(vane, new SampleTarget { Mode = SampleTargetMode.SnapHeading, Period = 4f });
    }

    private static Entity CreateBox(EntityManager em, float2 position)
    {
      var entity = em.CreateEntity();
      em.AddComponentData(entity, LocalTransform.FromPosition(position.x, position.y, 0f));
      em.AddComponentData(entity, new PhysicsBodyComponent { WorldIndex = 0, GravityScale = 1f });
      em.AddComponent<PhysicsDynamicTag>(entity);
      em.AddComponentData(entity, new PhysicsMaterial { Friction = 0.4f, Bounciness = 0.2f, Density = 1f });
      em.AddComponentData(entity, CollisionFilter.Default);
      em.AddComponentData(entity, new PhysicsShapeBox { Size = new float2(1f, 1f), Center = float2.zero, Rotation = 0f });
      return entity;
    }

    private static void AddControllers(EntityManager em, Entity entity, float2 target, float angle, StablePdSettings position, StablePdSettings rotation)
    {
      em.AddComponentData(entity, new PdPosition2D { Settings = position, TargetPosition = target, CompensateGravity = true });
      em.AddComponentData(entity, new PdPositionHistory2D());
      em.AddComponentData(entity, new PdRotation2D { Settings = rotation, TargetAngle = angle });
      em.AddComponentData(entity, new PdRotationHistory2D());
    }
  }
}
