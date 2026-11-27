using ECSPhysics2D;
using Unity.Entities;

namespace Yohash.Propulsion.Entities.Physics2D
{
  /// <summary>
  /// Runs Propulsion's 2D control each physics step: first the target-velocity derivation, then
  /// the position and rotation controllers.
  ///
  /// It sits in FixedStepSimulationSystemGroup after ECSPhysics2D has created this step's bodies
  /// and shapes (so new bodies are controlled on their first step, with their real mass), and
  /// before <c>PhysicsSimulationSystem</c> steps the world. The controllers add their forces to
  /// Box2D's accumulator alongside every other force applied in that window (ECSPhysics2D's
  /// force components, force fields, explosions, your own systems); the order among them doesn't
  /// matter, because each controller computes from the body's state, not from other forces.
  ///
  /// Order the systems that move your targets before this group:
  /// <c>[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]</c>
  /// <c>[UpdateBefore(typeof(Propulsion2DSystemGroup))]</c>.
  /// </summary>
  [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
  [UpdateAfter(typeof(BuildPhysicsWorldSystem))]
  [UpdateAfter(typeof(ShapeCreationSystem))]
  [UpdateAfter(typeof(CompoundShapeBuilderSystem))]
  [UpdateBefore(typeof(PhysicsSimulationSystem))]
  public partial class Propulsion2DSystemGroup : ComponentSystemGroup
  {
  }
}
