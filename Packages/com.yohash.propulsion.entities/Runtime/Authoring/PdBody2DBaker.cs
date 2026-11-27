using Unity.Entities;
using Unity.Mathematics;
using Yohash.Propulsion.Core;

namespace Yohash.Propulsion.Entities
{
  /// <summary>
  /// Bakes <see cref="PdBody2DAuthoring"/>. The targets start at the GameObject's world
  /// position (x, y) and its rotation about Z, so the body holds where it was placed.
  /// </summary>
  public class PdBody2DBaker : Baker<PdBody2DAuthoring>
  {
    public override void Bake(PdBody2DAuthoring authoring)
    {
      var entity = GetEntity(TransformUsageFlags.Dynamic);
      var transform = authoring.transform;
      float2 position = new float2(transform.position.x, transform.position.y);
      float angle = MotionMath.WrapAngle(math.radians(transform.eulerAngles.z));

      if (authoring.ControlPosition) {
        AddComponent(entity, new PdPosition2D {
          Settings = new StablePdSettings(authoring.PositionFrequency, authoring.PositionDamping),
          TargetPosition = position,
          CompensateGravity = authoring.CompensateGravity,
        });
        if (authoring.DerivePositionTargetVelocity) {
          AddComponent(entity, new PdPositionHistory2D());
        }
      }

      if (authoring.ControlRotation) {
        AddComponent(entity, new PdRotation2D {
          Settings = new StablePdSettings(authoring.RotationFrequency, authoring.RotationDamping),
          TargetAngle = angle,
        });
        if (authoring.DeriveRotationTargetVelocity) {
          AddComponent(entity, new PdRotationHistory2D());
        }
      }
    }
  }
}
