using Unity.Entities;
using Unity.Mathematics;

namespace Yohash.Propulsion.Entities.Samples
{
  /// <summary>
  /// Sample only: every Interval seconds, <see cref="SampleKickSystem"/> hits the body with an
  /// off-centre ECSPhysics2D impulse, which knocks it off its position and spins it.
  /// </summary>
  public struct SampleKick : IComponentData
  {
    public float Interval;
    public float2 Impulse;

    /// <summary>Where the impulse lands, relative to the body's position.</summary>
    public float2 Offset;

    public float Timer;
  }
}
