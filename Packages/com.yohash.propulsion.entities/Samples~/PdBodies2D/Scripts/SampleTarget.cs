using Unity.Entities;
using Unity.Mathematics;

namespace Yohash.Propulsion.Entities.Samples
{
  /// <summary>Sample only: a pattern for <see cref="SampleTargetSystem"/> to move this body's targets in.</summary>
  public struct SampleTarget : IComponentData
  {
    public SampleTargetMode Mode;
    public float2 Center;
    public float Radius;

    /// <summary>Seconds per circle, or per pair of heading snaps.</summary>
    public float Period;

    /// <summary>Physics time since the pattern started.</summary>
    public float Time;
  }
}
