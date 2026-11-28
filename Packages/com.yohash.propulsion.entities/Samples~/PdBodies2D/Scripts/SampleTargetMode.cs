namespace Yohash.Propulsion.Entities.Samples
{
  /// <summary>How <see cref="SampleTargetSystem"/> moves a body's targets.</summary>
  public enum SampleTargetMode : byte
  {
    /// <summary>Targets stay where they are.</summary>
    Hold,

    /// <summary>The position target circles Center at Radius, facing the direction of travel.</summary>
    Circle,

    /// <summary>The heading snaps between 0 and 90 degrees every half Period.</summary>
    SnapHeading,
  }
}
