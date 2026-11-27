using Unity.Entities;

namespace Yohash.Propulsion.Entities.Samples
{
  /// <summary>
  /// Sample only: a singleton created by <see cref="PdBodies2DSample"/>. The sample's systems
  /// require it, so they do nothing in any other scene.
  /// </summary>
  public struct PdBodies2DSampleTag : IComponentData
  {
  }
}
