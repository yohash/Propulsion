using System.Runtime.CompilerServices;

// the engine tests read body state through EcsPhysics2DState, the same conversions the systems use
[assembly: InternalsVisibleTo("yohash.propulsion.entities.tests.physics2d")]
