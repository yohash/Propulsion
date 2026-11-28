# Propulsion Entities

Stable PD control for Unity ECS physics bodies. Add a component, set a target, and the body is pushed towards it by **forces**, with exactly the frequency and damping you ask for. The forces are added to the physics engine's own accumulator, so they combine with everything else in the scene: gravity, contacts, explosions, force fields and your own systems. Knock a controlled body away and it recovers at its tuned rate; push it steadily and it leans into the push like a spring.

All the maths comes from [Propulsion Core](../com.yohash.propulsion.core). This package holds the components, authoring and per-engine systems.

| Physics backend | Status | Compiles when installed |
|---|---|---|
| **ECSPhysics2D** (`com.ecsphysics2d.core`, Box2D v3 via `UnityEngine.LowLevelPhysics2D`) | ✅ 0.1.0 | `com.ecsphysics2d.core` 0.3.0 or newer |
| **Unity Physics** (`com.unity.physics`) | Planned for 0.1.0, not in this build yet | — |

Each backend lives in its own assembly, which compiles only when its physics package is installed. Neither physics package is a hard dependency.

Requires Unity 6 (6000.3) or newer and Entities 1.4.4 or newer.

## Installation

Unity can't resolve a package's git-hosted dependencies by itself, so install them first, in this order. Use **+ > Add package from git URL...** in the Package Manager:

1. **Propulsion Core:** `https://github.com/yohash/Propulsion.git#upm-core`
2. **Your physics package**, e.g. ECSPhysics2D: `https://github.com/yohash/ECSPhysics2D.git#upm`
3. **Propulsion Entities:** `https://github.com/yohash/Propulsion.git#upm-entities`

Pin releases with tags instead, e.g. `#core-v0.1.0` and `#entities-v0.1.0`. Entities, Burst and Mathematics come from Unity's registry automatically.

## Quick start

**In the editor:** add **Propulsion > PD Body 2D** (`PdBody2DAuthoring`) to a GameObject in a subscene whose entity also gets an ECSPhysics2D body. The targets start where the GameObject is placed, so by default the body holds its position and heading. (The position target is for the body's **centre of mass**: a body whose shapes are offset from the GameObject shifts on start so that its centre of mass lands there.)

**In code:** add the components to any dynamic body.

```csharp
using Unity.Mathematics;
using Yohash.Propulsion.Core;
using Yohash.Propulsion.Entities;

entityManager.AddComponentData(entity, new PdPosition2D {
  Settings = new StablePdSettings(frequency: 2f, damping: 1f),
  TargetPosition = new float2(0f, 3f),
  CompensateGravity = true,
});
entityManager.AddComponentData(entity, new PdPositionHistory2D());   // optional: derive TargetVelocity

entityManager.AddComponentData(entity, new PdRotation2D {
  Settings = new StablePdSettings(frequency: 3f, damping: 0.8f),
  TargetAngle = math.radians(90f),
});
```

**Then move the targets** from your own system, ordered before the backend's group:

```csharp
[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateBefore(typeof(Propulsion2DSystemGroup))]
public partial struct MoveMyTargetsSystem : ISystem
{
  public void OnUpdate(ref SystemState state)
  {
    foreach (var controller in SystemAPI.Query<RefRW<PdPosition2D>>()) {
      controller.ValueRW.TargetPosition = new float2(0f, 3f);   // wherever it should go
    }
  }
}
```

## Components

| Component | What it does |
|---|---|
| `PdPosition2D` | Drives the body's **centre of mass** towards `TargetPosition`, with a force. Has `Settings` (frequency in Hz, damping ratio), `TargetVelocity`, `Feedforward` (an extra known acceleration) and `CompensateGravity`. |
| `PdRotation2D` | Turns the body towards `TargetAngle` (radians), the short way round, with a torque. Has `Settings`, `TargetAngularVelocity` (rad/s) and `Feedforward`. |
| `PdPositionHistory2D` | **Opt-in.** Derives `PdPosition2D.TargetVelocity` every step from how far the target moved. See [Target velocity](#target-velocity-supplied-or-derived). |
| `PdRotationHistory2D` | **Opt-in.** The same for `PdRotation2D.TargetAngularVelocity`, wrap-safe. |
| `PdGainsCache` | Embedded in each controller: the gains solved for the current settings, timestep and sub-step count. Internal; never authored. |

**Choosing settings:** with damping 1, the error to a stationary target falls to about 5% after `0.75 / frequency` seconds. Damping below 1 overshoots; above 1 is sluggish. Any frequency is stable. See Propulsion Core's README for the details.

## How the forces combine

Every physics step, after ECSPhysics2D creates the step's bodies and shapes and before it simulates, `Propulsion2DSystemGroup` reads each body's state, computes the acceleration its controllers ask for, and calls `ApplyForceToCenter(mass x acceleration)` and `ApplyTorque(inertia x angular acceleration)`.

- **Nothing on the body is ever overwritten.** The force and torque go into Box2D's accumulator, next to ECSPhysics2D's `PhysicsForce`, `PhysicsImpulse`, force fields and explosions, and anything your own systems apply.
- **Order doesn't matter** among those force writers: each controller computes from the body's position and velocity, not from the other forces.
- **Other forces are disturbances the controller resists, like a spring.** A steady push `F` settles the body `F / (m K)` off target, where K is its stiffness; a kick is recovered at the tuned frequency and damping. Passive effects such as drag zones only add damping, so they can't destabilise it.
- **Mass and inertia are read from the engine** every step, so the response is right whatever the body's shapes and density.
- **Sub-steps are handled exactly.** Box2D v3 splits each step into sub-steps (4 by default), and a force is held across them. The gains are solved for the world's actual sub-step count and timestep, so the response keeps exactly the requested frequency and damping. Gains solved for one step would lose that, and at low damping can go unstable.

## Target velocity: supplied or derived

The controller tracks the target's **velocity** as well as its position. A moving target with `TargetVelocity` left at zero is trailed by a steady lag of about `speed x D / K`: roughly 0.16 m for each 1 m/s of target speed, at 2 Hz and critically damped. There are two ways to give it the velocity.

**Supply it:** write `TargetVelocity` yourself, next to `TargetPosition`. Exact, and the right choice when you know it, e.g. when following another physics body (use its velocity) or a path with a known speed.

**Derive it:** add `PdPositionHistory2D` (or `PdRotationHistory2D`). Every step, before the controllers run, the backend sets

```
TargetVelocity = (TargetPosition - previous TargetPosition) / dt
```

using the physics engine's own `dt`. You then only ever write the target position.

- **It overwrites `TargetVelocity` every step.** Any value you write there is lost while the history component is present and enabled. Remove it or disable it (`SetComponentEnabled<PdPositionHistory2D>(entity, false)`) to supply the velocity yourself.
- **After a teleport or re-target, set `Initialized = false`.** Otherwise the jump reads as one step of enormous velocity, and the body gets a violent kick. With the reset, that step derives zero and derivation resumes from the new position.
- **It's exact for steady motion,** and one step late when the target changes speed or direction.
- **Angles are wrap-safe:** a target crossing ±180° reads as a small turn, not a full spin.
- `PdBody2DAuthoring` adds the history components by default ("Derive target velocity").

## Gravity

`CompensateGravity` (on by default in authoring) cancels exactly the gravity acting on the body: the physics world's gravity times the body's gravity scale, read each step, so a negative or changing gravity scale works too. Without it, a position controller settles `g / K` below its target, about 7.6 cm at 2 Hz with 60 Hz physics.

Only gravity is cancelled. Every other force still acts on the body, which is what lets a controlled body be pushed around. A body resting on the ground with a target above it lifts off and hovers there.

## Units and conventions

- World space, metres, seconds, **radians** and radians per second. Unity's 2D physics reports angular velocity in degrees per second; the backend converts it, so everything on these components is radians.
- Positions are the body's **centre of mass**. For shapes centred on the GameObject this is its position.
- Rotation is about Z, counter-clockwise.

## ECSPhysics2D backend details

- **Timestep:** `PhysicsWorldSingleton.FixedDeltaTime`, the `dt` ECSPhysics2D simulates its worlds with. It can differ from the fixed-step group's `Time.DeltaTime`.
- **Sub-steps:** read per body from the live physics world it belongs to, so several worlds with different sub-step counts are handled.
- **Schedule:** `Propulsion2DSystemGroup` is in `FixedStepSimulationSystemGroup`, after `BuildPhysicsWorldSystem`, `ShapeCreationSystem` and `CompoundShapeBuilderSystem`, and before `PhysicsSimulationSystem`. It runs `PdTargetVelocity2DSystem`, then `PdPosition2DSystem` and `PdRotation2DSystem`.
- **Sleeping bodies** stay asleep while either the controller's feedback (its response to position and velocity error) or the net acceleration they'd feel (its output plus gravity) is negligible. So a body at rest on its target stays asleep whether it's hovering with gravity compensated, resting on the ground without it, or hanging at its sag. Moving its target wakes it.
- The systems run on the main thread, as ECSPhysics2D's own force systems do.

## Sample

**PD bodies 2D (ECSPhysics2D):** import it from the package's **Samples** tab. Three boxes under gravity:

- one holds its pose and is kicked off-centre every 3 seconds;
- one follows a circling target with derived target velocities;
- one holds against a constant wind force while snapping its heading.

Open `PdBodies2D.unity`, enable **ECS Physics 2D > Debug Draw Enabled**, and press Play.

## Tests

- **`yohash.propulsion.entities.tests.editor`** needs no physics package. It covers:
  - the gains cache;
  - the control functions under Box2D's sub-stepped integrator (exact poles, gravity, offsets under other forces, the angle wrap);
  - target-velocity derivation, including teleport resets.
- **`yohash.propulsion.entities.tests.physics2d`** compiles only with ECSPhysics2D installed. It runs the backend in a real Entities world against Box2D v3. It measures frequency and damping in the engine, and checks gravity compensation, forces combining with ECSPhysics2D's own `PhysicsForce`, and moving-target tracking.

To run them from the package, add it to your project's `testables`:

```json
"testables": ["com.yohash.propulsion.entities"]
```

## Roadmap

- **0.1.0:** the Unity Physics backend (`com.unity.physics`), alongside ECSPhysics2D.
- **0.2.0:** force and torque limits (`MaxForce`, `MaxTorque`), for actuators that can be overpowered.

## License

MIT. See [LICENSE.md](LICENSE.md) and [Third Party Notices.md](Third%20Party%20Notices.md).
