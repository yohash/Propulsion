# Propulsion
Scripts to enable vehicular propulsion, using physics-based systems.

The Unity package lives in [`Assets/Propulsion`](Assets/Propulsion). See its [README](Assets/Propulsion/README.md) for usage, and the [CHANGELOG](Assets/Propulsion/CHANGELOG.md) for release notes and migration guides.

## Source Material

The implementation of the PID Controller is sourced from [vazgriz](https://github.com/vazgriz), specifically, [PID_Controller](https://github.com/vazgriz/PID_Controller). 

The Stable PD Controller is based on [Stable Proportional-Derivative Controllers](http://www.jie-tan.net/project/spd.pdf) (Tan, Liu & Turk) and the "backwards" PD controller from [digitalopus](http://digitalopus.ca/site/pd-controllers/) ([UsefulUtils](https://github.com/Phong13/UsefulUtils)). Its gains are solved exactly for Unity's physics integrator, so the response has the requested frequency and damping at any timestep, and it is unconditionally stable.

## Controllers

| Controller | Use it for |
|---|---|
| `StablePdController` | Matching a target position / velocity, or rotation / angular velocity, in 1 or 3 axes. Physically parameterized (frequency in Hz, damping ratio), scaled by mass or inertia, unconditionally stable. Known forces such as gravity can be cancelled exactly through its feedforward term. |
| `PidController` | A bounded, normalized throttle with an integral term, for actuators with limited strength or unknown persistent disturbances. Hand tuned. |
| `PropulsionMath` | Shared helpers: angle wrapping, quaternion to rotation vector, angular velocity from successive rotations, angular acceleration to torque. |

## Reference material

Papers and other background material live in [`Documentation/`](Documentation). They are kept out of the Unity package itself.

## Installation (Unity Package Manager)

Requires Unity 2022.3 or newer.

1. In the Package Manager, select **+ > Add package from git URL...**
2. Enter:
```
https://github.com/yohash/Propulsion.git#upm
```

## Releasing

The `upm` branch holds only the package folder, maintained as a subtree split of `Assets/Propulsion`. After merging a release into `main`:

1. Set the release date on its `CHANGELOG.md` entry (replacing `Unreleased`) and commit.
2. Update the `upm` branch and push it:
```
git subtree split --prefix=Assets/Propulsion --branch upm
git push origin upm
```
