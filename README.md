# Propulsion
Scripts to enable vehicular propulsion, using physics-based systems.

## Source Material

The implementation of the PID Controller is sourced from [vazgriz](https://github.com/vazgriz), specifically, [PID_Controller](https://github.com/vazgriz/PID_Controller). 

The Stable PD Controller is based on [Stable Proportional-Derivative Controllers](http://www.jie-tan.net/project/spd.pdf) (Tan, Liu & Turk) and the "backwards" PD controller from [digitalopus](http://digitalopus.ca/site/pd-controllers/) ([UsefulUtils](https://github.com/Phong13/UsefulUtils)). Its gains are solved exactly for Unity's physics integrator, so the response has the requested frequency and damping at any timestep, and it is unconditionally stable.

## Controllers

| Controller | Use it for |
|---|---|
| `StablePdController` | Matching a target position / velocity, or rotation / angular velocity, in 1 or 3 axes. Physically parameterized (frequency in Hz, damping ratio), scaled by mass or inertia, unconditionally stable. Known forces such as gravity can be cancelled exactly through its feedforward term. |
| `PidController` | A bounded, normalized throttle with an integral term, for actuators with limited strength or unknown persistent disturbances. Hand tuned. |
| `PropulsionMath` | Shared helpers: angle wrapping, quaternion to rotation vector, angular velocity from successive rotations, angular acceleration to torque. |

## Unity Package Manager support /#upm

Add to your project via the Unity Package Manager. 
1. In the Package Manger, select "Add package from Git URL..."
2. Type in 
```
https://github.com/yohash/Propulsion.git#upm
```

The `upm` branch is maintained us a current subtree via:
```
git subtree split --prefix=Assets/Propulsion --branch upm
```
