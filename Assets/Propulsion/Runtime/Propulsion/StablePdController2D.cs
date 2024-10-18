using System;
using UnityEngine;

namespace Yohash.Propulsion
{
  /// <summary>
  /// *******************************************************************
  /// This code was adapted from
  /// https://digitalopus.ca/site/controlling-objects-with-forces-and-torques-part-2/
  /// https://github.com/Phong13/UsefulUtils
  /// *******************************************************************
  ///
  /// The stable PD controller is based on the paper:
  /// Stable Proportional-Derivative Controllers
  /// Jie Tan
  /// http://www.jie-tan.net/project/spd.pdf
  /// </summary>
  [Serializable]
  public class StablePdController2D
  {
    // The damping value is a scalar that determines how the system will react.
    //    damping = 1, the system is critically damped
    //    damping > 1 the system is over damped (sluggish)
    //    damping is < 1 the system is under damped (it will oscillate a little)
    [SerializeField] private float damping = 1;

    // Frequency is the speed of convergence.
    // If damping is 1, frequency is the 1/time taken to reach ~95% of the target value.
    // i.e.a frequency of 6 will bring you very close to your target within 1/6 seconds.
    [SerializeField] private float frequency = 4;

    /// <summary>
    /// This computes the acceleration needed to match the position/velocity.
    /// If you need to apply a force, scale by mass
    /// </summary>
    /// <param name="dt"></param>
    /// <param name="currentPosition"></param>
    /// <param name="currentVelocity"></param>
    /// <param name="targetPosition"></param>
    /// <param name="targetVelocity"></param>
    /// <returns></returns>
    public float ComputeAcceleration1D(
      float dt,
      float currentPosition,
      float currentVelocity,
      float targetPosition,
      float targetVelocity
    )
    {
      var p0 = currentPosition;
      var v0 = currentVelocity;
      var pt0 = targetPosition;
      var vt0 = targetVelocity;

      var kp = (6f * frequency) * (6f * frequency) * 0.25f;
      var kd = 4.5f * frequency * damping;

      float pt1 = pt0 + dt * vt0;
      float vt1 = vt0;
      float p1 = p0 + dt * v0;
      float a = -kp * (p1 - pt1) - kd * (v0 - vt1);
      return a;
    }

    /// <summary>
    /// This uses the mass to compute the force necessary to match position/velocity
    /// </summary>
    public float ComputeForce1D(
      float dt,
      float currentPosition,
      float currentVelocity,
      float targetPosition,
      float targetVelocity,
      float mass
    )
    {
      float a = ComputeAcceleration1D(
        dt,
        currentPosition,
        currentVelocity,
        targetPosition,
        targetVelocity
      );
      float f = mass * a;
      return f;
    }


    /// <summary>
    /// This will compute the angular acceleration necessary to rotate a body and match
    /// a provided angle and angular velocity.
    /// 
    /// Input angles are in radians.
    /// Input Angular Velocities are in radians per second
    /// 
    /// The returned angular acceleration is in radians per second per second
    /// </summary>
    public float ComputeAngularAcceleration(
      float dt,
      float currentAngle,
      float currentAngularVelocity,
      float targetAngle,
      float targetAngularVelocity
    )
    {
      var kp = (6f * frequency) * (6f * frequency) * 0.25f;
      var kd = 4.5f * frequency * damping;

      var p0 = currentAngle;
      var v0 = currentAngularVelocity;
      var pt0 = targetAngle;
      var vt0 = targetAngularVelocity;

      // approximate the next step with Taylor series
      float pt1 = pt0 + dt * vt0;
      float vt1 = vt0;
      float p1 = p0 + dt * v0;

      float angleDelta = p1 - pt1;
      if (Mathf.Abs(angleDelta) > Mathf.PI) {
        // The difference between current angle and target angle is greater than 180
        // There is a shorter route to the target.
        angleDelta = AngleDifferenceAminusB_rad(p1, pt1);
      }

      float angularAcceleration = -kp * (angleDelta) - kd * (v0 - vt1);
      return angularAcceleration;
    }

    public static float AngleDifferenceAminusB_rad(float a_rad, float b_rad)
    {
      float diff_rad = (b_rad - a_rad + Mathf.PI) % (Mathf.PI * 2f) - Mathf.PI;
      return diff_rad < -Mathf.PI ? -(diff_rad + (Mathf.PI * 2f)) : -diff_rad;
    }

    /// <summary>
    /// This will compute the angular acceleration necessary to rotate a body and match
    /// a provided angle and angular velocity.
    /// 
    /// Input angles are in radians.
    /// Input Angular Velocities are in radians per second
    /// The input moment of inertia can be extracted from the Rigidbody2d:
    ///     rigidbody2d.inertia
    /// 
    /// The returned angular acceleration is scaled by the inertia to compute a torque
    /// </summary>
    public float ComputeTorque(
      float dt,
      float currentAngle,
      float currentAngularVelocity,
      float targetAngle,
      float targetAngularVelocity,
      float momentOfInertia
    )
    {
      float a = ComputeAngularAcceleration(dt, currentAngle, currentAngularVelocity, targetAngle, targetAngularVelocity);
      float f = momentOfInertia * a;
      return f;
    }
  }
}
