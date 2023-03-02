using System;
using UnityEngine;

namespace Yohash.Propulsion
{
  /// <summary>
  /// The stable torque PD controller is a backwards PD controller that
  /// is unconditionally stable. This torque controller computes the
  /// force needed to reach the desired input in the next time step.
  ///
  /// Sourced from http://digitalopus.ca/site/pd-controllers/
  /// </summary>
  [Serializable]
  public class BackwardsPdController
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

    public Vector3 Update(
      float dt,
      Vector3 currentPosition,
      Vector3 desiredPosition,
      Vector3 currentVelocity
    )
    {
      // majority of cases the desired velocity is v = (0,0,0), as the controller is needed
      // to calculate forces necessary to move to a given position and stop there, in a stable
      // manner
      return Update(dt, currentPosition, desiredPosition, currentVelocity, Vector3.zero);
    }

    /// <summary>
    /// The backwards PD controller for position and velocity matching
    /// </summary>
    public Vector3 Update(
      float dt,
      Vector3 currentPosition,
      Vector3 desiredPosition,
      Vector3 currentVelocity,
      Vector3 desiredVelocity
    )
    {
      var kp = (6f * frequency) * (6f * frequency) * 0.25f;
      var kd = 4.5f * frequency * damping;

      float g = 1 / (1 + kd * dt + kp * dt * dt);
      float kpg = kp * g;
      float kdg = (kd + kp * dt) * g;

      Vector3 pt0 = currentPosition;
      Vector3 vt0 = currentVelocity;
      Vector3 F = (desiredPosition - pt0) * kpg + (desiredVelocity - vt0) * kdg;

      return F;
    }

    /// <summary>
    /// The input to the Torque PD controller is a desired rotation (quaternion),
    /// The current rotation (quaternion) from Transform.Rotation, and the rigidbody
    /// of the object to be controlled.
    ///
    /// The Torque vector is returned, rather than applied outright to the input rigidbody,
    /// so that controlling elements can apply clamping or axis limiting, if desired.
    ///
    /// Make sure the output Torque vector is applied in an absolute manner, that is, using:
    ///     rigidbody.AddTorque(T)
    /// and not relative to the body (ie. rigidbody.AddRelativeTorque(T))
    /// </summary>
    public Vector3 BackwardTorque(float dt, Quaternion desiredRotation, Quaternion currentRotation, Rigidbody rigidbody)
    {
      var kp = (6f * frequency) * (6f * frequency) * 0.25f;
      var kd = 4.5f * frequency * damping;

      float g = 1 / (1 + kd * dt + kp * dt * dt);
      float kpg = kp * g;
      float kdg = (kd + kp * dt) * g;

      var q = desiredRotation * Quaternion.Inverse(currentRotation);

      // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
      // We want the equivalant short rotation eg. -10 degrees
      // Check if rotation is greater than 190 degees == q.w is negative
      if (q.w < 0) {
        // Convert the quaterion to equivalent "short way around" quaterion
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
        q.w = -q.w;
      }

      q.ToAngleAxis(out var xMag, out var x);
      x.Normalize();
      x *= Mathf.Deg2Rad;

      //var pidv = kp * x * xMag - kd * rigidbody.angularVelocity;
      var pidv = kpg * x * xMag - kdg * rigidbody.angularVelocity;
      var rotInertia2World = rigidbody.inertiaTensorRotation * currentRotation;

      pidv = Quaternion.Inverse(rotInertia2World) * pidv;
      pidv.Scale(rigidbody.inertiaTensor);
      pidv = rotInertia2World * pidv;

      return pidv;
    }
  }
}
