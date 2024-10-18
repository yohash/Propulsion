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
  /// 
  /// These controllers are stable if:
  ///     kd >= kp * dt
  /// or
  ///     damping >= 2 * frequency * dt 
  ///     
  /// If operating on Unity FixedDeltaTime
  /// 
  /// </summary>
  [Serializable]
  public class StablePdController
  {
    // The damping value is a scalar that determines how the system will react.
    //    damping = 1, the system is critically damped
    //    damping > 1 the system is over damped (sluggish)
    //    damping is < 1 the system is under damped (it will oscillate a little)
    [SerializeField] private float damping = 1;
    //[SerializeField] private float kp = 10;

    // Frequency is the speed of convergence.
    // If damping is 1, frequency is the 1/time taken to reach ~95% of the target value.
    // i.e.a frequency of 6 will bring you very close to your target within 1/6 seconds.
    [SerializeField] private float frequency = 4;
    //[SerializeField] private float kd = 4;

    /// <summary>
    /// This computes the acceleration needed to match the position/velocity.
    /// If you need to apply a force, scale by mass
    /// </summary>
    public Vector3 ComputeAcceleration(
      float dt,
      Vector3 currentPosition,
      Vector3 currentVelocity,
      Vector3 targetPosition,
      Vector3 targetVelocity
    )
    {
      var kp = (6f * frequency) * (6f * frequency) * 0.25f;
      var kd = 4.5f * frequency * damping;

      var p0 = currentPosition;
      var v0 = currentVelocity;
      var pt0 = targetPosition;
      var vt0 = targetVelocity;

      // approximate the next step with Taylor series
      var pt1 = pt0 + dt * vt0;
      var vt1 = vt0;
      var p1 = p0 + dt * v0;
      // Idealy we would want to use vt0 instead of v0.
      // But we would need to know the acceleration
      // for that. Assume vt0 == v0.
      var a = -kp * (p1 - pt1) - kd * (v0 - vt1);
      return a;
    }

    /// <summary>
    /// This uses the mass to compute the force necessary to match position/velocity
    /// </summary>
    public Vector3 ComputeForce(
      float dt,
      Vector3 currentPosition,
      Vector3 currentVelocity,
      Vector3 targetPosition,
      Vector3 targetVelocity,
      float mass
    )
    {
      var a = ComputeAcceleration(
        dt,
        currentPosition,
        currentVelocity,
        targetPosition,
        targetVelocity
      );
      var f = mass * a;
      return f;
    }

    /// <summary>
    /// The following comment is left in-place from the source material:
    /// https://github.com/Phong13/UsefulUtils/blob/main/digitalopus/Physics/Kinematics/SPDControllerV2.cs
    /// to provide valuable discussion and context
    /// 
    /// ------------------------ 
    /// Inertia Tensor stuff
    /// One complication with the Quaternion version of SPD is the inertia tensor.
    /// 
    ///     f = M * a
    /// 
    /// f is torque, a is angular acceleration.
    /// M is the inertia tensor and it is not a simple scalar like in the float and Vector3 version of SPD
    /// It is a matrix. However there exists a coordinate system in which the inertia tensor is a diagonal matrix.
    /// This coordinate system is called the principal axes coordinate system.
    ///     obj_inertiaTensorRotation_dyn
    /// rotates from inertia space to object local space. The axes of inertia-dynamics-space are called the principal axes coordinate system.
    ///     inertiaTensor
    /// is the diagonal entries of the matrix in the principal axes coordinate system. Basically the axes are scaled by these numbers.
    /// 
    /// Think of the inertia tensor as a coordinate transformation matrix:
    ///     - it transforms object-local space to principal-axis-space, com-is-pivot, scaled-by-inertiaTensor 
    ///     - the centerOfMass location is the pivot
    ///     - the body_intertiaTensorRotation_paxis is the rotation of the coordinate system where the matrix is diagonal
    ///     - the inertiaTensor is the scaling
    /// 
    /// 
    /// Note that because this calculator only deals with angular quantities we don't need to consider the
    /// pivot when converting between inertia space and object local space.
    /// 
    /// Now consider:
    ///     kinematics       <=>       dynamics
    ///       a  *  M         =           f
    /// 
    /// Kinematics quantities:
    ///    postition, velocity, angularVelocity, acceleration
    ///    
    /// Dynamics qunatities:
    ///    force, torque, momentum, angularMomentum
    ///    
    /// The inertia matrix converts quantites to and from objLocalSpace <=> inertiaDynamicsSpace
    ///    
    /// The SPD controller computes an accelereration in kinematic space. Then we need to use the intertiaTensor to 
    /// convert these quantities into dynamics quantities.
    /// ------------------------ 
    /// 
    /// Inputs:
    /// 
    ///     Quaternion currentRotation,
    ///     Vector3 currentAngularVelocity,
    ///     Quaternion targetRotation,
    ///     Vector3 targetAngularVelocity
    ///     
    /// Are in world-coordinates.
    /// 
    /// currentAngularVelocity and targetAngularVelocity are angular velocities,
    /// and can be extracted from the Rigidbody:
    ///     rigidbody.angularVelocity
    /// </summary>
    public Vector3 ComputeAngularAcceleration(
      float dt,
      Quaternion currentRotation,
      Vector3 currentAngularVelocity,
      Quaternion targetRotation,
      Vector3 targetAngularVelocity
    )
    {
      var kp = (6f * frequency) * (6f * frequency) * 0.25f;
      var kd = 4.5f * frequency * damping;

      var r0 = currentRotation;
      var v0 = currentAngularVelocity;
      var rt0 = targetRotation;
      var vt0 = targetAngularVelocity;

      // rt1 =  rt0 + dt * rt0
      // expected body rotation one frame in the future.
      Quaternion rt1;
      {
        float omega_t = targetAngularVelocity.magnitude;
        if (omega_t < 10e-8f) {
          rt1 = targetRotation;
        } else {
          var wld_delta_wld = Quaternion.AngleAxis(
            omega_t * dt * Mathf.Rad2Deg,
            targetAngularVelocity / omega_t
          );
          rt1 = wld_delta_wld * targetRotation;
        }
      }

      // r1 = r0 + dt * v0;
      // expected target rotation one frame in the future.
      Quaternion r1;
      {
        float omega = currentAngularVelocity.magnitude;
        if (omega < 10e-8f) {
          r1 = currentRotation;
        } else {
          var wld_delta_wld = Quaternion.AngleAxis(
            omega * dt * Mathf.Rad2Deg,
            currentAngularVelocity / omega
          );
          r1 = wld_delta_wld * currentRotation;
        }
      }

      // |delta_r| = r1 - rt1
      // difference between current angle and target angle
      float r_deltaMag;
      Vector3 r_delta;
      {
        var wld_deltaP_wld = r1 * Quaternion.Inverse(rt1);

        if (wld_deltaP_wld.w < 0f) {
          // The rotation is greater than 180 degrees (long way around the sphere). Convert to shortest path.
          wld_deltaP_wld.x = -wld_deltaP_wld.x;
          wld_deltaP_wld.y = -wld_deltaP_wld.y;
          wld_deltaP_wld.z = -wld_deltaP_wld.z;
          wld_deltaP_wld.w = -wld_deltaP_wld.w;
        }

        wld_deltaP_wld.ToAngleAxis(out r_deltaMag, out r_delta);
        r_delta.Normalize();
        r_deltaMag *= Mathf.Deg2Rad;
      }

      var a = -kp * r_delta * r_deltaMag - kd * (currentAngularVelocity - targetAngularVelocity);
      return a;
    }


    /// <summary>
    /// This computation calculates the Torque to-be-applied, and fullcomputation requires
    /// several values that can be extracted from the rigidbody.
    /// 
    /// The inputs for
    ///   - Angular Velocity
    ///   - Inertia Tensor Rotation  (rigidbody.inertiaTensorRotation)
    ///   - Inertia Tensor  (rigidbody.inertiaTensor)
    /// Can typically be extracted directly from a RigidBody or an ArticulationBody
    /// </summary>
    /// <param name="dt"></param>
    /// <param name="currentRotation"></param>
    /// <param name="currentAngularVelocity"></param>
    /// <param name="targetRotation"></param>
    /// <param name="targetAngularVelocity"></param>
    /// <param name="inertiaTensorRotation"></param>
    /// <param name="inertiaTensor"></param>
    public Vector3 ComputeTorque(
      float dt,
      Quaternion currentRotation,
      Vector3 currentAngularVelocity,
      Quaternion targetRotation,
      Vector3 targetAngularVelocity,
      Quaternion inertiaTensorRotation,
      Vector3 inertiaTensor
    )
    {
      // accelWorld is the kinematic angular acceleration in world frame.
      // Calculated using stable PD 
      var accelWorld = ComputeAngularAcceleration(
        dt,
        currentRotation,
        currentAngularVelocity,
        targetRotation,
        targetAngularVelocity
      );

      // Calc transform from world frame to objects principal axis frame
      var rotationPrincipalAxis = currentRotation * inertiaTensorRotation;

      // We need to convert the acceleration to torque in the principal axis frame.
      // In this frame the inertia matrix is diagonal. The inertiaTensor vector is the
      // non-zero diagonal entries of this matrix.
      var accelPrincipalAxis = Quaternion.Inverse(rotationPrincipalAxis) * accelWorld;

      // Now that we are in dynamics principal axis frame we can compute torque using:
      //      torque = intertiaTensor * angularAcceleration
      var torquePrincipalAxis = new Vector3(
        inertiaTensor.x * accelPrincipalAxis.x,
        inertiaTensor.y * accelPrincipalAxis.y,
        inertiaTensor.z * accelPrincipalAxis.z
      );

      // Now transform the torque from principal axis frame back to world frame.
      var torqueWorld = rotationPrincipalAxis * torquePrincipalAxis;
      return torqueWorld;
    }
  }
}