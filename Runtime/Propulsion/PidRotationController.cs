using System;
using UnityEngine;

namespace Yohash.Propulsion
{
  /// <summary>
  /// The PID rotation controller manages 3 individual PidController instances,
  /// one for each rotation axis.
  ///
  /// This controller attempts to take desired lookAt direction and localUp vectors,
  /// and project desired facings individually onto 3 separate axes. The output is a
  /// Vector3 of the axis-by-axis torque required to meet a given facing and up vector.
  ///
  /// This approach uses the PID angle-specific controller for each individual axis,
  /// and may not necessarily be stable for all orientations. It is best for simple
  /// rotations where you can guarantee an Up-vector that faces up
  /// </summary>
  [Serializable]
  public class PidRotationController
  {
    [Header("Activate axes")]
    public bool X;
    public bool Y;
    public bool Z;

    [Header("Tune PID controllers on a per-axis basis")]
    public PidController controllerX;
    public PidController controllerY;
    public PidController controllerZ;

    /// <summary>
    /// Returns a 3-axis Torque vector that can be applied to the to match
    /// supplied rotation info. Considers as input:
    ///   - current Transform (as Pose)
    ///   - desired "look at" vector
    ///   - desired "up" vector
    ///
    /// Make sure the output Torque vector is applied in a relative manner, that is, using:
    ///     rigidbody.AddRelativeTorque(T)
    /// and not relative to the body (ie. rigidbody.AddTorque(T))
    /// </summary>
    /// <param name="dt"></param>
    /// <param name="currentPose"></param>
    /// <param name="aimAtDirection"></param>
    /// <param name="aimAtUpVector"></param>
    public Vector3 Update(float dt, Pose currentPose, Vector3 aimAtDirection, Vector3 aimAtUpVector)
    {
      var currentForward = currentPose.forward;
      var desiredForward = aimAtDirection;

      var throttle = Vector3.zero;

      // ******************  X-axis
      // measure the x-axis (up/down: tilt) rotation delta in the local YZ-plane. Use this error
      // to determine the degrees around the x-axis we'll turn
      if (X) {
        var inPlaneDesired = Vector3.ProjectOnPlane(aimAtDirection, currentPose.right);
        if (Vector3.Dot(currentForward, inPlaneDesired) < 0) {
          inPlaneDesired = Vector3.Reflect(inPlaneDesired, currentPose.up);
        }

        var xCurrentAngle = Vector3.SignedAngle(Vector3.up, currentForward, currentPose.right);
        var xDesiredAngle = Vector3.SignedAngle(Vector3.up, inPlaneDesired, currentPose.right);

        //var xAngleDelta = Mathf.Abs(PIDController.AngleDifference(xCurrentAngle, xDesiredAngle));

        var xThrottle = new Vector3(
          controllerX.UpdateAngle(dt, xCurrentAngle, xDesiredAngle),
          0,
          0
        );

        throttle += xThrottle;
      }

      // ******************  Y-axis
      // measure the Y-rotation delta (left/right turns in the XZ-plane: yaw). Use this error to determine the
      // degrees around the y-axis we'll need to turn
      if (Y) {
        var xzCurrentForward = new Vector3(currentForward.x, 0, currentForward.z);
        var xzDesiredForward = new Vector3(desiredForward.x, 0, desiredForward.z);

        var xzCurrentAngle = Vector3.SignedAngle(Vector3.forward, xzCurrentForward, Vector3.up);
        var xzDesiredAngle = Vector3.SignedAngle(Vector3.forward, xzDesiredForward, Vector3.up);

        //var yAngleDelta = Mathf.Abs(PIDController.AngleDifference(xzCurrentAngle, xzDesiredAngle));

        var yThrottle = new Vector3(
          0,
          controllerY.UpdateAngle(dt, xzCurrentAngle, xzDesiredAngle),
          0
        );

        throttle += yThrottle;
      }

      // ******************  Z-axis
      // measure the z-axis ("lean" left/right: roll) rotation delta in the local XY-plane. Use this
      // error to determine the degrees around the z-axis we'll rotate. To determine Z, however,
      // we need to compare the current/desired UP-vector. For X/Y axis rotation, we're matching
      // the forward-pointing vectors. For Z, we're matching the up-vectors. In this case, we want
      // Z to always be (locally) straight up, so our desired will be (0,1,0)
      if (Z) {
        var currentUp = currentPose.up;
        var desiredUp = aimAtUpVector;

        var xyCurrentUp = Vector3.ProjectOnPlane(currentUp, currentPose.forward);
        var xyDesiredUp = Vector3.ProjectOnPlane(desiredUp, currentPose.forward);
        var xyUp = Vector3.ProjectOnPlane(Vector3.up, currentPose.forward);

        var xyCurrentAngle = Vector3.SignedAngle(xyUp, xyCurrentUp, currentPose.forward);
        var xyDesiredAngle = Vector3.SignedAngle(xyUp, xyDesiredUp, currentPose.forward);

        var zThrottle = new Vector3(
          0,
          0,
          controllerZ.UpdateAngle(dt, xyCurrentAngle, xyDesiredAngle)
        );

        throttle += zThrottle;
      }

      // ******************  Throttle
      // final throttle is the sum of all axes
      return throttle;
    }
  }
}
