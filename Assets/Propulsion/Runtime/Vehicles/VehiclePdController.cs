using UnityEngine;
using Yohash.Propulsion;

[System.Serializable]
public class VehiclePdController
{
  [Header("Assign these coefficients")]
  // PD coefficients
  [SerializeField] private float proportionalGain = 1;
  [SerializeField] private float derivativeGain = 1;

  [SerializeField] private PidRotationController turnCtrl;

  /// <summary>
  /// The vehicle PD controller update function takes in the Pose of the current
  /// object being controlled, as well as its trajectory. The Target trajectory
  /// belongs to the object whose position/velocity we are attempting to match.
  /// Tweak the P and D parameters to optimize trajectory matching.
  ///
  /// The return Tuple is in the format (Acceleration, Steer), where a (+)
  /// steering value indicates turning to the right
  /// </summary>
  /// <param name="pose"></param>
  /// <param name="target"></param>
  /// <param name="current"></param>
  /// <returns></returns>
  public (float, float) Update(float dt, Pose pose, Trajectory target, Trajectory current)
  {
    // determine the position and velocity error between the current trajectory
    // of the physics body, and the desired trajectory
    var positionError = target.Position - current.Position;
    var P = positionError * proportionalGain;

    var velocityError = target.Velocity - current.Velocity;
    var D = velocityError * derivativeGain;

    // the desired acceleration as a vector-3 is the P and D term
    var desiredAccel = P + D;

    // decompose the acceleration into a steering term ("left/right") and
    // an acceleration term ("forward/backward")
    // var steer = Vector3.Dot(desiredAccel, pose.right);
    var accel = Vector3.Dot(desiredAccel, pose.forward);

    // The original algorithm calls for returning this "steer" parameter directly. In tests,
    // however, it was discovered that this leads to directly coupling the acceleration
    // and steering responsiveness of the ai vehicle. That is, if one oscillates but the other
    // does not, attempting to compensate one (by tweaking P, D) will over-compensate the other.
    // return (accel, steer);
    // To correct this issue, we use a rotation PID controller for the facing.

    // when the ai vehicle approaches a stationary target, at some point, the position
    // error and velocity error (nearly) cancel out, leaving a very small desired
    // accel term. This term can lead to turning oscillations.
    var facing = Vector3.Lerp(positionError, desiredAccel, desiredAccel.sqrMagnitude);
    var steer = turnCtrl.Update(dt, pose, facing, Vector3.up).y;

    return (accel, steer);
  }
}
