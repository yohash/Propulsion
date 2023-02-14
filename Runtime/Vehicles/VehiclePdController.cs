using UnityEngine;

public class VehiclePdController
{
  [Header("Assign these coefficients")]
  // PD coefficients
  [SerializeField] private float proportionalGain = 1;
  [SerializeField] private float derivativeGain = 1;

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
  public (float, float) Update(Pose pose, Trajectory target, Trajectory current)
  {
    // determine the position and velocity error between the current trajectory
    // of the physics body, and the desired trajectory
    var positionError = current.Position - target.Position;
    var P = positionError * proportionalGain;

    var velocityError = current.Velocity - target.Velocity;
    var D = velocityError * derivativeGain;

    // the desired acceleration as a vector-3 is the P and D term
    var desiredAccel = P + D;

    // decompose the acceleration into a steering term ("left/right") and
    // an acceleration term ("forward/backward")
    var steer = Vector3.Dot(pose.right, desiredAccel);
    var accel = Vector3.Dot(pose.forward, desiredAccel);

    return (accel, steer);
  }
}
