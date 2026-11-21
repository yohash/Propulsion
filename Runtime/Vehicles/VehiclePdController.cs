using UnityEngine;
using Yohash.Propulsion;

/// <summary>
/// This controller was inspired by the technique outlined at
/// https://youtu.be/OcsD-1wINK0?si=cZ7uKyMrRu00W00g
/// </summary>
[System.Serializable]
public class VehiclePdController
{
  // PD coefficients
  [Header("Assign these coefficients")]
  // kp
  [SerializeField] private float proportionalGain = 1;
  // kd
  [SerializeField] private float derivativeGain = 1;

  [Header("Turning controller")]
  [SerializeField] private StablePdController turnsController;

  [SerializeField] private float currentAngle;
  [SerializeField] private float lastAngle;

  [SerializeField] private float currentTargetFacing;
  [SerializeField] private float lastTargetFacing;

  [SerializeField] private float angularVelocity;
  [SerializeField] private float targetAngularVelocity;

  // false until the first Update, so the first angular velocities are 0 rather than a jump
  // from 0 degrees. Not serialized, so a prefab or scene can never ship it already set.
  [System.NonSerialized] private bool initialized;

  // below this squared length a direction vector has no meaningful heading
  private const float MinDirectionSqrMagnitude = 1e-8f;

  public VehiclePdController() { }

  public VehiclePdController(float proportionalGain, float derivativeGain, StablePdController turnsController)
  {
    this.proportionalGain = proportionalGain;
    this.derivativeGain = derivativeGain;
    this.turnsController = turnsController;
  }

  /// <summary>
  /// Forgets the heading history, so the next Update starts from rest. Call this when the
  /// vehicle is teleported or re-targeted.
  /// </summary>
  public void Reset()
  {
    initialized = false;
  }

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
    if (dt == 0) { return (0, 0); }
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
    //var steer = Vector3.Dot(desiredAccel, pose.right);
    var accel = Vector3.Dot(desiredAccel, pose.forward);

    // The original algorithm calls for returning this "steer" parameter directly. In tests,
    // however, it was discovered that this leads to directly coupling the acceleration
    // and steering responsiveness of the ai vehicle. That is, if one oscillates but the other
    // does not, attempting to compensate one (by tweaking P, D) will over-compensate the other.
    // return (accel, steer);
    // To correct this issue, we use a stable PD controller for the facing.

    // the vehicle's heading, in degrees [-180, 180]. If it points straight up or down, its
    // flat forward has no direction, so keep the previous heading.
    var flatForward = Vector3.ProjectOnPlane(pose.forward, Vector3.up);
    float heading = flatForward.sqrMagnitude > MinDirectionSqrMagnitude
      ? Vector3.SignedAngle(Vector3.forward, flatForward, Vector3.up)
      : (initialized ? currentAngle : 0f);

    // when the ai vehicle approaches a stationary target, at some point, the position
    // error and velocity error (nearly) cancel out, leaving a very small desired
    // accel term. This term can lead to turning oscillations.
    var facing = Vector3.Lerp(positionError, desiredAccel, desiredAccel.sqrMagnitude);
    var flatFacing = Vector3.ProjectOnPlane(facing, Vector3.up);

    // Once the vehicle sits on its target with matching velocity, the facing vector is zero,
    // and SignedAngle of a zero vector returns 0 (world forward). Hold the previous target
    // facing instead (or the current heading, on the first update) rather than snapping.
    float targetFacing = flatFacing.sqrMagnitude > MinDirectionSqrMagnitude
      ? Vector3.SignedAngle(Vector3.forward, flatFacing, Vector3.up)
      : (initialized ? currentTargetFacing : heading);

    // On the first update there is no history: start from rest instead of differencing
    // against the 0 degree initial values.
    if (!initialized) {
      currentAngle = heading;
      currentTargetFacing = targetFacing;
      initialized = true;
    }

    lastAngle = currentAngle;
    lastTargetFacing = currentTargetFacing;
    currentAngle = heading;
    currentTargetFacing = targetFacing;

    // Mathf.DeltaAngle takes the short way around, so crossing +/-180 degrees is a small
    // step rather than a 360 degree jump (which would be 18,000 deg/s at 50 Hz).
    angularVelocity = Mathf.DeltaAngle(lastAngle, currentAngle) / dt;
    targetAngularVelocity = Mathf.DeltaAngle(lastTargetFacing, currentTargetFacing) / dt;

    var steer = turnsController.ComputeAngularAcceleration(
       dt,
       currentAngle * Mathf.Deg2Rad,
       angularVelocity * Mathf.Deg2Rad,
       currentTargetFacing * Mathf.Deg2Rad,
       targetAngularVelocity * Mathf.Deg2Rad
     );

    return (accel, steer);
  }
}
