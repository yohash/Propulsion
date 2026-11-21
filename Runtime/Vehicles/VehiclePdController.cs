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

  [Header("Arrival")]
  [Tooltip("Within this distance of the target (m, on the ground plane), and below Arrive Speed, " +
    "the vehicle has arrived: it holds its heading instead of steering towards the target.")]
  [SerializeField, Min(0f)] private float arriveRadius = 0.05f;
  [Tooltip("The relative speed (m/s, on the ground plane) below which the vehicle can count as arrived.")]
  [SerializeField, Min(0f)] private float arriveSpeed = 0.1f;

  [SerializeField] private float currentAngle;
  [SerializeField] private float lastAngle;

  [SerializeField] private float currentTargetFacing;
  [SerializeField] private float lastTargetFacing;

  [SerializeField] private float angularVelocity;
  [SerializeField] private float targetAngularVelocity;

  // false until the first Update, so the first angular velocities are 0 rather than a jump
  // from 0 degrees. Not serialized, so a prefab or scene can never ship it already set.
  [System.NonSerialized] private bool initialized;

  // true while the vehicle sits on its target and holds its heading
  [System.NonSerialized] private bool arrived;

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
    arrived = false;
  }

  /// <summary>
  /// The vehicle PD controller update function takes in the Pose of the current
  /// object being controlled, as well as its trajectory. The Target trajectory
  /// belongs to the object whose position/velocity we are attempting to match.
  /// Tweak the P and D parameters to optimize trajectory matching.
  ///
  /// The return Tuple is in the format (Acceleration, Steer):
  ///   - Acceleration: a throttle-like input along the vehicle's forward axis.
  ///   - Steer: the desired yaw angular acceleration, in rad/s^2, where (+) turns right.
  ///     It is a physical quantity, not a normalized input: convert it with
  ///     VehicleMotor.YawAccelerationToInput before clamping it to +/-1. Clamping it directly
  ///     saturates at a fraction of a degree of heading error, making the steering bang-bang.
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

    // Arrival: once the vehicle sits on its target with matching velocity, the facing vector
    // is only contact jitter (millimetres, in a random direction), and steering towards it
    // spins the vehicle in place. Inside the arrival dead-zone, hold the heading the vehicle
    // had on arriving. Latching the heading, rather than the last facing, also means a
    // teleport onto the target doesn't trigger a turn towards where it used to be.
    var flatPositionError = Vector3.ProjectOnPlane(positionError, Vector3.up);
    var flatVelocityError = Vector3.ProjectOnPlane(velocityError, Vector3.up);
    bool wasArrived = arrived && initialized;
    arrived = flatPositionError.magnitude < arriveRadius && flatVelocityError.magnitude < arriveSpeed;

    float targetFacing;
    if (arrived) {
      targetFacing = wasArrived ? currentTargetFacing : heading;
    } else if (flatFacing.sqrMagnitude > MinDirectionSqrMagnitude) {
      targetFacing = Vector3.SignedAngle(Vector3.forward, flatFacing, Vector3.up);
    } else {
      // a zero facing vector outside the dead-zone: SignedAngle would return world forward
      targetFacing = initialized ? currentTargetFacing : heading;
    }

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

    // A held facing is stationary. Entering or leaving the hold makes the target facing jump,
    // and that jump is a change of target, not a rotation: it must not register as a rate.
    if (arrived || wasArrived) {
      targetAngularVelocity = 0f;
    }

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
