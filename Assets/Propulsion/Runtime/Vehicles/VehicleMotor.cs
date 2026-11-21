using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class VehicleMotor : MonoBehaviour
{
  private Rigidbody rb;

  [Header("Declare physical paremeters")]
  // declaring a "non-physical" center of mass below the vehicle's chasis
  // can help prevent rolls/tipping
  [SerializeField] private Vector3 centerOfMassOffset = Vector3.down;

  [Header("Declare engine paremeters")]
  [SerializeField] private float enginePower;
  [SerializeField] private float turningPower;

  [Header("Declare environmental paremeters")]
  [SerializeField] private float forwardFrictionForce;
  [SerializeField] private float sidewaysFrictionForce;

  [Header("Values determined by environment")]
  [SerializeField] private int ground;
  // with 2 wheels, we need the number of ground contacts
  // to be 2+
  private bool isGrounded {
    get { return ground >= 2; }
  }

  private void Awake()
  {
    rb = GetComponent<Rigidbody>();
    rb.centerOfMass = centerOfMassOffset;
  }

  private void FixedUpdate()
  {
    // we need some sort of intelligent test to determine
    // if we've enough "wheels on the ground" to apply
    // driving forces
    if (!isGrounded) { return; }

    // every physics step, we apply environmental forces
    var relativeForward = Vector3.Dot(rb.linearVelocity, transform.forward) * transform.forward;
    var relativeSideways = Vector3.Dot(rb.linearVelocity, transform.right) * transform.right;

    var dragForward = -relativeForward * forwardFrictionForce;
    var dragSideways = -relativeSideways * sidewaysFrictionForce;

    var environmentForce = dragForward + dragSideways;

    rb.AddForce(environmentForce);
  }

  /// <summary>
  /// If the forward friction force is something determined the environment,
  /// such as changes in the road condition, we can apply those values here.
  /// </summary>
  /// <param name="f"></param>
  public void SetForwardFriction(float f)
  {
    forwardFrictionForce = f;
  }

  /// <summary>
  /// If the sideways friction force should be determined by the environment,
  /// apply those changes here. For example, an icy road should have lower
  /// friction forces.
  /// </summary>
  /// <param name="f"></param>
  public void SetSidewaysFriction(float f)
  {
    sidewaysFrictionForce = f;
  }

  /// <summary>The torque (N m, about the vehicle's local up axis) of a full turn input.</summary>
  public float TurningPower {
    get { return turningPower; }
    set { turningPower = value; }
  }

  /// <summary>
  /// The yaw angular acceleration (rad/s^2, about the vehicle's local up axis) that a full
  /// turn input of 1 produces on a free body: turningPower divided by the Rigidbody's
  /// effective moment of inertia about that axis.
  ///
  /// This is an upper bound. On the ground, wheel friction resists the turn; in the follower
  /// sample a full input delivered only about 0.36 of this from rest. To the extent friction
  /// scales the delivered turning by a factor f < 1, it lowers a turn controller's effective
  /// frequency by about sqrt(f), and its damping ratio by about sqrt(f) too (roughly 0.6x at
  /// f = 0.36), so the response is softer and less damped than tuned. Raise the controller's
  /// damping, or calibrate f, to compensate.
  /// </summary>
  public float MaxYawAcceleration {
    get {
      if (rb == null) { rb = GetComponent<Rigidbody>(); }
      return turningPower * YawAccelerationPerUnitTorque(rb.inertiaTensorRotation, rb.inertiaTensor);
    }
  }

  /// <summary>
  /// Converts a desired yaw angular acceleration (rad/s^2, positive turns right) into the turn
  /// input for ApplyForces. The result is unclamped: values beyond +/-1 ask for more than the
  /// motor can deliver, so clamp before applying.
  /// </summary>
  public float YawAccelerationToInput(float yawAcceleration)
  {
    return YawAccelerationToInput(yawAcceleration, MaxYawAcceleration);
  }

  /// <summary>
  /// The turn input that asks for `yawAcceleration` from a motor whose full input delivers
  /// `maxYawAcceleration` (both rad/s^2). Returns 0 when the motor can't turn at all.
  /// </summary>
  public static float YawAccelerationToInput(float yawAcceleration, float maxYawAcceleration)
  {
    return maxYawAcceleration > 0f ? yawAcceleration / maxYawAcceleration : 0f;
  }

  /// <summary>
  /// The angular acceleration about the body's local up axis per unit of torque applied about
  /// that axis (1 / kg m^2), from a Rigidbody's principal moments of inertia:
  ///
  ///     sum over principal axes i of  p_i^2 / I_i,    p = up, in the principal-axis frame
  ///
  /// With the principal axes aligned to the body, this is simply 1 / inertiaTensor.y.
  /// </summary>
  public static float YawAccelerationPerUnitTorque(Quaternion inertiaTensorRotation, Vector3 inertiaTensor)
  {
    var p = Quaternion.Inverse(inertiaTensorRotation) * Vector3.up;
    float k = 0f;
    if (inertiaTensor.x > 0f) { k += p.x * p.x / inertiaTensor.x; }
    if (inertiaTensor.y > 0f) { k += p.y * p.y / inertiaTensor.y; }
    if (inertiaTensor.z > 0f) { k += p.z * p.z / inertiaTensor.z; }
    return k;
  }

  /// <summary>
  /// Two inputs are acceleration and turning. Both can be positive or
  /// negative values. This method should be called during FixedUpdate().
  ///
  /// Torque considers a positive (+) value to turn right.
  /// </summary>
  /// <param name="acceleration"></param>
  /// <param name="torque"></param>
  public void ApplyForces(float acceleration, float torque)
  {
    // we need some sort of intelligent test to determine
    // if we've enough "wheels on the ground" to apply
    // driving forces
    if (!isGrounded) { return; }

    // compute acceleration force on the rigid body
    var accelerationForce = acceleration * enginePower * transform.forward;

    // compute the total torque as a rotation about the Y-axis
    var relativeTorque = torque * turningPower * Vector3.up;

    // apply the forces
    rb.AddForce(accelerationForce);
    rb.AddRelativeTorque(relativeTorque);
  }

  // ***********************************************************
  //			COLLISION HANDLING
  // ***********************************************************
  void OnCollisionEnter(Collision col)
  {
    // TODO - detect terrain, or viable "driving" surface
    ground++;
  }

  void OnCollisionExit(Collision col)
  {
    // TODO - detect terrain, or viable "driving" surface
    ground--;
  }
}
