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
    var relativeForward = Vector3.Dot(rb.velocity, transform.forward) * transform.forward;
    var relativeSideways = Vector3.Dot(rb.velocity, transform.right) * transform.right;

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
