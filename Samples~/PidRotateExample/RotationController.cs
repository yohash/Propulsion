using UnityEngine;
using Yohash.Propulsion;

public class RotationController : MonoBehaviour
{
  public enum Lock { X, Y }

  [Header("Define in Inspector")]
  public Transform AimAtTarget;
  public float Power = 10f;

  [Header("Activate axes")]
  public bool X;
  public bool Y;
  public bool Z;

  [Header("Tune PID controllers on a per-axis basis")]
  public PIDController controllerX;
  public PIDController controllerY;
  public PIDController controllerZ;

  [Header("Output Throttle")]
  public Vector3 Throttle;

  private Rigidbody rb;

  private void Awake()
  {
    rb = GetComponent<Rigidbody>();
  }

  private void FixedUpdate()
  {
    if (AimAtTarget == null) { return; }
    //transform.rotation = Quaternion.Euler(transform.rotation.eulerAngles.x, transform.rotation.eulerAngles.y, 0f);
    var aimAtDirection = AimAtTarget.transform.position - transform.position;
    //var aimRotation = Quaternion.LookRotation(aimAtDirection, Vector3.up);
    var currentForward = transform.forward;
    var desiredForward = aimAtDirection;

    // ******************  X-axis
    // measure the x-axis (up/down: tilt) rotation delta in the local YZ-plane. Use this error
    // to determine the degrees around the x-axis we'll turn
    var inPlaneDesired = Vector3.ProjectOnPlane(aimAtDirection, transform.right);
    if (Vector3.Dot(currentForward, inPlaneDesired) < 0) {
      inPlaneDesired = Vector3.Reflect(inPlaneDesired, transform.up);
    }

    var xCurrentAngle = Vector3.SignedAngle(Vector3.up, currentForward, transform.right);
    var xDesiredAngle = Vector3.SignedAngle(Vector3.up, inPlaneDesired, transform.right);

    var xAngleDelta = Mathf.Abs(PIDController.AngleDifference(xCurrentAngle, xDesiredAngle));

    var xThrottle = new Vector3(
      controllerX.UpdateAngle(Time.fixedDeltaTime, xCurrentAngle, xDesiredAngle),
      0,
      0
    );

    // ******************  Y-axis
    // measure the Y-rotation delta (left/right turns in the XZ-plane: yaw). Use this error to determine the
    // degrees around the y-axis we'll need to turn
    var xzCurrentForward = new Vector3(currentForward.x, 0, currentForward.z);
    var xzDesiredForward = new Vector3(desiredForward.x, 0, desiredForward.z);

    var xzCurrentAngle = Vector3.SignedAngle(Vector3.forward, xzCurrentForward, Vector3.up);
    var xzDesiredAngle = Vector3.SignedAngle(Vector3.forward, xzDesiredForward, Vector3.up);

    var yAngleDelta = Mathf.Abs(PIDController.AngleDifference(xzCurrentAngle, xzDesiredAngle));

    var yThrottle = new Vector3(
      0,
      controllerY.UpdateAngle(Time.fixedDeltaTime, xzCurrentAngle, xzDesiredAngle),
      0
    );

    // ******************  Z-axis
    // measure the z-axis ("lean" left/right: roll) rotation delta in the local XY-plane. Use this
    // error to determine the degrees around the z-axis we'll rotate. To determine Z, however,
    // we need to compare the current/desired UP-vector. For X/Y axis rotation, we're matching
    // the forward-pointing vectors. For Z, we're matching the up-vectors. In this case, we want
    // Z to always be (locally) straight up, so our desired will be (0,1,0)
    var currentUp = transform.up;
    var desiredUp = Vector3.up;

    var xyCurrentUp = Vector3.ProjectOnPlane(currentUp, transform.forward);
    var xyDesiredUp = Vector3.ProjectOnPlane(desiredUp, transform.forward);
    var xyUp = Vector3.ProjectOnPlane(Vector3.up, transform.forward);

    var xyCurrentAngle = Vector3.SignedAngle(xyUp, xyCurrentUp, transform.forward);
    var xyDesiredAngle = Vector3.SignedAngle(xyUp, xyDesiredUp, transform.forward);

    var zThrottle = new Vector3(
      0,
      0,
      controllerZ.UpdateAngle(Time.fixedDeltaTime, xyCurrentAngle, xyDesiredAngle)
    );

    // ******************  Throttle
    // final throttle is the sum of all axes
    Throttle =
        (X ? xThrottle : Vector3.zero) +
        (Y ? yThrottle : Vector3.zero) +
        (Z ? zThrottle : Vector3.zero);

    rb.AddRelativeTorque(Throttle * Power);
  }
}
