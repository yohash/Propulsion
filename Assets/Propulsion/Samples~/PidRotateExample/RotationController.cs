using UnityEngine;
using Yohash.Propulsion;

public class RotationController : MonoBehaviour
{
  [Header("Define in Inspector")]
  public Transform AimAtTarget;
  public float Power = 10f;

  [Header("Output Throttle")]
  public Vector3 Torque;

  [Header("Per-axis PID rotation controller")]
  public PidRotationController pidRoteController;

  [Header("Stable backwards PD controller")]
  public BackwardsPdController backwardsPdController;

  public enum Controller
  {
    PidRotation,
    BackwardsPd,
  }
  public Controller controller = Controller.BackwardsPd;

  // cached var
  private Rigidbody rb;

  private void Awake()
  {
    rb = GetComponent<Rigidbody>();
  }

  private void FixedUpdate()
  {
    if (AimAtTarget == null) { return; }

    var aimAtDirection = AimAtTarget.transform.position - transform.position;
    var desiredRotation = Quaternion.LookRotation(aimAtDirection, Vector3.up);

    switch (controller) {
      case Controller.PidRotation:
        Torque = pidRoteController.Update(
          Time.fixedDeltaTime,
          new Pose(transform.position, transform.rotation),
          aimAtDirection,
          Vector3.up
        );
        break;

      case Controller.BackwardsPd:
        Torque = backwardsPdController.BackwardTorque(
          Time.fixedDeltaTime,
          desiredRotation,
          transform.rotation,
          rb
        );
        break;
    }

    rb.AddTorque(Torque * Power);
  }
}
