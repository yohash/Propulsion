using UnityEngine;

namespace Yohash.Propulsion.Samples.Rotation3d
{
  public class StablePdRotationController : MonoBehaviour
  {
    [Header("Define in Inspector")]
    public Transform AimAtTarget;

    public Vector3 Torque;
    public Vector3 CurrentAimAt;
    public Vector3 LastAimAt;
    public Vector3 TargetAngularVelocity;

    [Header("Backwards PD controller")]
    public StablePdController stablePdController;

    // cached var
    private Rigidbody rb;

    private void Awake()
    {
      rb = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
      if (AimAtTarget == null) { return; }

      LastAimAt = CurrentAimAt;
      CurrentAimAt = AimAtTarget.transform.position - transform.position;
      TargetAngularVelocity = (CurrentAimAt - LastAimAt) / Time.fixedDeltaTime;

      var targetRotation = Quaternion.LookRotation(CurrentAimAt, Vector3.up);

      Torque = stablePdController.ComputeTorque(
        Time.fixedDeltaTime,
        transform.rotation,
        rb.angularVelocity,
        targetRotation,
        TargetAngularVelocity,
        rb.inertiaTensorRotation,
        rb.inertiaTensor
      );

      rb.AddTorque(Torque);
    }
  }
}
