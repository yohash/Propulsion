using UnityEngine;

namespace Yohash.Propulsion.Samples.Rotation2d
{
  public class StableRotation2d : MonoBehaviour
  {
    [Header("Define in Inspector")]
    public Transform AimAtTarget;

    public float CurrentAngle;
    public float CurrentToTargetAngle;
    public float LastToTarget;
    public float TargetAngularVelocity;

    public float Torque;

    [Header("Stable PD controller")]
    public StablePdController2D stablePdController;

    // cached var
    private Rigidbody2D rb;

    private void Awake()
    {
      rb = GetComponent<Rigidbody2D>();
    }

    private void FixedUpdate()
    {
      if (AimAtTarget == null) { return; }

      LastToTarget = CurrentToTargetAngle;

      CurrentAngle = transform.rotation.eulerAngles.z;
      var toTarget = (Vector2)(AimAtTarget.position - transform.position);
      CurrentToTargetAngle = Vector2.SignedAngle(Vector2.up, toTarget);

      TargetAngularVelocity = (CurrentToTargetAngle - LastToTarget) / Time.fixedDeltaTime;

      Torque = stablePdController.ComputeTorque(
        Time.fixedDeltaTime,
        CurrentAngle * Mathf.Deg2Rad,
        rb.angularVelocity * Mathf.Deg2Rad,
        CurrentToTargetAngle * Mathf.Deg2Rad,
        TargetAngularVelocity * Mathf.Deg2Rad,
        rb.inertia
      );

      rb.AddTorque(Torque);
    }
  }
}
