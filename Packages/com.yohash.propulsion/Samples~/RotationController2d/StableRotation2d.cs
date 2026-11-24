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
    public StablePdController stablePdController;

    // cached var
    private Rigidbody2D rb;
    private bool initialized;

    private void Awake()
    {
      rb = GetComponent<Rigidbody2D>();
    }

    private void FixedUpdate()
    {
      if (AimAtTarget == null) { return; }

      CurrentAngle = rb.rotation;
      var toTarget = (Vector2)AimAtTarget.position - rb.position;
      // on the first step there is no history, so assume the target is stationary
      LastToTarget = initialized ? CurrentToTargetAngle : Vector2.SignedAngle(Vector2.up, toTarget);
      CurrentToTargetAngle = Vector2.SignedAngle(Vector2.up, toTarget);
      initialized = true;

      // wrap-safe: crossing +/-180 degrees does not produce a velocity spike
      TargetAngularVelocity = Mathf.DeltaAngle(LastToTarget, CurrentToTargetAngle) / Time.fixedDeltaTime;

      Torque = stablePdController.ComputeTorque(
        Time.fixedDeltaTime,
        rb.inertia,
        CurrentAngle * Mathf.Deg2Rad,
        rb.angularVelocity * Mathf.Deg2Rad,
        CurrentToTargetAngle * Mathf.Deg2Rad,
        TargetAngularVelocity * Mathf.Deg2Rad
      );

      rb.AddTorque(Torque);
    }
  }
}
