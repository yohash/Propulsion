using UnityEngine;

namespace Yohash.Propulsion.Samples.Rotation2d
{
  public class PidRotation2d : MonoBehaviour
  {
    [Header("Define in Inspector")]
    public Transform AimAtTarget;
    public float Power = 10;
    public float Torque;

    [Header("PID controller")]
    public PidController Pid;

    public float CurrentAngle;
    public float ToTargetAngle;

    // cached var
    private Rigidbody2D rb;

    private void Awake()
    {
      rb = GetComponent<Rigidbody2D>();
      rb.centerOfMass = Vector3.zero;
    }

    private void FixedUpdate()
    {
      if (AimAtTarget == null) { return; }

      CurrentAngle = transform.rotation.eulerAngles.z;
      var toTarget = (Vector2)(AimAtTarget.position - transform.position);
      ToTargetAngle = Vector2.SignedAngle(Vector2.up, toTarget);

      Torque = Pid.ComputeTorque1D(Time.fixedDeltaTime, CurrentAngle, ToTargetAngle);
      rb.AddTorque(Torque * Power);
    }
  }
}
