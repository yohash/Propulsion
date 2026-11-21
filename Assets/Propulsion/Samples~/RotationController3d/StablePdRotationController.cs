using UnityEngine;

namespace Yohash.Propulsion.Samples.Rotation3d
{
  public class StablePdRotationController : MonoBehaviour
  {
    [Header("Define in Inspector")]
    public Transform AimAtTarget;

    public Vector3 Torque;
    public Vector3 TargetAngularVelocity;

    [Header("Stable PD controller")]
    public StablePdController stablePdController;

    // cached var
    private Rigidbody rb;
    private Quaternion lastTargetRotation;
    private bool initialized;

    private void Awake()
    {
      rb = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
      if (AimAtTarget == null) { return; }

      var aimAtDirection = AimAtTarget.position - rb.position;
      var targetRotation = Quaternion.LookRotation(aimAtDirection, Vector3.up);

      // the target's angular velocity, from successive target rotations (rad/s, world space).
      // On the first step there is no history, so assume the target is stationary.
      if (!initialized) { lastTargetRotation = targetRotation; initialized = true; }
      TargetAngularVelocity = PropulsionMath.AngularVelocity(
        lastTargetRotation,
        targetRotation,
        Time.fixedDeltaTime
      );
      lastTargetRotation = targetRotation;

      Torque = stablePdController.ComputeTorque(
        Time.fixedDeltaTime,
        rb.inertiaTensorRotation,
        rb.inertiaTensor,
        rb.rotation,
        rb.angularVelocity,
        targetRotation,
        TargetAngularVelocity
      );

      rb.AddTorque(Torque);
    }
  }
}
