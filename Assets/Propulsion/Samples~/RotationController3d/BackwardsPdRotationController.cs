using UnityEngine;

namespace Yohash.Propulsion.Samples.Rotation3d
{
  public class BackwardsPdRotationController : MonoBehaviour
  {
    [Header("Define in Inspector")]
    public Transform AimAtTarget;

    public Vector3 Torque;

    // This controller is fed no target angular velocity, to contrast with
    // StablePdRotationController which tracks it
    [Header("Stable PD controller")]
    public StablePdController backwardsPdController;

    // cached var
    private Rigidbody rb;

    private void Awake()
    {
      rb = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
      if (AimAtTarget == null) { return; }

      var aimAtDirection = AimAtTarget.position - rb.position;
      var desiredRotation = Quaternion.LookRotation(aimAtDirection, Vector3.up);

      Torque = backwardsPdController.ComputeTorque(
        Time.fixedDeltaTime,
        rb.inertiaTensorRotation,
        rb.inertiaTensor,
        rb.rotation,
        rb.angularVelocity,
        desiredRotation,
        Vector3.zero
      );

      rb.AddTorque(Torque);
    }
  }
}
