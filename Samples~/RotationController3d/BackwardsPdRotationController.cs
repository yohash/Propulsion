using UnityEngine;

namespace Yohash.Propulsion.Samples.Rotation3d
{
  public class BackwardsPdRotationController : MonoBehaviour
  {
    [Header("Define in Inspector")]
    public Transform AimAtTarget;

    public Vector3 Torque;

    [Header("Backwards PD controller")]
    public BackwardsPdController backwardsPdController;

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

      Torque = backwardsPdController.ComputeTorque(
        Time.fixedDeltaTime,
        desiredRotation,
        transform.rotation,
        rb.angularVelocity,
        rb.inertiaTensorRotation,
        rb.inertiaTensor
      );

      rb.AddTorque(Torque);
    }
  }
}
