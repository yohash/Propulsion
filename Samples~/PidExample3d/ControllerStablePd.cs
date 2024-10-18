using UnityEngine;

namespace Yohash.Propulsion.Samples.PidExample3d
{
  public class ControllerStablePd : MonoBehaviour
  {
    public Transform Target;

    public Vector3 Force;

    public StablePdController StableController;
    public Vector3 TargetPosition;
    public Vector3 TargetLastPosition;
    public Vector3 TargetVelocity;

    private Rigidbody rb;

    private void Awake()
    {
      rb = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
      if (Target == null) { return; }

      TargetLastPosition = TargetPosition;
      TargetPosition = Target.position;
      TargetVelocity = (TargetPosition - TargetLastPosition) / Time.fixedDeltaTime;

      Force = StableController.ComputeForce(
        Time.fixedDeltaTime,
        transform.position,
        rb.velocity,
        Target.transform.position,
        TargetVelocity,
        rb.mass
      );

      rb.AddForce(Force);
    }
  }
}
