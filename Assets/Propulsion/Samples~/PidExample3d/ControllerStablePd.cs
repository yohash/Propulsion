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
    private bool initialized;

    private void Awake()
    {
      rb = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
      if (Target == null) { return; }

      // on the first step there is no history, so assume the target is stationary
      TargetLastPosition = initialized ? TargetPosition : Target.position;
      TargetPosition = Target.position;
      TargetVelocity = (TargetPosition - TargetLastPosition) / Time.fixedDeltaTime;
      initialized = true;

      Force = StableController.ComputeForce(
        Time.fixedDeltaTime,
        rb.mass,
        rb.position,
        rb.linearVelocity,
        TargetPosition,
        TargetVelocity
      );

      rb.AddForce(Force);
    }
  }
}
