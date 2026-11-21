using UnityEngine;

namespace Yohash.Propulsion.Samples.PidExample3d
{
  public class ControllerBackwardPd : MonoBehaviour
  {
    public Transform Target;

    public Vector3 Force;

    // This controller is fed no target velocity, to contrast with ControllerStablePd which tracks it
    public StablePdController PdController;

    private Rigidbody rb;

    private void Awake()
    {
      rb = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
      if (Target == null) { return; }

      Force = PdController.ComputeForce(
        Time.fixedDeltaTime,
        rb.mass,
        rb.position,
        rb.linearVelocity,
        Target.position,
        Vector3.zero
      );

      rb.AddForce(Force);
    }
  }
}
