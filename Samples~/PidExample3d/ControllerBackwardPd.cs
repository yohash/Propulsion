using UnityEngine;

namespace Yohash.Propulsion.Samples.PidExample3d
{
  public class ControllerBackwardPd : MonoBehaviour
  {
    public Transform Target;

    public Vector3 Force;

    public BackwardsPdController PdController;

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
        transform.position,
        Target.transform.position,
        rb.velocity
      );

      rb.AddForce(Force);
    }
  }
}
