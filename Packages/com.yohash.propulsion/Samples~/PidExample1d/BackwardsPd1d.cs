using UnityEngine;

namespace Yohash.Propulsion.Samples.PidExample1d
{
  public class BackwardsPd1d : MonoBehaviour
  {
    public Transform Target;

    public float Force;

    // This controller is fed no target velocity, to contrast with StablePd1d which tracks it
    public StablePdController PdController;

    private Rigidbody2D rb;

    private void Awake()
    {
      rb = GetComponent<Rigidbody2D>();
    }

    private void FixedUpdate()
    {
      if (Target == null) { return; }

      Force = PdController.ComputeForce(
        Time.fixedDeltaTime,
        rb.mass,
        rb.position.x,
        rb.linearVelocity.x,
        Target.position.x,
        0f
      );

      rb.AddForce(new Vector2(Force, 0));
    }
  }
}
