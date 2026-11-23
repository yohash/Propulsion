using UnityEngine;

namespace Yohash.Propulsion.Samples.PidExample1d
{
  public class StablePd1d : MonoBehaviour
  {
    public Transform Target;

    public float Force;

    public StablePdController XStableController;
    public float TargetPosition;
    public float TargetLastPosition;
    public float TargetVelocity;

    private Rigidbody2D rb;
    private bool initialized;

    private void Awake()
    {
      rb = GetComponent<Rigidbody2D>();
    }

    private void FixedUpdate()
    {
      if (Target == null) { return; }

      // on the first step there is no history, so assume the target is stationary
      TargetLastPosition = initialized ? TargetPosition : Target.position.x;
      TargetPosition = Target.position.x;
      TargetVelocity = (TargetPosition - TargetLastPosition) / Time.fixedDeltaTime;
      initialized = true;

      Force = XStableController.ComputeForce(
        Time.fixedDeltaTime,
        rb.mass,
        rb.position.x,
        rb.linearVelocity.x,
        TargetPosition,
        TargetVelocity
      );

      rb.AddForce(new Vector2(Force, 0));
    }
  }
}
