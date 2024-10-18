using UnityEngine;

namespace Yohash.Propulsion.Samples.PidExample1d
{
  public class StablePd1d : MonoBehaviour
  {
    public Transform Target;

    public float Force;

    public StablePdController2D XStableController;
    public float TargetPosition;
    public float TargetLastPosition;
    public float TargetVelocity;

    private Rigidbody2D rb;

    private void Awake()
    {
      rb = GetComponent<Rigidbody2D>();
    }

    private void FixedUpdate()
    {
      if (Target == null) { return; }

      TargetLastPosition = TargetPosition;
      TargetPosition = Target.position.x;
      TargetVelocity = (TargetPosition - TargetLastPosition) / Time.fixedDeltaTime;

      var current = transform.position.x;
      var velocity = rb.velocity.x;

      Force = XStableController.ComputeForce1D(
        Time.fixedDeltaTime,
        current,
        velocity,
        TargetPosition,
        TargetVelocity,
        rb.mass
      );

      rb.AddForce(new Vector2(Force, 0));
    }
  }
}
