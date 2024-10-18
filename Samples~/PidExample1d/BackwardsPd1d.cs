using UnityEngine;

namespace Yohash.Propulsion.Samples.PidExample1d
{
  public class BackwardsPd1d : MonoBehaviour
  {
    public Transform Target;

    public float Force;

    public BackwardsPdController PdController;

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
        transform.position,
        Target.transform.position,
        rb.velocity
      ).x;

      rb.AddForce(new Vector2(Force, 0));
    }
  }
}
