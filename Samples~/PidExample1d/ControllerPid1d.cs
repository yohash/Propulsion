using UnityEngine;

namespace Yohash.Propulsion.Samples.PidExample1d
{
  public class ControllerPid1d : MonoBehaviour
  {
    public Transform Target;
    public float Power = 10f;
    public float Throttle;

    public PidController PidControllerX;

    private Rigidbody2D rb;

    private void Awake()
    {
      rb = GetComponent<Rigidbody2D>();
    }

    private void FixedUpdate()
    {
      if (Target == null) { return; }

      Throttle = PidControllerX.ComputeForce1D(Time.fixedDeltaTime, rb.position.x, Target.position.x);

      rb.AddForce(new Vector2(Throttle, 0) * Power);
    }
  }
}
