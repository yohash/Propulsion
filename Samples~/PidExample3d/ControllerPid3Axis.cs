using UnityEngine;

namespace Yohash.Propulsion.Samples.PidExample3d
{
  public class ControllerPid3Axis : MonoBehaviour
  {
    public Transform Target;
    public float Power = 10f;

    public Vector3 Throttle;

    public PidController PidControllerX;
    public PidController PidControllerY;
    public PidController PidControllerZ;

    private Rigidbody rb;

    private void Awake()
    {
      rb = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
      if (Target == null) { return; }

      Throttle = new Vector3(
        PidControllerX.ComputeForce1D(Time.fixedDeltaTime, rb.position.x, Target.position.x),
        PidControllerY.ComputeForce1D(Time.fixedDeltaTime, rb.position.y, Target.position.y),
        PidControllerZ.ComputeForce1D(Time.fixedDeltaTime, rb.position.z, Target.position.z)
      );

      rb.AddForce(Throttle * Power);
    }
  }
}
