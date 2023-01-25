using UnityEngine;
using Yohash.Propulsion;

public class Controller : MonoBehaviour
{
  public Transform Target;
  public float Power = 10f;

  public Vector3 throttle;

  public PIDController ControllerX;
  public PIDController ControllerY;
  public PIDController ControllerZ;

  private Rigidbody rb;

  private void Awake()
  {
    rb = GetComponent<Rigidbody>();
  }

  private void FixedUpdate()
  {
    if (Target == null) { return; }

    throttle = new Vector3(
      ControllerX.Update(Time.fixedDeltaTime, rb.position.x, Target.position.x),
      ControllerY.Update(Time.fixedDeltaTime, rb.position.y, Target.position.y),
      ControllerZ.Update(Time.fixedDeltaTime, rb.position.z, Target.position.z)
    );

    rb.AddForce(throttle * Power);
  }
}
