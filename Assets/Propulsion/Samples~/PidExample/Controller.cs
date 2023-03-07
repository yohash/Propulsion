using UnityEngine;
using Yohash.Propulsion;

public class Controller : MonoBehaviour
{
  public Transform Target;
  public float Power = 10f;

  public Vector3 throttle;

  public PidController PidControllerX;
  public PidController PidControllerY;
  public PidController PidControllerZ;

  public BackwardsPdController PdController;

  public enum CONTROLLER { Pid3Axis, BackwardsPd}
  public CONTROLLER Drive;

  private Rigidbody rb;

  private void Awake()
  {
    rb = GetComponent<Rigidbody>();
  }

  private void FixedUpdate()
  {
    if (Target == null) { return; }

    switch (Drive) {
      case CONTROLLER.Pid3Axis:
        throttle = new Vector3(
          PidControllerX.Update(Time.fixedDeltaTime, rb.position.x, Target.position.x),
          PidControllerY.Update(Time.fixedDeltaTime, rb.position.y, Target.position.y),
          PidControllerZ.Update(Time.fixedDeltaTime, rb.position.z, Target.position.z)
        );
        break;
      case CONTROLLER.BackwardsPd:
        throttle = PdController.UpdatePosition(
          Time.fixedDeltaTime,
          transform.position,
          Target.transform.position,
          rb.velocity
        );
        break;
    }

    rb.AddForce(throttle * Power);
  }
}
