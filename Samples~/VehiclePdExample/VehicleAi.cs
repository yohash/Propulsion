using UnityEngine;
using Yohash.Propulsion;

[RequireComponent(typeof(VehicleMotor))]
public class VehicleAi : MonoBehaviour
{
  private Rigidbody rb;
  [SerializeField] private VehicleMotor motor;

  [Header("Assign Target")]
  [SerializeField] private Rigidbody target;

  [Header("Assign controller values")]
  [SerializeField] private VehiclePdController controller;

  public float accel;
  public float steer;

  private void Awake()
  {
    motor = GetComponent<VehicleMotor>();
    rb = GetComponent<Rigidbody>();
  }

  private void FixedUpdate()
  {
    if (target == null) { return; }

    foreach (var col1 in GetComponentsInChildren<Collider>()) {
      foreach (var col2 in target.GetComponentsInChildren<Collider>()) {
        Physics.IgnoreCollision(col1, col2);
      }
    }

    (accel, steer) = controller.Update(
      Time.fixedDeltaTime,
      new Pose(transform.position, transform.rotation),
      new Trajectory(target),
      new Trajectory(rb)
    );

    accel = Mathf.Clamp(accel, -1, 1);
    steer = Mathf.Clamp(steer, -1, 1);

    motor.ApplyForces(accel, steer);
  }
}
