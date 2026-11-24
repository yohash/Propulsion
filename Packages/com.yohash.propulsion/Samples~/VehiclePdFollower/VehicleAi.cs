using UnityEngine;

namespace Yohash.Propulsion.Samples.VehiclePdFollower
{
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
    // the controller's desired yaw angular acceleration (rad/s^2), and the turn input it maps to
    public float yawAcceleration;
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

      (accel, yawAcceleration) = controller.Update(
        Time.fixedDeltaTime,
        new Pose(transform.position, transform.rotation),
        new Trajectory(target),
        new Trajectory(rb)
      );

      // the controller asks for a yaw acceleration; the motor knows how much one unit of turn
      // input delivers, so steering stays proportional until the car's turning limit
      accel = Mathf.Clamp(accel, -1, 1);
      steer = Mathf.Clamp(motor.YawAccelerationToInput(yawAcceleration), -1, 1);

      motor.ApplyForces(accel, steer);
    }
  }
}
