using UnityEngine;

[RequireComponent(typeof(VehicleMotor))]
public class VehicleControllerAiScene : MonoBehaviour
{
  [SerializeField] private VehicleMotor motor;

  private void Awake()
  {
    motor = GetComponent<VehicleMotor>();
  }

  private void FixedUpdate()
  {
    float accel = 0;
    float turn = 0;

    // use Unity's older Input Manager (vs. Input System)
    // to avoid additional dependencies in this package
    if (Input.GetKey(KeyCode.W)) {
      accel++;
    }
    if (Input.GetKey(KeyCode.S)) {
      accel--;
    }
    if (Input.GetKey(KeyCode.A)) {
      turn--;
    }
    if (Input.GetKey(KeyCode.D)) {
      turn++;
    }

    motor.ApplyForces(accel, turn);
  }
}
