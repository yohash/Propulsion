using UnityEngine;

[RequireComponent(typeof(VehicleMotor))]
public class VehicleController : MonoBehaviour
{
  [SerializeField] private VehicleMotor motor;

  [Header("Values determined by environment")]
  [SerializeField] private int ground;
  // with 2 wheels, we need the number of ground contacts
  // to be 2+
  private bool isGrounded {
    get { return ground >= 2; }
  }

  private void Awake()
  {
    motor = GetComponent<VehicleMotor>();
  }

  private void FixedUpdate()
  {
    // we need some sort of intelligent test to determine
    // if we've enough "wheels on the ground" to apply
    // driving forces
    if (!isGrounded) { return; }

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

  // ***********************************************************
  //			COLLISION HANDLING
  // ***********************************************************
  void OnCollisionEnter(Collision col)
  {
    // TODO - detect terrain, or viable "driving" surface
    ground++;
  }

  void OnCollisionExit(Collision col)
  {
    // TODO - detect terrain, or viable "driving" surface
    ground--;
  }
}
