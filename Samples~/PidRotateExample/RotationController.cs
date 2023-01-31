using UnityEngine;
using Yohash.Propulsion;

public class RotationController : MonoBehaviour
{
  [Header("Define in Inspector")]
  public Transform AimAtTarget;
  public float Power = 10f;

  [Header("Output Throttle")]
  public Vector3 Throttle;

  [Header("Tune controller rotational axes")]
  public PidRotationController controller;


  // cached var
  private Rigidbody rb;

  private void Awake()
  {
    rb = GetComponent<Rigidbody>();
  }

  private void FixedUpdate()
  {
    if (AimAtTarget == null) { return; }

    var aimAtDirection = AimAtTarget.transform.position - transform.position;

    Throttle = controller.Update(
      Time.fixedDeltaTime,
      new Pose(transform.position, transform.rotation),
      aimAtDirection,
      Vector3.up
    );

    rb.AddRelativeTorque(Throttle * Power);
  }
}
