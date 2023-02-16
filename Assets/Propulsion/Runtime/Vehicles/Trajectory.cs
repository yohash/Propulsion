using UnityEngine;

[System.Serializable]
public struct Trajectory
{
  public Vector3 Position;
  public Vector3 Velocity;

  public Trajectory(Rigidbody rb)
  {
    Position = rb.position;
    Velocity = rb.velocity;
  }

  public override string ToString()
  {
    return "p:" + Position.ToString() + ", v:" + Velocity.ToString();
  }
}
