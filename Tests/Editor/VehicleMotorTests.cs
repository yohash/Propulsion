using NUnit.Framework;
using UnityEngine;

namespace Yohash.Propulsion.Tests
{
  /// <summary>
  /// The conversion from VehiclePdController's desired yaw acceleration (rad/s^2) to the
  /// motor's turn input. Before 0.6 the acceleration was clamped to +/-1 directly, which
  /// saturated at about half a degree of heading error.
  /// </summary>
  public class VehicleMotorTests
  {
    private const float Dt = 0.02f;

    [Test]
    public void YawAccelerationPerUnitTorqueUsesTheUpAxisMoment()
    {
      var inertia = new Vector3(2f, 5f, 3f);

      // principal axes aligned with the body: just 1 / I_y
      Assert.AreEqual(1f / 5f, VehicleMotor.YawAccelerationPerUnitTorque(Quaternion.identity, inertia), 1e-6);

      // principal axes turned 90 degrees about X: the body's up axis is principal z
      var quarterTurn = Quaternion.AngleAxis(90f, Vector3.right);
      Assert.AreEqual(1f / 3f, VehicleMotor.YawAccelerationPerUnitTorque(quarterTurn, inertia), 1e-5);
    }

    [Test]
    public void YawAccelerationPerUnitTorqueMatchesTheInertiaTensor()
    {
      // any orientation: the up component of I^-1 * up, computed independently in principal axes
      var inertia = new Vector3(1f, 4f, 12f);
      var rotation = Quaternion.AngleAxis(30f, Vector3.up) * Quaternion.AngleAxis(45f, Vector3.right);

      var local = Quaternion.Inverse(rotation) * Vector3.up;
      var alpha = rotation * new Vector3(local.x / inertia.x, local.y / inertia.y, local.z / inertia.z);
      float expected = Vector3.Dot(alpha, Vector3.up);

      Assert.AreEqual(expected, VehicleMotor.YawAccelerationPerUnitTorque(rotation, inertia), 1e-5);
    }

    // the controller's steady yaw acceleration for a target `errorDegrees` to the right
    private static float YawAccelerationFor(float errorDegrees)
    {
      var c = new VehiclePdController(1f, 1f, new StablePdController(2f, 1f));
      var pose = new Pose(Vector3.zero, Quaternion.identity);
      var target = new Trajectory {
        Position = Quaternion.AngleAxis(errorDegrees, Vector3.up) * Vector3.forward * 10f,
      };
      c.Update(Dt, pose, target, new Trajectory());
      return c.Update(Dt, pose, target, new Trajectory()).Item2;
    }

    [Test]
    public void SmallHeadingErrorsGiveProportionalUnsaturatedSteering()
    {
      // a car like the follower sample: turningPower 120, 10 kg, a 2 m x 4 m body
      // (I_y = m (w^2 + l^2) / 12 = 16.7 kg m^2), so full turn input gives about 7.2 rad/s^2
      float maxYawAcceleration = 120f * VehicleMotor.YawAccelerationPerUnitTorque(
        Quaternion.identity, new Vector3(16.7f, 16.7f, 16.7f));

      float halfDegree = YawAccelerationFor(0.5f);
      float oneDegree = YawAccelerationFor(1f);

      // clamping the yaw acceleration directly saturated at half a degree...
      Assert.GreaterOrEqual(halfDegree, 1f, "0.5 degrees asks for over 1 rad/s^2");

      // ...converted through the motor's authority, it's a small, proportional input
      float halfInput = halfDegree / maxYawAcceleration;
      float oneInput = oneDegree / maxYawAcceleration;
      Assert.Greater(halfInput, 0f, "a target to the right steers right");
      Assert.Less(halfInput, 0.2f);
      Assert.AreEqual(2f, oneInput / halfInput, 0.02);
    }
  }
}
