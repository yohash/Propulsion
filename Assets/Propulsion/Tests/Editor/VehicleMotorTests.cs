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

    [Test]
    public void MaxYawAccelerationAndInputConversionUseTheRigidbody()
    {
      // the instance conversion VehicleAi uses, on a real Rigidbody with a known,
      // non-identity inertia tensor
      var go = new GameObject("VehicleMotorTest");
      try {
        var rb = go.AddComponent<Rigidbody>();
        rb.automaticInertiaTensor = false;
        rb.inertiaTensor = new Vector3(2f, 5f, 3f);
        rb.inertiaTensorRotation = Quaternion.AngleAxis(30f, Vector3.forward);
        var motor = go.AddComponent<VehicleMotor>();
        motor.TurningPower = 120f;

        float expected = 120f * VehicleMotor.YawAccelerationPerUnitTorque(rb.inertiaTensorRotation, rb.inertiaTensor);
        Assert.Greater(expected, 0f);
        Assert.AreEqual(expected, motor.MaxYawAcceleration, expected * 1e-4);
        Assert.AreEqual(3f / expected, motor.YawAccelerationToInput(3f), 1e-4);
        Assert.AreEqual(-0.5f, motor.YawAccelerationToInput(-0.5f * expected), 1e-4);

        // a motor that can't turn asks for nothing, rather than dividing by zero
        motor.TurningPower = 0f;
        float input = motor.YawAccelerationToInput(3f);
        Assert.IsFalse(float.IsNaN(input) || float.IsInfinity(input));
        Assert.AreEqual(0f, input, 0.0);
      } finally {
        Object.DestroyImmediate(go);
      }
    }

    // Closed loop: a 30 degree heading step through VehicleAi's chain (controller, conversion,
    // clamp to +/-1), with a yaw model where a full input delivers `efficiency` of the
    // motor's nominal yaw acceleration.
    private const float FollowerMaxYawAcceleration = 27.9f;   // measured on the follower sample

    private static void TurnStep(bool convert, float efficiency,
      out int signFlips, out float overshootDegrees, out float finalDegrees)
    {
      var c = new VehiclePdController(1f, 1f, new StablePdController(2f, 1f));
      var target = new Trajectory { Position = Quaternion.AngleAxis(30f, Vector3.up) * Vector3.forward * 1000f };
      float yaw = 0f, yawRate = 0f, peak = 0f, lastInput = 0f;
      signFlips = 0;
      for (int i = 0; i < 150; i++) {
        float yawAcceleration = c.Update(Dt, new Pose(Vector3.zero, Quaternion.AngleAxis(yaw, Vector3.up)), target, new Trajectory()).Item2;
        float raw = convert
          ? VehicleMotor.YawAccelerationToInput(yawAcceleration, FollowerMaxYawAcceleration)
          : yawAcceleration;   // the pre-0.6 VehicleAi: clamp the rad/s^2 directly
        float input = Mathf.Clamp(raw, -1f, 1f);
        if (i > 0 && Mathf.Abs(input) > 0.05f && Mathf.Sign(input) != Mathf.Sign(lastInput)) { signFlips++; }
        lastInput = input;

        yawRate += input * FollowerMaxYawAcceleration * efficiency * Mathf.Rad2Deg * Dt;
        yaw += yawRate * Dt;
        peak = Mathf.Max(peak, yaw);
      }
      overshootDegrees = peak - 30f;
      finalDegrees = yaw;
    }

    [Test]
    public void ConvertedSteeringTurnsWithoutChatter()
    {
      TurnStep(convert: true, efficiency: 1f, out int flips, out float overshoot, out float final);
      Assert.LessOrEqual(flips, 1, "steering chattered");
      Assert.Less(overshoot, 0.1f);
      Assert.AreEqual(30f, final, 0.01);

      // the pre-0.6 direct clamp: bang-bang, flipping almost every step, and never settling
      TurnStep(convert: false, efficiency: 1f, out int oldFlips, out _, out float oldFinal);
      Assert.Greater(oldFlips, 50);
      Assert.Greater(Mathf.Abs(oldFinal - 30f), 0.5f);
    }

    [Test]
    public void GroundFrictionMakesTheTurnLessDampedButItStillSettles()
    {
      // A full input delivered 0.36 of MaxYawAcceleration on the ground in the sample. As the
      // MaxYawAcceleration docs say, that lowers the damping ratio by about sqrt(0.36) = 0.6,
      // so the critically damped turn now overshoots a little, but it settles without chatter.
      TurnStep(convert: true, efficiency: 0.36f, out int flips, out float overshoot, out float final);
      Assert.LessOrEqual(flips, 1);
      Assert.Greater(overshoot, 1f, "expected the reduced damping to show");
      Assert.Less(overshoot, 6f);
      Assert.AreEqual(30f, final, 0.05);
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
