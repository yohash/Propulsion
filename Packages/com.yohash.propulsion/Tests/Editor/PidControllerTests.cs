using System;
using NUnit.Framework;
using UnityEngine;

namespace Yohash.Propulsion.Tests
{
  public class PidControllerTests
  {
    private const float Dt = 0.02f;

    [Test]
    public void ConvergesOnUnitMass()
    {
      var pid = new PidController(4f, 0.5f, 3f, -1f, 1f, 1f);
      const float power = 10f;
      float p = 0f, v = 0f;
      for (int i = 0; i < 1500; i++) {
        v += pid.Compute(Dt, p, 2f) * power * Dt;
        p += v * Dt;
      }
      Assert.AreEqual(2f, p, 1e-2);
    }

    [Test]
    public void IntegralRemovesConstantDisturbance()
    {
      var pid = new PidController(2f, 1f, 2f, -1f, 1f, integralSaturation: 1f);
      const float power = 10f;
      const float disturbance = -3f;   // e.g. gravity this controller knows nothing about
      float p = 0f, v = 0f;
      for (int i = 0; i < 5000; i++) {
        v += (pid.Compute(Dt, p, 1f) * power + disturbance) * Dt;
        p += v * Dt;
      }
      Assert.AreEqual(1f, p, 1e-2);
    }

    [Test]
    public void IntegralContributionIsLimitedBySaturation()
    {
      // P = D = 0, so the output is exactly the integral term
      var pid = new PidController(0f, 5f, 0f, -100f, 100f, integralSaturation: 2f);
      float output = 0f;
      for (int i = 0; i < 1000; i++) {
        output = pid.Compute(Dt, 0f, 10f);
      }
      Assert.AreEqual(2f, output, 1e-4);
    }

    [Test]
    public void IntegralDoesNotWindUpWhileOutputIsSaturated()
    {
      var pid = new PidController(1f, 1f, 0f, -1f, 1f, integralSaturation: 1f);
      // a large error saturates the output via P alone; the integral must not keep charging
      for (int i = 0; i < 1000; i++) {
        pid.Compute(Dt, 0f, 100f);
      }
      // at zero error, the output is the stored integral only, which should still be tiny
      float output = pid.Compute(Dt, 0f, 0f);
      Assert.Less(Mathf.Abs(output), 0.05f);
    }

    [Test]
    public void ResetClearsAllState()
    {
      var pid = new PidController(0f, 1f, 1f, -10f, 10f, 10f);
      for (int i = 0; i < 100; i++) {
        pid.Compute(Dt, 0f, 1f);
      }
      pid.Reset();
      // zero error, no history: nothing should come out
      Assert.AreEqual(0f, pid.Compute(Dt, 5f, 5f), 1e-6);
    }

    [Test]
    public void AngleTakesShortestPathAcrossWrap()
    {
      var pid = new PidController(1f, 0f, 0f, -10f, 10f, 0f);
      // at +179, aiming for -179: the short way is +2 degrees
      Assert.AreEqual(2f, pid.ComputeAngle(Dt, 179f, -179f), 1e-4);

      pid.Reset();
      Assert.AreEqual(-2f, pid.ComputeAngle(Dt, -179f, 179f), 1e-4);

      // works for angles far outside [-180, 180] too
      pid.Reset();
      Assert.AreEqual(10f, pid.ComputeAngle(Dt, 720f, -350f), 1e-3);
    }

    [Test]
    public void AngleDerivativeIsWrapSafe()
    {
      var pid = new PidController(0f, 0f, 1f, -1000f, 1000f, 0f,
        PidController.DerivativeMeasurement.Velocity);
      pid.ComputeAngle(Dt, 179f, 0f);
      // the body moved +2 degrees across the wrap: -(2 / dt), not +(358 / dt)
      Assert.AreEqual(-2f / Dt, pid.ComputeAngle(Dt, -179f, 0f), 1e-2);
    }

    [Test]
    public void NonPositiveTimestepThrows()
    {
      var pid = new PidController();
      Assert.Throws<ArgumentOutOfRangeException>(() => pid.Compute(0f, 0f, 1f));
      Assert.Throws<ArgumentOutOfRangeException>(() => pid.ComputeAngle(-1f, 0f, 1f));
    }
  }
}
