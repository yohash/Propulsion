using NUnit.Framework;
using Unity.Mathematics;

namespace Yohash.Propulsion.Core.Tests
{
  public class PidTests
  {
    private const float Dt = 0.02f;

    [Test]
    public void ConvergesOnUnitMass()
    {
      var s = new PidSettings(4f, 0.5f, 3f, -1f, 1f, 1f);
      var state = new PidState();
      const float power = 10f;
      float p = 0f, v = 0f;
      for (int i = 0; i < 1500; i++) {
        v += Pid.Compute(s, ref state, Dt, p, 2f) * power * Dt;
        p += v * Dt;
      }
      Assert.AreEqual(2f, p, 1e-2);
    }

    [Test]
    public void IntegralRemovesConstantDisturbance()
    {
      var s = new PidSettings(2f, 1f, 2f, -1f, 1f, integralSaturation: 1f);
      var state = new PidState();
      const float power = 10f;
      const float disturbance = -3f;   // e.g. gravity the controller knows nothing about
      float p = 0f, v = 0f;
      for (int i = 0; i < 5000; i++) {
        v += (Pid.Compute(s, ref state, Dt, p, 1f) * power + disturbance) * Dt;
        p += v * Dt;
      }
      Assert.AreEqual(1f, p, 1e-2);
    }

    [Test]
    public void IntegralContributionIsLimitedBySaturation()
    {
      // P = D = 0, so the output is exactly the integral term
      var s = new PidSettings(0f, 5f, 0f, -100f, 100f, integralSaturation: 2f);
      var state = new PidState();
      float output = 0f;
      for (int i = 0; i < 1000; i++) {
        output = Pid.Compute(s, ref state, Dt, 0f, 10f);
      }
      Assert.AreEqual(2f, output, 1e-4);
    }

    [Test]
    public void IntegralDoesNotWindUpWhileOutputIsSaturated()
    {
      var s = new PidSettings(1f, 1f, 0f, -1f, 1f, integralSaturation: 1f);
      var state = new PidState();
      // a large error saturates the output via P alone; the integral must not keep charging
      for (int i = 0; i < 1000; i++) {
        Pid.Compute(s, ref state, Dt, 0f, 100f);
      }
      Assert.Less(math.abs(Pid.Compute(s, ref state, Dt, 0f, 0f)), 0.05f);
    }

    [Test]
    public void ResetClearsAllState()
    {
      var s = new PidSettings(0f, 1f, 1f, -10f, 10f, 10f);
      var state = new PidState();
      for (int i = 0; i < 100; i++) {
        Pid.Compute(s, ref state, Dt, 0f, 1f);
      }
      state.Reset();
      Assert.AreEqual(default(PidState), state);
      // zero error, no history: nothing comes out
      Assert.AreEqual(0f, Pid.Compute(s, ref state, Dt, 5f, 5f), 1e-6);
    }

    [Test]
    public void AngleTakesShortestPathAcrossWrap()
    {
      var s = new PidSettings(1f, 0f, 0f, -10f, 10f, 0f);

      // at +179 deg, aiming for -179 deg: the short way is +2 deg
      var state = new PidState();
      Assert.AreEqual(math.radians(2f), Pid.ComputeAngle(s, ref state, Dt, math.radians(179f), math.radians(-179f)), 1e-4);

      state = new PidState();
      Assert.AreEqual(math.radians(-2f), Pid.ComputeAngle(s, ref state, Dt, math.radians(-179f), math.radians(179f)), 1e-4);

      // works for angles far outside [-PI, PI] too
      state = new PidState();
      Assert.AreEqual(math.radians(10f), Pid.ComputeAngle(s, ref state, Dt, math.radians(720f), math.radians(-350f)), 1e-3);
    }

    [Test]
    public void AngleDerivativeIsWrapSafe()
    {
      var s = new PidSettings(0f, 0f, 1f, -1000f, 1000f, 0f, PidDerivativeMeasurement.Velocity);
      var state = new PidState();
      Pid.ComputeAngle(s, ref state, Dt, math.radians(179f), 0f);
      // the body moved +2 degrees across the wrap: -(2 deg / dt), not +(358 deg / dt)
      Assert.AreEqual(-math.radians(2f) / Dt, Pid.ComputeAngle(s, ref state, Dt, math.radians(-179f), 0f), 1e-2);
    }

    [Test]
    public void NonPositiveTimestepReturnsZeroAndLeavesStateUntouched()
    {
      var s = PidSettings.Default;
      var state = new PidState();
      Pid.Compute(s, ref state, Dt, 0f, 1f);
      var before = state;

      Assert.AreEqual(0f, Pid.Compute(s, ref state, 0f, 0f, 1f), 0.0);
      Assert.AreEqual(0f, Pid.ComputeAngle(s, ref state, -1f, 0f, 1f), 0.0);
      Assert.AreEqual(0f, Pid.Compute(s, ref state, float.NaN, 0f, 1f), 0.0);
      Assert.AreEqual(before, state);
    }
  }
}
