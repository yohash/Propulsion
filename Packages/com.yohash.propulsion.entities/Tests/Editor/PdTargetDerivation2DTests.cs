using NUnit.Framework;
using Unity.Mathematics;
using Yohash.Propulsion.Core;

namespace Yohash.Propulsion.Entities.Tests
{
  public class PdTargetDerivation2DTests
  {
    private const float Dt = 1f / 60f;
    private const int SubSteps = 4;

    [Test]
    public void FirstStepDerivesZeroAndRecordsTheTarget()
    {
      var pd = new PdPosition2D { TargetPosition = new float2(4f, 5f), TargetVelocity = new float2(9f, 9f) };
      var history = new PdPositionHistory2D();

      PdTargetDerivation2D.DeriveVelocity(ref pd, ref history, Dt);

      Assert.AreEqual(float2.zero, pd.TargetVelocity);
      Assert.AreEqual(new float2(4f, 5f), history.PreviousTarget);
      Assert.IsTrue(history.Initialized);
    }

    [Test]
    public void ConstantVelocityIsDerivedExactly()
    {
      var pd = new PdPosition2D();
      var history = new PdPositionHistory2D();
      float2 velocity = new float2(1.5f, -2f);
      for (int k = 0; k < 10; k++) {
        pd.TargetPosition = velocity * (k * Dt);
        PdTargetDerivation2D.DeriveVelocity(ref pd, ref history, Dt);
      }
      Assert.Less(math.length(pd.TargetVelocity - velocity), 1e-3f);
    }

    [Test]
    public void ResettingAfterATeleportDerivesZeroInsteadOfAKick()
    {
      var pd = new PdPosition2D();
      var history = new PdPositionHistory2D();
      PdTargetDerivation2D.DeriveVelocity(ref pd, ref history, Dt);

      // without the reset, a 100 m jump reads as 6000 m/s for one step
      pd.TargetPosition = new float2(100f, 0f);
      var notReset = history;
      var notResetPd = pd;
      PdTargetDerivation2D.DeriveVelocity(ref notResetPd, ref notReset, Dt);
      Assert.Greater(notResetPd.TargetVelocity.x, 1000f);

      history.Initialized = false;
      PdTargetDerivation2D.DeriveVelocity(ref pd, ref history, Dt);
      Assert.AreEqual(float2.zero, pd.TargetVelocity);

      // and derivation resumes normally from the new position
      pd.TargetPosition = new float2(100f + Dt, 0f);
      PdTargetDerivation2D.DeriveVelocity(ref pd, ref history, Dt);
      Assert.AreEqual(1f, pd.TargetVelocity.x, 1e-3f);
    }

    [Test]
    public void InvalidTimestepChangesNothing()
    {
      var pd = new PdPosition2D { TargetPosition = new float2(1f, 1f), TargetVelocity = new float2(7f, 7f) };
      var history = new PdPositionHistory2D { PreviousTarget = new float2(-3f, 2f), Initialized = true };
      var rotation = new PdRotation2D { TargetAngle = 1f, TargetAngularVelocity = 7f };
      var rotationHistory = new PdRotationHistory2D { PreviousTarget = -1f, Initialized = true };
      var (pdBefore, historyBefore, rotationBefore, rotationHistoryBefore) = (pd, history, rotation, rotationHistory);

      foreach (float dt in new[] { 0f, -Dt, float.NaN }) {
        PdTargetDerivation2D.DeriveVelocity(ref pd, ref history, dt);
        PdTargetDerivation2D.DeriveAngularVelocity(ref rotation, ref rotationHistory, dt);
      }
      Assert.AreEqual(pdBefore, pd);
      Assert.AreEqual(historyBefore, history);
      Assert.AreEqual(rotationBefore, rotation);
      Assert.AreEqual(rotationHistoryBefore, rotationHistory);
    }

    [Test]
    public void AngularVelocityTakesTheShortWayAcrossTheWrap()
    {
      var pd = new PdRotation2D { TargetAngle = math.radians(179f) };
      var history = new PdRotationHistory2D();
      PdTargetDerivation2D.DeriveAngularVelocity(ref pd, ref history, Dt);
      Assert.AreEqual(0f, pd.TargetAngularVelocity);

      // 179 -> -179 degrees is a 2 degree turn, not a 358 degree spin
      pd.TargetAngle = math.radians(-179f);
      PdTargetDerivation2D.DeriveAngularVelocity(ref pd, ref history, Dt);
      Assert.AreEqual(math.radians(2f) / Dt, pd.TargetAngularVelocity, 1e-2f);
    }

    [Test]
    public void ResettingTheRotationHistoryDerivesZero()
    {
      var pd = new PdRotation2D { TargetAngle = 0f };
      var history = new PdRotationHistory2D();
      PdTargetDerivation2D.DeriveAngularVelocity(ref pd, ref history, Dt);

      pd.TargetAngle = 2f;
      history.Initialized = false;
      PdTargetDerivation2D.DeriveAngularVelocity(ref pd, ref history, Dt);
      Assert.AreEqual(0f, pd.TargetAngularVelocity);
      Assert.AreEqual(2f, history.PreviousTarget);
    }

    // ******************************************************************
    //    CLOSED LOOP: WHY THE HISTORY EXISTS
    // ******************************************************************

    private static float2 TrackMovingTarget(bool derive, out float stiffness, out float damper)
    {
      float2 targetVelocity = new float2(1f, 0f);
      var pd = new PdPosition2D { Settings = new StablePdSettings(2f, 1f) };
      var history = new PdPositionHistory2D();
      float2 p = float2.zero, v = float2.zero;

      for (int k = 0; k < 900; k++) {
        pd.TargetPosition = targetVelocity * (k * Dt);
        if (derive) { PdTargetDerivation2D.DeriveVelocity(ref pd, ref history, Dt); }
        float2 a = PdControl2D.Acceleration(ref pd, p, v, float2.zero, Dt, SubSteps);
        SubSteppedIntegrator.Step(ref p, ref v, a, Dt, SubSteps);
      }
      stiffness = pd.Cache.Gains.Stiffness;
      damper = pd.Cache.Gains.Damper;
      // the error at the start of the next step, against the target it will then have
      return p - targetVelocity * (900 * Dt);
    }

    [Test]
    public void DerivedVelocityTracksAMovingTargetWithoutLag()
    {
      float2 error = TrackMovingTarget(derive: true, out _, out _);
      Assert.Less(math.length(error), 1e-3f);
    }

    [Test]
    public void WithoutDerivationAMovingTargetIsTrailedBySpeedTimesDOverK()
    {
      float2 error = TrackMovingTarget(derive: false, out float stiffness, out float damper);
      // steady state: zero acceleration, so K e + D v_target = 0
      float expected = -1f * damper / stiffness;
      Assert.AreEqual(expected, error.x, 2e-3f);
      Assert.Less(expected, -0.1f, "the lag is large enough to matter");
    }
  }
}
