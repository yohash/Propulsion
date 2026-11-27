using NUnit.Framework;
using Unity.Mathematics;
using Yohash.Propulsion.Core;

namespace Yohash.Propulsion.Entities.Tests
{
  /// <summary>
  /// The engine-neutral control functions, driven through the sub-stepped integrator that
  /// Box2D v3 uses (4 sub-steps by default, force held across them).
  /// </summary>
  public class PdControl2DTests
  {
    private const float Dt = 1f / 60f;
    private const int SubSteps = 4;
    private static readonly float2 Gravity = new float2(0f, -9.81f);

    private static PdPosition2D Position(float frequency, float damping, float2 target, bool compensateGravity = false)
    {
      return new PdPosition2D {
        Settings = new StablePdSettings(frequency, damping),
        TargetPosition = target,
        CompensateGravity = compensateGravity,
      };
    }

    // ******************************************************************
    //    WIRING
    // ******************************************************************

    [Test]
    public void AccelerationIsTheCoreLawWithTheComponentsFields()
    {
      var pd = Position(2f, 0.7f, new float2(1f, 2f));
      pd.TargetVelocity = new float2(0.5f, -0.25f);
      pd.Feedforward = new float2(0.1f, 0.2f);
      float2 p = new float2(-1f, 3f), v = new float2(0.3f, 0.4f);

      float2 a = PdControl2D.Acceleration(ref pd, p, v, Gravity, Dt, SubSteps);

      var gains = StablePd.Gains(pd.Settings, Dt, SubSteps);
      float2 expected = StablePd.Acceleration(gains, p, v, pd.TargetPosition, pd.TargetVelocity, pd.Feedforward);
      Assert.Less(math.length(a - expected), 1e-5f);
      Assert.AreEqual(gains, pd.Cache.Gains, "the cache holds the gains it used");
    }

    [Test]
    public void FeedforwardSubtractsGravityOnlyWhenCompensating()
    {
      var pd = Position(2f, 1f, float2.zero);
      pd.Feedforward = new float2(1f, 1f);

      Assert.AreEqual(new float2(1f, 1f), PdControl2D.Feedforward(pd, Gravity));

      pd.CompensateGravity = true;
      Assert.AreEqual(new float2(1f, 1f + 9.81f), PdControl2D.Feedforward(pd, Gravity));
    }

    [Test]
    public void InvalidTimestepAppliesOnlyTheFeedforward()
    {
      var pd = Position(2f, 1f, new float2(5f, 5f), compensateGravity: true);
      float2 a = PdControl2D.Acceleration(ref pd, float2.zero, float2.zero, Gravity, 0f, SubSteps);
      Assert.AreEqual(-Gravity, a);
    }

    // ******************************************************************
    //    CLOSED LOOP, SUB-STEPPED
    // ******************************************************************

    // the error of a 2nd-order linear system obeys e[k+2] = S e[k+1] - P e[k], where S and P are
    // the sum and product of its discrete poles. Pole-matched gains make those exp(s dt).
    private static void AssertPolesAreExact(float[] error, float frequency, float damping)
    {
      double omega = 2.0 * math.PI_DBL * frequency;
      double decay = math.exp(-damping * omega * Dt);
      double omegaD = omega * math.sqrt(1.0 - damping * damping);
      double sum = 2.0 * decay * math.cos(omegaD * Dt);
      double product = decay * decay;

      for (int k = 0; k + 2 < error.Length; k++) {
        double residual = error[k + 2] - sum * error[k + 1] + product * error[k];
        Assert.Less(math.abs(residual), 2e-5 * math.max(1.0, math.abs(error[k])), $"step {k}");
      }
    }

    [TestCase(1)]
    [TestCase(2)]
    [TestCase(4)]
    [TestCase(8)]
    public void PositionHasExactlyTheRequestedFrequencyAndDampingUnderSubSteps(int subSteps)
    {
      const float frequency = 2f, damping = 0.3f;
      var pd = Position(frequency, damping, new float2(1f, 0f));
      float2 p = float2.zero, v = float2.zero;
      var error = new float[120];

      for (int k = 0; k < error.Length; k++) {
        error[k] = p.x - pd.TargetPosition.x;
        float2 a = PdControl2D.Acceleration(ref pd, p, v, float2.zero, Dt, subSteps);
        SubSteppedIntegrator.Step(ref p, ref v, a, Dt, subSteps);
      }
      AssertPolesAreExact(error, frequency, damping);
    }

    [Test]
    public void HoldsItsTargetUnderGravityWhenCompensating()
    {
      var pd = Position(2f, 1f, new float2(0f, 3f), compensateGravity: true);
      float2 p = new float2(0f, 3f), v = float2.zero;
      for (int k = 0; k < 600; k++) {
        float2 a = PdControl2D.Acceleration(ref pd, p, v, Gravity, Dt, SubSteps) + Gravity;
        SubSteppedIntegrator.Step(ref p, ref v, a, Dt, SubSteps);
      }
      Assert.Less(math.length(p - pd.TargetPosition), 1e-4f);
    }

    [Test]
    public void SagsByGravityOverStiffnessWithoutCompensation()
    {
      var pd = Position(2f, 1f, new float2(0f, 3f), compensateGravity: false);
      float2 p = new float2(0f, 3f), v = float2.zero;
      for (int k = 0; k < 600; k++) {
        float2 a = PdControl2D.Acceleration(ref pd, p, v, Gravity, Dt, SubSteps) + Gravity;
        SubSteppedIntegrator.Step(ref p, ref v, a, Dt, SubSteps);
      }
      float sag = 9.81f / pd.Cache.Gains.Stiffness;   // about 6 cm at 2 Hz
      Assert.AreEqual(3f - sag, p.y, 1e-4f);
      Assert.Greater(sag, 0.05f);
    }

    [Test]
    public void OtherForcesStillActAndSettleToAnOffset()
    {
      // e.g. a force field pushing +x on a 2 kg body: the controller resists it like a spring,
      // settling F / (m K) off target
      const float mass = 2f;
      float2 fieldForce = new float2(3f, 0f);
      var pd = Position(2f, 1f, float2.zero, compensateGravity: true);
      float2 p = float2.zero, v = float2.zero;
      for (int k = 0; k < 600; k++) {
        float2 controlForce = mass * PdControl2D.Acceleration(ref pd, p, v, Gravity, Dt, SubSteps);
        float2 a = (controlForce + fieldForce) / mass + Gravity;
        SubSteppedIntegrator.Step(ref p, ref v, a, Dt, SubSteps);
      }
      float offset = fieldForce.x / (mass * pd.Cache.Gains.Stiffness);
      Assert.AreEqual(offset, p.x, 1e-4f);
      Assert.AreEqual(0f, p.y, 1e-4f);
    }

    [Test]
    public void RotationHasExactlyTheRequestedFrequencyAndDampingUnderSubSteps()
    {
      const float frequency = 2f, damping = 0.3f;
      var pd = new PdRotation2D { Settings = new StablePdSettings(frequency, damping), TargetAngle = 1f };
      float angle = 0f, omega = 0f;
      var error = new float[120];

      for (int k = 0; k < error.Length; k++) {
        error[k] = angle - pd.TargetAngle;
        float alpha = PdControl2D.AngularAcceleration(ref pd, angle, omega, Dt, SubSteps);
        SubSteppedIntegrator.Step(ref angle, ref omega, alpha, Dt, SubSteps);
      }
      AssertPolesAreExact(error, frequency, damping);
    }

    [Test]
    public void RotationTurnsTheShortWayAcrossTheWrap()
    {
      // from 3 rad to -3 rad: the short way is +0.28 rad, through PI, not -6 rad through 0
      var pd = new PdRotation2D { Settings = new StablePdSettings(2f, 1f), TargetAngle = -3f };
      float angle = 3f, omega = 0f;
      for (int k = 0; k < 300; k++) {
        float alpha = PdControl2D.AngularAcceleration(ref pd, MotionMath.WrapAngle(angle), omega, Dt, SubSteps);
        SubSteppedIntegrator.Step(ref angle, ref omega, alpha, Dt, SubSteps);
        Assert.Greater(math.abs(MotionMath.WrapAngle(angle)), 2.9f, $"step {k}: went the long way");
      }
      Assert.AreEqual(0f, MotionMath.WrapAngle(angle - pd.TargetAngle), 1e-4f);
    }
  }
}
