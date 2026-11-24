using System;
using NUnit.Framework;
using Unity.Mathematics;

namespace Yohash.Propulsion.Core.Tests
{
  /// <summary>
  /// These tests drive StablePd with the integrator the physics engines use (semi-implicit Euler,
  /// optionally split into n sub-steps with the acceleration held constant across them), so they
  /// check the documented physical claims without a scene or a physics package.
  /// </summary>
  public class StablePdTests
  {
    private const float Dt = 0.02f;

    // one physics step of length dt, in n sub-steps, under a constant acceleration
    private static void Step(ref float p, ref float v, float a, float dt, int subSteps)
    {
      float h = dt / subSteps;
      for (int i = 0; i < subSteps; i++) {
        v += a * h;
        p += v * h;
      }
    }

    private static void Step(ref float3 p, ref float3 v, float3 a, float dt, int subSteps)
    {
      float h = dt / subSteps;
      for (int i = 0; i < subSteps; i++) {
        v += a * h;
        p += v * h;
      }
    }

    // release a unit-mass body from rest at `start`, towards a stationary target at 0
    private static float Simulate1D(
      in StablePdGains gains, int subSteps, float seconds, float start, out float maxAbsPosition,
      float externalAcceleration = 0f, float feedforward = 0f)
    {
      float p = start, v = 0f;
      maxAbsPosition = math.abs(p);
      int steps = (int)math.ceil(seconds / gains.Dt);
      for (int i = 0; i < steps; i++) {
        float a = StablePd.Acceleration(gains, p, v, 0f, 0f, feedforward) + externalAcceleration;
        Step(ref p, ref v, a, gains.Dt, subSteps);
        maxAbsPosition = math.max(maxAbsPosition, math.abs(p));
      }
      return p;
    }

    // trace and determinant of the closed loop's one-step map, measured by simulating it
    private static void MeasuredMap(in StablePdGains g, int subSteps, out double trace, out double determinant)
    {
      float p1 = 1f, v1 = 0f;
      Step(ref p1, ref v1, StablePd.Acceleration(g, 1f, 0f, 0f, 0f), g.Dt, subSteps);
      float p2 = 0f, v2 = 1f;
      Step(ref p2, ref v2, StablePd.Acceleration(g, 0f, 1f, 0f, 0f), g.Dt, subSteps);
      trace = (double)p1 + v2;
      determinant = (double)p1 * v2 - (double)p2 * v1;
    }

    // z1 + z2 and z1 * z2 for the poles exp(s dt) of the requested continuous response
    private static void TargetPoles(float frequency, float damping, float dt, out double sum, out double product)
    {
      double w = 2.0 * Math.PI * frequency;
      product = Math.Exp(-2.0 * damping * w * dt);
      if (damping < 1f) {
        sum = 2.0 * Math.Exp(-damping * w * dt) * Math.Cos(w * Math.Sqrt(1.0 - damping * damping) * dt);
      } else if (damping == 1f) {
        sum = 2.0 * Math.Exp(-w * dt);
      } else {
        double r = Math.Sqrt(damping * damping - 1.0);
        sum = Math.Exp(-w * (damping - r) * dt) + Math.Exp(-w * (damping + r) * dt);
      }
    }

    private static double LargestPole(double trace, double determinant)
    {
      double disc = trace * trace - 4.0 * determinant;
      if (disc >= 0) {
        double r = Math.Sqrt(disc);
        return Math.Max(Math.Abs((trace + r) / 2), Math.Abs((trace - r) / 2));
      }
      return Math.Sqrt(determinant);   // complex pair: |z|^2 = determinant
    }

    // ******************************************************************
    //    GAINS
    // ******************************************************************

    [Test]
    public void SettingsGainsAreTheContinuousOnes()
    {
      var s = new StablePdSettings(2f, 0.5f);
      float omega = 2f * math.PI * 2f;
      Assert.AreEqual(omega * omega, s.Kp, 1e-3);
      Assert.AreEqual(2f * 0.5f * omega, s.Kd, 1e-4);
    }

    [Test]
    public void DiscreteGainsTendToTheContinuousOnesAsDtShrinks([Values(1, 4)] int subSteps)
    {
      var s = new StablePdSettings(2f, 0.5f);
      var g = StablePd.Gains(s, 1e-4f, subSteps);
      Assert.AreEqual(s.Kp, g.Stiffness, s.Kp * 2e-3);
      Assert.AreEqual(s.Kd, g.Damper, s.Kd * 2e-3);
    }

    [Test]
    public void DiscretePolesExactlyMatchContinuousPoles(
      [Values(1, 2, 4, 8)] int subSteps,
      [Values(0.5f, 2f, 8f)] float frequency,
      [Values(0.2f, 1f, 3f)] float damping,
      [Values(0.005f, 0.02f)] float dt
    )
    {
      var g = StablePd.Gains(new StablePdSettings(frequency, damping), dt, subSteps);
      MeasuredMap(g, subSteps, out double trace, out double determinant);
      TargetPoles(frequency, damping, dt, out double sum, out double product);

      Assert.AreEqual(product, determinant, 2e-5);
      Assert.AreEqual(sum, trace, 2e-5);
    }

    [Test]
    public void SubStepsOnlyIncreaseTheDamper()
    {
      // K is the same for any sub-step count; D = D(1 step) + (1 - c) K dt
      var s = new StablePdSettings(4f, 0.7f);
      var one = StablePd.Gains(s, Dt, 1);
      var four = StablePd.Gains(s, Dt, 4);
      Assert.AreEqual(one.Stiffness, four.Stiffness, 1e-3);
      Assert.AreEqual(one.Damper + (1f - 5f / 8f) * one.Stiffness * Dt, four.Damper, 1e-3);
    }

    [Test]
    public void OneStepGainsLoseTheirTuningUnderSubSteps()
    {
      // Why Gains takes a sub-step count: gains solved for 1 step, run under 4 sub-steps...
      // ...at 10 Hz, damping 0.1, go unstable,
      var low = StablePd.Gains(new StablePdSettings(10f, 0.1f), Dt, 1);
      MeasuredMap(low, 4, out double trace, out double determinant);
      Assert.Greater(LargestPole(trace, determinant), 1.0, "expected the mismatched gains to diverge");

      // while the same settings solved for 4 sub-steps are stable
      var matched = StablePd.Gains(new StablePdSettings(10f, 0.1f), Dt, 4);
      MeasuredMap(matched, 4, out trace, out determinant);
      Assert.Less(LargestPole(trace, determinant), 1.0);
    }

    [Test]
    public void InvalidInputGivesZeroGainsAndNoFeedback()
    {
      var s = StablePdSettings.Default;
      foreach (var g in new[] { StablePd.Gains(s, 0f), StablePd.Gains(s, -Dt), StablePd.Gains(s, float.NaN), StablePd.Gains(s, Dt, 0) }) {
        Assert.IsFalse(g.IsValid);
        // only the feedforward term remains
        Assert.AreEqual(2.5f, StablePd.Acceleration(g, 10f, 3f, 0f, 0f, 2.5f), 0.0);
        Assert.AreEqual(0f, math.length(StablePd.AngularAcceleration(g, quaternion.identity, new float3(1f), quaternion.AxisAngle(new float3(0f, 1f, 0f), 1f), float3.zero)), 0.0);
      }
      Assert.IsTrue(StablePd.Gains(s, Dt).IsValid);
    }

    // ******************************************************************
    //    LINEAR RESPONSE
    // ******************************************************************

    [Test]
    public void CriticallyDampedStepSettlesWithoutOvershoot([Values(1, 4)] int subSteps)
    {
      var g = StablePd.Gains(new StablePdSettings(1f, 1f), Dt, subSteps);

      // documented: the error falls to ~5% after ~0.75 / frequency seconds
      Assert.Less(math.abs(Simulate1D(g, subSteps, 0.75f, 1f, out _)), 0.08f);

      float p = 1f, v = 0f, minP = 1f;
      for (int i = 0; i < 500; i++) {
        Step(ref p, ref v, StablePd.Acceleration(g, p, v, 0f, 0f), Dt, subSteps);
        minP = math.min(minP, p);
      }
      Assert.Less(math.abs(p), 1e-4f);
      Assert.Greater(minP, -0.01f, "a critically damped response should not overshoot");
    }

    [Test]
    public void UnderdampedResponseMatchesContinuousTime()
    {
      // e(t) = exp(-zeta w t) (cos(wd t) + zeta w / wd sin(wd t)) for a release from rest
      const float frequency = 1f, damping = 0.3f;
      var g = StablePd.Gains(new StablePdSettings(frequency, damping), Dt);
      double w = 2.0 * Math.PI * frequency;
      double wd = w * Math.Sqrt(1.0 - damping * damping);

      float p = 1f, v = 0f;
      double worst = 0;
      for (int i = 1; i <= 150; i++) {
        Step(ref p, ref v, StablePd.Acceleration(g, p, v, 0f, 0f), Dt, 1);
        double t = i * Dt;
        double expected = Math.Exp(-damping * w * t) * (Math.Cos(wd * t) + damping * w / wd * Math.Sin(wd * t));
        worst = Math.Max(worst, Math.Abs(p - expected));
      }
      // the poles match exactly; the transient's initial shape differs by O(omega dt), because
      // the acceleration is held constant across each step
      Assert.Less(worst, 0.05);
    }

    [Test]
    public void UnconditionallyStableForAnyGainsTimestepAndSubSteps(
      [Values(1, 4)] int subSteps,
      [Values(0.1f, 2f, 20f, 200f, 5000f)] float frequency,
      [Values(0f, 0.2f, 1f, 10f)] float damping,
      [Values(0.001f, 0.02f, 0.1f)] float dt
    )
    {
      var g = StablePd.Gains(new StablePdSettings(frequency, damping), dt, subSteps);
      float final = Simulate1D(g, subSteps, 20f, 1f, out float peak);

      Assert.IsFalse(float.IsNaN(final) || float.IsInfinity(final));
      if (damping > 0f) {
        // a released, damped body never swings beyond where it started
        Assert.LessOrEqual(peak, 1.0001f, "the error must never grow beyond its initial value");
        Assert.LessOrEqual(math.abs(final), 1f);
      } else {
        // an undamped oscillator neither decays nor grows
        Assert.Less(peak, 1.5f);
      }
    }

    [Test]
    public void TracksConstantVelocityTargetWithZeroSteadyStateError([Values(1, 4)] int subSteps)
    {
      var g = StablePd.Gains(StablePdSettings.Default, Dt, subSteps);
      float3 p = float3.zero, v = float3.zero;
      var targetVelocity = new float3(3f, -1f, 2f);
      var target = new float3(5f, 0f, 0f);

      for (int i = 0; i < 300; i++) {
        Step(ref p, ref v, StablePd.Acceleration(g, p, v, target, targetVelocity), Dt, subSteps);
        target += targetVelocity * Dt;
      }
      Assert.Less(math.length(p - target), 1e-3f);
      Assert.Less(math.length(v - targetVelocity), 1e-3f);
    }

    [Test]
    public void RampLagWithoutTrackingIsDampingOverStiffnessTimesSpeed()
    {
      // a target at constant speed, fed without its velocity, settles to a lag of D / K * v
      var g = StablePd.Gains(StablePdSettings.Default, Dt);
      const float speed = 1f;
      float p = 0f, v = 0f, target = 0f;
      for (int i = 0; i < 500; i++) {
        Step(ref p, ref v, StablePd.Acceleration(g, p, v, target, 0f), Dt, 1);
        target += speed * Dt;
      }
      Assert.AreEqual(g.Damper / g.Stiffness * speed, target - p, 1e-4);
      Assert.AreEqual(0.16f, target - p, 0.005f);   // 16 cm at 2 Hz, as in the samples
    }

    [Test]
    public void TracksAcceleratingTargetWithFeedforward()
    {
      var g = StablePd.Gains(StablePdSettings.Default, Dt);
      float3 p = float3.zero, v = float3.zero;
      float3 target = new float3(1f), targetVelocity = float3.zero;
      var targetAcceleration = new float3(0f, 2f, -1f);

      for (int i = 0; i < 300; i++) {
        Step(ref p, ref v, StablePd.Acceleration(g, p, v, target, targetVelocity, targetAcceleration), Dt, 1);
        targetVelocity += targetAcceleration * Dt;
        target += targetVelocity * Dt;
      }
      Assert.Less(math.length(p - target), 1e-3f);
    }

    [Test]
    public void GravityFeedforwardRemovesSteadyStateSag([Values(1, 4)] int subSteps)
    {
      var g = StablePd.Gains(StablePdSettings.Default, Dt, subSteps);
      const float gravity = -9.81f;

      float sag = Simulate1D(g, subSteps, 10f, 0f, out _, externalAcceleration: gravity);
      Assert.AreEqual(gravity / g.Stiffness, sag, 1e-4, "without feedforward the body hangs g / K below the target");

      float held = Simulate1D(g, subSteps, 10f, 0f, out _, externalAcceleration: gravity, feedforward: -gravity);
      Assert.AreEqual(0f, held, 1e-5);
    }

    [Test]
    public void VectorOverloadsMatchTheScalarOnePerAxis()
    {
      var g = StablePd.Gains(new StablePdSettings(3f, 0.8f), Dt);
      var p = new float3(1f, -2f, 0.5f);
      var v = new float3(0.3f, 0f, -1f);
      var t = new float3(-1f, 4f, 2f);
      var tv = new float3(0f, 1f, 0.25f);
      var ff = new float3(0.1f, -9.81f, 0f);

      float3 a3 = StablePd.Acceleration(g, p, v, t, tv, ff);
      float2 a2 = StablePd.Acceleration(g, p.xy, v.xy, t.xy, tv.xy, ff.xy);
      for (int i = 0; i < 3; i++) {
        float a1 = StablePd.Acceleration(g, p[i], v[i], t[i], tv[i], ff[i]);
        Assert.AreEqual(a1, a3[i], 1e-4);
        if (i < 2) { Assert.AreEqual(a1, a2[i], 1e-4); }
      }
    }

    // ******************************************************************
    //    ANGULAR, 1 AXIS
    // ******************************************************************

    [Test]
    public void SingleAxisAngleTakesShortestPathAcrossWrap()
    {
      var g = StablePd.Gains(StablePdSettings.Default, Dt);
      // at +179 deg, aiming for -179 deg: the short way is +2 deg, so accelerate positively
      float alpha = StablePd.AngularAcceleration(g, math.radians(179f), 0f, math.radians(-179f), 0f);
      Assert.Greater(alpha, 0f);
      Assert.AreEqual(StablePd.AngularAcceleration(g, 0f, 0f, math.radians(2f), 0f), alpha, 1e-3);
    }

    [Test]
    public void SingleAxisAngleConvergesFromAnyStart([Values(1, 4)] int subSteps)
    {
      var g = StablePd.Gains(StablePdSettings.Default, Dt, subSteps);
      for (float start = -10f; start <= 10f; start += 0.7f) {
        float angle = start, w = 0f;
        const float target = 1f;
        for (int i = 0; i < 400; i++) {
          Step(ref angle, ref w, StablePd.AngularAcceleration(g, angle, w, target, 0f), Dt, subSteps);
        }
        Assert.Less(math.abs(MotionMath.WrapAngle(angle - target)), 1e-3f);
      }
    }

    // ******************************************************************
    //    ANGULAR, QUATERNION
    // ******************************************************************

    private static quaternion AxisAngleDegrees(float3 axis, float degrees)
    {
      return quaternion.AxisAngle(math.normalize(axis), math.radians(degrees));
    }

    // A free rigid body with an anisotropic, rotated inertia tensor, driven by torque: the
    // angular acceleration is I_world^-1 * torque, and the rotation is integrated with the new w.
    private static quaternion SimulateRotation(
      in StablePdGains g, quaternion start, Func<int, quaternion> targetAt, float3 targetAngularVelocity,
      int steps, float3 inertiaTensor, quaternion inertiaTensorRotation, out float3 angularVelocity, out float maxOffAxis,
      float3 axis = default, Func<float3, quaternion, float3> torqueOverride = null)
    {
      quaternion r = start;
      float3 w = float3.zero;
      maxOffAxis = 0f;
      for (int i = 0; i < steps; i++) {
        float3 alphaCommand = StablePd.AngularAcceleration(g, r, w, targetAt(i), targetAngularVelocity);
        float3 torque = torqueOverride != null
          ? torqueOverride(alphaCommand, r)
          : MotionMath.AngularAccelerationToTorque(alphaCommand, r, inertiaTensorRotation, inertiaTensor);

        // the body's true response, inverting the inertia independently of the code under test
        quaternion principalToWorld = math.mul(r, inertiaTensorRotation);
        float3 local = math.rotate(math.inverse(principalToWorld), torque);
        float3 alpha = math.rotate(principalToWorld, local / inertiaTensor);

        w += alpha * g.Dt;
        r = MotionMath.Integrate(r, w, g.Dt);
        if (math.lengthsq(axis) > 0f) {
          maxOffAxis = math.max(maxOffAxis, math.length(w - math.dot(w, axis) * axis));
        }
      }
      angularVelocity = w;
      return r;
    }

    private static float ErrorDegrees(quaternion a, quaternion b)
    {
      return math.degrees(math.length(MotionMath.ToRotationVector(math.mul(a, math.inverse(b)))));
    }

    [Test]
    public void RotationConvergesWithAnisotropicInertia([Values(1, 4)] int subSteps)
    {
      var g = StablePd.Gains(StablePdSettings.Default, Dt, subSteps);
      var target = AxisAngleDegrees(new float3(0.3f, 1f, -0.2f), -40f);
      var start = AxisAngleDegrees(new float3(1f, -0.5f, 0.25f), 170f);

      var final = SimulateRotation(g, start, _ => target, float3.zero, 400,
        new float3(1f, 4f, 0.25f), AxisAngleDegrees(new float3(1f, 2f, 3f), 30f), out var w, out _);

      Assert.Less(ErrorDegrees(final, target), 0.05f);
      Assert.Less(math.length(w), 1e-3f);
    }

    [Test]
    public void RotationTracksSpinningTarget()
    {
      var g = StablePd.Gains(StablePdSettings.Default, Dt);
      var spin = new float3(0f, 2f, 0.5f);
      var target0 = AxisAngleDegrees(new float3(1f, 0f, 0f), 60f);
      Func<int, quaternion> targetAt = i => MotionMath.Integrate(target0, spin, i * Dt);

      const int steps = 400;
      var final = SimulateRotation(g, quaternion.identity, targetAt, spin, steps,
        new float3(1f, 4f, 0.25f), AxisAngleDegrees(new float3(1f, 2f, 3f), 30f), out var w, out _);

      Assert.Less(ErrorDegrees(final, targetAt(steps)), 0.1f);
      Assert.Less(math.length(w - spin), 1e-2f);
    }

    [Test]
    public void RotationWithMisalignedInertiaHasNoCrossAxisMotion()
    {
      // inertia (1, 4, 12), principal axes at AngleAxis(30, Y) * AngleAxis(45, X), turned 90
      // degrees about one world axis: the angular velocity must stay on that axis
      var g = StablePd.Gains(StablePdSettings.Default, Dt);
      var inertia = new float3(1f, 4f, 12f);
      var inertiaRotation = math.mul(AxisAngleDegrees(new float3(0f, 1f, 0f), 30f), AxisAngleDegrees(new float3(1f, 0f, 0f), 45f));
      var axis = new float3(0f, 1f, 0f);
      var target = AxisAngleDegrees(axis, 90f);

      var final = SimulateRotation(g, quaternion.identity, _ => target, float3.zero, 400,
        inertia, inertiaRotation, out _, out float offAxis, axis);
      Assert.Less(offAxis, 1e-3f, "angular velocity strayed off the rotation axis");
      Assert.Less(ErrorDegrees(final, target), 0.05f);

      // sensitivity check: the reversed composition (inertiaTensorRotation * rotation) couples the axes
      Func<float3, quaternion, float3> reversed = (alpha, r) => {
        quaternion wrong = math.mul(inertiaRotation, r);
        return math.rotate(wrong, math.rotate(math.inverse(wrong), alpha) * inertia);
      };
      SimulateRotation(g, quaternion.identity, _ => target, float3.zero, 400,
        inertia, inertiaRotation, out _, out float wrongOffAxis, axis, reversed);
      Assert.Greater(wrongOffAxis, 100f * math.max(offAxis, 1e-5f));
    }

    [Test]
    public void RotationTakesShortestPath()
    {
      var g = StablePd.Gains(StablePdSettings.Default, Dt);
      // 270 degrees around +Y is 90 degrees the other way
      var alpha = StablePd.AngularAcceleration(g, quaternion.identity, float3.zero, AxisAngleDegrees(new float3(0f, 1f, 0f), 270f), float3.zero);
      Assert.Less(alpha.y, 0f);
      Assert.AreEqual(0f, alpha.x, 1e-4);
      Assert.AreEqual(0f, alpha.z, 1e-4);
    }

    [Test]
    public void RotationAtTargetProducesNoAcceleration()
    {
      var g = StablePd.Gains(StablePdSettings.Default, Dt);
      var q = AxisAngleDegrees(new float3(1f, 1f, 0f), 33f);
      var alpha = StablePd.AngularAcceleration(g, q, float3.zero, q, float3.zero);
      Assert.IsFalse(math.any(math.isnan(alpha)));
      Assert.Less(math.length(alpha), 1e-4f);
    }
  }
}
