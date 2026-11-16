using System;
using NUnit.Framework;
using UnityEngine;

namespace Yohash.Propulsion.Tests
{
  /// <summary>
  /// These tests drive the controller with the same semi-implicit Euler integrator that
  /// PhysX and Box2D use (v += a * dt; p += v * dt), so they verify the physical claims made in
  /// the controller's documentation without needing a scene.
  /// </summary>
  public class StablePdControllerTests
  {
    private const float Dt = 0.02f;

    // simulate a unit-mass body on one axis; returns final position, and tracks peak overshoot
    private static float Simulate1D(
      StablePdController controller,
      float dt,
      float seconds,
      float startPosition,
      out float maxAbsPosition,
      float externalAcceleration = 0f,
      float feedforward = 0f
    )
    {
      float p = startPosition;
      float v = 0f;
      maxAbsPosition = Mathf.Abs(p);
      int steps = Mathf.CeilToInt(seconds / dt);
      for (int i = 0; i < steps; i++) {
        float a = controller.ComputeAcceleration(dt, p, v, 0f, 0f, feedforward) + externalAcceleration;
        v += a * dt;
        p += v * dt;
        maxAbsPosition = Mathf.Max(maxAbsPosition, Mathf.Abs(p));
      }
      return p;
    }

    [Test]
    public void GainsMatchNaturalFrequencyAndDampingRatio()
    {
      var c = new StablePdController(frequency: 2f, damping: 0.5f);
      float omega = 2f * Mathf.PI * 2f;
      Assert.AreEqual(omega * omega, c.Kp, 1e-3);
      Assert.AreEqual(2f * 0.5f * omega, c.Kd, 1e-4);

      // the discrete gains converge on the continuous ones as dt -> 0
      c.GetDiscreteGains(1e-4f, out float k, out float d);
      Assert.AreEqual(c.Kp, k, c.Kp * 2e-3);
      Assert.AreEqual(c.Kd, d, c.Kd * 2e-3);
    }

    [Test]
    public void DiscretePolesExactlyMatchContinuousPoles(
      [Values(0.5f, 2f, 8f)] float frequency,
      [Values(0.2f, 1f, 3f)] float damping,
      [Values(0.005f, 0.02f)] float dt
    )
    {
      var c = new StablePdController(frequency, damping);
      c.GetDiscreteGains(dt, out float k, out float d);

      // semi-implicit Euler with a = -k e - d de has characteristic polynomial
      //   z^2 - (2 - k dt^2 - d dt) z + (1 - d dt)
      double omega = 2.0 * Math.PI * frequency;
      double product = Math.Exp(-2.0 * damping * omega * dt);
      double sum = damping < 1f
        ? 2.0 * Math.Exp(-damping * omega * dt) * Math.Cos(omega * Math.Sqrt(1.0 - damping * damping) * dt)
        : damping == 1f
          ? 2.0 * Math.Exp(-omega * dt)
          : Math.Exp(-omega * (damping - Math.Sqrt(damping * damping - 1.0)) * dt)
            + Math.Exp(-omega * (damping + Math.Sqrt(damping * damping - 1.0)) * dt);

      Assert.AreEqual(product, 1.0 - d * dt, 1e-5);
      Assert.AreEqual(sum, 2.0 - k * dt * dt - d * dt, 1e-5);
    }

    [Test]
    public void UnderdampedResponseMatchesContinuousTime()
    {
      // e(t) = exp(-zeta w t) (cos(wd t) + zeta w / wd sin(wd t)) for a release from rest
      const float frequency = 1f, damping = 0.3f;
      var c = new StablePdController(frequency, damping);
      double w = 2.0 * Math.PI * frequency;
      double wd = w * Math.Sqrt(1.0 - damping * damping);

      float p = 1f, v = 0f;
      double worst = 0;
      for (int i = 1; i <= 150; i++) {
        v += c.ComputeAcceleration(Dt, p, v, 0f, 0f) * Dt;
        p += v * Dt;
        double t = i * Dt;
        double expected = Math.Exp(-damping * w * t) * (Math.Cos(wd * t) + damping * w / wd * Math.Sin(wd * t));
        worst = Math.Max(worst, Math.Abs(p - expected));
      }
      // the poles (frequency and decay) match exactly; the transient's initial shape differs by
      // O(omega * dt), because the control acceleration is held constant across each step
      Assert.Less(worst, 0.05);
    }

    [Test]
    public void CriticallyDampedStepSettlesWithoutOvershoot()
    {
      var c = new StablePdController(frequency: 1f, damping: 1f);

      // documented: error falls to ~5% after ~0.75 / frequency seconds
      float at075 = Simulate1D(c, Dt, 0.75f, 1f, out _);
      Assert.Less(Mathf.Abs(at075), 0.08f);

      float p = 1f, v = 0f, minP = 1f;
      for (int i = 0; i < 500; i++) {
        v += c.ComputeAcceleration(Dt, p, v, 0f, 0f) * Dt;
        p += v * Dt;
        minP = Mathf.Min(minP, p);
      }
      Assert.Less(Mathf.Abs(p), 1e-4f);
      Assert.Greater(minP, -0.01f, "critically damped response should not overshoot");
    }

    [Test]
    public void UnconditionallyStableForAnyGainsAndTimestep(
      [Values(0.1f, 2f, 20f, 200f, 5000f)] float frequency,
      [Values(0f, 0.2f, 1f, 10f)] float damping,
      [Values(0.001f, 0.02f, 0.1f)] float dt
    )
    {
      var c = new StablePdController(frequency, damping);
      float final = Simulate1D(c, dt, 20f, 1f, out float peak);

      Assert.IsFalse(float.IsNaN(final) || float.IsInfinity(final));
      if (damping > 0f) {
        // a released, damped body can never swing beyond where it started
        Assert.LessOrEqual(peak, 1.0001f, "the error must never grow beyond its initial value");
        Assert.LessOrEqual(Mathf.Abs(final), 1f);
      } else {
        // an undamped oscillator neither decays nor grows
        Assert.Less(peak, 1.5f);
      }
    }

    [Test]
    public void TracksConstantVelocityTargetWithZeroSteadyStateError()
    {
      var c = new StablePdController(frequency: 2f, damping: 1f);
      var p = Vector3.zero;
      var v = Vector3.zero;
      var targetVelocity = new Vector3(3f, -1f, 2f);
      var target = new Vector3(5f, 0f, 0f);

      for (int i = 0; i < 300; i++) {
        var a = c.ComputeAcceleration(Dt, p, v, target, targetVelocity);
        v += a * Dt;
        p += v * Dt;
        target += targetVelocity * Dt;
      }
      Assert.Less((p - target).magnitude, 1e-3f);
      Assert.Less((v - targetVelocity).magnitude, 1e-3f);
    }

    [Test]
    public void TracksAcceleratingTargetWithFeedforward()
    {
      var c = new StablePdController(frequency: 2f, damping: 1f);
      var p = Vector3.zero;
      var v = Vector3.zero;
      var target = Vector3.one;
      var targetVelocity = Vector3.zero;
      var targetAcceleration = new Vector3(0f, 2f, -1f);

      for (int i = 0; i < 300; i++) {
        var a = c.ComputeAcceleration(Dt, p, v, target, targetVelocity, targetAcceleration);
        v += a * Dt;
        p += v * Dt;
        targetVelocity += targetAcceleration * Dt;
        target += targetVelocity * Dt;
      }
      Assert.Less((p - target).magnitude, 1e-3f);
    }

    [Test]
    public void GravityFeedforwardRemovesSteadyStateSag()
    {
      var c = new StablePdController(frequency: 2f, damping: 1f);
      const float g = -9.81f;

      c.GetDiscreteGains(Dt, out float stiffness, out _);
      float sag = Simulate1D(c, Dt, 10f, 0f, out _, externalAcceleration: g);
      Assert.AreEqual(g / stiffness, sag, 1e-4, "without feedforward the body hangs g / stiffness below the target");

      float held = Simulate1D(c, Dt, 10f, 0f, out _, externalAcceleration: g, feedforward: -g);
      Assert.AreEqual(0f, held, 1e-5);
    }

    [Test]
    public void ForceAndTorqueScaleByMassAndInertia()
    {
      var c = new StablePdController(2f, 1f);
      float a = c.ComputeAcceleration(Dt, 1f, 0.5f, 0f, 0f);
      Assert.AreEqual(7f * a, c.ComputeForce(Dt, 7f, 1f, 0.5f, 0f, 0f), 1e-3);

      float alpha = c.ComputeAngularAcceleration(Dt, 1f, 0.5f, 0f, 0f);
      Assert.AreEqual(3f * alpha, c.ComputeTorque(Dt, 3f, 1f, 0.5f, 0f, 0f), 1e-3);
    }

    [Test]
    public void SingleAxisAngleTakesShortestPathAcrossWrap()
    {
      var c = new StablePdController(2f, 1f);
      // at +179 deg, aiming for -179 deg: the short way is +2 deg, so accelerate positively
      float alpha = c.ComputeAngularAcceleration(
        Dt, 179f * Mathf.Deg2Rad, 0f, -179f * Mathf.Deg2Rad, 0f);
      Assert.Greater(alpha, 0f);

      // and the magnitude matches a plain +2 degree error
      float reference = c.ComputeAngularAcceleration(Dt, 0f, 0f, 2f * Mathf.Deg2Rad, 0f);
      Assert.AreEqual(reference, alpha, 1e-3);
    }

    [Test]
    public void SingleAxisAngleConvergesFromAnyStart()
    {
      var c = new StablePdController(2f, 1f);
      for (float start = -10f; start <= 10f; start += 0.7f) {
        float angle = start;
        float w = 0f;
        float target = 1f;
        for (int i = 0; i < 400; i++) {
          w += c.ComputeAngularAcceleration(Dt, angle, w, target, 0f) * Dt;
          angle += w * Dt;
        }
        Assert.Less(Mathf.Abs(PropulsionMath.WrapAngle(angle - target)), 1e-3f);
      }
    }

    // simulate a free-floating rigid body with an anisotropic, rotated inertia tensor.
    // PhysX: angular acceleration = I_world^-1 * torque, then rotation integrated with the new w
    private static Quaternion SimulateRotation(
      StablePdController c,
      Quaternion start,
      Func<int, Quaternion> targetAt,
      Vector3 targetAngularVelocity,
      int steps,
      out Vector3 angularVelocity
    )
    {
      var inertiaTensor = new Vector3(1f, 4f, 0.25f);
      var inertiaTensorRotation = Quaternion.AngleAxis(30f, new Vector3(1f, 2f, 3f));

      var r = start;
      var w = Vector3.zero;
      for (int i = 0; i < steps; i++) {
        var torque = c.ComputeTorque(
          Dt, inertiaTensorRotation, inertiaTensor, r, w, targetAt(i), targetAngularVelocity);

        // invert the inertia independently of the code under test
        var principalToWorld = r * inertiaTensorRotation;
        var local = Quaternion.Inverse(principalToWorld) * torque;
        var alpha = principalToWorld * new Vector3(
          local.x / inertiaTensor.x, local.y / inertiaTensor.y, local.z / inertiaTensor.z);

        w += alpha * Dt;
        r = PropulsionMath.Integrate(r, w, Dt);
      }
      angularVelocity = w;
      return r;
    }

    [Test]
    public void RotationConvergesWithAnisotropicInertia()
    {
      var c = new StablePdController(2f, 1f);
      var target = Quaternion.AngleAxis(-40f, new Vector3(0.3f, 1f, -0.2f));
      var start = Quaternion.AngleAxis(170f, new Vector3(1f, -0.5f, 0.25f));

      var final = SimulateRotation(c, start, _ => target, Vector3.zero, 400, out var w);

      float errorDegrees = PropulsionMath.ToRotationVector(final * Quaternion.Inverse(target)).magnitude * Mathf.Rad2Deg;
      Assert.Less(errorDegrees, 0.05f);
      Assert.Less(w.magnitude, 1e-3f);
    }

    [Test]
    public void RotationTracksSpinningTarget()
    {
      var c = new StablePdController(2f, 1f);
      var spin = new Vector3(0f, 2f, 0.5f);
      var target0 = Quaternion.AngleAxis(60f, Vector3.right);
      Func<int, Quaternion> targetAt = i => PropulsionMath.Integrate(target0, spin, i * Dt);

      const int steps = 400;
      var final = SimulateRotation(c, Quaternion.identity, targetAt, spin, steps, out var w);

      var targetNow = targetAt(steps);
      float errorDegrees = PropulsionMath.ToRotationVector(final * Quaternion.Inverse(targetNow)).magnitude * Mathf.Rad2Deg;
      Assert.Less(errorDegrees, 0.1f);
      Assert.Less((w - spin).magnitude, 1e-2f);
    }

    [Test]
    public void RotationTakesShortestPath()
    {
      var c = new StablePdController(2f, 1f);
      // target is 270 degrees around +Y, i.e. 90 degrees the other way
      var target = Quaternion.AngleAxis(270f, Vector3.up);
      var alpha = c.ComputeAngularAcceleration(Dt, Quaternion.identity, Vector3.zero, target, Vector3.zero);
      Assert.Less(alpha.y, 0f);
      Assert.AreEqual(0f, alpha.x, 1e-4);
      Assert.AreEqual(0f, alpha.z, 1e-4);
    }

    [Test]
    public void RotationAtTargetProducesNoTorque()
    {
      var c = new StablePdController(2f, 1f);
      var q = Quaternion.AngleAxis(33f, new Vector3(1f, 1f, 0f));
      var torque = c.ComputeTorque(Dt, Quaternion.identity, Vector3.one, q, Vector3.zero, q, Vector3.zero);
      Assert.IsFalse(float.IsNaN(torque.x) || float.IsNaN(torque.y) || float.IsNaN(torque.z));
      Assert.Less(torque.magnitude, 1e-4f);
    }

    [Test]
    public void NonPositiveTimestepThrows()
    {
      var c = new StablePdController();
      Assert.Throws<ArgumentOutOfRangeException>(() => c.ComputeAcceleration(0f, 0f, 0f, 1f, 0f));
      Assert.Throws<ArgumentOutOfRangeException>(() => c.ComputeAcceleration(-Dt, Vector3.zero, Vector3.zero, Vector3.one, Vector3.zero));
      Assert.Throws<ArgumentOutOfRangeException>(() => c.ComputeAngularAcceleration(0f, Quaternion.identity, Vector3.zero, Quaternion.identity, Vector3.zero));
    }
  }
}
