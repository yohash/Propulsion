using NUnit.Framework;
using Unity.Mathematics;

namespace Yohash.Propulsion.Core.Tests
{
  public class MotionMathTests
  {
    private static quaternion AxisAngleDegrees(float3 axis, float degrees)
    {
      return quaternion.AxisAngle(math.normalize(axis), math.radians(degrees));
    }

    [Test]
    public void WrapAngleMapsIntoPlusMinusPi()
    {
      Assert.AreEqual(0f, MotionMath.WrapAngle(2f * math.PI), 1e-5);
      Assert.AreEqual(-math.PI / 2f, MotionMath.WrapAngle(3f * math.PI / 2f), 1e-5);
      Assert.AreEqual(0.5f, MotionMath.WrapAngle(0.5f - 6f * math.PI), 1e-4);
      Assert.AreEqual(-math.PI, MotionMath.WrapAngle(math.PI), 1e-5);
    }

    [Test]
    public void RotationVectorMatchesAxisTimesAngle()
    {
      var axis = math.normalize(new float3(1f, -2f, 0.5f));
      var q = AxisAngleDegrees(axis, 75f);
      var r = MotionMath.ToRotationVector(q);
      Assert.Less(math.length(r - axis * math.radians(75f)), 1e-4f);

      // q and -q are the same rotation
      Assert.Less(math.length(MotionMath.ToRotationVector(new quaternion(-q.value)) - r), 1e-4f);
    }

    [Test]
    public void RotationVectorTakesShortestPath()
    {
      var r = MotionMath.ToRotationVector(AxisAngleDegrees(new float3(0f, 0f, 1f), 300f));
      Assert.AreEqual(math.radians(-60f), r.z, 1e-4);
    }

    [Test]
    public void RotationVectorIsRobustNearIdentityAndForUnnormalizedInput()
    {
      Assert.AreEqual(0f, math.length(MotionMath.ToRotationVector(quaternion.identity)), 1e-7);

      // a slightly non-unit quaternion (w > 1) makes acos-based conversions return NaN
      var r = MotionMath.ToRotationVector(new quaternion(1e-5f, 0f, 0f, 1.0000002f));
      Assert.IsFalse(math.any(math.isnan(r)));
      Assert.AreEqual(2e-5f, r.x, 1e-6);
    }

    [Test]
    public void RotationVectorRoundTrips()
    {
      var r = new float3(0.3f, -1.2f, 2.0f);
      Assert.Less(math.length(MotionMath.ToRotationVector(MotionMath.FromRotationVector(r)) - r), 1e-4f);
    }

    [Test]
    public void AngularVelocityRecoversIntegratedSpin()
    {
      var w = new float3(0.4f, 3f, -1f);
      var q0 = AxisAngleDegrees(new float3(1f, 0f, 0f), 20f);
      var q1 = MotionMath.Integrate(q0, w, 0.02f);
      Assert.Less(math.length(MotionMath.AngularVelocity(q0, q1, 0.02f) - w), 1e-3f);
      Assert.AreEqual(0f, math.length(MotionMath.AngularVelocity(q0, q1, 0f)), 0.0);
    }

    [Test]
    public void ScalarAngularVelocityIsWrapSafe()
    {
      // +179 deg to -179 deg is a +2 deg step, not -358
      float w = MotionMath.AngularVelocity(math.radians(179f), math.radians(-179f), 0.02f);
      Assert.AreEqual(math.radians(2f) / 0.02f, w, 1e-3);
      Assert.AreEqual(0f, MotionMath.AngularVelocity(0f, 1f, 0f), 0.0);
    }

    [Test]
    public void TorqueUsesBodyThenPrincipalRotationOrder()
    {
      // torque = R D R^-1 alpha, with R = rotation * inertiaTensorRotation
      var rotation = AxisAngleDegrees(new float3(0f, 1f, 0f), 90f);
      var inertiaTensorRotation = AxisAngleDegrees(new float3(1f, 0f, 0f), 90f);
      var inertia = new float3(1f, 2f, 3f);

      var principalToWorld = math.mul(rotation, inertiaTensorRotation);
      for (int i = 0; i < 3; i++) {
        var principalAxis = new float3(i == 0 ? 1 : 0, i == 1 ? 1 : 0, i == 2 ? 1 : 0);
        var worldAxis = math.rotate(principalToWorld, principalAxis);
        var torque = MotionMath.AngularAccelerationToTorque(worldAxis, rotation, inertiaTensorRotation, inertia);
        // accelerating about a principal axis needs torque along that axis, scaled by its moment
        Assert.Less(math.length(torque - worldAxis * inertia[i]), 1e-4f);
      }

      // the reversed order (inertiaTensorRotation * rotation) gives a different answer here
      var wrong = math.mul(inertiaTensorRotation, rotation);
      Assert.Greater(math.length(math.rotate(wrong, new float3(1f, 0f, 0f)) - math.rotate(principalToWorld, new float3(1f, 0f, 0f))), 0.5f);
    }
  }
}
