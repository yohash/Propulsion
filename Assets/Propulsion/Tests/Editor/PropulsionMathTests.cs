using NUnit.Framework;
using UnityEngine;

namespace Yohash.Propulsion.Tests
{
  public class PropulsionMathTests
  {
    [Test]
    public void WrapAngleMapsIntoPlusMinusPi()
    {
      Assert.AreEqual(0f, PropulsionMath.WrapAngle(2f * Mathf.PI), 1e-5);
      Assert.AreEqual(-Mathf.PI / 2f, PropulsionMath.WrapAngle(3f * Mathf.PI / 2f), 1e-5);
      Assert.AreEqual(0.5f, PropulsionMath.WrapAngle(0.5f - 6f * Mathf.PI), 1e-4);
    }

    [Test]
    public void RotationVectorMatchesAxisTimesAngle()
    {
      var axis = new Vector3(1f, -2f, 0.5f).normalized;
      var q = Quaternion.AngleAxis(75f, axis);
      var r = PropulsionMath.ToRotationVector(q);
      Assert.Less((r - axis * (75f * Mathf.Deg2Rad)).magnitude, 1e-4f);

      // q and -q are the same rotation
      var negated = new Quaternion(-q.x, -q.y, -q.z, -q.w);
      Assert.Less((PropulsionMath.ToRotationVector(negated) - r).magnitude, 1e-4f);
    }

    [Test]
    public void RotationVectorTakesShortestPath()
    {
      var q = Quaternion.AngleAxis(300f, Vector3.forward);
      var r = PropulsionMath.ToRotationVector(q);
      Assert.AreEqual(-60f * Mathf.Deg2Rad, r.z, 1e-4);
    }

    [Test]
    public void RotationVectorIsRobustNearIdentityAndForUnnormalizedInput()
    {
      var r0 = PropulsionMath.ToRotationVector(Quaternion.identity);
      Assert.AreEqual(0f, r0.magnitude, 1e-7);

      // a slightly non-unit quaternion (w > 1) makes acos-based conversions return NaN
      var r1 = PropulsionMath.ToRotationVector(new Quaternion(1e-5f, 0f, 0f, 1.0000002f));
      Assert.IsFalse(float.IsNaN(r1.x) || float.IsNaN(r1.y) || float.IsNaN(r1.z));
      Assert.AreEqual(2e-5f, r1.x, 1e-6);
    }

    [Test]
    public void RotationVectorRoundTrips()
    {
      var r = new Vector3(0.3f, -1.2f, 2.0f);
      var back = PropulsionMath.ToRotationVector(PropulsionMath.FromRotationVector(r));
      Assert.Less((back - r).magnitude, 1e-4f);
    }

    [Test]
    public void AngularVelocityRecoversIntegratedSpin()
    {
      var w = new Vector3(0.4f, 3f, -1f);
      var q0 = Quaternion.AngleAxis(20f, Vector3.right);
      var q1 = PropulsionMath.Integrate(q0, w, 0.02f);
      Assert.Less((PropulsionMath.AngularVelocity(q0, q1, 0.02f) - w).magnitude, 1e-3f);
    }

    [Test]
    public void ScalarAngularVelocityIsWrapSafe()
    {
      // +179 deg to -179 deg is a +2 deg step, not -358
      float w = PropulsionMath.AngularVelocity(179f * Mathf.Deg2Rad, -179f * Mathf.Deg2Rad, 0.02f);
      Assert.AreEqual(2f * Mathf.Deg2Rad / 0.02f, w, 1e-3);
    }

    [Test]
    public void TorqueUsesBodyThenPrincipalRotationOrder()
    {
      // torque = R D R^-1 alpha, with R = rotation * inertiaTensorRotation.
      var rotation = Quaternion.AngleAxis(90f, Vector3.up);
      var inertiaTensorRotation = Quaternion.AngleAxis(90f, Vector3.right);
      var inertia = new Vector3(1f, 2f, 3f);

      // R maps principal x->world -z... check by construction: rotate each principal axis
      var R = rotation * inertiaTensorRotation;
      for (int i = 0; i < 3; i++) {
        var principalAxis = new Vector3(i == 0 ? 1 : 0, i == 1 ? 1 : 0, i == 2 ? 1 : 0);
        var worldAxis = R * principalAxis;
        var torque = PropulsionMath.AngularAccelerationToTorque(worldAxis, rotation, inertiaTensorRotation, inertia);
        // accelerating about a principal axis needs torque along that same axis, scaled by its moment
        float moment = i == 0 ? inertia.x : i == 1 ? inertia.y : inertia.z;
        Assert.Less((torque - worldAxis * moment).magnitude, 1e-4f);
      }

      // the reversed order (inertiaTensorRotation * rotation) gives a different answer here
      var wrongR = inertiaTensorRotation * rotation;
      Assert.Greater((wrongR * Vector3.right - R * Vector3.right).magnitude, 0.5f);
    }
  }
}
