using UnityEngine;

namespace Yohash.Propulsion
{
  /// <summary>
  /// Shared math helpers for the controllers in this package.
  ///
  /// Conventions:
  ///   - Angles are in radians, angular velocities in radians per second.
  ///   - Angular velocities are world-space vectors (as reported by Rigidbody.angularVelocity
  ///     and ArticulationBody.angularVelocity).
  ///   - A "rotation vector" is axis * angle (radians), i.e. the quaternion logarithm map.
  /// </summary>
  public static class PropulsionMath
  {
    private const float SmallAngle = 1e-6f;

    /// <summary>
    /// Wraps an angle (radians) to the range [-PI, PI].
    /// </summary>
    public static float WrapAngle(float radians)
    {
      return Mathf.Repeat(radians + Mathf.PI, 2f * Mathf.PI) - Mathf.PI;
    }

    /// <summary>
    /// Converts a quaternion to a rotation vector (axis * angle, radians), always taking the
    /// shortest path (angle in [0, PI]).
    ///
    /// Unlike Quaternion.ToAngleAxis, this tolerates slightly non-unit quaternions (it never
    /// produces NaN from acos(w > 1)) and it is smooth through the identity rotation.
    /// </summary>
    public static Vector3 ToRotationVector(Quaternion q)
    {
      float norm = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
      if (norm < SmallAngle) { return Vector3.zero; }

      float inv = (q.w < 0f ? -1f : 1f) / norm;
      var v = new Vector3(q.x * inv, q.y * inv, q.z * inv);
      float w = q.w * inv;

      float s = v.magnitude;
      if (s < SmallAngle) {
        // sin(angle/2) ~ angle/2, so axis * angle ~ 2 * v
        return 2f * v;
      }

      float angle = 2f * Mathf.Atan2(s, w);
      return v * (angle / s);
    }

    /// <summary>
    /// Converts a rotation vector (axis * angle, radians) back into a quaternion.
    /// </summary>
    public static Quaternion FromRotationVector(Vector3 rotationVector)
    {
      float angle = rotationVector.magnitude;
      if (angle < SmallAngle) {
        var half = 0.5f * rotationVector;
        return Normalize(new Quaternion(half.x, half.y, half.z, 1f));
      }

      float halfAngle = 0.5f * angle;
      float k = Mathf.Sin(halfAngle) / angle;
      return new Quaternion(
        rotationVector.x * k,
        rotationVector.y * k,
        rotationVector.z * k,
        Mathf.Cos(halfAngle)
      );
    }

    /// <summary>
    /// Advances a rotation by a world-space angular velocity over dt seconds.
    /// </summary>
    public static Quaternion Integrate(Quaternion rotation, Vector3 angularVelocity, float dt)
    {
      return FromRotationVector(angularVelocity * dt) * rotation;
    }

    /// <summary>
    /// The world-space angular velocity that rotates <paramref name="from"/> into
    /// <paramref name="to"/> in dt seconds, along the shortest path.
    ///
    /// Use this to derive a target's angular velocity from two successive target rotations.
    /// </summary>
    public static Vector3 AngularVelocity(Quaternion from, Quaternion to, float dt)
    {
      if (dt <= 0f) { return Vector3.zero; }
      return ToRotationVector(to * Quaternion.Inverse(from)) / dt;
    }

    /// <summary>
    /// The rate of change (radians per second) between two angles (radians), along the
    /// shortest path. Safe across the +/-PI wrap.
    /// </summary>
    public static float AngularVelocity(float fromRadians, float toRadians, float dt)
    {
      if (dt <= 0f) { return 0f; }
      return WrapAngle(toRadians - fromRadians) / dt;
    }

    /// <summary>
    /// Converts a world-space angular acceleration into the world-space torque that produces it
    /// on a rigid body, using the body's principal moments of inertia.
    ///
    ///     torque = R * diag(inertiaTensor) * R^-1 * angularAcceleration
    ///     R      = rotation * inertiaTensorRotation   (principal-axis frame -> world)
    ///
    /// Inputs map directly onto Rigidbody / ArticulationBody:
    ///     rotation              = rigidbody.rotation
    ///     inertiaTensorRotation = rigidbody.inertiaTensorRotation
    ///     inertiaTensor         = rigidbody.inertiaTensor
    ///
    /// Gyroscopic (w x Iw) terms are not included; PhysX does not apply them by default.
    /// Apply the result with rigidbody.AddTorque (world space), not AddRelativeTorque.
    /// </summary>
    public static Vector3 AngularAccelerationToTorque(
      Vector3 angularAcceleration,
      Quaternion rotation,
      Quaternion inertiaTensorRotation,
      Vector3 inertiaTensor
    )
    {
      var principalToWorld = rotation * inertiaTensorRotation;
      var principal = Quaternion.Inverse(principalToWorld) * angularAcceleration;
      principal.Scale(inertiaTensor);
      return principalToWorld * principal;
    }

    private static Quaternion Normalize(Quaternion q)
    {
      float norm = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
      return new Quaternion(q.x / norm, q.y / norm, q.z / norm, q.w / norm);
    }
  }
}
