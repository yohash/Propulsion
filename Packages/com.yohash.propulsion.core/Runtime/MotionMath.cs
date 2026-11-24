using Unity.Mathematics;

namespace Yohash.Propulsion.Core
{
  /// <summary>
  /// Rotation and angle helpers shared by the controllers. Burst-compatible: plain static
  /// functions over Unity.Mathematics, with no managed types and no exceptions.
  ///
  /// Conventions:
  ///   - Angles are in radians, angular velocities in radians per second.
  ///   - 3D angular velocities are world-space vectors.
  ///   - A "rotation vector" is axis * angle (radians): the quaternion logarithm map.
  ///   - Quaternions are expected to be unit length, as Unity.Mathematics' rotate/mul assume.
  /// </summary>
  public static class MotionMath
  {
    private const float SmallAngle = 1e-6f;

    /// <summary>Wraps an angle (radians) into [-PI, PI).</summary>
    public static float WrapAngle(float radians)
    {
      float t = radians + math.PI;
      t -= math.PI2 * math.floor(t / math.PI2);
      return t - math.PI;
    }

    /// <summary>
    /// Converts a quaternion to a rotation vector (axis * angle, radians), always along the
    /// shortest path (angle in [0, PI]). Tolerates slightly non-unit input (no acos of w > 1),
    /// and is smooth through the identity rotation.
    /// </summary>
    public static float3 ToRotationVector(quaternion q)
    {
      float4 v = q.value;
      float norm = math.length(v);
      if (norm < SmallAngle) { return float3.zero; }

      v *= (v.w < 0f ? -1f : 1f) / norm;
      float3 axisPart = v.xyz;
      float s = math.length(axisPart);
      if (s < SmallAngle) {
        // sin(angle / 2) ~ angle / 2, so axis * angle ~ 2 * xyz
        return 2f * axisPart;
      }

      float angle = 2f * math.atan2(s, v.w);
      return axisPart * (angle / s);
    }

    /// <summary>Converts a rotation vector (axis * angle, radians) back into a unit quaternion.</summary>
    public static quaternion FromRotationVector(float3 rotationVector)
    {
      float angle = math.length(rotationVector);
      if (angle < SmallAngle) {
        return math.normalize(new quaternion(new float4(0.5f * rotationVector, 1f)));
      }

      math.sincos(0.5f * angle, out float sinHalf, out float cosHalf);
      return new quaternion(new float4(rotationVector * (sinHalf / angle), cosHalf));
    }

    /// <summary>Advances a rotation by a world-space angular velocity over dt seconds.</summary>
    public static quaternion Integrate(quaternion rotation, float3 angularVelocity, float dt)
    {
      return math.mul(FromRotationVector(angularVelocity * dt), rotation);
    }

    /// <summary>
    /// The world-space angular velocity that turns <paramref name="from"/> into
    /// <paramref name="to"/> in dt seconds, along the shortest path. Returns zero for dt &lt;= 0.
    /// Use it to derive a target's angular velocity from two successive target rotations.
    /// </summary>
    public static float3 AngularVelocity(quaternion from, quaternion to, float dt)
    {
      if (!(dt > 0f)) { return float3.zero; }
      return ToRotationVector(math.mul(to, math.inverse(from))) / dt;
    }

    /// <summary>
    /// The rate of change (rad/s) between two angles (radians), along the shortest path, so it
    /// is safe across the +/-PI wrap. Returns zero for dt &lt;= 0.
    /// </summary>
    public static float AngularVelocity(float fromRadians, float toRadians, float dt)
    {
      if (!(dt > 0f)) { return 0f; }
      return WrapAngle(toRadians - fromRadians) / dt;
    }

    /// <summary>
    /// Converts a world-space angular acceleration into the world-space torque that produces it on
    /// a rigid body with the given principal moments of inertia:
    ///
    ///     torque = R * diag(inertiaTensor) * R^-1 * angularAcceleration,
    ///     R      = rotation * inertiaTensorRotation   (principal-axis frame -> world)
    ///
    /// Gyroscopic (w x Iw) terms are not included.
    /// </summary>
    public static float3 AngularAccelerationToTorque(
      float3 angularAcceleration,
      quaternion rotation,
      quaternion inertiaTensorRotation,
      float3 inertiaTensor
    )
    {
      quaternion principalToWorld = math.mul(rotation, inertiaTensorRotation);
      float3 principal = math.rotate(math.inverse(principalToWorld), angularAcceleration);
      return math.rotate(principalToWorld, principal * inertiaTensor);
    }
  }
}
