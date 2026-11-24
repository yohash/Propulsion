using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Yohash.Propulsion.Core.Tests.Burst
{
  /// <summary>
  /// Proves the core is Burst-compatible: a Burst-compiled job calls every public function, and
  /// must (a) actually run as Burst code, not fall back to managed code, and (b) agree with the
  /// same calls made from managed code.
  ///
  /// This assembly only compiles when com.unity.burst is installed (see its asmdef's
  /// versionDefines); the core package itself doesn't depend on Burst.
  /// </summary>
  public class BurstCompatibilityTests
  {
    private const int ScalarCount = 8;
    private const int VectorCount = 4;

    [BurstCompile(CompileSynchronously = true)]
    private struct ControllersJob : IJob
    {
      public NativeArray<float> Scalars;
      public NativeArray<float3> Vectors;
      public NativeArray<int> RanAsManagedCode;

      public void Execute()
      {
        int managed = 0;
        MarkManaged(ref managed);
        RanAsManagedCode[0] = managed;
        Evaluate(ref Scalars, ref Vectors);
      }

      // Burst strips [BurstDiscard] calls, so this only sets the flag when the job runs as
      // managed code (Burst disabled, or the job failed to Burst-compile).
      [BurstDiscard]
      private static void MarkManaged(ref int managed)
      {
        managed = 1;
      }
    }

    // every public entry point of the core, with fixed inputs
    private static void Evaluate(ref NativeArray<float> scalars, ref NativeArray<float3> vectors)
    {
      var gains = StablePd.Gains(new StablePdSettings(2f, 0.7f), 0.02f, 4);
      var rotation = quaternion.AxisAngle(math.normalize(new float3(1f, 2f, 0.5f)), 1.3f);
      var target = quaternion.AxisAngle(new float3(0f, 1f, 0f), -2.1f);

      scalars[0] = gains.Stiffness;
      scalars[1] = gains.Damper;
      scalars[2] = StablePd.Acceleration(gains, 1f, 0.5f, -0.25f, 0.1f, 9.81f);
      scalars[3] = StablePd.AngularAcceleration(gains, 3f, 0.1f, -3f, 0.2f, 0f);
      scalars[4] = StablePd.Acceleration(gains, new float2(1f, -1f), new float2(0.5f), float2.zero, float2.zero).y;
      scalars[5] = MotionMath.WrapAngle(7f) + MotionMath.AngularVelocity(3f, -3f, 0.02f);

      var pidSettings = new PidSettings(1f, 0.5f, 0.2f);
      var pidState = new PidState();
      float output = 0f;
      for (int i = 0; i < 10; i++) {
        output = Pid.Compute(pidSettings, ref pidState, 0.02f, 0f, 1f);
      }
      scalars[6] = output;
      var angleState = new PidState();
      scalars[7] = Pid.ComputeAngle(pidSettings, ref angleState, 0.02f, 3f, -3f);

      vectors[0] = StablePd.Acceleration(gains, new float3(1f, 2f, 3f), new float3(0.1f), float3.zero, new float3(1f, 0f, 0f), new float3(0f, 9.81f, 0f));
      vectors[1] = StablePd.AngularAcceleration(gains, rotation, new float3(0.2f, -0.1f, 0.4f), target, new float3(0f, 0.5f, 0f));
      vectors[2] = MotionMath.AngularAccelerationToTorque(vectors[1], rotation, quaternion.AxisAngle(new float3(1f, 0f, 0f), 0.7f), new float3(1f, 4f, 12f));
      vectors[3] = MotionMath.ToRotationVector(MotionMath.Integrate(rotation, new float3(1f, 0f, 2f), 0.02f))
        + MotionMath.AngularVelocity(rotation, target, 0.02f);
    }

    [Test]
    public void CoreRunsInABurstCompiledJobAndMatchesManagedCode()
    {
      if (!BurstCompiler.IsEnabled) {
        Assert.Ignore("Burst compilation is disabled in this editor (Jobs > Burst > Enable Compilation).");
      }

      var scalars = new NativeArray<float>(ScalarCount, Allocator.TempJob);
      var vectors = new NativeArray<float3>(VectorCount, Allocator.TempJob);
      var managedFlag = new NativeArray<int>(1, Allocator.TempJob);
      var expectedScalars = new NativeArray<float>(ScalarCount, Allocator.Temp);
      var expectedVectors = new NativeArray<float3>(VectorCount, Allocator.Temp);
      try {
        new ControllersJob { Scalars = scalars, Vectors = vectors, RanAsManagedCode = managedFlag }.Run();
        Assert.AreEqual(0, managedFlag[0], "the job ran as managed code: the core failed to Burst-compile");

        Evaluate(ref expectedScalars, ref expectedVectors);
        for (int i = 0; i < ScalarCount; i++) {
          Assert.AreEqual(expectedScalars[i], scalars[i], 1e-4 * math.max(1f, math.abs(expectedScalars[i])), $"scalar {i}");
        }
        for (int i = 0; i < VectorCount; i++) {
          Assert.Less(math.length(expectedVectors[i] - vectors[i]), 1e-4f * math.max(1f, math.length(expectedVectors[i])), $"vector {i}");
        }
      } finally {
        scalars.Dispose();
        vectors.Dispose();
        managedFlag.Dispose();
        expectedScalars.Dispose();
        expectedVectors.Dispose();
      }
    }
  }
}
