using NUnit.Framework;
using UnityEngine;

namespace Yohash.Propulsion.Tests
{
  /// <summary>
  /// The steering inputs VehiclePdController feeds its turn controller. Before 0.6 they were
  /// raw differences of headings in [-180, 180], so they spiked when a heading crossed +/-180,
  /// on the first update, and when the facing vector vanished on reaching the target.
  /// </summary>
  public class VehiclePdControllerTests
  {
    private const float Dt = 0.02f;

    private static VehiclePdController NewController()
    {
      return new VehiclePdController(1f, 1f, new StablePdController(2f, 1f));
    }

    // a vehicle at the origin, heading `yaw` degrees clockwise from world forward
    private static Pose Heading(float yaw)
    {
      return new Pose(Vector3.zero, Quaternion.AngleAxis(yaw, Vector3.up));
    }

    private static Trajectory At(Vector3 position)
    {
      return new Trajectory { Position = position, Velocity = Vector3.zero };
    }

    // a point `distance` from the origin, in the direction `yaw` degrees from world forward
    private static Vector3 Toward(float yaw, float distance)
    {
      return Quaternion.AngleAxis(yaw, Vector3.up) * Vector3.forward * distance;
    }

    private static float Steer(VehiclePdController c, float yaw, Vector3 target)
    {
      return c.Update(Dt, Heading(yaw), At(target), At(Vector3.zero)).Item2;
    }

    [Test]
    public void HeadingCrossingPlusMinus180DoesNotSpike()
    {
      // the vehicle turns steadily through 180 degrees while the target stays at 90 degrees
      var c = NewController();
      var target = Toward(90f, 100f);

      float previous = Steer(c, 170f, target);
      float maxSteer = 0f;
      float maxChange = 0f;
      for (float yaw = 171f; yaw <= 200f; yaw += 1f) {
        float steer = Steer(c, yaw, target);
        maxSteer = Mathf.Max(maxSteer, Mathf.Abs(steer));
        maxChange = Mathf.Max(maxChange, Mathf.Abs(steer - previous));
        previous = steer;
      }

      // smooth across the wrap (~250 peak, <20 per step). The raw 360 degree jump used to
      // produce a velocity of 18,000 deg/s and a steer of over 5,000 at the crossing.
      Assert.Less(maxSteer, 400f);
      Assert.Less(maxChange, 40f);
    }

    [Test]
    public void FirstUpdateStartsFromRest()
    {
      // heading 120 degrees, target at 30 degrees: the first update must match a steady
      // update, not difference against the 0 degree initial values
      var c = NewController();
      var target = Toward(30f, 10f);

      float first = Steer(c, 120f, target);
      float steady = Steer(c, 120f, target);
      Assert.AreEqual(steady, first, 1e-3);

      // Reset forgets the history, so the next update starts from rest again
      Steer(c, -150f, Toward(0f, 10f));
      c.Reset();
      Assert.AreEqual(steady, Steer(c, 120f, target), 1e-3);
    }

    [Test]
    public void ReachingTheTargetHoldsTheLastFacing()
    {
      // approach with the target dead ahead at 90 degrees...
      var c = NewController();
      for (int i = 0; i < 5; i++) {
        Steer(c, 90f, Toward(90f, 10f));
      }

      // ...then sit exactly on it with matching velocity: the facing vector is zero. The
      // previous target facing (90, the current heading) must be held, so there's nothing
      // to steer. SignedAngle of the zero vector would otherwise snap it to 0 degrees.
      float steer = Steer(c, 90f, Vector3.zero);
      Assert.AreEqual(0f, steer, 1e-3);
    }

    [Test]
    public void SittingOnTheTargetWithContactJitterDoesNotSpin()
    {
      // In play, a vehicle parked on its target never has an exactly-zero error: contact jitter
      // leaves a few millimetres in a random direction. Close the loop with a simple yaw model,
      // where the clamped steer drives the yaw rate as VehicleAi and VehicleMotor do, and check
      // that the heading holds.
      var c = NewController();
      var random = new System.Random(1234);
      // deg/s^2 per unit of (clamped) steer. Stiff enough that the old exact-zero guard spins
      // through ~280 degrees in these 6 s, close to the ~330 deg/s spin seen in the editor.
      const float yawAcceleration = 500f;
      float yaw = 131f;
      float yawRate = 0f;

      // arrive from an approach whose facing differs from the heading (as after a teleport)
      Steer(c, yaw, Toward(40f, 10f));

      float minYaw = yaw, maxYaw = yaw;
      for (int i = 0; i < 300; i++) {
        var jitter = new Vector3(
          (float)(random.NextDouble() * 2 - 1) * 0.002f,
          0f,
          (float)(random.NextDouble() * 2 - 1) * 0.002f);
        var target = new Trajectory {
          Position = jitter,
          Velocity = new Vector3((float)(random.NextDouble() * 2 - 1) * 0.01f, 0f, 0f),
        };
        float steer = c.Update(Dt, Heading(yaw), target, At(Vector3.zero)).Item2;

        yawRate += Mathf.Clamp(steer, -1f, 1f) * yawAcceleration * Dt;
        yaw += yawRate * Dt;
        minYaw = Mathf.Min(minYaw, yaw);
        maxYaw = Mathf.Max(maxYaw, yaw);
      }

      // the heading latched on arrival holds; without the dead-zone it wanders through 360
      Assert.Less(maxYaw - minYaw, 1f, "the parked vehicle turned");
    }

    [Test]
    public void LeavingTheArrivalHoldDoesNotSpike()
    {
      // park on the target, holding a heading of 90 degrees...
      var c = NewController();
      for (int i = 0; i < 5; i++) {
        Steer(c, 90f, Vector3.zero);
      }

      // ...then the target moves off to 30 degrees. The facing jumps from the held 90 to 30;
      // that is a new target, not a rotation, so the first steer must match a steady one.
      var target = Toward(30f, 10f);
      float leaving = Steer(c, 90f, target);
      float steady = Steer(c, 90f, target);
      Assert.AreEqual(steady, leaving, 1e-3);
    }

    [Test]
    public void ZeroFacingOnTheFirstUpdateUsesTheHeading()
    {
      var c = NewController();
      Assert.AreEqual(0f, Steer(c, -45f, Vector3.zero), 1e-3);
    }
  }
}
