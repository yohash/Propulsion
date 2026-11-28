using ECSPhysics2D;
using NUnit.Framework;
using Unity.Entities;
using Unity.Mathematics;
using Yohash.Propulsion.Core;

namespace Yohash.Propulsion.Entities.Tests.Physics2D
{
  /// <summary>
  /// The ECSPhysics2D backend in a real Entities world, against the real Box2D v3 engine
  /// (UnityEngine.LowLevelPhysics2D), with its default 4 sub-steps.
  /// </summary>
  public class Physics2DBackendTests
  {
    private Physics2DTestWorld test;

    [SetUp]
    public void SetUp() => test = new Physics2DTestWorld();

    [TearDown]
    public void TearDown() => test.Dispose();

    private PdPosition2D AddPosition(Entity entity, float frequency, float damping, float2 target, bool compensateGravity)
    {
      var controller = new PdPosition2D {
        Settings = new StablePdSettings(frequency, damping),
        TargetPosition = target,
        CompensateGravity = compensateGravity,
      };
      test.EntityManager.AddComponentData(entity, controller);
      return controller;
    }

    private PdPosition2D Position(Entity entity) => test.EntityManager.GetComponentData<PdPosition2D>(entity);

    // ******************************************************************
    //    SET-UP AND WIRING
    // ******************************************************************

    [Test]
    public void GainsAreSolvedForTheEnginesTimestepAndSubSteps()
    {
      var ball = test.CreateBall(float2.zero, gravityScale: 0f);
      AddPosition(ball, 2f, 1f, new float2(1f, 0f), compensateGravity: false);
      test.Step();

      var cache = Position(ball).Cache;
      Assert.AreEqual(test.Dt, cache.Gains.Dt, "ECSPhysics2D's FixedDeltaTime");
      Assert.AreEqual(4, cache.SolvedSubSteps, "the default world's sub-steps");
      Assert.Greater(test.Body(ball).mass, 0f, "shapes exist before control runs");
    }

    // ******************************************************************
    //    MEASURED IN THE ENGINE
    // ******************************************************************

    [TestCase(2f, 0.3f)]
    [TestCase(6f, 0.2f)]
    [TestCase(10f, 0.1f)]
    public void PositionHasTheRequestedFrequencyAndDampingInTheEngine(float frequency, float damping)
    {
      var ball = test.CreateBall(float2.zero, gravityScale: 0f);
      AddPosition(ball, frequency, damping, new float2(1f, 0f), compensateGravity: false);
      test.Step();   // creates the body; the error is recorded from here

      var error = new float[90];
      for (int k = 0; k < error.Length; k++) {
        error[k] = test.CenterOfMass(ball).x - 1f;
        test.Step();
      }

      var measured = MeasuredResponse.Fit(error, test.Dt);
      TestContext.WriteLine($"requested {frequency} Hz, damping {damping}: measured {measured}");
      Assert.AreEqual(frequency, measured.Frequency, 0.005 * frequency);
      Assert.AreEqual(damping, measured.Damping, 0.01);
    }

    [Test]
    public void RotationHasTheRequestedFrequencyAndDampingInTheEngine()
    {
      const float frequency = 2f, damping = 0.3f;
      var ball = test.CreateBall(float2.zero, gravityScale: 0f);
      test.EntityManager.AddComponentData(ball, new PdRotation2D {
        Settings = new StablePdSettings(frequency, damping),
        TargetAngle = 1f,
      });
      test.Step();

      var error = new float[90];
      for (int k = 0; k < error.Length; k++) {
        error[k] = test.Angle(ball) - 1f;
        test.Step();
      }

      // unconverted degrees per second would make the damping ~57x too strong
      var measured = MeasuredResponse.Fit(error, test.Dt);
      TestContext.WriteLine($"requested {frequency} Hz, damping {damping}: measured {measured}");
      Assert.AreEqual(frequency, measured.Frequency, 0.01 * frequency);
      Assert.AreEqual(damping, measured.Damping, 0.01);
    }

    [Test]
    public void RotationTurnsTheShortWayAcrossTheWrap()
    {
      var ball = test.CreateBall(float2.zero, gravityScale: 0f);
      test.EntityManager.AddComponentData(ball, new PdRotation2D {
        Settings = new StablePdSettings(2f, 1f),
        TargetAngle = -3f,
      });
      // start at +3 rad: the short way to -3 is +0.28 rad through PI
      var transform = test.EntityManager.GetComponentData<Unity.Transforms.LocalTransform>(ball);
      transform.Rotation = quaternion.RotateZ(3f);
      test.EntityManager.SetComponentData(ball, transform);
      test.Step();

      for (int k = 0; k < 180; k++) {
        Assert.Greater(math.abs(test.Angle(ball)), 2.9f, $"step {k}: went the long way");
        test.Step();
      }
      Assert.AreEqual(0f, MotionMath.WrapAngle(test.Angle(ball) + 3f), 1e-3f);
    }

    // ******************************************************************
    //    GRAVITY
    // ******************************************************************

    [Test]
    public void HoldsItsTargetUnderGravityWhenCompensating()
    {
      var start = new float2(0f, 5f);
      var ball = test.CreateBall(start, gravityScale: 1f);
      AddPosition(ball, 2f, 1f, start, compensateGravity: true);
      for (int k = 0; k < 300; k++) { test.Step(); }

      Assert.Less(math.length(test.CenterOfMass(ball) - start), 1e-3f);
    }

    [Test]
    public void FollowsTheBodysGravityScale()
    {
      // e.g. gravity that differs per team or per body: a negative scale flips it
      var start = new float2(0f, 5f);
      var ball = test.CreateBall(start, gravityScale: -2f);
      AddPosition(ball, 2f, 1f, start, compensateGravity: true);
      for (int k = 0; k < 300; k++) { test.Step(); }

      Assert.Less(math.length(test.CenterOfMass(ball) - start), 1e-3f);
    }

    [Test]
    public void SagsByGravityOverStiffnessWithoutCompensation()
    {
      var start = new float2(0f, 5f);
      var ball = test.CreateBall(start, gravityScale: 1f);
      AddPosition(ball, 2f, 1f, start, compensateGravity: false);
      for (int k = 0; k < 300; k++) { test.Step(); }

      float gravity = -test.Physics.GetWorld(0).gravity.y;
      float sag = gravity / Position(ball).Cache.Gains.Stiffness;
      Assert.AreEqual(start.y - sag, test.CenterOfMass(ball).y, 1e-3f);
    }

    [Test]
    public void ThePositionControllerDrivesTheCentreOfMass()
    {
      // the circle sits 0.5 m right of the body's origin, so the centre of mass does too
      var target = new float2(0f, 5f);
      var ball = test.CreateBall(target, gravityScale: 1f, shapeOffset: new float2(0.5f, 0f));
      AddPosition(ball, 2f, 1f, target, compensateGravity: true);
      for (int k = 0; k < 300; k++) { test.Step(); }

      Assert.Less(math.length(test.CenterOfMass(ball) - target), 1e-3f, "the centre of mass is on target");
      Assert.Less(math.length(test.Origin(ball) - (target - new float2(0.5f, 0f))), 1e-3f,
        "so the origin, where the target was baked, has moved by the offset");
    }

    // ******************************************************************
    //    SLEEPING
    // ******************************************************************

    [Test]
    public void ACompensatedBodyOnItsTargetFallsAsleepAndStaysAsleepUntilTheTargetMoves()
    {
      var start = new float2(0f, 5f);
      var ball = test.CreateBall(start, gravityScale: 1f);
      AddPosition(ball, 2f, 1f, start, compensateGravity: true);
      test.EntityManager.AddComponentData(ball, new PdRotation2D { Settings = new StablePdSettings(2f, 1f), TargetAngle = 0f });

      int asleepAt = -1;
      for (int k = 0; k < 600 && asleepAt < 0; k++) {
        test.Step();
        if (!test.Awake(ball)) { asleepAt = k; }
      }
      Assert.GreaterOrEqual(asleepAt, 0, "Box2D puts a body at rest on its target to sleep");

      // the controllers' output includes -gravity, but the net acceleration is zero: no wake-ups
      for (int k = 0; k < 120; k++) {
        test.Step();
        Assert.IsFalse(test.Awake(ball), $"woken {k + 1} steps after falling asleep");
      }
      Assert.Less(math.length(test.CenterOfMass(ball) - start), 1e-3f, "asleep where it was held");

      // moving the target wakes it, and it goes there
      var controller = Position(ball);
      controller.TargetPosition = start + new float2(1f, 0f);
      test.EntityManager.SetComponentData(ball, controller);
      test.Step();
      Assert.IsTrue(test.Awake(ball), "woken by the new target");
      for (int k = 0; k < 300; k++) { test.Step(); }
      Assert.Less(math.length(test.CenterOfMass(ball) - controller.TargetPosition), 1e-3f);
    }

    // ******************************************************************
    //    FORCES COMBINE
    // ******************************************************************

    [Test]
    public void OtherForcesStillActAndSettleToAnOffset()
    {
      // ECSPhysics2D's own continuous force, applied to the same body: both add to Box2D's
      // accumulator, and the controller resists it like a spring, settling F / (m K) off target
      var ball = test.CreateBall(float2.zero, gravityScale: 0f);
      AddPosition(ball, 2f, 1f, float2.zero, compensateGravity: false);
      test.EntityManager.AddComponentData(ball, PhysicsForce.CreateCentralForce(new float2(3f, 0f)));
      for (int k = 0; k < 300; k++) { test.Step(); }

      float offset = 3f / (test.Body(ball).mass * Position(ball).Cache.Gains.Stiffness);
      Assert.AreEqual(offset, test.CenterOfMass(ball).x, 1e-3f);
      Assert.AreEqual(0f, test.CenterOfMass(ball).y, 1e-3f);
    }

    // ******************************************************************
    //    TARGET VELOCITY
    // ******************************************************************

    private float TrackingLag(bool derive)
    {
      var ball = test.CreateBall(float2.zero, gravityScale: 0f);
      AddPosition(ball, 2f, 1f, float2.zero, compensateGravity: false);
      if (derive) { test.EntityManager.AddComponentData(ball, new PdPositionHistory2D()); }

      // the target moves at 1 m/s; only its position is written
      float time = 0f;
      for (int k = 0; k < 300; k++) {
        var controller = Position(ball);
        controller.TargetPosition = new float2(time, 0f);
        test.EntityManager.SetComponentData(ball, controller);
        test.Step();
        time += test.Dt;
      }
      // the error at the start of the next step, against that step's target
      return test.CenterOfMass(ball).x - time;
    }

    [Test]
    public void TheHistoryComponentTracksAMovingTargetWithoutLag()
    {
      Assert.AreEqual(0f, TrackingLag(derive: true), 2e-3f);
    }

    [Test]
    public void WithoutTheHistoryAMovingTargetIsTrailed()
    {
      Assert.Less(TrackingLag(derive: false), -0.1f);
    }

    [Test]
    public void DisablingTheHistoryLeavesTheSuppliedVelocityAlone()
    {
      var ball = test.CreateBall(float2.zero, gravityScale: 0f);
      var controller = AddPosition(ball, 2f, 1f, new float2(1f, 0f), compensateGravity: false);
      controller.TargetVelocity = new float2(0.5f, 0f);
      test.EntityManager.SetComponentData(ball, controller);
      test.EntityManager.AddComponentData(ball, new PdPositionHistory2D());
      test.EntityManager.SetComponentEnabled<PdPositionHistory2D>(ball, false);

      test.Step();
      Assert.AreEqual(new float2(0.5f, 0f), Position(ball).TargetVelocity);

      test.EntityManager.SetComponentEnabled<PdPositionHistory2D>(ball, true);
      test.Step();
      Assert.AreEqual(float2.zero, Position(ball).TargetVelocity, "first enabled step derives zero");
    }
  }
}
