using NUnit.Framework;
using Yohash.Propulsion.Core;

namespace Yohash.Propulsion.Entities.Tests
{
  public class PdGainsCacheTests
  {
    private const float Dt = 1f / 60f;
    private static readonly StablePdSettings Settings = new StablePdSettings(2f, 0.7f);

    [Test]
    public void DefaultCacheSolvesOnFirstUse()
    {
      var cache = new PdGainsCache();
      var gains = cache.Resolve(Settings, Dt, 4);
      var expected = StablePd.Gains(Settings, Dt, 4);

      Assert.IsTrue(gains.IsValid);
      Assert.AreEqual(expected.Stiffness, gains.Stiffness);
      Assert.AreEqual(expected.Damper, gains.Damper);
      Assert.AreEqual(Dt, gains.Dt);
      Assert.AreEqual(expected, cache.Gains);
    }

    [Test]
    public void ReusesTheGainsWhileTheInputsAreUnchanged()
    {
      var cache = new PdGainsCache();
      cache.Resolve(Settings, Dt, 4);

      // a sentinel only survives if Resolve doesn't re-solve
      cache.Gains.Stiffness = 12345f;
      Assert.AreEqual(12345f, cache.Resolve(Settings, Dt, 4).Stiffness);
      Assert.AreEqual(12345f, cache.Resolve(new StablePdSettings(2f, 0.7f), Dt, 4).Stiffness);
    }

    [TestCase(1f / 50f, 4, 2f, 0.7f, TestName = "ReSolvesWhenTheTimestepChanges")]
    [TestCase(1f / 60f, 1, 2f, 0.7f, TestName = "ReSolvesWhenTheSubStepCountChanges")]
    [TestCase(1f / 60f, 4, 3f, 0.7f, TestName = "ReSolvesWhenTheFrequencyChanges")]
    [TestCase(1f / 60f, 4, 2f, 1.0f, TestName = "ReSolvesWhenTheDampingChanges")]
    public void ReSolvesWhenAnInputChanges(float dt, int subSteps, float frequency, float damping)
    {
      var cache = new PdGainsCache();
      cache.Resolve(Settings, Dt, 4);
      cache.Gains.Stiffness = 12345f;

      var changed = new StablePdSettings(frequency, damping);
      var gains = cache.Resolve(changed, dt, subSteps);
      var expected = StablePd.Gains(changed, dt, subSteps);

      Assert.AreEqual(expected, gains);
      Assert.AreEqual(changed, cache.SolvedSettings);
      Assert.AreEqual(subSteps, cache.SolvedSubSteps);
    }

    [Test]
    public void InvalidInputGivesZeroGainsAndRecovers()
    {
      var cache = new PdGainsCache();
      Assert.IsTrue(cache.Resolve(Settings, Dt, 4).IsValid);

      Assert.IsFalse(cache.Resolve(Settings, 0f, 4).IsValid);
      Assert.IsFalse(cache.Resolve(Settings, -Dt, 4).IsValid);
      Assert.IsFalse(cache.Resolve(Settings, float.NaN, 4).IsValid);
      Assert.IsFalse(cache.Resolve(Settings, Dt, 0).IsValid);
      Assert.AreEqual(0f, cache.Resolve(Settings, Dt, 0).Stiffness);

      Assert.AreEqual(StablePd.Gains(Settings, Dt, 4), cache.Resolve(Settings, Dt, 4));
    }
  }
}
