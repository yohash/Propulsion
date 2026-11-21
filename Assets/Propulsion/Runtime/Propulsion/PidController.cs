using System;
using UnityEngine;

namespace Yohash.Propulsion
{
  /// <summary>
  /// *******************************************************************
  /// This code was adapted from https://vazgriz.com/621/pid-controllers/
  /// *******************************************************************
  ///
  /// A classic single-axis PID controller that outputs a normalized "throttle" in
  /// [outputMin, outputMax]. The caller scales it into a force or torque (e.g. throttle * power).
  ///
  /// Use this controller when you need what the StablePdController cannot provide:
  ///   - an integral term that removes steady-state error caused by unknown, persistent
  ///     disturbances (an unknown load, unmodelled friction, an unknown mass)
  ///   - a bounded output, matching an actuator with limited strength (an engine, a servo)
  ///
  /// Unlike the StablePdController, the gains here are not normalized by mass or timestep,
  /// so they are tuned by hand for each body. Large gains can make it oscillate or diverge.
  /// If the disturbance is known (gravity, for example), feeding it forward through the
  /// StablePdController is usually more accurate than integrating it out.
  ///
  /// Example usage for a linear controller:
  ///
  ///    void FixedUpdate()
  ///    {
  ///      float throttle = controller.Compute(Time.fixedDeltaTime, rigidbody.position.y, targetPosition.y);
  ///      rigidbody.AddForce(new Vector3(0, throttle * power, 0));
  ///    }
  ///
  /// Example usage for an angular controller (angles in degrees):
  ///
  ///    void FixedUpdate()
  ///    {
  ///      var targetPosition = target.position;
  ///      targetPosition.y = rigidbody.position.y;    //ignore difference in Y
  ///      var targetDir = (targetPosition - rigidbody.position).normalized;
  ///      var forwardDir = rigidbody.rotation * Vector3.forward;
  ///
  ///      var currentAngle = Vector3.SignedAngle(Vector3.forward, forwardDir, Vector3.up);
  ///      var targetAngle = Vector3.SignedAngle(Vector3.forward, targetDir, Vector3.up);
  ///
  ///      float input = controller.ComputeAngle(Time.fixedDeltaTime, currentAngle, targetAngle);
  ///      rigidbody.AddTorque(new Vector3(0, input * power, 0));
  ///    }
  ///
  /// </summary>
  [Serializable]
  public class PidController
  {
    public enum DerivativeMeasurement
    {
      // derivative of the measured value; avoids a "kick" when the target jumps
      Velocity,
      // derivative of the error; also responds to target motion
      ErrorRateOfChange
    }

    [Header("Assign these coefficients")]
    [SerializeField] private float proportionalGain = 1;
    [SerializeField] private float integralGain = 1;
    [SerializeField] private float derivativeGain = 1;

    [SerializeField] private float outputMin = -1;
    [SerializeField] private float outputMax = 1;
    [Tooltip("The largest contribution the integral term may make to the output.\n" +
      "Setting this equal to outputMax lets the integral alone saturate the output.")]
    [SerializeField, Min(0f)] private float integralSaturation = 1;
    [SerializeField] private DerivativeMeasurement derivativeMeasurement;

    [Header("Internal Variables")]
    [SerializeField] private float valueLast;
    [SerializeField] private float errorLast;
    [SerializeField] private float integrationStored;
    [SerializeField] private float velocity;  //only used for the info display
    [SerializeField] private bool derivativeInitialized;

    public PidController() { }

    public PidController(
      float proportionalGain,
      float integralGain,
      float derivativeGain,
      float outputMin = -1,
      float outputMax = 1,
      float integralSaturation = 1,
      DerivativeMeasurement derivativeMeasurement = DerivativeMeasurement.Velocity
    )
    {
      this.proportionalGain = proportionalGain;
      this.integralGain = integralGain;
      this.derivativeGain = derivativeGain;
      this.outputMin = outputMin;
      this.outputMax = outputMax;
      this.integralSaturation = Mathf.Max(0f, integralSaturation);
      this.derivativeMeasurement = derivativeMeasurement;
    }

    /// <summary>
    /// Clears all internal state: the stored integral and the derivative history.
    /// Call this when the controller is re-targeted or re-enabled, so stale history
    /// does not produce a jolt.
    /// </summary>
    public void Reset()
    {
      valueLast = 0;
      errorLast = 0;
      integrationStored = 0;
      velocity = 0;
      derivativeInitialized = false;
    }

    /// <summary>
    /// Update function for linear motion along a single axis.
    /// Returns a throttle in [outputMin, outputMax].
    /// </summary>
    public float Compute(float dt, float currentValue, float targetValue)
    {
      ValidateTimestep(dt);

      float error = targetValue - currentValue;
      float errorRateOfChange = (error - errorLast) / dt;
      float valueRateOfChange = (currentValue - valueLast) / dt;

      errorLast = error;
      valueLast = currentValue;

      return Evaluate(dt, error, errorRateOfChange, valueRateOfChange);
    }

    /// <summary>
    /// Update function for angular motion around a single axis. Angles are in degrees,
    /// and the error always takes the shortest way around.
    /// Returns a throttle in [outputMin, outputMax].
    /// </summary>
    public float ComputeAngle(float dt, float currentAngle, float targetAngle)
    {
      ValidateTimestep(dt);

      float error = Mathf.DeltaAngle(currentAngle, targetAngle);
      float errorRateOfChange = Mathf.DeltaAngle(errorLast, error) / dt;
      float valueRateOfChange = Mathf.DeltaAngle(valueLast, currentAngle) / dt;

      errorLast = error;
      valueLast = currentAngle;

      return Evaluate(dt, error, errorRateOfChange, valueRateOfChange);
    }

    private float Evaluate(float dt, float error, float errorRateOfChange, float valueRateOfChange)
    {
      // P term
      float P = proportionalGain * error;

      // D term. The first update has no history, so it contributes nothing.
      float D = 0;
      if (derivativeInitialized) {
        velocity = valueRateOfChange;
        D = derivativeGain * (derivativeMeasurement == DerivativeMeasurement.Velocity
          ? -valueRateOfChange
          : errorRateOfChange);
      } else {
        derivativeInitialized = true;
      }

      // I term, with two anti-windup measures:
      //  1. the integral's contribution to the output is limited to +/- integralSaturation
      //  2. while the output is saturated, the integral may not grow further into saturation
      float integralLimit = integralGain != 0 ? integralSaturation / Mathf.Abs(integralGain) : 0;
      float candidate = Mathf.Clamp(integrationStored + error * dt, -integralLimit, integralLimit);

      float unclamped = P + integralGain * candidate + D;
      bool windingUp =
        (unclamped > outputMax && integralGain * (candidate - integrationStored) > 0) ||
        (unclamped < outputMin && integralGain * (candidate - integrationStored) < 0);

      if (!windingUp) {
        integrationStored = candidate;
      }
      integrationStored = Mathf.Clamp(integrationStored, -integralLimit, integralLimit);

      float I = integralGain * integrationStored;

      return Mathf.Clamp(P + I + D, outputMin, outputMax);
    }

    private static void ValidateTimestep(float dt)
    {
      if (!(dt > 0f)) {
        throw new ArgumentOutOfRangeException(nameof(dt), dt, "Timestep must be positive.");
      }
    }
  }
}
