﻿using System;
using UnityEngine;

namespace Yohash.Propulsion
{
  /// <summary>
  /// *******************************************************************
  /// This code was adapted from https://vazgriz.com/621/pid-controllers/
  /// *******************************************************************
  ///
  /// Example usage for cartesian controller:
  ///
  ///    void FixedUpdate()
  ///    {
  ///      float throttle = controller.Update(Time.fixedDeltaTime, rigidbody.position.y, targetPosition.y);
  ///      rigidbody.AddForce(new Vector3(0, throttle * power, 0));
  ///    }
  ///
  /// Example usage for angular controller:
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
  ///      float input = controller.UpdateAngle(Time.fixedDeltaTime, currentAngle, targetAngle);
  ///      rigidbody.AddTorque(new Vector3(0, input * power, 0));
  ///    }
  ///
  /// </summary>

  [Serializable]
  public class PIDController
  {
    public enum DerivativeMeasurement
    {
      Velocity,
      ErrorRateOfChange
    }

    //PID coefficients
    [SerializeField] private float proportionalGain;
    [SerializeField] private float integralGain;
    [SerializeField] private float derivativeGain;

    [SerializeField] private float outputMin = -1;
    [SerializeField] private float outputMax = 1;
    [SerializeField] private float integralSaturation;
    [SerializeField] private DerivativeMeasurement derivativeMeasurement;

    [SerializeField] private float valueLast;
    [SerializeField] private float errorLast;
    [SerializeField] private float integrationStored;
    [SerializeField] private float velocity;  //only used for the info display
    [SerializeField] private bool derivativeInitialized;

    public void Reset()
    {
      derivativeInitialized = false;
    }

    /// <summary>
    /// Update function for linear motion along some major axiss
    /// </summary>
    /// <param name="dt"></param>
    /// <param name="currentValue"></param>
    /// <param name="targetValue"></param>
    public float Update(float dt, float currentValue, float targetValue)
    {
      if (dt <= 0) throw new ArgumentOutOfRangeException(nameof(dt));

      float error = targetValue - currentValue;

      //calculate P term
      float P = proportionalGain * error;

      //calculate I term
      integrationStored = Mathf.Clamp(integrationStored + (error * dt), -integralSaturation, integralSaturation);
      float I = integralGain * integrationStored;

      //calculate both D terms
      float errorRateOfChange = (error - errorLast) / dt;
      errorLast = error;

      float valueRateOfChange = (currentValue - valueLast) / dt;
      valueLast = currentValue;
      velocity = valueRateOfChange;

      //choose D term to use
      float deriveMeasure = 0;

      if (derivativeInitialized) {
        if (derivativeMeasurement == DerivativeMeasurement.Velocity) {
          deriveMeasure = -valueRateOfChange;
        } else {
          deriveMeasure = errorRateOfChange;
        }
      } else {
        derivativeInitialized = true;
      }

      float D = derivativeGain * deriveMeasure;

      float result = P + I + D;

      return Mathf.Clamp(result, outputMin, outputMax);
    }

    /// <summary>
    /// Update function for angular motion
    /// </summary>
    /// <param name="dt"></param>
    /// <param name="currentAngle"></param>
    /// <param name="targetAngle"></param>
    public float UpdateAngle(float dt, float currentAngle, float targetAngle)
    {
      if (dt <= 0) throw new ArgumentOutOfRangeException(nameof(dt));
      float error = angleDifference(targetAngle, currentAngle);
      errorLast = error;

      //calculate P term
      float P = proportionalGain * error;

      //calculate I term
      integrationStored = Mathf.Clamp(integrationStored + (error * dt), -integralSaturation, integralSaturation);
      float I = integralGain * integrationStored;

      //calculate both D terms
      float errorRateOfChange = angleDifference(error, errorLast) / dt;
      errorLast = error;

      float valueRateOfChange = angleDifference(currentAngle, valueLast) / dt;
      valueLast = currentAngle;
      velocity = valueRateOfChange;

      //choose D term to use
      float deriveMeasure = 0;

      if (derivativeInitialized) {
        if (derivativeMeasurement == DerivativeMeasurement.Velocity) {
          deriveMeasure = -valueRateOfChange;
        } else {
          deriveMeasure = errorRateOfChange;
        }
      } else {
        derivativeInitialized = true;
      }

      float D = derivativeGain * deriveMeasure;

      float result = P + I + D;

      return Mathf.Clamp(result, outputMin, outputMax);
    }

    /// <summary>
    /// Calculate modular difference, and remap to [-180, 180]
    /// </summary>
    /// <param name="a"></param>
    /// <param name="b"></param>
    /// <returns></returns>
    private float angleDifference(float a, float b)
    {
      return (a - b + 540) % 360 - 180;
    }
  }
}