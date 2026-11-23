using UnityEngine;
#if PROPULSION_INPUT_SYSTEM && ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace Yohash.Propulsion.Samples.VehiclePdFollower
{
  /// <summary>
  /// Drives the VehicleMotor from the keyboard: W / S accelerate and brake, A / D steer.
  ///
  /// Works with any Active Input Handling setting (Project Settings > Player > Other Settings):
  ///   - Input System Package (New), or Both: reads the Input System's Keyboard. The package is
  ///     detected through the PROPULSION_INPUT_SYSTEM define, set in this sample's asmdef.
  ///   - Input Manager (Old): reads the legacy UnityEngine.Input class.
  /// This package itself does not depend on the Input System package.
  /// </summary>
  [RequireComponent(typeof(VehicleMotor))]
  public class VehicleControllerAiScene : MonoBehaviour
  {
    [SerializeField] private VehicleMotor motor;

    private void Awake()
    {
      motor = GetComponent<VehicleMotor>();
#if !ENABLE_LEGACY_INPUT_MANAGER && !(PROPULSION_INPUT_SYSTEM && ENABLE_INPUT_SYSTEM)
      Debug.LogWarning(
        "VehicleControllerAiScene: no keyboard input is available. Install the Input System package, or set " +
        "Active Input Handling to Input Manager (Old) or Both.", this);
#endif
    }

    private void FixedUpdate()
    {
      ReadKeys(out float accel, out float turn);
      motor.ApplyForces(accel, turn);
    }

    // accel: +1 forward (W), -1 back (S). turn: +1 right (D), -1 left (A).
    private static void ReadKeys(out float accel, out float turn)
    {
      accel = 0;
      turn = 0;

#if PROPULSION_INPUT_SYSTEM && ENABLE_INPUT_SYSTEM
      var keyboard = Keyboard.current;
      if (keyboard != null) {
        if (keyboard.wKey.isPressed) { accel++; }
        if (keyboard.sKey.isPressed) { accel--; }
        if (keyboard.aKey.isPressed) { turn--; }
        if (keyboard.dKey.isPressed) { turn++; }
        return;
      }
#endif

#if ENABLE_LEGACY_INPUT_MANAGER
      if (Input.GetKey(KeyCode.W)) { accel++; }
      if (Input.GetKey(KeyCode.S)) { accel--; }
      if (Input.GetKey(KeyCode.A)) { turn--; }
      if (Input.GetKey(KeyCode.D)) { turn++; }
#endif
    }
  }
}
