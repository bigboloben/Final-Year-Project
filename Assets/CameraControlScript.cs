using UnityEngine;
using System;

public class CameraControlScript : MonoBehaviour
{
    private Camera[] cameras = new Camera[2];
    private CarControls controls;
    private Action<UnityEngine.InputSystem.InputAction.CallbackContext> performedCallback;
    private Action<UnityEngine.InputSystem.InputAction.CallbackContext> canceledCallback;

    void Awake()
    {
        controls = new CarControls();

        // Create the callbacks once and store them as variables
        performedCallback = ctx =>
        {
            if (cameras[0] != null && cameras[1] != null)
            {
                cameras[0].enabled = false;
                cameras[1].enabled = true;
            }
        };

        canceledCallback = ctx =>
        {
            if (cameras[0] != null && cameras[1] != null)
            {
                cameras[0].enabled = true;
                cameras[1].enabled = false;
            }
        };
    }

    public void SetupCameras(Camera camera0, Camera camera1)
    {
        // Remove any previous callbacks if they exist
        RemoveCallbacks();

        cameras[0] = camera0;
        cameras[1] = camera1;

        // Only add callbacks if the cameras are valid
        if (camera0 != null && camera1 != null)
        {
            controls.Camera.LookBehind.performed += performedCallback;
            controls.Camera.LookBehind.canceled += canceledCallback;
        }
    }

    private void RemoveCallbacks()
    {
        // Safely remove existing callbacks to prevent calling with destroyed cameras
        if (performedCallback != null)
        {
            controls.Camera.LookBehind.performed -= performedCallback;
        }

        if (canceledCallback != null)
        {
            controls.Camera.LookBehind.canceled -= canceledCallback;
        }
    }

    private void OnEnable()
    {
        controls.Enable();
    }

    private void OnDisable()
    {
        controls.Disable();
    }

    private void OnDestroy()
    {
        // Make sure to remove all callbacks when the script is destroyed
        RemoveCallbacks();

        // Clear camera references
        cameras[0] = null;
        cameras[1] = null;
    }
}