using UnityEngine;
using UnityEngine.InputSystem;
using System.Collections;

public class ApplicationShutdownHandler : MonoBehaviour
{
    private void OnEnable()
    {
        Application.wantsToQuit += OnWantsToQuit;
    }

    private void OnDisable()
    {
        Application.wantsToQuit -= OnWantsToQuit;
    }

    private bool OnWantsToQuit()
    {
        StartCoroutine(CleanupAndQuit());
        return false; // Prevent immediate quit, let our coroutine handle it
    }

    private IEnumerator CleanupAndQuit()
    {
        // Disable all input devices first
        var devices = InputSystem.devices;
        foreach (var device in devices)
        {
            if (device != null && device.enabled)
            {
                InputSystem.DisableDevice(device);
            }
        }

        // Wait a frame to ensure input system has processed the disable requests
        yield return null;

        // Remove all devices
        foreach (var device in devices)
        {
            if (device != null)
            {
                InputSystem.RemoveDevice(device);
            }
        }

        // Wait another frame for cleanup
        yield return null;

        // Final cleanup of the Input System
        InputSystem.Update();

        // Now we can safely quit
        Application.Quit();
    }
}