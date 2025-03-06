using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;
using System.Linq;
using TMPro;

public class SimulationSpeedController : MonoBehaviour
{
    [Header("Speed Settings")]
    [Range(0.1f, 10f)]
    public float defaultTimeScale = 1.0f;
    [Range(0.05f, 1.0f)]
    public float slowTimeScale = 0.25f;
    [Range(1.0f, 10f)]
    public float fastTimeScale = 3.0f;

    [Header("UI Elements")]
    public TextMeshProUGUI speedText;
    public KeyCode speedUpKey = KeyCode.Equals; // '='
    public KeyCode speedDownKey = KeyCode.Minus; // '-'
    public KeyCode resetSpeedKey = KeyCode.Backspace;
    public KeyCode slowModeKey = KeyCode.RightShift;
    public KeyCode fastModeKey = KeyCode.RightControl;

    [Header("Debug Settings")]
    public KeyCode toggleDebugKey = KeyCode.F1;
    public KeyCode cycleAgentKey = KeyCode.Tab;
    public KeyCode pauseKey = KeyCode.P;

    private List<CarAgent> allAgents = new List<CarAgent>();
    private int focusedAgentIndex = 0;
    private bool isPaused = false;
    private bool isRewardLoggingEnabled = true;
    private bool isResetLoggingEnabled = true;

    void Start()
    {
        // Set initial time scale
        Time.timeScale = defaultTimeScale;
        UpdateSpeedText();

        // Find all car agents in the scene
        FindAllAgents();

        Debug.Log("Simulation Speed Controller initialized:");
        Debug.Log("  Speed Controls: '+'/'-' to adjust, Backspace to reset");
        Debug.Log("  Right Shift = Slow mode, Right Ctrl = Fast mode");
        Debug.Log("  F1 = Toggle debug, Tab = Focus next agent, P = Pause");
    }

    void Update()
    {
        HandleSpeedControls();
        HandleDebugControls();
    }

    void HandleSpeedControls()
    {
        bool changed = false;

        // Speed up
        if (Input.GetKeyDown(speedUpKey))
        {
            Time.timeScale = Mathf.Min(Time.timeScale + 0.25f, 10f);
            changed = true;
        }

        // Slow down
        if (Input.GetKeyDown(speedDownKey))
        {
            Time.timeScale = Mathf.Max(Time.timeScale - 0.25f, 0.1f);
            changed = true;
        }

        // Reset to default
        if (Input.GetKeyDown(resetSpeedKey))
        {
            Time.timeScale = defaultTimeScale;
            changed = true;
        }

        // Slow mode (hold key)
        if (Input.GetKeyDown(slowModeKey))
        {
            Time.timeScale = slowTimeScale;
            changed = true;
        }
        else if (Input.GetKeyUp(slowModeKey))
        {
            Time.timeScale = defaultTimeScale;
            changed = true;
        }

        // Fast mode (hold key)
        if (Input.GetKeyDown(fastModeKey))
        {
            Time.timeScale = fastTimeScale;
            changed = true;
        }
        else if (Input.GetKeyUp(fastModeKey))
        {
            Time.timeScale = defaultTimeScale;
            changed = true;
        }

        // Pause/unpause
        if (Input.GetKeyDown(pauseKey))
        {
            isPaused = !isPaused;
            Time.timeScale = isPaused ? 0f : defaultTimeScale;
            changed = true;
        }

        if (changed)
        {
            UpdateSpeedText();
        }
    }

    void HandleDebugControls()
    {
        // Toggle debug logging
        if (Input.GetKeyDown(toggleDebugKey))
        {
            isRewardLoggingEnabled = !isRewardLoggingEnabled;
            isResetLoggingEnabled = !isResetLoggingEnabled;

            // Update all agents
            foreach (var agent in allAgents)
            {
                if (agent != null)
                {
                    agent.enableRewardLogging = isRewardLoggingEnabled;
                    agent.enableResetReasonLogging = isResetLoggingEnabled;
                    agent.visualizeTargetDirection = isResetLoggingEnabled;
                }
            }

            Debug.Log($"Debug logging {(isRewardLoggingEnabled ? "enabled" : "disabled")}");
        }

        // Cycle focused agent
        if (Input.GetKeyDown(cycleAgentKey))
        {
            FocusNextAgent();
        }
    }

    void FindAllAgents()
    {
        allAgents.Clear();
        foreach (var agent in GetComponents<CarAgent>())
        {
            allAgents.Add(agent);
        }

        Debug.Log($"Found {allAgents.Count} car agents");

        // Enable debug visualization on the first agent
        if (allAgents.Count > 0)
        {
            SetAgentFocus(0);
        }
    }

    void FocusNextAgent()
    {
        if (allAgents.Count == 0) return;

        focusedAgentIndex = (focusedAgentIndex + 1) % allAgents.Count;
        SetAgentFocus(focusedAgentIndex);
    }

    void SetAgentFocus(int index)
    {
        if (index < 0 || index >= allAgents.Count) return;

        // Disable visualization on all agents
        foreach (var agent in allAgents)
        {
            if (agent != null)
            {
                agent.visualizeTargetDirection = false;
                agent.enableCheckpointDebugging = false;
            }
        }

        // Enable on the focused agent
        CarAgent focusedAgent = allAgents[index];
        if (focusedAgent != null)
        {
            focusedAgent.visualizeTargetDirection = true;
            focusedAgent.enableCheckpointDebugging = true;

            // Follow this agent with the camera if possible
            if (Camera.main != null)
            {
                // Try to find or add a simple follow script
                SimpleCameraFollow follow = Camera.main.GetComponent<SimpleCameraFollow>();
                if (follow == null)
                {
                    follow = Camera.main.gameObject.AddComponent<SimpleCameraFollow>();
                }

                follow.SetTarget(focusedAgent.transform);
            }

            Debug.Log($"Now focusing on agent: {focusedAgent.gameObject.name} (Index {index})");
        }
    }

    void UpdateSpeedText()
    {
        if (speedText != null)
        {
            speedText.text = $"Speed: {Time.timeScale:F2}x {(isPaused ? "[PAUSED]" : "")}";
        }
        else
        {
            Debug.Log($"Simulation speed: {Time.timeScale:F2}x {(isPaused ? "[PAUSED]" : "")}");
        }
    }
}

// Simple camera follow script
public class SimpleCameraFollow : MonoBehaviour
{
    public Transform target;
    public Vector3 offset = new Vector3(0, 10, -10);
    public float smoothSpeed = 0.25f;

    public void SetTarget(Transform newTarget)
    {
        target = newTarget;

        // Immediately snap to target
        if (target != null)
        {
            transform.position = target.position + offset;
            transform.LookAt(target);
        }
    }

    void LateUpdate()
    {
        if (target == null) return;

        Vector3 desiredPosition = target.position + offset;
        Vector3 smoothedPosition = Vector3.Lerp(transform.position, desiredPosition, smoothSpeed);
        transform.position = smoothedPosition;

        transform.LookAt(target);
    }
}