using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;
using Assets.TrackGeneration;
using System;

/// <summary>
/// A specialized version of CarAgent that is optimized for inference only.
/// This removes all training-specific code and focuses only on using the trained model.
/// </summary>
public class InferenceCarAgent : Agent
{
    [Header("References")]
    public CarControlScript carController;
    public TrackSensingSystem sensorSystem;
    public TrackHandler trackHandler;
    public RaceManager raceManager;

    [Header("Debug")]
    public bool visualizeRays = false;
    public bool visualizeTargetDirection = false;
    public bool isDebugLoggingEnabled = false;
    public bool enableCheckpointDebugging = false;

    [Header("Track Memory")]
    public bool useTrackMemory = true;
    private Vector3[] trackKnots;     // Center track knots
    private Vector3[] leftWallKnots;  // Left wall knots 
    private Vector3[] rightWallKnots; // Right wall knots
    private bool[] discoveredKnots;   // Which knots we've seen
    private int closestKnotIndex = 0;
    private int lookAheadCount = 2;   // How many points to predict ahead
    public float trackMemoryUpdateInterval = 0.2f; // Update 5 times per second instead of every frame
    private float lastTrackMemoryUpdateTime = 0f;
    private int cachedClosestKnotIndex = -1;

    // Store positions of checkpoints for decision making
    private List<Checkpoint> checkpoints;
    private int nextCheckpointIndex = 0;
    private float distanceToNextCheckpoint;
    private Vector3 directionToNextCheckpoint;
    private float lastCheckpointTime;
    private int totalCheckpointsPassed = 0;
    private int currentLap = 0;
    private bool lastRaceActiveState = false;

    // Memory for observations
    private float[] previousRaySensorData = new float[36];

    // Action interpolation system
    private float[] previousActions = new float[3]; // [accelerate, steer, brake]
    private float[] currentActions = new float[3];
    private bool previousHandbrake = false;
    private bool currentHandbrake = false;
    private float interpolationTime = 0f;
    private float decisionInterval = 0.1f; // Will be updated dynamically

    void Start()
    {
        InitializeAgent();

        // Initialize ray history with proper size
        if (sensorSystem != null)
        {
            float[] currentData = sensorSystem.GetAllSensorData();
            previousRaySensorData = new float[currentData.Length];
            System.Array.Copy(currentData, previousRaySensorData, currentData.Length);
        }

        // Initialize action arrays for interpolation
        for (int i = 0; i < 3; i++)
        {
            previousActions[i] = 0f;
            currentActions[i] = 0f;
        }

        // Get initial decision interval
        var decisionRequester = GetComponent<Unity.MLAgents.DecisionRequester>();
        if (decisionRequester != null)
        {
            decisionInterval = Time.fixedDeltaTime * decisionRequester.DecisionPeriod;
        }
    }

    protected override void Awake()
    {
        base.Awake();

        // Verify BehaviorParameters configuration
        var behaviorParams = GetComponent<Unity.MLAgents.Policies.BehaviorParameters>();
        if (behaviorParams != null)
        {
            // Force the correct observation size
            int expectedObservations = 40;
            if (behaviorParams.BrainParameters.VectorObservationSize != expectedObservations)
            {
                Debug.Log($"Correcting observation size from {behaviorParams.BrainParameters.VectorObservationSize} to {expectedObservations}");
                behaviorParams.BrainParameters.VectorObservationSize = expectedObservations;
            }

            // Verify action spec configuration
            var actionSpec = behaviorParams.BrainParameters.ActionSpec;
            if (actionSpec.NumContinuousActions != 3 || actionSpec.BranchSizes.Length != 1 || actionSpec.BranchSizes[0] != 2)
            {
                Debug.Log("Correcting action space configuration");
                var newActionSpec = new Unity.MLAgents.Actuators.ActionSpec();
                newActionSpec.NumContinuousActions = 3;
                newActionSpec.BranchSizes = new int[] { 2 };
                behaviorParams.BrainParameters.ActionSpec = newActionSpec;
            }

            // Ensure model is assigned (if it was set elsewhere but not correctly applied)
            if (behaviorParams.Model != null)
            {
                Debug.Log($"ONNX model verified in Awake: {behaviorParams.Model.name}");
            }
            else
            {
                Debug.LogWarning("No ONNX model assigned to BehaviorParameters!");
                // If you have a default model you can assign it here
                // behaviorParams.Model = defaultModel;
            }
        }
    }

    public void InitializeAgent()
    {
        // Make sure references are set up
        if (carController == null)
            carController = GetComponent<CarControlScript>();

        if (sensorSystem == null)
            sensorSystem = GetComponent<TrackSensingSystem>();

        // Initialize the checkpoint system
        if (trackHandler != null)
        {
            checkpoints = trackHandler.GetCheckpoints();
            if (checkpoints != null && checkpoints.Count > 0)
            {
                Debug.Log($"Inference Agent: Found {checkpoints.Count} checkpoints");
                UpdateCheckpointInfo();
            }
            else
            {
                Debug.LogWarning("Inference Agent: No checkpoints found!");
            }
        }
        else
        {
            Debug.LogError("Inference Agent: TrackHandler reference not set!");
        }

        // Register with RaceManager if available
        if (raceManager != null)
        {
            raceManager.RegisterAgent(gameObject, gameObject.name);
            if (isDebugLoggingEnabled) Debug.Log($"[Agent {gameObject.name}] Registered with RaceManager");

            // Subscribe to checkpoint events
            raceManager.OnCheckpointPassed += OnRaceManagerCheckpointPassed;
        }

        // Make sure the car is properly set up for AI control
        if (carController != null)
        {
            // Make sure the car knows this is AI controlled
            carController.isPlayerControlled = false;
        }

        // Initialize track memory system
        InitializeTrackMemory();

        // Initialize timers
        lastCheckpointTime = Time.time;
    }

    private void InitializeTrackMemory()
    {
        if (!useTrackMemory || trackHandler == null) return;

        // Get the spline containers
        var centerSpline = trackHandler.trackSpline;
        var leftSpline = trackHandler.leftSpline;
        var rightSpline = trackHandler.rightSpline;

        if (centerSpline == null || leftSpline == null || rightSpline == null)
        {
            Debug.LogWarning("Could not find track splines for sampling");
            return;
        }

        // Get knot count (they should all have the same count)
        int knotCount = centerSpline.Spline.Count;
        trackKnots = new Vector3[knotCount];
        leftWallKnots = new Vector3[knotCount];
        rightWallKnots = new Vector3[knotCount];
        discoveredKnots = new bool[knotCount];

        // Extract the knot positions from each spline
        for (int i = 0; i < knotCount; i++)
        {
            trackKnots[i] = centerSpline.Spline[i].Position;
            leftWallKnots[i] = leftSpline.Spline[i].Position;
            rightWallKnots[i] = rightSpline.Spline[i].Position;
            discoveredKnots[i] = false; // Start undiscovered
        }

        Debug.Log($"Track memory initialized with {knotCount} knot points");
    }

    public override void OnEpisodeBegin()
    {
        // Reset if needed (typically not used in inference mode, but can be called manually)
        if (carController != null)
        {
            carController.ResetVehicle();
        }

        // Reset checkpoint tracking
        nextCheckpointIndex = 0;
        totalCheckpointsPassed = 0;
        currentLap = 0;
        lastCheckpointTime = Time.time;

        // Force synchronize with RaceManager if available
        if (raceManager != null)
        {
            nextCheckpointIndex = raceManager.GetPlayerNextCheckpoint(gameObject);
            if (isDebugLoggingEnabled) Debug.Log($"[Agent {gameObject.name}] Starting episode with checkpoint index: {nextCheckpointIndex} (from RaceManager)");
        }

        UpdateCheckpointInfo();

        // Reset action interpolation
        interpolationTime = 0f;
        for (int i = 0; i < 3; i++)
        {
            previousActions[i] = 0f;
            currentActions[i] = 0f;
        }
        previousHandbrake = false;
        currentHandbrake = false;
    }

    void Update()
    {
        // Check and log race state changes
        if (raceManager != null)
        {
            bool isRaceActive = raceManager.IsRaceActive();

            // Log when race state changes
            if (isRaceActive != lastRaceActiveState)
            {
                Debug.Log($"[Agent {gameObject.name}] Race active state changed: {lastRaceActiveState} -> {isRaceActive}");
                lastRaceActiveState = isRaceActive;
            }

            // Periodically sync checkpoint data with RaceManager
            if (isRaceActive && Time.frameCount % 60 == 0)
            {
                SyncCheckpointWithRaceManager();
            }
        }
    }

    void FixedUpdate()
    {
        // Update interpolation time
        interpolationTime += Time.fixedDeltaTime;

        // Calculate interpolation factor (0 to 1)
        float t = Mathf.Clamp01(interpolationTime / decisionInterval);

        // Interpolate actions
        float interpolatedAccelerate = Mathf.Lerp(previousActions[0], currentActions[0], t);
        float interpolatedSteer = Mathf.Lerp(previousActions[1], currentActions[1], t);
        float interpolatedBrake = Mathf.Lerp(previousActions[2], currentActions[2], t);

        // For handbrake (binary action), use threshold
        bool useHandbrake = t > 0.5f ? currentHandbrake : previousHandbrake;

        // Apply interpolated inputs to car controller
        SetCarInputs(interpolatedAccelerate, interpolatedSteer, interpolatedBrake, useHandbrake);

        // Update track memory at intervals rather than every frame
        if (Time.time - lastTrackMemoryUpdateTime > trackMemoryUpdateInterval)
        {
            UpdateTrackMemory();
            lastTrackMemoryUpdateTime = Time.time;
        }
    }

    private void UpdateTrackMemory()
    {
        if (!useTrackMemory || trackKnots == null) return;

        // Find closest track knot
        closestKnotIndex = FindClosestKnot();

        // Mark several knots around the current position as discovered
        if (closestKnotIndex >= 0)
        {
            int range = 10; // Mark 10 knots in each direction
            for (int offset = -range; offset <= range; offset++)
            {
                int idx = (closestKnotIndex + offset + trackKnots.Length) % trackKnots.Length;
                discoveredKnots[idx] = true;
            }
        }
    }

    private int FindClosestKnot()
    {
        if (trackKnots == null) return -1;

        // If we already have a closest knot, only check nearby knots first
        if (cachedClosestKnotIndex >= 0)
        {
            float minDist = float.MaxValue;
            int closestIndex = cachedClosestKnotIndex;
            int knotCount = trackKnots.Length;

            // Check 10 knots ahead and behind current knot
            int searchRange = 10;
            for (int offset = -searchRange; offset <= searchRange; offset++)
            {
                int i = (cachedClosestKnotIndex + offset + knotCount) % knotCount;
                float dist = Vector3.Distance(transform.position, trackKnots[i]);
                if (dist < minDist)
                {
                    minDist = dist;
                    closestIndex = i;
                }
            }

            // If we found a knot significantly closer or haven't checked all knots in a while,
            // perform a full search (every ~2 seconds)
            if (minDist < 5f || Time.frameCount % 120 == 0)
                return closestIndex;

            // Full search if we didn't find a good match
            return FindAllKnots();
        }

        // Fall back to checking all knots if we don't have a cached value
        return FindAllKnots();
    }

    private int FindAllKnots()
    {
        float minDist = float.MaxValue;
        int closestIndex = -1;

        for (int i = 0; i < trackKnots.Length; i++)
        {
            float dist = Vector3.Distance(transform.position, trackKnots[i]);
            if (dist < minDist)
            {
                minDist = dist;
                closestIndex = i;
            }
        }

        // Cache result for next time
        cachedClosestKnotIndex = closestIndex;
        return closestIndex;
    }

    // Sync checkpoint data with RaceManager
    private void SyncCheckpointWithRaceManager()
    {
        if (raceManager == null) return;

        int raceManagerNextCheckpoint = raceManager.GetPlayerNextCheckpoint(gameObject);

        if (nextCheckpointIndex != raceManagerNextCheckpoint)
        {
            if (enableCheckpointDebugging)
            {
                Debug.Log($"[Agent {gameObject.name}] Syncing checkpoint index: {nextCheckpointIndex} -> {raceManagerNextCheckpoint}");
            }

            nextCheckpointIndex = raceManagerNextCheckpoint;
            UpdateCheckpointInfo(); // Update direction ray
        }
    }

    // Called when RaceManager reports checkpoint passed
    private void OnRaceManagerCheckpointPassed(GameObject player, int checkpointIndex, int totalCheckpoints)
    {
        // Only process events for this agent
        if (player == this.gameObject)
        {
            if (enableCheckpointDebugging)
            {
                Debug.Log($"[Agent {gameObject.name}] RECEIVED checkpoint event from RaceManager: " +
                          $"index={checkpointIndex}, total={totalCheckpoints}");
            }

            // Get next expected checkpoint
            int previousIndex = nextCheckpointIndex;
            nextCheckpointIndex = (checkpointIndex + 1) % checkpoints.Count;

            if (enableCheckpointDebugging)
            {
                Debug.Log($"[Agent {gameObject.name}] Updated checkpoint index: {previousIndex} -> {nextCheckpointIndex}");
            }

            // Update direction info
            UpdateCheckpointInfo();

            // Update timers and counters
            lastCheckpointTime = Time.time;
            totalCheckpointsPassed++;

            // Check for lap completion - if we just hit the last checkpoint and next is 0
            if (checkpointIndex == checkpoints.Count - 1 && nextCheckpointIndex == 0)
            {
                currentLap++;
                if (isDebugLoggingEnabled)
                {
                    Debug.Log($"[Agent {gameObject.name}] Completed lap {currentLap}");
                }
            }
        }
    }

    private void UpdateCheckpointInfo()
    {
        try
        {
            if (checkpoints == null || checkpoints.Count == 0)
            {
                if (enableCheckpointDebugging)
                    Debug.LogWarning($"[Agent {gameObject.name}] UpdateCheckpointInfo: Checkpoints list is null or empty!");
                return;
            }

            if (nextCheckpointIndex < checkpoints.Count)
            {
                Vector3 nextCheckpointPosition = checkpoints[nextCheckpointIndex].transform.position;

                distanceToNextCheckpoint = Vector3.Distance(transform.position, nextCheckpointPosition);
                directionToNextCheckpoint = nextCheckpointPosition - transform.position;

                // Debug visualization
                if (visualizeTargetDirection)
                {
                    Debug.DrawLine(transform.position, nextCheckpointPosition, Color.green, 0.1f);

                    if (enableCheckpointDebugging && Time.frameCount % 300 == 0)
                    {
                        Debug.Log($"[Agent {gameObject.name}] Drawing ray to checkpoint {nextCheckpointIndex} at {nextCheckpointPosition}");
                    }
                }
            }
            else
            {
                if (enableCheckpointDebugging)
                    Debug.LogError($"[Agent {gameObject.name}] Invalid nextCheckpointIndex: {nextCheckpointIndex} (max: {checkpoints.Count - 1})");

                // Fix invalid index
                nextCheckpointIndex = 0;
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Error in UpdateCheckpointInfo: {e.Message}");

            // Set reasonable defaults in case of error
            distanceToNextCheckpoint = 50f;
            directionToNextCheckpoint = transform.forward * 50f;
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Make sure sensor system is initialized
        if (sensorSystem == null)
        {
            Debug.LogError("SensorSystem is null in CollectObservations!");
            // Add blank observations to prevent errors
            for (int i = 0; i < 40; i++)
            {
                sensor.AddObservation(0f);
            }
            return;
        }

        // 1. Get ray sensor data (distances to track boundaries)
        float[] raySensorData = sensorSystem.GetAllSensorData();

        // Add ray sensor data
        foreach (float rayDistance in raySensorData)
        {
            sensor.AddObservation(rayDistance);
        }

        // 2. Calculate and add ray velocity data (rate of change of distances)
        for (int i = 0; i < (sensorSystem.totalRayCount * 2) + 6; i++)
        {
            // Calculate rate of change between current and previous reading
            float velocity = i < raySensorData.Length && i < previousRaySensorData.Length ?
                            (raySensorData[i] - previousRaySensorData[i]) / Time.deltaTime : 0f;

            // Clamp to reasonable range to match training
            velocity = Mathf.Clamp(velocity, -10f, 10f);
            sensor.AddObservation(velocity);
        }

        // Store current rays for next frame's velocity calculation
        System.Array.Copy(raySensorData, previousRaySensorData, Mathf.Min(raySensorData.Length, previousRaySensorData.Length));

        // 3. Add normalized direction to next checkpoint
        float forwardDot = Vector3.Dot(transform.forward, directionToNextCheckpoint.normalized);
        float rightDot = Vector3.Dot(transform.right, directionToNextCheckpoint.normalized);
        sensor.AddObservation(forwardDot); // How aligned we are with the next checkpoint
        sensor.AddObservation(rightDot);   // If the checkpoint is to our right or left

        // 4. Add normalized distance to next checkpoint
        sensor.AddObservation(Mathf.Clamp01(distanceToNextCheckpoint / 100f));

        // 5. Add car speed information
        float speed = carController.GetCurrentSpeed();
        sensor.AddObservation(speed / carController.maxVelocity); // Normalized speed

        // 6. Add car-specific information
        sensor.AddObservation(carController.isDrifting ? 1f : 0f); // Is the car drifting

        // 7. Add time since last decision (helps agent understand time scale)
        float timeSinceLastDecision = Time.fixedDeltaTime * Time.timeScale * GetComponent<Unity.MLAgents.DecisionRequester>().DecisionPeriod;
        sensor.AddObservation(timeSinceLastDecision);

        // 8. Add track memory observations (if used in training)
        if (useTrackMemory && trackKnots != null && closestKnotIndex >= 0)
        {
            int knotCount = trackKnots.Length;

            // Sample the upcoming knots
            for (int i = 1; i <= lookAheadCount; i++)
            {
                int knotIndex = (closestKnotIndex + i) % knotCount;

                if (discoveredKnots[knotIndex])
                {
                    // Get the knot positions
                    Vector3 centerPos = trackKnots[knotIndex];
                    Vector3 leftPos = leftWallKnots[knotIndex];
                    Vector3 rightPos = rightWallKnots[knotIndex];

                    // Calculate relative position to car (in local space)
                    Vector3 localCenter = transform.InverseTransformPoint(centerPos);

                    // Calculate distance to this knot
                    float distance = Vector3.Distance(transform.position, centerPos);

                    // Calculate track width
                    float width = Vector3.Distance(leftPos, rightPos);

                    // Calculate track direction
                    int nextIndex = (knotIndex + 1) % knotCount;
                    Vector3 nextPos = trackKnots[nextIndex];
                    Vector3 trackDirection = (nextPos - centerPos).normalized;
                    float trackAngle = Vector3.SignedAngle(transform.forward, trackDirection, Vector3.up);

                    // Calculate banking angle
                    float heightDiff = rightPos.y - leftPos.y;
                    float bankAngle = Mathf.Atan2(heightDiff, width) * Mathf.Rad2Deg;

                    // Add observations (normalized)
                    sensor.AddObservation(Mathf.Clamp01(distance / 100f));  // Distance to knot
                    sensor.AddObservation(localCenter.x / 50f);            // X position relative to car
                    sensor.AddObservation(localCenter.z / 50f);            // Z position relative to car
                    sensor.AddObservation(width / 20f);                    // Track width
                    sensor.AddObservation(trackAngle / 180f);              // Track angle
                    sensor.AddObservation(bankAngle / 45f);                // Track banking
                }
                else
                {
                    // Add zeros for undiscovered knots
                    for (int j = 0; j < 6; j++)
                        sensor.AddObservation(0f);
                }
            }
        }
        else
        {
            // Add zeros to match expected observation space if track memory is disabled
            for (int i = 0; i < lookAheadCount * 6; i++)
            {
                sensor.AddObservation(0f);
            }
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        try
        {
            // Store previous actions
            for (int i = 0; i < 3; i++)
            {
                previousActions[i] = currentActions[i];
            }
            previousHandbrake = currentHandbrake;

            // Extract continuous actions
            if (actionBuffers.ContinuousActions.Length >= 3)
            {
                currentActions[0] = actionBuffers.ContinuousActions[0]; // accelerate
                currentActions[1] = actionBuffers.ContinuousActions[1]; // steer
                currentActions[2] = actionBuffers.ContinuousActions[2]; // brake

                // Debug visualization
                if (visualizeRays)
                {
                    Debug.DrawRay(transform.position, transform.forward * currentActions[0] * 5f, Color.green);
                    Debug.DrawRay(transform.position, transform.right * currentActions[1] * 5f, Color.red);
                }
            }

            // Extract discrete action (handbrake)
            if (actionBuffers.DiscreteActions.Length > 0)
            {
                currentHandbrake = actionBuffers.DiscreteActions[0] == 1;
            }

            // Reset interpolation timer
            interpolationTime = 0f;

            // Update decision interval dynamically
            var decisionRequester = GetComponent<Unity.MLAgents.DecisionRequester>();
            if (decisionRequester != null)
            {
                decisionInterval = Time.fixedDeltaTime * decisionRequester.DecisionPeriod;
            }

            // Check if race is active
            bool isRaceActive = raceManager != null && raceManager.IsRaceActive();

            // Make sure we're synchronized with RaceManager
            if (isRaceActive && raceManager != null)
            {
                SyncCheckpointWithRaceManager();
            }

            // Update checkpoint information
            UpdateCheckpointInfo();
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"Error in OnActionReceived: {ex.Message}");
        }
    }

    // Process checkpoint passes - called by Checkpoints directly
    public void CheckpointPassed(int checkpointIndex)
    {
        if (checkpointIndex == nextCheckpointIndex)
        {
            // Update tracking
            nextCheckpointIndex = (nextCheckpointIndex + 1) % checkpoints.Count;

            // Update distance to next checkpoint
            UpdateCheckpointInfo();

            // Record time
            lastCheckpointTime = Time.time;

            // Update count
            totalCheckpointsPassed++;

            // Check for lap completion
            if (checkpointIndex == checkpoints.Count - 1)
            {
                currentLap++;
                Debug.Log($"AI car completed lap {currentLap}");
            }

            if (isDebugLoggingEnabled)
                Debug.Log($"AI car passed checkpoint {checkpointIndex}, next is {nextCheckpointIndex}");
        }
    }

    // Set input values on the car controller
    private void SetCarInputs(float accelerate, float steer, float brake, bool handbrake)
    {
        if (carController != null)
        {
            carController.SetAccelerationInput(accelerate);
            carController.SetSteeringInput(steer);
            carController.SetReverseInput(brake);
            carController.SetHandbrakeInput(handbrake);
        }
    }

    // Simple collision reporting for debugging
    public void ReportCollision()
    {
        Debug.Log("AI car collision detected");
    }

    // Reset the car if needed (can be called externally)
    public void ResetCar()
    {
        if (carController != null)
        {
            carController.ResetVehicle();
        }

        // Reset checkpoint tracking
        nextCheckpointIndex = 0;
        totalCheckpointsPassed = 0;
        currentLap = 0;
        lastCheckpointTime = Time.time;

        // Force sync with RaceManager if available
        if (raceManager != null)
        {
            raceManager.ResetAgent(gameObject);
            nextCheckpointIndex = raceManager.GetPlayerNextCheckpoint(gameObject);
        }

        UpdateCheckpointInfo();
    }

    protected override void OnDisable()
    {
        base.OnDisable();

        // Unsubscribe from events
        if (raceManager != null)
        {
            raceManager.OnCheckpointPassed -= OnRaceManagerCheckpointPassed;
        }
    }

    void OnDestroy()
    {
        // Unsubscribe from events
        if (raceManager != null)
        {
            raceManager.OnCheckpointPassed -= OnRaceManagerCheckpointPassed;
        }
    }
}