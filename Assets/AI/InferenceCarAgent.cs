using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;
using Assets.TrackGeneration;
using System;
using System.Collections;
using System.Linq;
using System.Text;

public class InferenceCarAgent : Agent
{
    [Header("References")]
    public CarControlScript carController;
    public TrackSensingSystem sensorSystem;
    public TrackHandler trackHandler;
    public RaceManager raceManager;

    [Header("Checkpoint System")]
    private List<Checkpoint> checkpoints;
    private float distanceToNextCheckpoint;
    private Vector3 directionToNextCheckpoint;
    private float lastCheckpointTime;
    private int totalCheckpointsPassed = 0;

    [Header("Checkpoint Debugging")]
    public bool enableCheckpointDebugging = false;
    private bool lastRaceActiveState = false;

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

    [Header("Debug")]
    [Tooltip("Visualize direction to next checkpoint")]
    public bool visualizeTargetDirection = true;
    [Tooltip("Show debug messages")]
    public bool isDebugLoggingEnabled = false;

    [Header("Visualization")]
    public bool visualizeRaySensors = false;
    public bool visualizeActionIntentions = false;
    public Color raySensorHitColor = Color.red;
    public Color raySensorNoHitColor = Color.green;
    public float visualizationDuration = 0.1f;

    // Safety reset
    private float safetyResetHeight = -50f;

    // Race debug timer
    private float lastRaceDebugTime = 0f;

    // Memory for observations
    private Vector3 previousPosition;
    private Vector3 previousForward;
    private float[] previousRaySensorData = new float[0];

    // For visualization
    private float[] latestContinuousActions = new float[0];
    private int[] latestDiscreteActions = new int[0];

    // Action interpolation system
    private float[] previousActions = new float[3]; // [accelerate, steer, brake]
    private float[] currentActions = new float[3];
    private bool previousHandbrake = false;
    private bool currentHandbrake = false;
    private float interpolationTime = 0f;
    private float decisionInterval = 0.1f; // Will be updated dynamically

    public void InitializeAgent()
    {
        // Make sure we have all the required components
        if (carController == null)
            carController = GetComponent<CarControlScript>();

        if (sensorSystem == null)
            sensorSystem = GetComponentInChildren<TrackSensingSystem>();

        // Initialize the CarControlScript with the TrackHandler
        if (carController != null && trackHandler != null)
        {
            carController.trackHandler = trackHandler;
        }

        // Reset the car and setup the track
        ResetCar();

        // Get checkpoints
        if (trackHandler != null)
        {
            checkpoints = trackHandler.GetCheckpoints();
            if (checkpoints != null && checkpoints.Count > 0)
            {
                if (isDebugLoggingEnabled) Debug.Log($"Car {gameObject.name} found {checkpoints.Count} checkpoints");
            }
            else
            {
                Debug.LogWarning($"Car {gameObject.name} couldn't find checkpoints");
            }
        }

        if (raceManager != null)
        {
            // Explicitly register with RaceManager
            raceManager.RegisterAgent(gameObject, gameObject.name);
            Debug.Log($"[Agent {gameObject.name}] Registered with RaceManager");

            // Subscribe to checkpoint events
            raceManager.OnCheckpointPassed += OnRaceManagerCheckpointPassed;
            raceManager.OnWrongCheckpoint += OnRaceManagerWrongCheckpoint;
        }

        // Initialize memory variables
        previousPosition = transform.position;
        previousForward = transform.forward;

        // Initialize ray history with proper size when sensor system is available
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

        InitializeTrackMemory();
    }

    protected override void Awake()
    {
        base.Awake();

        // Verify BehaviorParameters configuration
        var behaviorParams = GetComponent<Unity.MLAgents.Policies.BehaviorParameters>();
        if (behaviorParams != null)
        {
            int expectedObservations = 52;
            if (behaviorParams.BrainParameters.VectorObservationSize != expectedObservations)
            {
                Debug.Log($"Updating vector observation size from {behaviorParams.BrainParameters.VectorObservationSize} to {expectedObservations}");
                behaviorParams.BrainParameters.VectorObservationSize = expectedObservations;
            }

            // Verify action spec configuration
            var actionSpec = behaviorParams.BrainParameters.ActionSpec;
            if (actionSpec.NumContinuousActions != 3 || actionSpec.BranchSizes.Length != 1 || actionSpec.BranchSizes[0] != 2)
            {
                var newActionSpec = new Unity.MLAgents.Actuators.ActionSpec();
                newActionSpec.NumContinuousActions = 3;
                newActionSpec.BranchSizes = new int[] { 2 };
                behaviorParams.BrainParameters.ActionSpec = newActionSpec;
            }
        }
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
            if (isRaceActive && Time.frameCount % 30 == 0)
            {
                SyncCheckpointWithRaceManager();
            }

            // Periodically log race and checkpoint stats
            if (Time.time > lastRaceDebugTime + 5f)
            {
                if (enableCheckpointDebugging)
                {
                    int nextCheckpointIndex = raceManager.GetPlayerNextCheckpoint(gameObject);
                    int currentLap = raceManager.GetPlayerCurrentLap(gameObject);
                    bool hasCompletedFullCircuit = raceManager.HasPlayerCompletedFullCircuit(gameObject);

                    Debug.Log($"[Agent {gameObject.name}] Race active: {isRaceActive}, " +
                              $"Next checkpoint: {nextCheckpointIndex}, " +
                              $"Lap: {currentLap}, " +
                              $"Full circuit: {hasCompletedFullCircuit}, " +
                              $"Total passed: {raceManager.GetPlayerTotalCheckpointsPassed(gameObject)}");
                }
                lastRaceDebugTime = Time.time;
            }
        }

        // Safety check - if car falls below a certain height, force a reset
        if (transform.position.y < safetyResetHeight)
        {
            Debug.LogWarning($"[RESET] Car {gameObject.name} fell below safety height ({safetyResetHeight}). Forcing reset.");
            ResetCar();
            EndEpisode();
            return;
        }

        // Visualize agent perception
        VisualizeAgentPerception();
    }

    // Sync checkpoint data with RaceManager
    private void SyncCheckpointWithRaceManager()
    {
        if (raceManager == null) return;

        // Simply update our direction information - we'll get all state from RaceManager
        UpdateCheckpointInfo();
    }

    // Action interpolation in FixedUpdate
    void FixedUpdate()
    {
        // Skip if car controller is null
        if (carController == null)
            return;

        // Update interpolation time
        interpolationTime += Time.fixedDeltaTime;

        // Update decision interval dynamically if needed
        if (decisionInterval <= 0.001f)
        {
            var decisionRequester = GetComponent<Unity.MLAgents.DecisionRequester>();
            if (decisionRequester != null)
            {
                decisionInterval = Time.fixedDeltaTime * decisionRequester.DecisionPeriod;
            }
            else
            {
                decisionInterval = 0.1f; // Fallback
            }
        }

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

    // Optimize FindClosestKnot to avoid checking every knot every time
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

    // Visualize ray sensors and agent intentions
    private void VisualizeAgentPerception()
    {
        if (!visualizeRaySensors && !visualizeActionIntentions)
            return;

        // Visualize ray sensors
        if (visualizeRaySensors && sensorSystem != null)
        {
            VisualizeTrackKnots();
            Vector3[] rayOrigins = sensorSystem.GetRayOrigins();
            Vector3[] rayDirections = sensorSystem.GetRayDirections();
            float[] rayDistances = sensorSystem.GetAllSensorData();

            for (int i = 0; i < rayOrigins.Length && i < rayDirections.Length && i < rayDistances.Length; i++)
            {
                // Get actual ray hit distance (if available)
                float maxRayDistance = sensorSystem.GetMaxRayDistance();
                float hitDistance = rayDistances[i] * maxRayDistance;

                // Draw the ray - red for hits, green for no hits
                bool hitDetected = rayDistances[i] < 0.99f; // Not at max range
                Color rayColor = hitDetected ? raySensorHitColor : raySensorNoHitColor;

                // Draw the ray up to hit point or full length
                Vector3 endPoint = rayOrigins[i] + rayDirections[i] * hitDistance;
                Debug.DrawLine(rayOrigins[i], endPoint, rayColor, visualizationDuration);

                // Draw a small sphere at the hit point if there was a hit
                if (hitDetected)
                {
                    Debug.DrawLine(endPoint, endPoint + Vector3.up * 0.5f, Color.yellow, visualizationDuration);
                }
            }
        }

        // Visualize agent action intentions
        if (visualizeActionIntentions)
        {
            // Get the most recent action values
            float accelerate = latestContinuousActions.Length > 0 ? latestContinuousActions[0] : 0f;
            float steer = latestContinuousActions.Length > 1 ? latestContinuousActions[1] : 0f;
            float brake = latestContinuousActions.Length > 2 ? latestContinuousActions[2] : 0f;

            // Draw action vectors
            // Acceleration/braking intention (forward/backward)
            Color accelColor = accelerate > 0 ? Color.green : Color.yellow;
            Debug.DrawLine(transform.position, transform.position + transform.forward * accelerate * 3f,
                          accelColor, visualizationDuration);

            // Steering intention (left/right)
            Color steerColor = steer > 0 ? Color.blue : Color.magenta;
            Debug.DrawLine(transform.position, transform.position + transform.right * steer * 2f,
                          steerColor, visualizationDuration);

            // Braking (shown as red)
            if (brake > 0.1f)
            {
                Debug.DrawLine(transform.position, transform.position - transform.forward * brake * 2f,
                              Color.red, visualizationDuration);
            }
        }
    }

    // Add to your visualization method
    private void VisualizeTrackKnots()
    {
        if (!useTrackMemory || !visualizeRaySensors || trackKnots == null) return;

        int knotCount = trackKnots.Length;

        // Draw lines between knots we've discovered
        for (int i = 0; i < knotCount; i++)
        {
            if (discoveredKnots[i])
            {
                int nextIndex = (i + 1) % knotCount;
                if (discoveredKnots[nextIndex])
                {
                    // Draw center line
                    Debug.DrawLine(trackKnots[i], trackKnots[nextIndex], Color.yellow, 0.1f);

                    // Draw wall lines
                    Debug.DrawLine(leftWallKnots[i], leftWallKnots[nextIndex], Color.cyan, 0.1f);
                    Debug.DrawLine(rightWallKnots[i], rightWallKnots[nextIndex], Color.cyan, 0.1f);

                    // Draw track width
                    Debug.DrawLine(leftWallKnots[i], rightWallKnots[i], Color.white, 0.1f);
                }
            }
        }

        // Highlight the closest knot
        if (closestKnotIndex >= 0)
        {
            // Draw a strong red line from car to closest knot
            Debug.DrawLine(transform.position, trackKnots[closestKnotIndex], Color.red, 0.1f);

            // Draw a sphere at the closest knot position
            Debug.DrawLine(trackKnots[closestKnotIndex], trackKnots[closestKnotIndex] + Vector3.up * 2f, Color.red, 0.1f);

            // Highlight the look-ahead knots (these are exactly the knots used as observations)
            for (int i = 1; i <= lookAheadCount; i++)
            {
                int knotIndex = (closestKnotIndex + i) % knotCount;
                if (discoveredKnots[knotIndex])
                {
                    // Color gradient from green to blue for observation knots
                    Color observationColor = Color.Lerp(Color.green, Color.blue, i / (float)lookAheadCount);

                    // Draw a direct line from car to observation knot (these are the knots used in observations)
                    Debug.DrawLine(
                        transform.position,
                        trackKnots[knotIndex],
                        observationColor,
                        0.1f
                    );

                    // Draw a vertical marker at each observation knot
                    Debug.DrawLine(
                        trackKnots[knotIndex],
                        trackKnots[knotIndex] + Vector3.up * (3f - i * 0.5f), // Vary height based on look ahead index
                        observationColor,
                        0.1f
                    );

                    // Get track data for the knot
                    Vector3 center = trackKnots[knotIndex];
                    Vector3 leftPos = leftWallKnots[knotIndex];
                    Vector3 rightPos = rightWallKnots[knotIndex];

                    // Calculate track width and height difference
                    float width = Vector3.Distance(leftPos, rightPos);
                    float heightDiff = rightPos.y - leftPos.y;
                    float bankAngle = Mathf.Atan2(heightDiff, width) * Mathf.Rad2Deg;

                    // Draw track width at this observation knot
                    Debug.DrawLine(leftPos, rightPos, observationColor, 0.1f);

                    // Visualize the banking angle
                    Vector3 bankingDirection = Quaternion.Euler(0, 0, bankAngle) * Vector3.up * 3f;
                    Debug.DrawRay(center, bankingDirection, Color.magenta, 0.1f);

                    // Calculate track direction to next knot
                    int nextIndex = (knotIndex + 1) % knotCount;
                    if (discoveredKnots[nextIndex])
                    {
                        Vector3 nextPos = trackKnots[nextIndex];
                        Vector3 trackDirection = (nextPos - center).normalized * 5f;
                        Debug.DrawRay(center, trackDirection, observationColor, 0.1f);
                    }
                }
            }
        }
    }

    // Update checkpoint information
    private void UpdateCheckpointInfo()
    {
        try
        {
            if (checkpoints == null || checkpoints.Count == 0 || raceManager == null)
            {
                if (enableCheckpointDebugging)
                    Debug.LogWarning($"[Agent {gameObject.name}] UpdateCheckpointInfo: Checkpoints list is null or empty!");
                return;
            }

            // Get next checkpoint index from RaceManager
            int nextIndex = raceManager.GetPlayerNextCheckpoint(gameObject);

            if (nextIndex < checkpoints.Count)
            {
                Vector3 nextCheckpointPosition = checkpoints[nextIndex].transform.position;

                distanceToNextCheckpoint = Vector3.Distance(transform.position, nextCheckpointPosition);
                directionToNextCheckpoint = nextCheckpointPosition - transform.position;

                // Debug visualization
                if (visualizeTargetDirection)
                {
                    Debug.DrawLine(transform.position, nextCheckpointPosition, Color.green, 0.1f);

                    if (enableCheckpointDebugging && Time.frameCount % 300 == 0)
                    {
                        Debug.Log($"[Agent {gameObject.name}] Drawing ray to checkpoint {nextIndex} at {nextCheckpointPosition}");
                    }
                }
            }
            else
            {
                if (enableCheckpointDebugging)
                    Debug.LogError($"[Agent {gameObject.name}] Invalid nextCheckpointIndex: {nextIndex} (max: {checkpoints.Count - 1})");
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

    private void ResetCar()
    {
        // Before doing anything, make sure car controller is valid
        if (carController == null)
        {
            Debug.LogWarning("CarController is null in ResetCar, looking for it now...");
            carController = GetComponent<CarControlScript>();
        }

        // Simply use the car controller's reset method which we know works
        if (carController != null)
        {
            try
            {
                // Use the car controller's reset method directly
                carController.ResetPosition();
            }
            catch (Exception e)
            {
                Debug.LogError($"Error in ResetCar: {e.Message}\n{e.StackTrace}");
            }
        }
        else
        {
            Debug.LogError($"Cannot reset car position: carController is still null");
        }
    }

    public override void OnEpisodeBegin()
    {
        // Reset the car position and state
        ResetCar();
        ForceCheckpointReset();

        // Initialize checkpoint tracking
        if (trackHandler != null)
        {
            checkpoints = trackHandler.GetCheckpoints();
        }

        // Reset timers and state tracking
        lastCheckpointTime = Time.time;
        totalCheckpointsPassed = 0;
        lastTrackMemoryUpdateTime = Time.time;

        // Reset memory variables
        previousPosition = transform.position;
        previousForward = transform.forward;

        // Reset interpolation values
        interpolationTime = 0f;
        for (int i = 0; i < 3; i++)
        {
            previousActions[i] = 0f;
            currentActions[i] = 0f;
        }
        previousHandbrake = false;
        currentHandbrake = false;

        // Reset car inputs
        if (carController != null)
        {
            carController.SetAccelerationInput(0f);
            carController.SetSteeringInput(0f);
            carController.SetReverseInput(0f);
            carController.SetHandbrakeInput(false);
        }

        // Initial measurement to next checkpoint
        UpdateCheckpointInfo();
    }

    // Reset checkpoint state for new episode
    private void ForceCheckpointReset()
    {
        // Reset related variables
        lastCheckpointTime = Time.time;

        // Tell RaceManager to reset this agent's checkpoint
        if (raceManager != null)
        {
            try
            {
                raceManager.ResetAgent(gameObject);

                // Update checkpoint info after reset
                UpdateCheckpointInfo();
            }
            catch (Exception e)
            {
                Debug.LogWarning($"Failed to reset checkpoint in RaceManager: {e.Message}");
            }
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Make sure we have the latest checkpoint info
        if (raceManager != null)
        {
            UpdateCheckpointInfo();
        }

        // Make sure sensor system is initialized
        if (sensorSystem == null)
        {
            Debug.LogError("SensorSystem is null in CollectObservations!");
            // Add blank observations to prevent errors
            int observations = (sensorSystem != null ? sensorSystem.totalRayCount * 2 : 20) + 6;
            for (int i = 0; i < observations; i++)
            {
                sensor.AddObservation(0f);
            }
            return;
        }

        // Get current ray sensor data
        float[] raySensorData = sensorSystem.GetAllSensorData();

        // 1. Add ray sensor data (distances to walls)
        for (int i = 0; i < sensorSystem.totalRayCount; i++)
        {
            sensor.AddObservation(i < raySensorData.Length ? raySensorData[i] : 1.0f);
        }

        // 2. Add ray velocity data (rate of change of distances)
        float[] velocityData = sensorSystem.GetVelocityData();
        for (int i = 0; i < sensorSystem.totalRayCount; i++)
        {
            // Clamp values to reasonable ranges to prevent instability
            float velocityValue = (i < velocityData.Length) ?
                                  Mathf.Clamp(velocityData[i], -10f, 10f) : 0f;
            sensor.AddObservation(velocityValue);
        }

        // 3. Add normalized direction to next checkpoint (2 values)
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

        // 8. Add track observations
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
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // Store latest actions for visualization
        if (actionBuffers.ContinuousActions.Length > 0)
        {
            latestContinuousActions = new float[actionBuffers.ContinuousActions.Length];
            for (int i = 0; i < actionBuffers.ContinuousActions.Length; i++)
            {
                latestContinuousActions[i] = actionBuffers.ContinuousActions[i];
            }
        }

        if (actionBuffers.DiscreteActions.Length > 0)
        {
            latestDiscreteActions = new int[actionBuffers.DiscreteActions.Length];
            for (int i = 0; i < actionBuffers.DiscreteActions.Length; i++)
            {
                latestDiscreteActions[i] = actionBuffers.DiscreteActions[i];
            }
        }

        try
        {
            // Extract continuous actions and store for interpolation
            if (actionBuffers.ContinuousActions.Length >= 3)
            {
                // Move current actions to previous 
                for (int i = 0; i < 3; i++)
                {
                    previousActions[i] = currentActions[i];
                }

                // Store new current actions
                currentActions[0] = actionBuffers.ContinuousActions[0]; // accelerate
                currentActions[1] = actionBuffers.ContinuousActions[1]; // steer
                currentActions[2] = actionBuffers.ContinuousActions[2]; // brake

                // Debug visualization only if enabled
                if (visualizeTargetDirection)
                {
                    Debug.DrawRay(transform.position, transform.forward * currentActions[0] * 5f, Color.green);
                    Debug.DrawRay(transform.position, transform.right * currentActions[1] * 5f, Color.red);
                }
            }
            else
            {
                Debug.LogWarning($"Expected 3 continuous actions, but got {actionBuffers.ContinuousActions.Length}");
            }

            // Extract discrete action (handbrake) and store for interpolation
            if (actionBuffers.DiscreteActions.Length > 0)
            {
                previousHandbrake = currentHandbrake;
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

            // Make sure we're synchronized with RaceManager
            if (raceManager != null && raceManager.IsRaceActive())
            {
                SyncCheckpointWithRaceManager();
            }

            UpdateCheckpointInfo();
        }
        catch (Exception ex)
        {
            Debug.LogError($"Error in OnActionReceived: {ex.Message}");
        }
    }

    // Called when a checkpoint is passed - get event from RaceManager
    private void OnRaceManagerCheckpointPassed(GameObject player, int checkpointIndex, int totalCheckpoints)
    {
        // Only process events for this agent
        if (player != this.gameObject) return;

        if (enableCheckpointDebugging)
        {
            int expectedIndex = raceManager.GetPlayerNextCheckpoint(gameObject);
            int currentLap = raceManager.GetPlayerCurrentLap(gameObject);
            bool fullCircuit = raceManager.HasPlayerCompletedFullCircuit(gameObject);

            Debug.Log($"[Agent {gameObject.name}] CHECKPOINT PASSED: index={checkpointIndex}, " +
                      $"expected={expectedIndex}, " +
                      $"lap={currentLap}, " +
                      $"full_circuit={fullCircuit}");
        }

        // Check if this was the correct checkpoint by comparing with expected next checkpoint
        int expectedNextIndex = raceManager.GetPlayerNextCheckpoint(gameObject);
        int previousIndex = (expectedNextIndex - 1 + checkpoints.Count) % checkpoints.Count;

        if (checkpointIndex == previousIndex)
        {
            // This was the expected checkpoint
            totalCheckpointsPassed++;

            // Update direction info
            UpdateCheckpointInfo();

            // Update timers
            lastCheckpointTime = Time.time;
        }
    }

    // Handle wrong checkpoint - new method
    private void OnRaceManagerWrongCheckpoint(GameObject player, int attemptedCheckpoint, int expectedCheckpoint)
    {
        // Only process events for this agent
        if (player != this.gameObject) return;

        if (enableCheckpointDebugging)
        {
            Debug.LogWarning($"[Agent {gameObject.name}] WRONG CHECKPOINT: Attempted={attemptedCheckpoint}, Expected={expectedCheckpoint}");
        }
    }

    // Set input values on the car controller
    private void SetCarInputs(float accelerate, float steer, float brake, bool handbrake)
    {
        carController.SetAccelerationInput(accelerate);
        carController.SetSteeringInput(steer);
        carController.SetReverseInput(brake);
        carController.SetHandbrakeInput(handbrake);
    }

    // Collision handling
    public void ReportCollision()
    {
        // In inference mode, we just log it
        if (isDebugLoggingEnabled)
            Debug.Log($"[Agent {gameObject.name}] Collision detected");
    }

    protected override void OnDisable()
    {
        base.OnDisable();

        // Unsubscribe from events
        if (raceManager != null)
        {
            raceManager.OnCheckpointPassed -= OnRaceManagerCheckpointPassed;
            raceManager.OnWrongCheckpoint -= OnRaceManagerWrongCheckpoint;
        }
    }

    void OnDestroy()
    {
        // Unsubscribe from events
        if (raceManager != null)
        {
            raceManager.OnCheckpointPassed -= OnRaceManagerCheckpointPassed;
            raceManager.OnWrongCheckpoint -= OnRaceManagerWrongCheckpoint;
        }
    }
}