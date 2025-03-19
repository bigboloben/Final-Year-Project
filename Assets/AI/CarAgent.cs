using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;
using Assets.TrackGeneration;
using System;
using System.Collections;
using System.Text;
using System.Linq;

public class CarAgent : Agent
{
    [Header("References")]
    public CarControlScript carController;
    public TrackSensingSystem sensorSystem;
    public TrackHandler trackHandler;
    public RaceManager raceManager;

    [Header("Checkpoint System")]
    private List<Checkpoint> checkpoints;
    private int nextCheckpointIndex = 0;
    private float distanceToNextCheckpoint;
    private Vector3 directionToNextCheckpoint;
    private float lastCheckpointTime;
    private int totalCheckpointsPassed = 0;
    private int currentLap = 0;

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

    [Header("Reward Parameters")]
    [Tooltip("Reward for passing a checkpoint")]
    public float checkpointReward = 2.0f;
    [Tooltip("Base reward for completing a lap")]
    public float lapCompletionReward = 5.0f;
    [Tooltip("Penalty for hitting walls")]
    public float collisionPenalty = -1.0f; // Increased from -0.1f
    [Tooltip("Small reward for maintaining speed")]
    public float speedReward = 0.05f;
    [Tooltip("Penalty for going backwards")]
    public float backwardsPenalty = -0.5f; // Increased from -0.1f
    [Tooltip("Penalty for not making progress")]
    public float noProgressPenalty = -0.05f; // Increased from -0.01f
    [Tooltip("Penalty for jerky steering")]
    public float jerkySteeringPenalty = -0.01f; // Slightly increased
    [Tooltip("Reward for facing the right direction")]
    public float directionAlignmentReward = 0.1f; // Increased from 0.05f
    [Tooltip("Penalty for using reverse")]
    public float reversePenalty = -0.3f;
    [Tooltip("Bonus reward for efficient lap completion")]
    public float lapTimeBonus = 3.0f;
    [Tooltip("Number of laps to complete before ending episode")]
    public int lapsToCompleteBeforeReset;

    [Header("Training Parameters")]
    [Tooltip("Seconds before reset if no checkpoint is passed")]
    public float maxTimeWithoutCheckpoint = 30f;
    [Tooltip("Maximum episode length in seconds")]
    public float maxEpisodeTime = 500f;
    [Tooltip("Threshold for detecting no progress")]
    public float noProgressThreshold = 0.5f;
    [Tooltip("Flag to enable/disable reset on collision")]
    public bool resetOnCollision = true;
    [Tooltip("Flag to reset when returning to a previous checkpoint")]
    public bool resetOnPreviousCheckpoint = true;
    [Tooltip("Minimum distance change to be considered progress")]
    public float noProgressThresholdTime = 5.0f;
    [HideInInspector]
    public int completedCheckpointsBeforeNewTrack = int.MaxValue;

    [Header("Debug")]
    [Tooltip("Visualize direction to next checkpoint")]
    public bool visualizeTargetDirection = true;
    [Tooltip("Show debug messages")]
    public bool isDebugLoggingEnabled = false;

    [Header("Debug Enhancement")]
    public bool enableRewardLogging = false;
    public bool enableResetReasonLogging = false;
    private string lastResetReason = "None";

    [Header("Visualization")]
    public bool visualizeRaySensors = false;
    public bool visualizeActionIntentions = false;
    public Color raySensorHitColor = Color.red;
    public Color raySensorNoHitColor = Color.green;
    public float visualizationDuration = 0.1f;

    // Episode tracking
    private int agentEpisodeCount = 0;

    // Reward tracking
    private Dictionary<string, float> rewardComponents = new Dictionary<string, float>();
    private float episodeTotalReward = 0f;
    private float lastLoggedRewardTime = 0f;
    private float rewardLoggingInterval = 5f; // Log rewards every 5 seconds

    // State tracking
    private bool hasCollided = false;
    private float episodeStartTime;
    private float lapStartTime;
    private float prevDistanceToCheckpoint; // Previous distance to next checkpoint (for progress calculation)
    private Vector3 prevPosition;
    private TrainingManager trainingManager;

    // Progress tracking
    private float noProgressTimer = 0f;
    private Vector3 lastPositionCheck;
    private float positionCheckInterval = 1.0f;
    private float lastPositionCheckTime = 0f;
    private bool isMovingBackwards = false;

    // Advanced reward tracking
    private float prevSteer = 0f;
    private bool isUsingHandbrake = false;
    private float totalDistanceTraveled = 0f;
    private float straightLineDistanceToStart = 0f;
    private float lapProgress = 0f;
    private int currentLapCheckpoints = 0;
    private float centerlineDeviation = 0f;
    private float lapEfficiencyScore = 1.0f;

    // Recovery tracking
    private bool isRecovering = false;
    private Vector3 collisionPosition;
    private float recoveryStartTime;
    private float recoveryCheckTime = 2.0f;

    // Used to detect movement patterns
    private Vector3 lastPosition;
    private List<Vector3> positionHistory = new List<Vector3>();
    private int positionHistoryMaxSize = 10;
    private float positionCheckFrequency = 0.2f;
    private float lastPositionHistoryTime = 0f;

    // Track rebuild coordination
    private bool isWaitingForTrackRebuild = false;
    private bool isReadyForRebuild = false;
    private float rebuilRequestTime = 0f;
    private float maxTimeToFinishBeforeRebuild = 10f;
    private float safetyResetHeight = -50f; // Height at which to force reset if car falls

    // Race debug timer
    private float lastRaceDebugTime = 0f;

    // Memory for observations
    private Vector3 previousPosition;
    private Vector3 previousForward;
    private float[] previousRaySensorData = new float[0]; // Will initialize properly in Start

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

    public override void Initialize()
    {
        // Get the TrainingManager singleton
        trainingManager = TrainingManager.GetInstance();

        // Get references from TrainingManager if possible
        if (trainingManager != null)
        {
            trackHandler = trainingManager.GetTrackHandler();
            raceManager = trainingManager.raceManager;

            // Subscribe to track rebuild events
            trainingManager.OnTrackRebuilt += OnTrackRebuilt;

            if (isDebugLoggingEnabled) Debug.Log($"Car {gameObject.name} successfully initialized with TrainingManager references");
        }
        else
        {
            Debug.LogWarning("TrainingManager singleton not found, trying direct scene lookup");

            // Fallback - try to find in scene directly
            trackHandler = GetComponent<TrackHandler>();
            raceManager = GetComponent<RaceManager>();

            if (trackHandler == null)
            {
                Debug.LogError($"Car {gameObject.name}: Could not find TrackHandler in scene! Car behavior will be incorrect.");
            }
        }

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
            lapsToCompleteBeforeReset = raceManager.totalLaps;
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

    protected override void Awake()
    {
        base.Awake();

        // Verify BehaviorParameters configuration
        var behaviorParams = GetComponent<Unity.MLAgents.Policies.BehaviorParameters>();
        if (behaviorParams != null)
        {
            
            int expectedObservations = 40;
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

            // Periodically log race and checkpoint stats
            if (Time.time > lastRaceDebugTime + 5f)
            {
                if (enableCheckpointDebugging)
                {
                    Debug.Log($"[Agent {gameObject.name}] Race active: {isRaceActive}, " +
                              $"Next checkpoint: {nextCheckpointIndex}, " +
                              $"RaceManager next: {(raceManager != null ? raceManager.GetPlayerNextCheckpoint(gameObject) : -1)}, " +
                              $"Total passed: {totalCheckpointsPassed}");
                }
                lastRaceDebugTime = Time.time;
            }
            if (isReadyForRebuild)
            {
                float stuckTime = Time.time - rebuilRequestTime;
                // If stuck for more than 30 seconds, force reset
                if (stuckTime > 30f)
                {
                    Debug.LogWarning($"[SAFETY] Car {gameObject.name} stuck in rebuild state for {stuckTime:F1}s - forcing reset");

                    // Reset critical flags
                    isReadyForRebuild = false;
                    isWaitingForTrackRebuild = false;

                    // Re-enable required components
                    if (carController != null)
                        carController.enabled = true;

                    Rigidbody rb = GetComponent<Rigidbody>();
                    if (rb != null)
                        rb.isKinematic = false;

                    // Reset car
                    ResetCar();
                    EndEpisode();
                }
            }
        }

        // Add this new block for more detailed progress tracking
        if (Time.frameCount % 120 == 0) // Log every 120 frames
        {
            float progressSpeed = carController.GetCurrentSpeed();
            float distToCheckpoint = distanceToNextCheckpoint;
            float timeSinceCP = Time.time - lastCheckpointTime;

            if (enableResetReasonLogging && timeSinceCP > maxTimeWithoutCheckpoint * 0.5f)
            {
                Debug.Log($"[WARN] Car {gameObject.name} progress: " +
                        $"CP#{nextCheckpointIndex}, " +
                        $"Distance={distToCheckpoint:F1}m, " +
                        $"Speed={progressSpeed:F1}, " +
                        $"TimeSinceCP={timeSinceCP:F1}s/" +
                        $"{maxTimeWithoutCheckpoint:F1}s");
            }
        }

        // Safety check - if car falls below a certain height, force a reset
        if (transform.position.y < safetyResetHeight)
        {
            if (enableResetReasonLogging)
                Debug.LogWarning($"[RESET] Car {gameObject.name} fell below safety height ({safetyResetHeight}). Forcing reset.");
            lastResetReason = "Fell below safety height";
            ResetCar();
            EndEpisode();
            return;
        }

        // Check if we need to force rebuild due to timeout
        if (isWaitingForTrackRebuild && !isReadyForRebuild)
        {
            float timeSinceRequest = Time.time - rebuilRequestTime;
            if (timeSinceRequest > maxTimeToFinishBeforeRebuild)
            {
                if (isDebugLoggingEnabled) Debug.Log($"Car {gameObject.name} forcing finish due to rebuild request timeout");
                NotifyReadyForRebuild();
            }
        }

        // Check for backwards movement at intervals
        if (Time.time - lastPositionCheckTime > positionCheckInterval)
        {
            CheckMovementDirection();
            lastPositionCheckTime = Time.time;
        }

        // Track position history for pattern detection (using for recovery behaviors)
        if (Time.time - lastPositionHistoryTime > positionCheckFrequency)
        {
            UpdatePositionHistory();
            lastPositionHistoryTime = Time.time;
        }


        // Log rewards periodically
        if (Time.time > lastLoggedRewardTime + rewardLoggingInterval && enableRewardLogging)
        {
            LogRewardSummary();
            lastLoggedRewardTime = Time.time;
        }

        // Visualize agent perception
        VisualizeAgentPerception();
    }

    // Action interpolation in FixedUpdate
    void FixedUpdate()
    {
        // Skip if waiting for rebuild or if car controller is null
        if (isReadyForRebuild || carController == null)
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

                    // Annotate with info about what's being observed at this knot
                    Vector3 localCenter = transform.InverseTransformPoint(center);
                    float distance = Vector3.Distance(transform.position, center);

                    // Show important values being observed by drawing different size markers
                    // based on these values (to visually indicate what the agent sees)
                    float observationScale = 0.5f + (lookAheadCount - i) * 0.3f; // Closer observations are larger
                    float distanceScale = Mathf.Clamp01(1.0f - (distance / 100f)) * 2f; // Closer knots get larger markers
                    float finalScale = observationScale * distanceScale;

                    // Draw a cross marker sized according to distance and observation importance
                    Vector3 crossSize = new Vector3(finalScale, finalScale, finalScale);
                    Debug.DrawRay(center, Vector3.right * crossSize.x, observationColor, 0.1f);
                    Debug.DrawRay(center, Vector3.left * crossSize.x, observationColor, 0.1f);
                    Debug.DrawRay(center, Vector3.forward * crossSize.z, observationColor, 0.1f);
                    Debug.DrawRay(center, Vector3.back * crossSize.z, observationColor, 0.1f);
                }
            }
        }
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

    // Update checkpoint information
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

    private void UpdatePositionHistory()
    {
        Vector3 currentPos = transform.position;

        // Only add if we've moved a significant distance
        if (lastPosition != Vector3.zero && Vector3.Distance(currentPos, lastPosition) > 0.1f)
        {
            positionHistory.Add(currentPos);

            // Keep history limited to the max size
            if (positionHistory.Count > positionHistoryMaxSize)
            {
                positionHistory.RemoveAt(0);
            }
        }

        lastPosition = currentPos;
    }

    // Check if the car is moving backwards relative to the target direction
    private void CheckMovementDirection()
    {
        if (checkpoints == null || checkpoints.Count == 0) return;

        Vector3 currentPosition = transform.position;

        if (lastPositionCheck != Vector3.zero)
        {
            Vector3 movementVector = currentPosition - lastPositionCheck;
            totalDistanceTraveled += movementVector.magnitude;

            // Check if movement is opposite to the direction to checkpoint
            float movementAlignment = Vector3.Dot(movementVector.normalized, directionToNextCheckpoint.normalized);
            bool wasMovingBackwards = isMovingBackwards;
            isMovingBackwards = movementAlignment < -0.3f; // If moving significantly away from checkpoint

            // Only apply penalty if this is a new backwards movement
            if (isMovingBackwards && !wasMovingBackwards)
            {
                AddTrackedReward(backwardsPenalty, "backwards");
                if (enableRewardLogging) Debug.Log($"[REWARD] Car {gameObject.name} is moving backwards - applying penalty {backwardsPenalty:F3}");
            }
        }

        lastPositionCheck = currentPosition;
    }

    // Called when the track is rebuilt by the TrainingManager
    private void OnTrackRebuilt()
    {
        // Re-initialize checkpoints when track is rebuilt
        if (trackHandler != null)
        {
            checkpoints = trackHandler.GetCheckpoints();
            nextCheckpointIndex = 0;
            totalCheckpointsPassed = 0;
            currentLap = 0;
            isWaitingForTrackRebuild = false;
            isReadyForRebuild = false;

            // Re-enable components that might have been disabled
            if (carController != null)
            {
                carController.enabled = true;
            }

            // Reset physics state
            Rigidbody rb = GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.isKinematic = false;
            }

            if (isDebugLoggingEnabled) Debug.Log($"Car {gameObject.name} received track rebuild notification and updated checkpoints");
        }
    }

    public override void OnEpisodeBegin()
    {
        // Increment agent's episode counter
        agentEpisodeCount++;

        // Report episode start to training manager
        if (trainingManager != null)
        {
            trainingManager.ReportAgentEpisodeCompleted(this, agentEpisodeCount);
        }

        // Get hyperparameters from Academy if needed
        maxTimeWithoutCheckpoint = Academy.Instance.EnvironmentParameters.GetWithDefault("max_time_without_checkpoint", maxTimeWithoutCheckpoint);
        speedReward = Academy.Instance.EnvironmentParameters.GetWithDefault("speed_reward", speedReward);
        checkpointReward = Academy.Instance.EnvironmentParameters.GetWithDefault("checkpoint_reward", checkpointReward);
        backwardsPenalty = Academy.Instance.EnvironmentParameters.GetWithDefault("backwards_penalty", backwardsPenalty);
        noProgressPenalty = Academy.Instance.EnvironmentParameters.GetWithDefault("no_progress_penalty", noProgressPenalty);
        directionAlignmentReward = Academy.Instance.EnvironmentParameters.GetWithDefault("direction_alignment_reward", directionAlignmentReward);
        collisionPenalty = Academy.Instance.EnvironmentParameters.GetWithDefault("collision_penalty", collisionPenalty);
        noProgressThresholdTime = Academy.Instance.EnvironmentParameters.GetWithDefault("no_progress_threshold_time", noProgressThresholdTime);

        // Log parameters for debugging
        if (isDebugLoggingEnabled)
        {
            Debug.Log($"[PARAMS] Agent {gameObject.name} starting episode {agentEpisodeCount}, " +
                     $"maxTimeWithoutCheckpoint: {maxTimeWithoutCheckpoint}, " +
                     $"noProgressThresholdTime: {noProgressThresholdTime}");
        }

        // Reset flags
        isWaitingForTrackRebuild = false;
        isReadyForRebuild = false;
        isRecovering = false;

        // Reset reward tracking
        rewardComponents.Clear();
        episodeTotalReward = 0f;
        lastLoggedRewardTime = Time.time;

        // Reset the car position and state
        ResetCar();
        ForceCheckpointReset();

        // Initialize checkpoint tracking
        if (trackHandler != null)
        {
            checkpoints = trackHandler.GetCheckpoints();

            // Force synchronize with RaceManager if available
            if (raceManager != null)
            {
                nextCheckpointIndex = raceManager.GetPlayerNextCheckpoint(gameObject);
                if (isDebugLoggingEnabled) Debug.Log($"[Agent {gameObject.name}] Starting episode with checkpoint index: {nextCheckpointIndex} (from RaceManager)");
            }
            else
            {
                nextCheckpointIndex = 0; // Fallback if race manager not available
                if (isDebugLoggingEnabled) Debug.Log($"[Agent {gameObject.name}] Starting episode with checkpoint index: 0 (fallback)");
            }

            distanceToNextCheckpoint = 0f;
            currentLap = 0;
        }
        else
        {
            Debug.LogError("TrackHandler is null in CarAgent.OnEpisodeBegin");

            // Try to recover by getting the reference again
            if (trainingManager != null)
            {
                trackHandler = trainingManager.trackHandler;
                if (trackHandler != null)
                {
                    checkpoints = trackHandler.GetCheckpoints();
                    if (isDebugLoggingEnabled) Debug.Log("Recovered TrackHandler reference");
                }
            }
        }

        // Reset timers and state tracking
        episodeStartTime = Time.time;
        lapStartTime = Time.time;
        lastCheckpointTime = Time.time;
        hasCollided = false;
        totalCheckpointsPassed = 0;
        noProgressTimer = 0f;
        lastPositionCheck = transform.position;
        lastPositionCheckTime = Time.time;
        isMovingBackwards = false;

        // Reset driving stats
        prevSteer = 0f;
        totalDistanceTraveled = 0f;
        lapProgress = 0f;
        currentLapCheckpoints = 0;

        // Clear position history
        positionHistory.Clear();
        lastPosition = transform.position;
        lastPositionHistoryTime = Time.time;

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
        prevDistanceToCheckpoint = distanceToNextCheckpoint;  // Set initial previous distance
        prevPosition = transform.position;

        // Calculate straight-line distance from start to first checkpoint (for efficiency scoring)
        if (checkpoints != null && checkpoints.Count > 0)
        {
            straightLineDistanceToStart = Vector3.Distance(transform.position, checkpoints[0].transform.position);
        }
    }

    // Called by TrainingManager to request this agent to prepare for track rebuild
    public void RequestTrackRebuild()
    {
        if (!isWaitingForTrackRebuild)
        {
            if (isDebugLoggingEnabled) Debug.Log($"Car {gameObject.name} received track rebuild request");
            isWaitingForTrackRebuild = true;
            rebuilRequestTime = Time.time;

            // If we're already in a problematic state, signal ready immediately
            if (hasCollided || transform.position.y < 0)
            {
                NotifyReadyForRebuild();
            }
        }
    }

    // Notify the training manager that this agent is ready for rebuild
    private void NotifyReadyForRebuild()
    {
        if (!isReadyForRebuild)
        {
            isReadyForRebuild = true;

            // Notify training manager we're ready for rebuild
            if (trainingManager != null)
            {
                trainingManager.NotifyAgentReadyForRebuild();
            }

            // Disable movement to prevent further issues
            if (carController != null)
            {
                carController.enabled = false;
            }

            // Keep the agent active but freeze it in place
            Rigidbody rb = GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.isKinematic = true;
            }
        }
    }

    // Called by TrainingManager to reset this agent for a new track
    public void ResetForNewTrack()
    {
        // Reset flags
        isWaitingForTrackRebuild = false;
        isReadyForRebuild = false;
        isRecovering = false;

        // Re-enable components
        if (carController != null)
        {
            carController.enabled = true;
        }

        Rigidbody rb = GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.isKinematic = false;
        }

        // Reset checkpoint-related data
        nextCheckpointIndex = 0;
        totalCheckpointsPassed = 0;
        lastCheckpointTime = Time.time;
        currentLap = 0;

        // Reset episode count for this agent only if configured to do so
        bool shouldResetEpisodes = (trainingManager != null) ? trainingManager.resetEpisodesAfterRebuild : true;

        if (shouldResetEpisodes)
        {
            agentEpisodeCount = 0;
            if (isDebugLoggingEnabled) Debug.Log($"Car {gameObject.name} episode counter reset to 0 for new track");
        }
        else
        {
            if (isDebugLoggingEnabled) Debug.Log($"Car {gameObject.name} maintaining episode count at {agentEpisodeCount} for new track");
        }

        // Update checkpoints list
        if (trackHandler != null)
        {
            checkpoints = trackHandler.GetCheckpoints();
            if (isDebugLoggingEnabled) Debug.Log($"Car {gameObject.name} reset for new track with {checkpoints.Count} checkpoints");
        }
        InitializeTrackMemory();
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

                //Debug.Log($"Car {gameObject.name} reset to rotation: {transform.rotation.eulerAngles}");
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

    public override void CollectObservations(VectorSensor sensor)
    {
        // Make sure sensor system is initialized
        if (sensorSystem == null)
        {
            Debug.LogError("SensorSystem is null in CollectObservations!");
            // Add blank observations to prevent errors
            int observations = (sensorSystem.totalRayCount * 2) + 6;
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

        // If we're waiting for rebuild, don't process actions
        if (isReadyForRebuild)
            return;

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

                // Calculate steering smoothness penalty
                float steerDelta = Mathf.Abs(currentActions[1] - prevSteer);
                if (steerDelta > 0.3f) // Only penalize large, abrupt steering changes
                {
                    AddTrackedReward(steerDelta * jerkySteeringPenalty, "jerky_steering");
                }
                prevSteer = currentActions[1];

                // Apply penalty for using reverse
                if (currentActions[2] > 0.5f)
                {
                    AddTrackedReward(reversePenalty * currentActions[2], "reverse");
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
                isUsingHandbrake = currentHandbrake; // Keep this for other logic
            }

            // Reset interpolation timer
            interpolationTime = 0f;

            // Update decision interval dynamically
            var decisionRequester = GetComponent<Unity.MLAgents.DecisionRequester>();
            if (decisionRequester != null)
            {
                decisionInterval = Time.fixedDeltaTime * decisionRequester.DecisionPeriod;
            }

            // Remove direct SetCarInputs call - it will be called in FixedUpdate with interpolation
            // SetCarInputs(accelerate, steer, brake, isUsingHandbrake);

            // Check if race is active
            bool isRaceActive = raceManager != null && raceManager.IsRaceActive();

            // Check timeouts unless we're waiting for rebuild
            float timeSinceLastCheckpoint = Time.time - lastCheckpointTime;
            float episodeTime = Time.time - episodeStartTime;

            if (!isWaitingForTrackRebuild && isRaceActive &&
                (timeSinceLastCheckpoint > maxTimeWithoutCheckpoint ||
                episodeTime > maxEpisodeTime))
            {
                // Add clear logging about which timeout occurred
                if (timeSinceLastCheckpoint > maxTimeWithoutCheckpoint)
                {
                    if (enableResetReasonLogging)
                        Debug.LogWarning($"[RESET] Car {gameObject.name} - Checkpoint timeout after {timeSinceLastCheckpoint:F1}s > {maxTimeWithoutCheckpoint:F1}s");
                    lastResetReason = "Checkpoint timeout";
                }
                else if (episodeTime > maxEpisodeTime)
                {
                    if (enableResetReasonLogging)
                        Debug.LogWarning($"[RESET] Car {gameObject.name} - Episode timeout after {episodeTime:F1}s > {maxEpisodeTime:F1}s");
                    lastResetReason = "Episode timeout";
                }

                // Penalty for timeout
                AddTrackedReward(-1.0f, "timeout");

                // Reset this agent (no synchronization required)
                EndEpisode();
                return;
            }

            // Make sure we're synchronized with RaceManager
            if (isRaceActive && raceManager != null)
            {
                SyncCheckpointWithRaceManager();
            }

            UpdateCheckpointInfo();
            CalculateRewards();
            CheckForNoProgress();

            prevDistanceToCheckpoint = distanceToNextCheckpoint;
            prevPosition = transform.position;

            // If we've been asked to prepare for rebuild and we hit a wall, notify
            if (isWaitingForTrackRebuild && hasCollided)
            {
                NotifyReadyForRebuild();
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"Error in OnActionReceived: {ex.Message}");
        }
    }

    // Check if car is making progress toward the next checkpoint
    private void CheckForNoProgress()
    {
        // Make sure we have valid previous distance
        if (prevDistanceToCheckpoint <= 0)
        {
            prevDistanceToCheckpoint = distanceToNextCheckpoint;
            return;
        }

        float distanceDelta = Mathf.Abs(prevDistanceToCheckpoint - distanceToNextCheckpoint);

        // If we're not making significant progress
        if (distanceDelta < noProgressThreshold)
        {
            noProgressTimer += Time.deltaTime;

            // Apply penalty if we've been not making progress for too long
            if (noProgressTimer > noProgressThresholdTime)
            {
                AddTrackedReward(noProgressPenalty, "no_progress");

                // Add warning when getting close to reset threshold
                if (enableResetReasonLogging && noProgressTimer > noProgressThresholdTime * 1.5f)
                {
                    Debug.LogWarning($"[WARN] Car {gameObject.name} - No progress for {noProgressTimer:F1}s - may reset soon");
                    lastResetReason = "No progress timeout";
                }

                // Reset timer to avoid continuous penalties
                noProgressTimer = 0f;
            }
        }
        else
        {
            // Reset the timer if we're making progress
            noProgressTimer = 0f;
        }
    }

    // Modified reward calculation for better cornering behavior
    private void CalculateRewards()
    {
        // Make sure we have valid previous distance
        if (prevDistanceToCheckpoint <= 0)
        {
            prevDistanceToCheckpoint = distanceToNextCheckpoint;
            return;
        }

        // 1. Calculate progress reward toward checkpoint
        float distanceDelta = prevDistanceToCheckpoint - distanceToNextCheckpoint;

        // Stronger signals for progress rewards
        float progressReward = 0.0f;
        if (distanceDelta > 0)
        {
            // Give much stronger reward for making progress toward checkpoint
            progressReward = distanceDelta * 0.3f;

            // Additional boost for early training
            if (totalCheckpointsPassed < 5)
            {
                progressReward *= 3.0f; // Triple reward for very early progress
            }
        }
        else
        {
            // Milder penalty for moving away
            progressReward = distanceDelta * 0.05f;
        }

        AddTrackedReward(progressReward, "progress");

        // 2. Improved cornering reward for arcade racing techniques
        float speed = carController.GetCurrentSpeed();
        float steeringMagnitude = Mathf.Abs(prevSteer);
        bool isTurning = steeringMagnitude > 0.3f; // Threshold for significant turning
        bool isDrifting = carController.isDrifting;

        // Normal speed reward (will be applied whether cornering or not)
        float speedRewardValue = speed * speedReward;
        AddTrackedReward(speedRewardValue, "speed");

        // 3. Direction alignment reward - more important now
        float forwardDot = Vector3.Dot(transform.forward, directionToNextCheckpoint.normalized);
        // Use a stronger non-linear function to emphasize proper alignment
        // Square the dot product and ensure it's only positive
        float alignmentReward = Mathf.Pow(Mathf.Max(0, forwardDot), 2) * directionAlignmentReward;
        // Additional bonus for perfect alignment
        if (forwardDot > 0.9f)
        {
            alignmentReward *= 1.5f; // Bonus for near-perfect alignment
        }
        AddTrackedReward(alignmentReward, "alignment");

        // 4. Survival reward for staying on track
        float stayingOnTrackReward = 0.01f * Time.deltaTime;
        AddTrackedReward(stayingOnTrackReward, "survival");

        // 5. Improved collision handling
        if (hasCollided)
        {
            // Handle collision based on the current training mode
            bool useStrictMode = trainingManager != null && trainingManager.IsUsingStrictResets();

            // Apply stronger collision penalty
            AddTrackedReward(collisionPenalty, "collision");

            if (enableResetReasonLogging)
                Debug.LogWarning($"[COLLISION] Car {gameObject.name} collided with obstacle");

            // If we're waiting for rebuild, notify immediately
            if (isWaitingForTrackRebuild)
            {
                hasCollided = false;
                lastResetReason = "Collision during rebuild request";
                NotifyReadyForRebuild();
                return;
            }

            // Modified collision handling: Only reset in strict mode
            // No longer reset on high speed collisions automatically
            if (resetOnCollision && useStrictMode)
            {
                // Apply more severe penalty for crashing in strict mode
                AddTrackedReward(-1.0f, "severe_collision");
                lastResetReason = "Collision with strict reset";

                // End episode for this agent
                EndEpisode();
            }
            else
            {
                // In recovery mode - start tracking recovery
                collisionPosition = transform.position;
                recoveryStartTime = Time.time;
                isRecovering = true;              
            }

            // Reset collision flag
            hasCollided = false;
        }
    }

    // Called when a checkpoint is passed
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

            // Apply rewards
            CalculateCheckpointReward(checkpointIndex);

            // Update timers and counters
            lastCheckpointTime = Time.time;
            totalCheckpointsPassed++;

            // Check for lap completion - if we just hit the last checkpoint and next is 0
            if (checkpointIndex == checkpoints.Count - 1 && nextCheckpointIndex == 0)
            {
                currentLap++;
            }
        }
    }

    private void ForceCheckpointReset()
    {
        // Reset all checkpoint tracking
        nextCheckpointIndex = 0;
        totalCheckpointsPassed = 0;
        currentLap = 0;

        // Reset related variables
        lastCheckpointTime = Time.time;
        currentLapCheckpoints = 0;
        lapProgress = 0f;

        // Tell RaceManager to reset this agent's checkpoint
        if (raceManager != null)
        {
            try
            {
                raceManager.ResetAgent(gameObject);

                // Sync with RaceManager's checkpoint
                nextCheckpointIndex = raceManager.GetPlayerNextCheckpoint(gameObject);
            }
            catch (Exception e)
            {
                Debug.LogWarning($"Failed to reset checkpoint in RaceManager: {e.Message}");
                // Fallback to 0
                nextCheckpointIndex = 0;
            }
        }

        // Update checkpoint info
        UpdateCheckpointInfo();
    }

    // Calculate checkpoint rewards without changing checkpoint index
    private void CalculateCheckpointReward(int checkpointIndex)
    {
        // Base checkpoint reward
        float checkpointBonus = checkpointReward * (1 + Mathf.Min(1.0f, totalCheckpointsPassed / 20.0f));
        AddTrackedReward(checkpointBonus, "checkpoint");

        // Update lap progress tracking
        currentLapCheckpoints++;
        if (checkpoints.Count > 0)
        {
            lapProgress = currentLapCheckpoints / (float)checkpoints.Count;
        }

        // Check for lap completion - if we just hit the last checkpoint and next is 0
        if (checkpointIndex == checkpoints.Count - 1 && nextCheckpointIndex == 0)
        {
            ApplyLapCompletionRewards();
        }
    }

    // Apply rewards for completing a lap
    private void ApplyLapCompletionRewards()
    {
        // Calculate lap time and efficiency
        float lapTime = Time.time - lapStartTime;
        lapStartTime = Time.time;

        // Calculate lap efficiency
        CalculateLapEfficiency();

        // Base reward for completing lap
        AddTrackedReward(lapCompletionReward, "lap_completion");

        // Time bonus for completing lap quickly
        float timeBonus = Mathf.Max(0, (maxEpisodeTime - lapTime) / maxEpisodeTime) * lapTimeBonus;
        AddTrackedReward(timeBonus, "lap_time_bonus");

        // Efficiency bonus
        AddTrackedReward(lapEfficiencyScore * 2.0f, "lap_efficiency");

        // Notify TrainingManager of lap completion
        if (trainingManager != null)
        {
            trainingManager.RecordLapCompletion(lapTime);

            // Report the agent's episode count
            trainingManager.ReportAgentEpisodeCompleted(this, agentEpisodeCount);
        }

        // Reset lap tracking
        currentLapCheckpoints = 0;
        totalDistanceTraveled = 0f;

        if (enableCheckpointDebugging)
        {
            Debug.Log($"[Agent {gameObject.name}] Completed full lap in {lapTime:F2}s with efficiency {lapEfficiencyScore:F2}");
        }

        // NEW CODE: Check if we've completed enough laps to end the episode
        if (currentLap >= lapsToCompleteBeforeReset)
        {
            if (enableResetReasonLogging)
                Debug.Log($"[RESET] Car {gameObject.name} completed {currentLap} laps - ending episode");

            lastResetReason = "Completed target number of laps";

            // End the episode which will trigger OnEpisodeBegin() and increment the episode counter
            EndEpisode();
        }
    }

    private void CalculateLapEfficiency()
    {
        if (totalDistanceTraveled > 0 && checkpoints != null && checkpoints.Count > 0)
        {
            // Calculate ideal path length (approximate with checkpoint distances)
            float idealPathLength = 0f;
            for (int i = 0; i < checkpoints.Count; i++)
            {
                int nextIdx = (i + 1) % checkpoints.Count;
                idealPathLength += Vector3.Distance(
                    checkpoints[i].transform.position,
                    checkpoints[nextIdx].transform.position);
            }

            // Calculate efficiency ratio (ideal / actual)
            // Add a buffer factor to account for necessary steering
            float bufferFactor = 1.2f; // Allow 20% more distance than straight-line path
            lapEfficiencyScore = Mathf.Clamp01(idealPathLength * bufferFactor / totalDistanceTraveled);

            if (isDebugLoggingEnabled)
            {
                Debug.Log($"Lap efficiency: {lapEfficiencyScore:F2} (Ideal: {idealPathLength:F1}, Actual: {totalDistanceTraveled:F1})");
            }
        }
    }

    // Called when the car collides with something
    public void ReportCollision()
    {
        hasCollided = true;
    }

    // Set input values on the car controller
    private void SetCarInputs(float accelerate, float steer, float brake, bool handbrake)
    {
        carController.SetAccelerationInput(accelerate);
        carController.SetSteeringInput(steer);
        carController.SetReverseInput(brake);
        carController.SetHandbrakeInput(handbrake);
    }

    // Reward tracking methods
    private void AddTrackedReward(float reward, string component)
    {
        // Call the base AddReward
        AddReward(reward);

        // Track the total
        episodeTotalReward += reward;

        // Track this component
        if (!rewardComponents.ContainsKey(component))
            rewardComponents[component] = 0f;

        rewardComponents[component] += reward;

        // Log if reward is significant or if debug is enabled
        if (enableRewardLogging && (Mathf.Abs(reward) > 0.1f || component == "checkpoint" || component == "lap"))
        {
            Debug.Log($"[REWARD] Car {gameObject.name} - {component}: {reward:F3}, Total: {episodeTotalReward:F3}");
        }
    }

    private void LogRewardSummary()
    {
        if (rewardComponents.Count == 0) return;

        StringBuilder sb = new StringBuilder();
        sb.AppendLine($"[REWARD SUMMARY] Car {gameObject.name} - Total: {episodeTotalReward:F2}");

        foreach (var kvp in rewardComponents.OrderByDescending(x => Mathf.Abs(x.Value)))
        {
            sb.AppendLine($"  {kvp.Key}: {kvp.Value:F2}");
        }

        Debug.Log(sb.ToString());
    }

    // Reset checkpoint state for new episode
    private void ResetCheckpointState()
    {
        nextCheckpointIndex = 0;
        totalCheckpointsPassed = 0;
        currentLap = 0;
        lastCheckpointTime = Time.time;

        // Force sync with RaceManager if available
        if (raceManager != null)
        {
            nextCheckpointIndex = raceManager.GetPlayerNextCheckpoint(gameObject);
        }

        UpdateCheckpointInfo();
    }

    protected override void OnDisable()
    {
        base.OnDisable();

        // Unsubscribe from events
        if (trainingManager != null)
        {
            trainingManager.OnTrackRebuilt -= OnTrackRebuilt;
        }
        if (raceManager != null)
        {
            raceManager.OnCheckpointPassed -= OnRaceManagerCheckpointPassed;
        }
    }

    void OnDestroy()
    {
        // Unsubscribe from events
        if (trainingManager != null)
        {
            trainingManager.OnTrackRebuilt -= OnTrackRebuilt;
        }
        if (raceManager != null)
        {
            raceManager.OnCheckpointPassed -= OnRaceManagerCheckpointPassed;
        }
    }
}