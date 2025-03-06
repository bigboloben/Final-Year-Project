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
    // Use an integer array for better performance than HashSet
    private readonly int[] visitedCheckpoints = new int[100]; // 100 is more than enough
    private int visitedCheckpointCount = 0;

    [Header("Checkpoint Debugging")]
    public bool enableCheckpointDebugging = true;
    private bool lastRaceActiveState = false;

    [Header("Reward Parameters")]
    [Tooltip("Reward for passing a checkpoint")]
    public float checkpointReward = 5.0f;
    [Tooltip("Base reward for completing a lap")]
    public float lapCompletionReward = 5.0f;
    [Tooltip("Penalty for hitting walls")]
    public float collisionPenalty = -0.1f;
    [Tooltip("Small reward for maintaining speed")]
    public float speedReward = 0.05f;
    [Tooltip("Penalty for going backwards")]
    public float backwardsPenalty = -0.1f;
    [Tooltip("Penalty for not making progress")]
    public float noProgressPenalty = -0.01f;
    [Tooltip("Penalty for jerky steering")]
    public float jerkySteeringPenalty = -0.005f;
    [Tooltip("Reward for facing the right direction")]
    public float directionAlignmentReward = 0.05f;
    [Tooltip("Penalty for using reverse")]
    public float reversePenalty = -0.3f;
    [Tooltip("Bonus reward for efficient lap completion")]
    public float lapTimeBonus = 3.0f;
    [Tooltip("Reward for skillful cornering techniques (drifting, handbrake turns)")]
    public float corneringReward = 0.15f;
    [Tooltip("Reward for recovering from difficult situations")]
    public float recoveryReward = 0.5f;

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
    public bool isDebugLoggingEnabled = true;

    [Header("Debug Enhancement")]
    public bool enableRewardLogging = true;
    public bool enableResetReasonLogging = true;
    private string lastResetReason = "None";

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

    // Reset coordination
    private bool resetRequested = false;
    private bool isReadyForReset = false;
    private float resetRequestTime = 0f;
    private float maxTimeToFinishAfterResetRequest = 10f;
    private bool isForcedReset = false;
    private float safetyResetHeight = -10f; // Height at which to force reset if car falls
    private bool isTrackRebuildPending = false;

    // Race debug timer
    private float lastRaceDebugTime = 0f;

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

            // Subscribe to episode reset events
            trainingManager.OnEpisodeReset += OnEpisodeReset;

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
        }
    }

    void Awake()
    {
        base.Awake();

        // Verify BehaviorParameters configuration
        var behaviorParams = GetComponent<Unity.MLAgents.Policies.BehaviorParameters>();
        if (behaviorParams != null)
        {
            // Force the correct observation size
            int expectedObservations = 14; // 9 rays + 5 other values

            if (behaviorParams.BrainParameters.VectorObservationSize != expectedObservations)
            {
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
        }

        // Add this new block for more detailed progress tracking
        if (!resetRequested && Time.frameCount % 120 == 0) // Log every 120 frames
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
            isForcedReset = true;
            NotifyReadyForReset();
            return;
        }

        // Check if we need to force finish due to reset request timeout
        if (resetRequested && !isReadyForReset)
        {
            float timeSinceRequest = Time.time - resetRequestTime;
            if (timeSinceRequest > maxTimeToFinishAfterResetRequest)
            {
                if (isDebugLoggingEnabled) Debug.Log($"Car {gameObject.name} forcing finish due to reset request timeout");
                isForcedReset = true;
                NotifyReadyForReset();
            }
        }

        // Check for backwards movement at intervals - only do this if we're not in reset mode
        if (!resetRequested && Time.time - lastPositionCheckTime > positionCheckInterval)
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

        // Check for recovery progress
        if (isRecovering && Time.time - recoveryStartTime > recoveryCheckTime)
        {
            CheckRecoveryProgress();
        }

        // Apply random perturbation based on curriculum (when not in reset mode)
        if (!resetRequested && trainingManager != null)
        {
            // Only apply this in "recovery learning" phase
            if (!trainingManager.IsUsingStrictResets())
            {
                trainingManager.MaybeApplyRandomPerturbation(this);
            }
        }

        // Log rewards periodically
        if (Time.time > lastLoggedRewardTime + rewardLoggingInterval && enableRewardLogging)
        {
            LogRewardSummary();
            lastLoggedRewardTime = Time.time;
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

    // Check if the car has successfully recovered from a collision
    private void CheckRecoveryProgress()
    {
        if (!isRecovering) return;

        // Check if we've moved a significant distance from collision point
        float distanceFromCollision = Vector3.Distance(transform.position, collisionPosition);

        // Check if we're making progress toward the checkpoint
        float forwardDot = Vector3.Dot(transform.forward, directionToNextCheckpoint.normalized);
        bool facingRightDirection = forwardDot > 0.7f;

        // If we've moved away from collision point and are facing the right way
        if (distanceFromCollision > 5.0f && facingRightDirection)
        {
            // Reward successful recovery
            AddTrackedReward(recoveryReward, "recovery");
            if (enableRewardLogging) Debug.Log($"[REWARD] Car {gameObject.name} successfully recovered from collision - applying reward {recoveryReward:F3}");
        }

        // End recovery tracking
        isRecovering = false;
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

    // Called when all agents are ready and the TrainingManager signals a global reset
    private void OnEpisodeReset()
    {
        if (isReadyForReset)
        {
            // Reset flags
            resetRequested = false;
            isReadyForReset = false;
            isForcedReset = false;
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

            // Start a new episode
            EndEpisode();
        }
    }

    public override void OnEpisodeBegin()
    {
        // Get hyperparameters from Academy if needed
        maxTimeWithoutCheckpoint = Academy.Instance.EnvironmentParameters.GetWithDefault("max_time_without_checkpoint", maxTimeWithoutCheckpoint);
        speedReward = Academy.Instance.EnvironmentParameters.GetWithDefault("speed_reward", speedReward);
        checkpointReward = Academy.Instance.EnvironmentParameters.GetWithDefault("checkpoint_reward", checkpointReward);
        backwardsPenalty = Academy.Instance.EnvironmentParameters.GetWithDefault("backwards_penalty", backwardsPenalty);
        noProgressPenalty = Academy.Instance.EnvironmentParameters.GetWithDefault("no_progress_penalty", noProgressPenalty);
        directionAlignmentReward = Academy.Instance.EnvironmentParameters.GetWithDefault("direction_alignment_reward", directionAlignmentReward);
        collisionPenalty = Academy.Instance.EnvironmentParameters.GetWithDefault("collision_penalty", collisionPenalty);
        corneringReward = Academy.Instance.EnvironmentParameters.GetWithDefault("cornering_reward", corneringReward);
        noProgressThresholdTime = Academy.Instance.EnvironmentParameters.GetWithDefault("no_progress_threshold_time", noProgressThresholdTime);

        // Log parameters for debugging
        if (isDebugLoggingEnabled)
        {
            Debug.Log($"[PARAMS] maxTimeWithoutCheckpoint: {maxTimeWithoutCheckpoint}, " +
                     $"noProgressThresholdTime: {noProgressThresholdTime}, " +
                     $"corneringReward: {corneringReward}");
        }

        // Reset flags
        resetRequested = false;
        isReadyForReset = false;
        isForcedReset = false;
        isRecovering = false;

        // Reset reward tracking
        rewardComponents.Clear();
        episodeTotalReward = 0f;
        lastLoggedRewardTime = Time.time;

        // Reset the car position and state
        ResetCar();

        // Initialize checkpoint tracking
        if (trackHandler != null)
        {
            checkpoints = trackHandler.GetCheckpoints();

            // Force synchronize with RaceManager if available
            if (raceManager != null)
            {
                nextCheckpointIndex = raceManager.GetPlayerNextCheckpoint(gameObject);
                Debug.Log($"[Agent {gameObject.name}] Starting episode with checkpoint index: {nextCheckpointIndex} (from RaceManager)");
            }
            else
            {
                nextCheckpointIndex = 0; // Fallback if race manager not available
                Debug.Log($"[Agent {gameObject.name}] Starting episode with checkpoint index: 0 (fallback)");
            }

            distanceToNextCheckpoint = 0f;

            // Clear visited checkpoints
            visitedCheckpointCount = 0;
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

    // Called by TrainingManager to request this agent to finish and reset
    public void RequestFinishForReset(bool forRebuild = false)
    {
        if (!resetRequested)
        {
            if (isDebugLoggingEnabled) Debug.Log($"Car {gameObject.name} received {(forRebuild ? "rebuild" : "reset")} request");
            resetRequested = true;
            resetRequestTime = Time.time;
            isTrackRebuildPending = forRebuild;

            // If we're already in a collision or problematic state, finish immediately
            if (hasCollided || transform.position.y < 0)
            {
                NotifyReadyForReset();
            }
        }
        else
        {
            // Already in reset mode - update flag if needed
            isTrackRebuildPending = isTrackRebuildPending || forRebuild;
        }
    }

    // Notify the training manager that this agent is ready for reset
    private void NotifyReadyForReset()
    {
        if (!isReadyForReset)
        {
            isReadyForReset = true;

            // Notify training manager we're ready for reset
            if (trainingManager != null)
            {
                trainingManager.NotifyAgentReadyForReset(isTrackRebuildPending);
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

    // Called when the track is rebuilt by the TrainingManager
    private void OnTrackRebuilt()
    {
        // Re-initialize checkpoints when track is rebuilt
        if (trackHandler != null)
        {
            checkpoints = trackHandler.GetCheckpoints();
            nextCheckpointIndex = 0;
            totalCheckpointsPassed = 0;
            visitedCheckpointCount = 0; // Clear visited checkpoints
            isTrackRebuildPending = false;

            if (isDebugLoggingEnabled) Debug.Log($"Car {gameObject.name} received track rebuild notification and updated checkpoints");
        }
    }

    // Called by TrainingManager to reset this agent for a new track
    public void ResetForNewTrack()
    {
        // Reset flags
        resetRequested = false;
        isReadyForReset = false;
        isForcedReset = false;
        isTrackRebuildPending = false;
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
        visitedCheckpointCount = 0;

        // Update checkpoints list
        if (trackHandler != null)
        {
            checkpoints = trackHandler.GetCheckpoints();
            if (isDebugLoggingEnabled) Debug.Log($"Car {gameObject.name} reset for new track with {checkpoints.Count} checkpoints");
        }
    }

    // Reset the car position and state
    private void ResetCar()
    {
        if (carController != null)
        {
            try
            {
                carController.ResetPosition();
            }
            catch (Exception e)
            {
                // Use manual reset as fallback
                if (trackHandler != null)
                {
                    // Get start position from track
                    (Vector3 startPosition, Vector3 startPosition2, Quaternion startRotation) = trackHandler.GetTrackStartTransform();

                    // Manually reset the car position and rotation
                    transform.position = startPosition;
                    transform.rotation = startRotation;

                    // Reset physics if rigidbody exists
                    Rigidbody rb = GetComponent<Rigidbody>();
                    if (rb != null)
                    {
                        rb.linearVelocity = Vector3.zero;
                        rb.angularVelocity = Vector3.zero;
                    }

                    if (isDebugLoggingEnabled) Debug.LogWarning($"Used manual reset for {gameObject.name} due to error: {e.Message}");
                }
                else
                {
                    Debug.LogError($"Cannot reset car position: both carController and trackHandler are not usable");
                }
            }
        }
        else
        {
            if (isDebugLoggingEnabled) Debug.LogWarning("CarController is null in ResetCar, looking for it now...");
            carController = GetComponent<CarControlScript>();

            if (carController != null)
            {
                try
                {
                    carController.ResetPosition();
                }
                catch (Exception ex)
                {
                    Debug.LogWarning($"Still failed to reset position: {ex.Message}");
                }
            }
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Add ray sensor data (distances to walls)
        float[] raySensorData = sensorSystem.GetAllSensorData();

        // Add each ray sensor observation individually
        foreach (float rayDistance in raySensorData)
        {
            sensor.AddObservation(rayDistance);
        }

        // Add normalized direction to next checkpoint (2 values representing a 2D direction vector)
        float forwardDot = Vector3.Dot(transform.forward, directionToNextCheckpoint.normalized);
        float rightDot = Vector3.Dot(transform.right, directionToNextCheckpoint.normalized);

        // Add direction observations
        sensor.AddObservation(forwardDot); // How aligned we are with the next checkpoint
        sensor.AddObservation(rightDot);   // If the checkpoint is to our right or left

        // Add normalized distance to next checkpoint
        sensor.AddObservation(Mathf.Clamp01(distanceToNextCheckpoint / 100f));

        // Add car speed information
        float speed = carController.GetCurrentSpeed();
        sensor.AddObservation(speed / carController.maxVelocity); // Normalized speed

        // Add minimal car-specific information
        sensor.AddObservation(carController.isDrifting ? 1f : 0f); // Is the car drifting
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // If we're waiting for reset, don't process actions
        if (isReadyForReset)
            return;

        try
        {
            // Extract continuous actions
            float accelerate = 0f;
            float steer = 0f;
            float brake = 0f;

            if (actionBuffers.ContinuousActions.Length >= 3)
            {
                accelerate = actionBuffers.ContinuousActions[0];
                steer = actionBuffers.ContinuousActions[1];
                brake = actionBuffers.ContinuousActions[2];

                // Debug visualization only if enabled
                if (visualizeTargetDirection)
                {
                    Debug.DrawRay(transform.position, transform.forward * accelerate * 5f, Color.green);
                    Debug.DrawRay(transform.position, transform.right * steer * 5f, Color.red);
                }

                // Calculate steering smoothness penalty
                float steerDelta = Mathf.Abs(steer - prevSteer);
                if (steerDelta > 0.3f) // Only penalize large, abrupt steering changes
                {
                    AddTrackedReward(steerDelta * jerkySteeringPenalty, "jerky_steering");
                }
                prevSteer = steer;

                // Apply penalty for using reverse
                if (brake > 0.5f)
                {
                    AddTrackedReward(reversePenalty * brake, "reverse");
                }
            }
            else
            {
                Debug.LogWarning($"Expected 3 continuous actions, but got {actionBuffers.ContinuousActions.Length}");
            }

            // Extract discrete action (handbrake)
            isUsingHandbrake = false;
            if (actionBuffers.DiscreteActions.Length > 0)
            {
                isUsingHandbrake = actionBuffers.DiscreteActions[0] == 1;
            }

            // Apply inputs to car controller
            SetCarInputs(accelerate, steer, brake, isUsingHandbrake);

            // Check if race is active
            bool isRaceActive = raceManager != null && raceManager.IsRaceActive();

            // Check timeouts unless we're in reset mode
            float timeSinceLastCheckpoint = Time.time - lastCheckpointTime;
            float episodeTime = Time.time - episodeStartTime;

            if (!resetRequested && isRaceActive &&
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

                // Request a synchronized episode reset
                if (trainingManager != null)
                {
                    trainingManager.RequestEpisodeReset();
                }
                else
                {
                    // If no TrainingManager, just end this episode
                    EndEpisode();
                }
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

            // If we've been asked to finish for reset and we hit a wall, notify
            if (resetRequested && hasCollided)
            {
                NotifyReadyForReset();
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

        if (isTurning)
        {
            // Base cornering reward - maintaining speed through corners is good
            float normalizedSpeed = speed / carController.maxVelocity;
            float cornerSpeedReward = normalizedSpeed * corneringReward;

            // Additional reward for successful handbrake turns in tight corners
            if (isUsingHandbrake && steeringMagnitude > 0.6f && speed > carController.maxVelocity * 0.4f)
            {
                // Reward handbrake usage in tight corners
                AddTrackedReward(corneringReward * 1.5f, "handbrake_turn");

                if (isDrifting)
                {
                    // Extra reward for controlled drifting
                    AddTrackedReward(corneringReward * 0.5f, "drift");
                }
            }

            // Reward for maintaining direction to next checkpoint while cornering
            float forwardDotDuringCorner = Vector3.Dot(transform.forward, directionToNextCheckpoint.normalized);
            if (forwardDotDuringCorner > 0.5f) // Still generally pointing toward checkpoint
            {
                AddTrackedReward(cornerSpeedReward, "cornering");
            }
        }

        // Normal speed reward (will be applied whether cornering or not)
        float speedRewardValue = speed * speedReward;
        AddTrackedReward(speedRewardValue, "speed");

        // 3. Direction alignment reward - more important now
        float forwardDot = Vector3.Dot(transform.forward, directionToNextCheckpoint.normalized);
        // Square the dot product to emphasize proper alignment
        float alignmentReward = Mathf.Pow(Mathf.Max(0, forwardDot), 2) * directionAlignmentReward;
        AddTrackedReward(alignmentReward, "alignment");

        // 4. Survival reward for staying on track
        float stayingOnTrackReward = 0.01f * Time.deltaTime;
        AddTrackedReward(stayingOnTrackReward, "survival");

        // 5. Improved collision handling
        if (hasCollided)
        {
            // Handle collision based on the current training mode
            bool useStrictMode = trainingManager != null && trainingManager.IsUsingStrictResets();

            // Always add the collision penalty (reduced from before)
            AddTrackedReward(collisionPenalty, "collision");

            if (enableResetReasonLogging)
                Debug.LogWarning($"[COLLISION] Car {gameObject.name} collided with obstacle");

            // If we're in reset mode, notify immediately
            if (resetRequested)
            {
                hasCollided = false;
                lastResetReason = "Collision during reset request";
                NotifyReadyForReset();
                return;
            }

            // If using strict resets and not in recovery mode
            if (resetOnCollision && useStrictMode)
            {
                // Apply more severe penalty for crashing
                AddTrackedReward(-1.0f, "severe_collision");

                lastResetReason = "Collision with strict reset";

                // Request synchronized reset
                if (trainingManager != null)
                {
                    trainingManager.RequestEpisodeReset();
                }
                else
                {
                    EndEpisode();
                }
            }
            else
            {
                // In recovery mode - start tracking recovery
                collisionPosition = transform.position;
                recoveryStartTime = Time.time;
                isRecovering = true;

                // Apply a gentler velocity reduction to simulate impact
                if (speed > 10f)
                {
                    Rigidbody rb = GetComponent<Rigidbody>();
                    if (rb != null) rb.linearVelocity *= 0.9f;
                }
            }

            // Reset collision flag
            hasCollided = false;
        }
    }

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

            // Get next expected checkpoint directly from RaceManager
            int previousIndex = nextCheckpointIndex;
            nextCheckpointIndex = raceManager.GetPlayerNextCheckpoint(gameObject);

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

            // Update the visited checkpoints array
            if (visitedCheckpointCount < visitedCheckpoints.Length)
            {
                visitedCheckpoints[visitedCheckpointCount++] = checkpointIndex;
            }
        }
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
        }

        // Reset lap tracking
        currentLapCheckpoints = 0;
        totalDistanceTraveled = 0f;

        // Clear visited checkpoints at the start of a new lap
        visitedCheckpointCount = 0;
        // Add start checkpoint as visited
        visitedCheckpoints[visitedCheckpointCount++] = 0;

        // Increment episode counter in TrainingManager
        if (trainingManager != null)
        {
            trainingManager.IncrementEpisode();
        }

        if (enableCheckpointDebugging)
        {
            Debug.Log($"[Agent {gameObject.name}] Completed full lap in {lapTime:F2}s with efficiency {lapEfficiencyScore:F2}");
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

    private void UpdateCheckpointInfo()
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
            Vector3 previousDirection = directionToNextCheckpoint; // Store previous for comparison

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

            // Verify direction changed
            if (enableCheckpointDebugging && previousDirection != Vector3.zero && Time.frameCount % 300 == 0)
            {
                bool directionChanged = Vector3.Angle(previousDirection.normalized, directionToNextCheckpoint.normalized) > 5f;
                if (directionChanged)
                {
                    Debug.Log($"[Agent {gameObject.name}] Direction to checkpoint {nextCheckpointIndex} CHANGED to {directionToNextCheckpoint.normalized}");
                }
            }
        }
        else
        {
            if (enableCheckpointDebugging)
                Debug.LogError($"[Agent {gameObject.name}] Invalid nextCheckpointIndex: {nextCheckpointIndex} (max: {checkpoints.Count - 1})");
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

    // Override the heuristic for manual control testing
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        var discreteActionsOut = actionsOut.DiscreteActions;

        continuousActionsOut[0] = Input.GetAxis("Vertical");    // Accelerate with W/Up
        continuousActionsOut[1] = Input.GetAxis("Horizontal");  // Steer with A/D or Left/Right
        continuousActionsOut[2] = Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow) ? 1f : 0f;  // Brake with S/Down

        discreteActionsOut[0] = Input.GetKey(KeyCode.Space) ? 1 : 0;  // Handbrake with Space
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

    protected override void OnDisable()
    {
        base.OnDisable();

        // Unsubscribe from events
        if (trainingManager != null)
        {
            trainingManager.OnTrackRebuilt -= OnTrackRebuilt;
            trainingManager.OnEpisodeReset -= OnEpisodeReset;
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
            trainingManager.OnEpisodeReset -= OnEpisodeReset;
        }
        if (raceManager != null)
        {
            raceManager.OnCheckpointPassed -= OnRaceManagerCheckpointPassed;
        }
    }
}