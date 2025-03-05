using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;
using Assets.TrackGeneration;
using System;
using System.Collections;

public class CarAgent : Agent
{
    [Header("References")]
    public CarControlScript carController;
    public TrackSensingSystem sensorSystem;
    public TrackHandler trackHandler;
    public RaceManager raceManager; // Add reference to RaceManager

    [Header("Checkpoint System")]
    private List<Checkpoint> checkpoints;
    private int nextCheckpointIndex = 0;
    private float distanceToNextCheckpoint;
    private Vector3 directionToNextCheckpoint;
    private float lastCheckpointTime;
    private int totalCheckpointsPassed = 0;
    private HashSet<int> visitedCheckpoints = new HashSet<int>(); // Track visited checkpoints

    [Header("Training Parameters")]
    public float maxTimeWithoutCheckpoint = 30f; // Seconds before a reset if no checkpoint is passed
    public float maxEpisodeTime = 700f; // Maximum episode length in seconds
    public float checkpointReward = 1.0f; // Reward for passing a checkpoint
    public float lapCompletionReward = 5.0f; // Reward for completing a lap
    public float collisionPenalty = -0.5f; // Penalty for collisions
    public float speedReward = 0.01f; // Small reward for maintaining speed
    public float backwardsPenalty = -0.2f; // Penalty for going backwards
    public float noProgressPenalty = -0.05f; // Penalty for not making progress
    public float noProgressThreshold = 0.5f; // Threshold for detecting no progress
    public float reversePenalty = -0.3f; // Penalty for using reverse
    [HideInInspector] // Hide this field as it's now controlled by TrainingManager
    public int completedCheckpointsBeforeNewTrack = int.MaxValue; // Set to max value to disable independent track generation

    // Add to force reset on collision
    public bool resetOnCollision = true; // Flag to enable/disable reset on collision
    public bool resetOnPreviousCheckpoint = true; // Flag to reset when returning to a previous checkpoint

    [Header("Debug")]
    public bool visualizeTargetDirection = true;

    private bool hasCollided = false;
    private float episodeStartTime;
    private float prevDistanceToCheckpoint;
    private Vector3 prevPosition;
    private TrainingManager trainingManager;
    private float noProgressTimer = 0f; // Timer to track no progress
    private float noProgressThresholdTime = 5f; // Time threshold for no progress penalty

    // Movement tracking
    private Vector3 lastPositionCheck;
    private float positionCheckInterval = 1.0f;
    private float lastPositionCheckTime = 0f;
    private bool isMovingBackwards = false;

    // Episode synchronization
    private bool resetRequested = false;
    private bool isReadyForReset = false;
    private float resetRequestTime = 0f;
    private float maxTimeToFinishAfterResetRequest = 10f; // Force finish after 10 seconds
    private bool isForcedReset = false;
    private float safetyResetHeight = -10f; // Height at which to force reset if car falls
    private bool isTrackRebuildPending = false;

    public override void Initialize()
    {
        // First, find TrainingManager singleton
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

            Debug.Log($"Car {gameObject.name} successfully initialized with TrainingManager references");
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
                Debug.Log($"Car {gameObject.name} found {checkpoints.Count} checkpoints");
            }
            else
            {
                Debug.LogWarning($"Car {gameObject.name} couldn't find checkpoints");
            }
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
        // Safety check - if car falls below a certain height, force a reset
        if (transform.position.y < safetyResetHeight)
        {
            Debug.LogWarning($"Car {gameObject.name} fell below safety height ({safetyResetHeight}). Forcing reset.");
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
                Debug.Log($"Car {gameObject.name} forcing finish due to reset request timeout");
                isForcedReset = true;
                NotifyReadyForReset();
            }
        }

        // Check for backwards movement at intervals
        if (Time.time - lastPositionCheckTime > positionCheckInterval)
        {
            CheckMovementDirection();
            lastPositionCheckTime = Time.time;
        }
    }

    // New method to check if the car is moving backwards relative to track direction
    private void CheckMovementDirection()
    {
        if (checkpoints == null || checkpoints.Count == 0) return;

        Vector3 currentPosition = transform.position;
        if (lastPositionCheck != Vector3.zero)
        {
            Vector3 movementVector = currentPosition - lastPositionCheck;

            // Check if movement is opposite to the direction to checkpoint
            float movementAlignment = Vector3.Dot(movementVector.normalized, directionToNextCheckpoint.normalized);
            isMovingBackwards = movementAlignment < -0.3f; // If moving significantly away from checkpoint

            if (isMovingBackwards)
            {
                // Apply penalty for moving backwards
                AddReward(backwardsPenalty);
                Debug.Log($"Car {gameObject.name} is moving backwards - applying penalty");
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
        maxTimeWithoutCheckpoint = Academy.Instance.EnvironmentParameters.GetWithDefault("max_time_without_checkpoint", maxTimeWithoutCheckpoint);
        speedReward = Academy.Instance.EnvironmentParameters.GetWithDefault("speed_reward", speedReward);
        checkpointReward = Academy.Instance.EnvironmentParameters.GetWithDefault("checkpoint_reward", checkpointReward);
        backwardsPenalty = Academy.Instance.EnvironmentParameters.GetWithDefault("backwards_penalty", backwardsPenalty);
        noProgressPenalty = Academy.Instance.EnvironmentParameters.GetWithDefault("no_progress_penalty", noProgressPenalty);

        // Reset flags
        resetRequested = false;
        isReadyForReset = false;
        isForcedReset = false;

        // Reset the car position and state
        ResetCar();

        // Initialize checkpoint tracking
        if (trackHandler != null)
        {
            checkpoints = trackHandler.GetCheckpoints();
            nextCheckpointIndex = 0;
            distanceToNextCheckpoint = 0f;
            visitedCheckpoints.Clear(); // Clear visited checkpoints
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
                    Debug.Log("Recovered TrackHandler reference");
                }
            }
        }

        // Reset timer and collision flag
        episodeStartTime = Time.time;
        lastCheckpointTime = Time.time;
        hasCollided = false;
        totalCheckpointsPassed = 0;
        noProgressTimer = 0f;
        lastPositionCheck = transform.position;
        lastPositionCheckTime = Time.time;
        isMovingBackwards = false;

        // Initial measurement to next checkpoint
        UpdateCheckpointInfo();
        prevDistanceToCheckpoint = distanceToNextCheckpoint;
        prevPosition = transform.position;
    }

    // Called from TrainingManager when a reset is requested (either normal or rebuild)
    public void RequestFinishForReset(bool forRebuild = false)
    {
        if (!resetRequested)
        {
            Debug.Log($"Car {gameObject.name} received {(forRebuild ? "rebuild" : "reset")} request");
            resetRequested = true;
            resetRequestTime = Time.time;
            isTrackRebuildPending = forRebuild;

            // If we're already in a collision or problematic state, finish immediately
            if (hasCollided || transform.position.y < 0)
            {
                NotifyReadyForReset();
            }
        }
    }

    // Notify the training manager this agent is ready for reset
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

    // New method to handle track rebuilds from TrainingManager
    private void OnTrackRebuilt()
    {
        // Re-initialize checkpoints when track is rebuilt
        if (trackHandler != null)
        {
            checkpoints = trackHandler.GetCheckpoints();
            nextCheckpointIndex = 0;
            totalCheckpointsPassed = 0;
            visitedCheckpoints.Clear(); // Clear visited checkpoints
            isTrackRebuildPending = false;
            Debug.Log($"Car {gameObject.name} received track rebuild notification and updated checkpoints");
        }
    }

    // Public method that can be called from TrainingManager
    public void ResetForNewTrack()
    {
        // Reset flags
        resetRequested = false;
        isReadyForReset = false;
        isForcedReset = false;
        isTrackRebuildPending = false;

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
        visitedCheckpoints.Clear(); // Clear visited checkpoints

        // Update checkpoints list
        if (trackHandler != null)
        {
            checkpoints = trackHandler.GetCheckpoints();
            Debug.Log($"Car {gameObject.name} reset for new track with {checkpoints.Count} checkpoints");
        }
    }

    // Replace the ResetCar method in CarAgent.cs with this safer version
    private void ResetCar()
    {
        if (carController != null)
        {
            try
            {
                carController.ResetPosition();
            }
            catch (NullReferenceException e)
            {
                Debug.LogWarning($"Error resetting car position: {e.Message}. Using manual reset instead.");

                // Fallback manual reset
                if (trackHandler != null)
                {
                    // Get start position from track
                    (Vector3 startPosition, Vector3 startPosition2, Quaternion startRotation) = trackHandler.GetTrackStartTransform();

                    // Manually reset the car
                    transform.position = startPosition;
                    transform.rotation = startRotation;

                    // Reset physics if rigidbody exists
                    Rigidbody rb = GetComponent<Rigidbody>();
                    if (rb != null)
                    {
                        rb.linearVelocity = Vector3.zero;
                        rb.angularVelocity = Vector3.zero;
                    }
                }
                else
                {
                    Debug.LogError($"Cannot reset car position: both carController and trackHandler are not usable");
                }
            }
        }
        else
        {
            Debug.LogWarning("CarController is null in ResetCar, looking for it now...");
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
        foreach (float rayDistance in raySensorData)
        {
            sensor.AddObservation(rayDistance);
        }

        // Add normalized direction to next checkpoint (2 values representing a 2D direction vector)
        float forwardDot = Vector3.Dot(transform.forward, directionToNextCheckpoint.normalized);
        float rightDot = Vector3.Dot(transform.right, directionToNextCheckpoint.normalized);

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
            // Extract continuous actions - make sure we're accessing valid indices
            float accelerate = 0f;
            float steer = 0f;
            float brake = 0f;

            if (actionBuffers.ContinuousActions.Length >= 3)
            {
                accelerate = actionBuffers.ContinuousActions[0];
                steer = actionBuffers.ContinuousActions[1];
                brake = actionBuffers.ContinuousActions[2];

                // Debug visualization
                Debug.DrawRay(transform.position, transform.forward * accelerate * 5f, Color.green);
                Debug.DrawRay(transform.position, transform.right * steer * 5f, Color.red);

                // Apply penalty for using reverse
                if (brake > 0.5f)
                {
                    AddReward(reversePenalty * brake);
                }
            }
            else
            {
                Debug.LogWarning($"Expected 3 continuous actions, but got {actionBuffers.ContinuousActions.Length}");
            }

            // Extract discrete action (handbrake)
            bool useHandbrake = false;
            if (actionBuffers.DiscreteActions.Length > 0)
            {
                useHandbrake = actionBuffers.DiscreteActions[0] == 1;
            }
            else
            {
                Debug.LogWarning("No discrete actions received for handbrake");
            }

            // Apply inputs to car controller
            SetCarInputs(accelerate, steer, brake, useHandbrake);

            // Rest of your reward calculation code...
            float timeSinceLastCheckpoint = Time.time - lastCheckpointTime;
            float episodeTime = Time.time - episodeStartTime;

            // Check for timeouts unless we're in reset mode
            if (!resetRequested &&
                (timeSinceLastCheckpoint > maxTimeWithoutCheckpoint ||
                episodeTime > maxEpisodeTime))
            {
                // Penalty for timeout
                AddReward(-1.0f);

                // Request a synchronized episode reset
                if (trainingManager != null && trainingManager.synchronizeEpisodes)
                {
                    trainingManager.RequestEpisodeReset();
                }
                else
                {
                    // If not using synchronized episodes, just end this one
                    EndEpisode();
                }
                return;
            }

            UpdateCheckpointInfo();
            CalculateRewards();

            // Check for lack of progress
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

    // New method to detect and penalize lack of progress
    private void CheckForNoProgress()
    {
        float distanceDelta = Mathf.Abs(prevDistanceToCheckpoint - distanceToNextCheckpoint);

        // If we're not making significant progress
        if (distanceDelta < noProgressThreshold)
        {
            noProgressTimer += Time.deltaTime;

            // Apply penalty if we've been not making progress for too long
            if (noProgressTimer > noProgressThresholdTime)
            {
                AddReward(noProgressPenalty);
                Debug.Log($"Car {gameObject.name} not making progress - applying penalty");

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

    private void CalculateRewards()
    {
        // Calculate reward for progress toward checkpoint
        float distanceDelta = prevDistanceToCheckpoint - distanceToNextCheckpoint;

        // Reward forward progress toward the next checkpoint
        if (distanceDelta > 0)
        {
            // Reward is proportional to progress made
            AddReward(distanceDelta * 0.01f);
        }
        else
        {
            // Increased penalty for moving away from the checkpoint
            AddReward(distanceDelta * 0.01f); // Doubled from original
        }

        // Reward based on speed (encourages the car to go faster)
        float speed = carController.GetCurrentSpeed();
        AddReward(speed * speedReward);

        // Penalty for collisions
        if (hasCollided)
        {
            AddReward(collisionPenalty);
            hasCollided = false;

            // If we're in reset mode, notify
            if (resetRequested)
            {
                NotifyReadyForReset();
                return;
            }

            // If using collision reset and not in reset mode, request synchronized reset
            if (resetOnCollision && !resetRequested)
            {
                // Apply more severe penalty for crashing
                AddReward(-1.0f);

                // Request a synchronized episode reset
                if (trainingManager != null && trainingManager.synchronizeEpisodes)
                {
                    trainingManager.RequestEpisodeReset();
                }
                else
                {
                    // If not using synchronized episodes, just end this one
                    EndEpisode();
                }
            }
        }
    }

    public void CheckpointPassed(int checkpointIndex)
    {
        if (checkpointIndex == nextCheckpointIndex)
        {
            // Check if we've already visited this checkpoint and it's not the start (index 0)
            if (visitedCheckpoints.Contains(checkpointIndex) && checkpointIndex != 0)
            {
                // We've already visited this checkpoint before - potential wrong direction
                Debug.Log($"Car {gameObject.name} returned to previously visited checkpoint {checkpointIndex}");

                // Apply a penalty
                AddReward(-1.0f);

                // Reset if configured to do so
                if (resetOnPreviousCheckpoint)
                {
                    if (trainingManager != null && trainingManager.synchronizeEpisodes)
                    {
                        trainingManager.RequestEpisodeReset();
                    }
                    else
                    {
                        EndEpisode();
                    }
                    return;
                }
            }

            // Mark this checkpoint as visited
            visitedCheckpoints.Add(checkpointIndex);

            // Reward for hitting the correct checkpoint
            AddReward(checkpointReward);

            // Update tracking
            nextCheckpointIndex = (nextCheckpointIndex + 1) % checkpoints.Count;
            lastCheckpointTime = Time.time;
            totalCheckpointsPassed++;

            // Additional reward for completing a lap
            if (nextCheckpointIndex == 0)
            {
                AddReward(lapCompletionReward);

                // Clear visited checkpoints at the start of a new lap
                visitedCheckpoints.Clear();
                visitedCheckpoints.Add(0); // Add start checkpoint as visited

                // Increment training manager episode counter if we're not using synchronized episodes
                if (trainingManager != null && !trainingManager.synchronizeEpisodes)
                {
                    trainingManager.IncrementEpisodeCounter();
                }
            }

            // Update distance to next checkpoint
            UpdateCheckpointInfo();
        }
        else
        {
            // Small penalty for hitting checkpoints out of order
            AddReward(-0.1f);
        }
    }

    private void UpdateCheckpointInfo()
    {
        if (checkpoints == null || checkpoints.Count == 0) return;

        if (nextCheckpointIndex < checkpoints.Count)
        {
            Vector3 nextCheckpointPosition = checkpoints[nextCheckpointIndex].transform.position;
            distanceToNextCheckpoint = Vector3.Distance(transform.position, nextCheckpointPosition);
            directionToNextCheckpoint = nextCheckpointPosition - transform.position;

            // Debug visualization
            if (visualizeTargetDirection)
            {
                Debug.DrawLine(transform.position, nextCheckpointPosition, Color.green);
            }
        }
    }

    // Modified collision handler
    public void ReportCollision()
    {
        hasCollided = true;

        // Apply the collision penalty
        AddReward(collisionPenalty);

        // If we're in reset mode, this is a good time to notify
        if (resetRequested)
        {
            NotifyReadyForReset();
            return;
        }

        // If using collision reset and not in reset mode, request synchronized reset
        if (resetOnCollision && !resetRequested)
        {
            // Apply more severe penalty for crashing
            AddReward(-1.0f);

            // Request a synchronized episode reset
            if (trainingManager != null && trainingManager.synchronizeEpisodes)
            {
                trainingManager.RequestEpisodeReset();
            }
            else
            {
                // If not using synchronized episodes, just end this one
                EndEpisode();
            }
        }
    }

    // Interface with the car controller to set inputs
    private void SetCarInputs(float accelerate, float steer, float brake, bool handbrake)
    {
        carController.SetAccelerationInput(accelerate);
        carController.SetSteeringInput(steer);
        carController.SetReverseInput(brake);
        carController.SetHandbrakeInput(handbrake);
    }

    // Override the heuristic (manual control) for testing
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        var discreteActionsOut = actionsOut.DiscreteActions;

        continuousActionsOut[0] = 0f;
        continuousActionsOut[1] = 0;
        continuousActionsOut[2] = 0;

        discreteActionsOut[0] = 0;
    }

    protected override void OnDisable()
    {
        base.OnDisable(); // Call the base implementation first

        // Unsubscribe from events when disabled
        if (trainingManager != null)
        {
            trainingManager.OnTrackRebuilt -= OnTrackRebuilt;
            trainingManager.OnEpisodeReset -= OnEpisodeReset;
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
    }
}