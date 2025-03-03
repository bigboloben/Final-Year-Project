using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;
using Assets.TrackGeneration;
using System;

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

    [Header("Training Parameters")]
    public float maxTimeWithoutCheckpoint = 30f; // Seconds before a reset if no checkpoint is passed
    public float maxEpisodeTime = 300f; // Maximum episode length in seconds
    public float checkpointReward = 1.0f; // Reward for passing a checkpoint
    public float lapCompletionReward = 5.0f; // Reward for completing a lap
    public float collisionPenalty = -0.5f; // Penalty for collisions
    public float speedReward = 0.001f; // Small reward for maintaining speed
    [HideInInspector] // Hide this field as it's now controlled by TrainingManager
    public int completedCheckpointsBeforeNewTrack = int.MaxValue; // Set to max value to disable independent track generation

    [Header("Debug")]
    public bool visualizeTargetDirection = true;

    private bool hasCollided = false;
    private float episodeStartTime;
    private float prevDistanceToCheckpoint;
    private Vector3 prevPosition;
    private TrainingManager trainingManager;

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

    public override void OnEpisodeBegin()
    {
        // Reset the car position and state
        ResetCar();

        // Initialize checkpoint tracking
        if (trackHandler != null)
        {
            checkpoints = trackHandler.GetCheckpoints();
            nextCheckpointIndex = 0;
            distanceToNextCheckpoint = 0f;
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

        // Initial measurement to next checkpoint
        UpdateCheckpointInfo();
        prevDistanceToCheckpoint = distanceToNextCheckpoint;
        prevPosition = transform.position;
    }

    // Custom method to end episode and notify training manager
    public void EndEpisodeAndNotify()
    {
        // Notify TrainingManager about episode completion
        if (trainingManager != null)
        {
            trainingManager.IncrementEpisodeCounter();
        }

        // Call the standard EndEpisode method
        EndEpisode();
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
            Debug.Log($"Car {gameObject.name} received track rebuild notification and updated checkpoints");
        }
    }

    // Public method that can be called from TrainingManager
    public void ResetForNewTrack()
    {
        // Reset checkpoint-related data
        nextCheckpointIndex = 0;
        totalCheckpointsPassed = 0;
        lastCheckpointTime = Time.time;

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
        // Extract continuous actions
        float accelerate = actionBuffers.ContinuousActions[0];
        float steer = actionBuffers.ContinuousActions[1];
        float brake = actionBuffers.ContinuousActions[2];

        // Add visual indicator for action values
        Debug.DrawRay(transform.position, transform.forward * accelerate * 5f, Color.green);
        Debug.DrawRay(transform.position, transform.right * steer * 5f, Color.red);

        // Extract discrete actions
        int handbrakeAction = actionBuffers.DiscreteActions[0]; // 0 or 1

        // Apply these inputs to the car controller
        SetCarInputs(accelerate, steer, brake, handbrakeAction == 1);

        // Check if we need to end the episode
        float timeSinceLastCheckpoint = Time.time - lastCheckpointTime;
        float episodeTime = Time.time - episodeStartTime;

        if (timeSinceLastCheckpoint > maxTimeWithoutCheckpoint ||
            episodeTime > maxEpisodeTime)
        {
            // Penalty for timeout
            AddReward(-1.0f);
            EndEpisodeAndNotify();
            return;
        }

        // Give rewards based on progress and speed
        UpdateCheckpointInfo();
        CalculateRewards();

        // Update previous values for next frame
        prevDistanceToCheckpoint = distanceToNextCheckpoint;
        prevPosition = transform.position;
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
            // Small penalty for moving away from the checkpoint
            AddReward(distanceDelta * 0.005f);
        }

        // Reward based on speed (encourages the car to go faster)
        float speed = carController.GetCurrentSpeed();
        AddReward(speed * speedReward);

        // Penalty for collisions
        if (hasCollided)
        {
            AddReward(collisionPenalty);
            hasCollided = false;
        }
    }

    public void CheckpointPassed(int checkpointIndex)
    {
        if (checkpointIndex == nextCheckpointIndex)
        {
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

    // This will be called by collision detection in your car controller
    public void ReportCollision()
    {
        hasCollided = true;

        // Apply the collision penalty
        AddReward(collisionPenalty);

        // Optional: End the episode immediately on collision
        // EndEpisodeAndNotify();
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

        // Get keyboard input for testing
        float vertical = Input.GetAxis("Vertical");
        float horizontal = Input.GetAxis("Horizontal");

        // Map keyboard inputs to agent actions
        continuousActionsOut[0] = vertical > 0 ? vertical : 0f;     // Accelerate
        continuousActionsOut[1] = horizontal;                       // Steer
        continuousActionsOut[2] = vertical < 0 ? -vertical : 0f;    // Brake/Reverse

        // Handbrake control (example: space bar)
        discreteActionsOut[0] = Input.GetKey(KeyCode.Space) ? 1 : 0;
    }

    protected override void OnDisable()
    {
        base.OnDisable(); // Call the base implementation first

        // Unsubscribe from events when disabled
        if (trainingManager != null)
        {
            trainingManager.OnTrackRebuilt -= OnTrackRebuilt;
        }
    }

    void OnDestroy()
    {
        // Unsubscribe from events
        if (trainingManager != null)
        {
            trainingManager.OnTrackRebuilt -= OnTrackRebuilt;
        }
    }
}