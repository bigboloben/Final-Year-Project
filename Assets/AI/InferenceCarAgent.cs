using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Assets.TrackGeneration;

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

    [Header("Debug")]
    public bool visualizeRays = false;
    public bool visualizeTargetDirection = false;

    // Store positions of checkpoints for decision making
    private System.Collections.Generic.List<Checkpoint> checkpoints;
    private int nextCheckpointIndex = 0;
    private float distanceToNextCheckpoint;
    private Vector3 directionToNextCheckpoint;


    void Start()
    {
        InitializeAgent();
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

        // Make sure the car is properly set up for AI control
        if (carController != null)
        {
            // Make sure the car knows this is AI controlled
            carController.isPlayerControlled = false;
        }
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
        UpdateCheckpointInfo();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Add ray sensor data (distances to track boundaries)
        float[] raySensorData = sensorSystem.GetAllSensorData();
        foreach (float rayDistance in raySensorData)
        {
            sensor.AddObservation(rayDistance);
        }

        // Add normalized direction to next checkpoint
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
        try
        {
            // Extract continuous actions
            float accelerate = actionBuffers.ContinuousActions[0];
            float steer = actionBuffers.ContinuousActions[1];
            float brake = actionBuffers.ContinuousActions[2];

            // Debug visualization
            if (visualizeRays)
            {
                Debug.DrawRay(transform.position, transform.forward * accelerate * 5f, Color.green);
                Debug.DrawRay(transform.position, transform.right * steer * 5f, Color.red);
            }

            // Extract discrete action (handbrake)
            bool useHandbrake = actionBuffers.DiscreteActions[0] == 1;

            // Apply inputs to car controller
            carController.SetAccelerationInput(accelerate);
            carController.SetSteeringInput(steer);
            carController.SetReverseInput(brake);
            carController.SetHandbrakeInput(useHandbrake);

            // Update checkpoint information
            UpdateCheckpointInfo();
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"Error in OnActionReceived: {ex.Message}");
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

    // Process checkpoint passes
    public void CheckpointPassed(int checkpointIndex)
    {
        if (checkpointIndex == nextCheckpointIndex)
        {
            // Update tracking
            nextCheckpointIndex = (nextCheckpointIndex + 1) % checkpoints.Count;

            // Update distance to next checkpoint
            UpdateCheckpointInfo();

            Debug.Log($"AI car passed checkpoint {checkpointIndex}, next is {nextCheckpointIndex}");
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
        UpdateCheckpointInfo();
    }
}