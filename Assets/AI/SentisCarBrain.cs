//using UnityEngine;
//using Unity.Sentis;
//using System.Collections.Generic;
//using Assets.TrackGeneration;

//public class SentisCarBrain : MonoBehaviour
//{
//    [Header("Model Configuration")]
//    public ModelAsset onnxModel; // Your trained ML-Agents ONNX model
//    private Worker worker;
//    private Model runtimeModel;

//    [Header("References")]
//    public TrackSensingSystem sensorSystem;
//    public CarControlScript carController;
//    public TrackHandler trackHandler;

//    [Header("Checkpoint Tracking")]
//    private List<Checkpoint> checkpoints;
//    private int nextCheckpointIndex = 0;
//    private float distanceToNextCheckpoint;
//    private Vector3 directionToNextCheckpoint;

//    [Header("AI Performance Settings")]
//    [Range(0.5f, 1.5f)] public float skillMultiplier = 1.0f; // Adjust AI performance
//    [Range(0.0f, 0.5f)] public float reactionDelay = 0.0f; // Add delay for more human-like behavior

//    [Header("Debug")]
//    public bool visualizeTargetDirection = true;
//    public bool visualizeSensors = false;
//    public bool debugTensorShapes = true;

//    // Delayed action application
//    private float timeSinceLastDecision = 0f;
//    private float[] lastContinuousActions = new float[3]; // steering, acceleration, brake
//    private int[] lastDiscreteActions = new int[1]; // handbrake (0 or 1)
//    private bool isModelInitialized = false;

//    // Input tensor
//    private Tensor<float> inputTensor;

//    // Observation size
//    private int observationSize = 14; // 9 rays + 5 other values

//    void Start()
//    {
//        InitializeModel();
//        InitializeCheckpoints();
//    }

//    void Update()
//    {
//        // Skip if model isn't ready
//        if (!isModelInitialized) return;

//        // Update timer for reaction delay
//        timeSinceLastDecision += Time.deltaTime;

//        // Apply delayed actions if necessary
//        if (reactionDelay > 0 && timeSinceLastDecision >= reactionDelay)
//        {
//            ExecuteActions(lastContinuousActions, lastDiscreteActions);
//            timeSinceLastDecision = 0f;
//        }

//        // Visual debug for target checkpoint
//        if (visualizeTargetDirection && checkpoints != null && checkpoints.Count > 0 && nextCheckpointIndex < checkpoints.Count)
//        {
//            Debug.DrawLine(transform.position, checkpoints[nextCheckpointIndex].transform.position, Color.blue);
//        }
//    }

//    void FixedUpdate()
//    {
//        // Skip if model isn't ready
//        if (!isModelInitialized) return;

//        // Execute immediately if no delay is set
//        if (reactionDelay <= 0)
//        {
//            MakeDecision();
//        }
//        else if (timeSinceLastDecision >= reactionDelay)
//        {
//            // Make a new decision but don't apply it yet (it will be applied with delay)
//            MakeDecision();
//        }

//        // Update checkpoint information
//        UpdateCheckpointInfo();
//    }

//    void MakeDecision()
//    {
//        try
//        {
//            // 1. Collect observations just like in CarAgent
//            var observations = CollectObservations();

//            // 2. Update input tensor with new observations
//            if (inputTensor == null)
//            {
//                // Create a new tensor with the right shape
//                inputTensor = new Tensor<float>(new TensorShape(1, observationSize));

//                // Fill with initial values
//                for (int i = 0; i < observations.Length; i++)
//                {
//                    inputTensor[0, i] = observations[i];
//                }
//            }
//            else
//            {
//                // Update existing tensor with new observations
//                for (int i = 0; i < observations.Length; i++)
//                {
//                    inputTensor[0, i] = observations[i];
//                }
//            }

//            // 3. Execute the model
//            var inputs = new Dictionary<string, Tensor>
//            {
//                { "obs_0", inputTensor } // Standard ML-Agents observation input name
//            };

//            var outputs = worker.Schedule(inputs);

//            if (debugTensorShapes)
//            {
//                Debug.Log($"Model execution completed with {outputs.Count} output tensors");
//                foreach (var output in outputs)
//                {
//                    Debug.Log($"Output tensor: {output.Key}, Type: {output.Value.GetType().Name}, Shape: {output.Value.shape}");
//                }
//            }

//            // 4. Get results (continuous and discrete actions)
//            var continuousActions = GetContinuousActions(outputs);
//            var discreteActions = GetDiscreteActions(outputs);

//            // 5. Apply skill multiplier to continuous actions
//            continuousActions[0] *= skillMultiplier; // Accelerate
//            continuousActions[2] *= skillMultiplier; // Brake
//                                                     // Don't modify steering (index 1) with skill multiplier

//            // Store actions for delayed execution if needed
//            lastContinuousActions = continuousActions;
//            lastDiscreteActions = discreteActions;

//            // 6. Execute immediately if no delay
//            if (reactionDelay <= 0)
//            {
//                ExecuteActions(continuousActions, discreteActions);
//            }

//            timeSinceLastDecision = 0f;
//        }
//        catch (System.Exception e)
//        {
//            Debug.LogError($"Error during AI decision making: {e.Message}");
//        }
//    }

//    private void ExecuteActions(float[] continuousActions, int[] discreteActions)
//    {
//        if (carController == null) return;

//        // Extract actions
//        float accelerate = continuousActions[0];
//        float steer = continuousActions[1];
//        float brake = continuousActions[2];
//        bool useHandbrake = discreteActions[0] == 1;

//        // Apply inputs to car controller
//        carController.SetAccelerationInput(accelerate);
//        carController.SetSteeringInput(steer);
//        carController.SetReverseInput(brake);
//        carController.SetHandbrakeInput(useHandbrake);

//        // Debug visualization
//        if (visualizeSensors)
//        {
//            Debug.DrawRay(transform.position, transform.forward * accelerate * 5f, Color.green, 0.1f);
//            Debug.DrawRay(transform.position, transform.right * steer * 5f, Color.red, 0.1f);
//        }
//    }

//    private float[] CollectObservations()
//    {
//        // This should match exactly the same structure used in CarAgent.CollectObservations
//        // Getting this wrong will cause unpredictable AI behavior

//        List<float> observations = new List<float>();

//        // 1. Add ray sensor data (distances to walls)
//        float[] raySensorData = sensorSystem.GetAllSensorData();
//        foreach (float rayDistance in raySensorData)
//        {
//            observations.Add(rayDistance);
//        }

//        // 2. Add normalized direction to next checkpoint
//        float forwardDot = Vector3.Dot(transform.forward, directionToNextCheckpoint.normalized);
//        float rightDot = Vector3.Dot(transform.right, directionToNextCheckpoint.normalized);

//        observations.Add(forwardDot); // How aligned we are with the next checkpoint
//        observations.Add(rightDot);   // If the checkpoint is to our right or left

//        // 3. Add normalized distance to next checkpoint
//        observations.Add(Mathf.Clamp01(distanceToNextCheckpoint / 100f));

//        // 4. Add car speed information
//        float speed = carController.GetCurrentSpeed();
//        observations.Add(speed / carController.maxVelocity); // Normalized speed

//        // 5. Add minimal car-specific information
//        observations.Add(carController.isDrifting ? 1f : 0f); // Is the car drifting

//        // Ensure observation size matches expected size
//        if (observations.Count != observationSize)
//        {
//            Debug.LogWarning($"Expected {observationSize} observations but got {observations.Count}");
//        }

//        return observations.ToArray();
//    }

//    private float[] GetContinuousActions(Dictionary<string, Tensor> outputs)
//    {
//        float[] actions = new float[3]; // steering, acceleration, brake

//        try
//        {
//            // Try different possible output tensor names for continuous actions
//            string[] possibleNames = { "continuous_actions", "action", "continuous_action", "continuous" };
//            Tensor<float> continuousActionsTensor = null;

//            // First try standard names
//            foreach (var name in possibleNames)
//            {
//                if (outputs.TryGetValue(name, out Tensor tensor) && tensor is Tensor<float>)
//                {
//                    continuousActionsTensor = tensor as Tensor<float>;
//                    Debug.Log($"Found continuous actions with name: {name}");
//                    break;
//                }
//            }

//            // If not found, try the first float tensor
//            if (continuousActionsTensor == null)
//            {
//                foreach (var output in outputs)
//                {
//                    if (output.Value is Tensor<float>)
//                    {
//                        continuousActionsTensor = output.Value as Tensor<float>;
//                        Debug.Log($"Using tensor {output.Key} for continuous actions");
//                        break;
//                    }
//                }
//            }

//            if (continuousActionsTensor != null)
//            {
//                // Debug tensor shape
//                if (debugTensorShapes)
//                {
//                    Debug.Log($"Continuous actions tensor shape: {continuousActionsTensor.shape}");
//                }

//                // Extract values based on tensor shape
//                if (continuousActionsTensor.shape.rank >= 2 && continuousActionsTensor.shape[1] >= 3)
//                {
//                    // Shape [1,3] - batch of actions
//                    for (int i = 0; i < 3 && i < continuousActionsTensor.shape[1]; i++)
//                    {
//                        actions[i] = continuousActionsTensor[0, i];
//                    }
//                }
//                else if (continuousActionsTensor.shape.rank >= 1 && continuousActionsTensor.shape[0] >= 3)
//                {
//                    // Shape [3] - direct list of actions
//                    for (int i = 0; i < 3 && i < continuousActionsTensor.shape[0]; i++)
//                    {
//                        actions[i] = continuousActionsTensor[i];
//                    }
//                }
//                else
//                {
//                    Debug.LogWarning($"Unexpected tensor shape: {continuousActionsTensor.shape}");
//                }
//            }
//            else
//            {
//                Debug.LogWarning("No continuous actions tensor found, using zero values");
//            }
//        }
//        catch (System.Exception e)
//        {
//            Debug.LogError($"Error getting continuous actions: {e.Message}");
//        }

//        return actions;
//    }

//    private int[] GetDiscreteActions(Dictionary<string, Tensor> outputs)
//    {
//        int[] actions = new int[1]; // handbrake

//        try
//        {
//            // Try different possible output tensor names for discrete actions
//            string[] possibleNames = { "discrete_actions", "discrete_action", "discrete" };
//            Tensor<int> discreteActionsTensor = null;

//            // First try standard names
//            foreach (var name in possibleNames)
//            {
//                if (outputs.TryGetValue(name, out Tensor tensor) && tensor is Tensor<int>)
//                {
//                    discreteActionsTensor = tensor as Tensor<int>;
//                    Debug.Log($"Found discrete actions with name: {name}");
//                    break;
//                }
//            }

//            // If not found, try the first int tensor
//            if (discreteActionsTensor == null)
//            {
//                foreach (var output in outputs)
//                {
//                    if (output.Value is Tensor<int>)
//                    {
//                        discreteActionsTensor = output.Value as Tensor<int>;
//                        Debug.Log($"Using tensor {output.Key} for discrete actions");
//                        break;
//                    }
//                }
//            }

//            if (discreteActionsTensor != null)
//            {
//                // Debug tensor shape
//                if (debugTensorShapes)
//                {
//                    Debug.Log($"Discrete actions tensor shape: {discreteActionsTensor.shape}");
//                }

//                // Extract values based on tensor shape
//                if (discreteActionsTensor.shape.rank >= 2)
//                {
//                    actions[0] = discreteActionsTensor[0, 0];
//                }
//                else if (discreteActionsTensor.shape.rank >= 1)
//                {
//                    actions[0] = discreteActionsTensor[0];
//                }
//            }
//            else
//            {
//                // If no discrete tensor found, check if continuous tensor has an extra value for handbrake
//                foreach (var output in outputs)
//                {
//                    if (output.Value is Tensor<float> floatTensor)
//                    {
//                        if ((floatTensor.shape.rank >= 2 && floatTensor.shape[1] >= 4) ||
//                            (floatTensor.shape.rank == 1 && floatTensor.shape[0] >= 4))
//                        {
//                            float handbrakeValue = floatTensor.shape.rank >= 2 ?
//                                floatTensor[0, 3] : floatTensor[3];

//                            actions[0] = handbrakeValue > 0.5f ? 1 : 0;
//                            Debug.Log("Using continuous tensor for discrete action (handbrake)");
//                            break;
//                        }
//                    }
//                }
//            }
//        }
//        catch (System.Exception e)
//        {
//            Debug.LogError($"Error getting discrete actions: {e.Message}");
//        }

//        return actions;
//    }

//    private void InitializeModel()
//    {
//        try
//        {
//            if (onnxModel == null)
//            {
//                Debug.LogError("No ONNX model asset assigned!");
//                return;
//            }

//            // Create the runtime model from the asset
//            runtimeModel = ModelLoader.Load(onnxModel);

//            // Log model info
//            Debug.Log($"Model inputs: {string.Join(", ", runtimeModel.inputs)}");
//            Debug.Log($"Model outputs: {string.Join(", ", runtimeModel.outputs)}");

//            // Create an engine for inference
//            worker = Engine.Create();

//            // Set flag that model is ready
//            isModelInitialized = true;

//            Debug.Log($"Successfully initialized Sentis model: {onnxModel.name}");
//        }
//        catch (System.Exception e)
//        {
//            Debug.LogError($"Error initializing Sentis model: {e.Message}");
//        }
//    }

//    private void InitializeCheckpoints()
//    {
//        if (trackHandler == null)
//        {
//            trackHandler = FindObjectOfType<TrackHandler>();
//        }

//        if (trackHandler != null)
//        {
//            checkpoints = trackHandler.GetCheckpoints();
//            nextCheckpointIndex = 0;

//            // Initial checkpoint info
//            UpdateCheckpointInfo();
//        }
//    }

//    private void UpdateCheckpointInfo()
//    {
//        if (checkpoints == null || checkpoints.Count == 0) return;

//        if (nextCheckpointIndex < checkpoints.Count)
//        {
//            Vector3 nextCheckpointPosition = checkpoints[nextCheckpointIndex].transform.position;
//            distanceToNextCheckpoint = Vector3.Distance(transform.position, nextCheckpointPosition);
//            directionToNextCheckpoint = nextCheckpointPosition - transform.position;
//        }
//    }

//    public void CheckpointPassed(int checkpointIndex)
//    {
//        if (checkpointIndex == nextCheckpointIndex)
//        {
//            nextCheckpointIndex = (nextCheckpointIndex + 1) % checkpoints.Count;
//            UpdateCheckpointInfo();
//        }
//    }

//    public void ReportCollision()
//    {
//        // Optional: Implement collision response for the AI
//        // For example, you might want to temporarily slow down or adjust behavior
//    }

//    void OnDestroy()
//    {
//        // Clean up engine when component is destroyed
//        if (worker != null)
//        {
//            worker.Dispose();
//            worker = null;
//        }
//    }
//}