using UnityEngine;
using Assets.TrackGeneration;
using Unity.MLAgents.Policies;
using Unity.Sentis;
using Unity.MLAgents.Actuators;
using System.Collections;

public class CarSpawner : MonoBehaviour
{
    [Header("Car Setup")]
    public GameObject carPrefab; // Player car prefab 
    public GameObject aiCarPrefab;
    public TrackHandler trackHandler;
    // Using track's second starting position for AI car

    [Header("AI Setup")]
    public ModelAsset onnxModel; // Reference to your trained ML-Agents model (.onnx file)
    public bool spawnAI = false;
    [Tooltip("Set this to the same number of rays as used during training")]
    public int aiRayCount = 9;
    [Tooltip("Set this to the same ray angle as used during training")]
    public float aiRayAngle = 180f;
    [Tooltip("Set this to the same sensor length as used during training")]
    public float aiSensorLength = 20f;

    [Header("Camera Setup")]
    public Vector3 cameraOffset = new Vector3(0, 3, -8);
    public Vector3 rearCameraOffset = new Vector3(0, 3, 8);
    public float cameraFollowSpeed = 6f;
    public float cameraRotationSpeed = 5f;
    public float cameraLookSensitivity = 100f;

    private GameObject playerCar;
    private GameObject aiCar;
    private Transform carBody;
    private Camera[] playerCameras = new Camera[2];

    [Header("AI Layer Setup")]
    public string aiLayerName = "Agent"; // Use same layer as in TrainingManager

    void Start()
    {
        //SpawnCars();
    }

    public void SpawnCars()
    {
        Debug.Log("SpawnCars method called");

        if (carPrefab == null || trackHandler == null)
        {
            Debug.LogError($"Required components not assigned!");

            return;
        }

        Debug.Log("Clearing existing cars...");
        ClearExisting();

        Debug.Log("Getting start position from track");
        // Get start position from track
        var (startPos, startPos2, startRot) = trackHandler.GetTrackStartTransform();
        Debug.Log($"Start position: {startPos}, rotation: {startRot}");

        Debug.Log("Spawning player car...");
        SpawnPlayerCar(startPos, startRot);
        Debug.Log("Player car spawned");

        // Set up the camera
        Debug.Log("Setting up player camera...");
        SetupPlayerCamera();
        Debug.Log("Player camera setup complete");


        if (spawnAI)
        {// Spawn AI car at position2 from track handler
            if (aiCarPrefab == null)
            {
                Debug.LogError("AI car prefab not assigned!");
                return;
            }
            Debug.Log("Spawning AI car at second track position...");
            SpawnAICar(startPos2, startRot);
            Debug.Log("AI car spawned");
        }
    }

    private void SpawnPlayerCar(Vector3 startPos, Quaternion startRot)
    {
        // Spawn the player car
        playerCar = Instantiate(carPrefab, startPos, startRot);
        playerCar.tag = "Player";
        playerCar.name = "PlayerCar";

        // Configure car components
        CarControlScript carControl = playerCar.GetComponent<CarControlScript>();
        if (carControl != null)
        {
            carControl.trackHandler = trackHandler;
            // Flag this as player-controlled if your car script supports it
            var isPlayerControlledField = carControl.GetType().GetField("isPlayerControlled");
            if (isPlayerControlledField != null)
            {
                isPlayerControlledField.SetValue(carControl, true);
            }
        }
        else
        {
            Debug.LogError("CarControlScript not found on car prefab!");
        }

        // Find car body for camera following
        carBody = playerCar.transform.Find("CarBody");
        if (carBody == null)
        {
            Debug.LogWarning("CarBody not found in car prefab! Using car root transform instead.");
            carBody = playerCar.transform;
        }
    }

    private void SpawnAICar(Vector3 spawnPosition, Quaternion startRotation)
    {
        try
        {
            // Instantiate the car at the provided position
            GameObject carObject = Instantiate(aiCarPrefab, spawnPosition, startRotation);
            carObject.name = "AICar";
            carObject.tag = "AI";

            // Set the car's layer (important for consistent physics)
            int agentLayer = LayerMask.NameToLayer(aiLayerName);
            if (agentLayer != -1)
            {
                SetLayerRecursively(carObject, agentLayer);
            }
            else
            {
                Debug.LogWarning($"Layer '{aiLayerName}' not found! Using default layer.");
            }

            // Configure CarControlScript first (same as in TrainingManager)
            CarControlScript carControl = carObject.GetComponent<CarControlScript>();
            if (carControl != null)
            {
                carControl.isPlayerControlled = false;
                Debug.Log("Set car control to AI mode (isPlayerControlled = false)");


                carControl.trackHandler = trackHandler;

                // Disable visual effects for better performance (same as training)
                carControl.enableVisualEffects = true;

                // Make sure it has a rigidbody
                if (carControl.GetComponent<Rigidbody>() == null)
                {
                    Debug.LogWarning("No RB on AI CAR");
                }
            }
            else
            {
                Debug.LogError("CarControlScript not found on AI car prefab!");
                Destroy(carObject);
                return;
            }

            // Wait for car control to finish initialization before adding CarAgent
            StartCoroutine(AddAgentComponentsAfterDelay(carObject));

            // Store the AI car reference
            aiCar = carObject;
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error spawning AI car: {e.Message}\n{e.StackTrace}");
        }
    }

    // In your CarSpawner.cs, update your AddAgentComponentsAfterDelay method:

    private IEnumerator AddAgentComponentsAfterDelay(GameObject carObject)
    {
        // Wait two frames to ensure everything is initialized properly
        yield return null;
        yield return null;

        try
        {
            // Ensure trackHandler is available
            if (trackHandler == null)
            {
                Debug.LogError("TrackHandler is null in AddAgentComponentsAfterDelay!");
                yield break;
            }

            // First disable any player input control
            CarControlScript carControl = carObject.GetComponent<CarControlScript>();
            if (carControl != null)
            {
                carControl.isPlayerControlled = false;
                carControl.trackHandler = trackHandler;
            }
            else
            {
                Debug.LogError("CarControlScript not found on AI car!");
                yield break;
            }

            // Configure track sensing system first
            TrackSensingSystem sensorSystem = carObject.GetComponent<TrackSensingSystem>();
            if (sensorSystem == null)
            {
                sensorSystem = carObject.AddComponent<TrackSensingSystem>();
                sensorSystem.raycastOrigin = carObject.transform;
                sensorSystem.rayCount = aiRayCount;
                sensorSystem.rayAngle = aiRayAngle;
                sensorSystem.sensorLength = aiSensorLength;
                sensorSystem.visualizeRays = false;
            }
            else
            {
                sensorSystem.raycastOrigin = carObject.transform;
                sensorSystem.rayCount = aiRayCount;
                sensorSystem.rayAngle = aiRayAngle;
                sensorSystem.sensorLength = aiSensorLength;
                sensorSystem.visualizeRays = false;
            }

            // IMPORTANT: Use InferenceCarAgent instead of regular CarAgent
            InferenceCarAgent inferenceAgent = carObject.GetComponent<InferenceCarAgent>();
            if (inferenceAgent == null)
            {
                inferenceAgent = carObject.AddComponent<InferenceCarAgent>();
                Debug.Log("Added InferenceCarAgent component");
            }

            // Setup references
            inferenceAgent.carController = carControl;
            inferenceAgent.sensorSystem = sensorSystem;
            inferenceAgent.trackHandler = trackHandler;

            // Configure behavior parameters
            Unity.MLAgents.Policies.BehaviorParameters behaviorParams =
                carObject.GetComponent<Unity.MLAgents.Policies.BehaviorParameters>();

            if (behaviorParams == null)
            {
                behaviorParams = carObject.AddComponent<Unity.MLAgents.Policies.BehaviorParameters>();
            }

            // Setup for observation and action spaces
            int totalObservations = sensorSystem.rayCount + 5; // rays + other values

            // IMPORTANT: Set model before behavior type
            if (onnxModel != null)
            {
                behaviorParams.Model = onnxModel;
                Debug.Log($"ONNX model assigned to BehaviorParameters: {onnxModel.name}");

                // Then set behavior type
                behaviorParams.BehaviorName = "CarAgent";
                behaviorParams.BehaviorType = Unity.MLAgents.Policies.BehaviorType.InferenceOnly;
            }
            else
            {
                Debug.LogError("No ONNX model provided! AI car will not work correctly.");
                behaviorParams.BehaviorType = Unity.MLAgents.Policies.BehaviorType.Default;
            }

            // Configure observation space
            behaviorParams.BrainParameters.VectorObservationSize = totalObservations;
            behaviorParams.BrainParameters.NumStackedVectorObservations = 1;

            // Configure action space (must match training)
            var actionSpec = new Unity.MLAgents.Actuators.ActionSpec();
            actionSpec.NumContinuousActions = 3; // steering, acceleration, braking
            actionSpec.BranchSizes = new int[] { 2 }; // handbrake (on/off)
            behaviorParams.BrainParameters.ActionSpec = actionSpec;

            // Add decision requester
            Unity.MLAgents.DecisionRequester decisionRequester =
                carObject.GetComponent<Unity.MLAgents.DecisionRequester>();

            if (decisionRequester == null)
            {
                decisionRequester = carObject.AddComponent<Unity.MLAgents.DecisionRequester>();
                decisionRequester.DecisionPeriod = 5;
                decisionRequester.TakeActionsBetweenDecisions = true;
            }

            // Initialize the inference agent
            inferenceAgent.InitializeAgent();

            Debug.Log("AI car setup complete with InferenceCarAgent");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error configuring AI car: {e.Message}\n{e.StackTrace}");
        }
    }

    // Recursive method to set layer on all children (same as in TrainingManager)
    private void SetLayerRecursively(GameObject obj, int layer)
    {
        obj.layer = layer;

        foreach (Transform child in obj.transform)
        {
            SetLayerRecursively(child.gameObject, layer);
        }
    }

    private void SetupPlayerCamera()
    {
        GameObject cameraControlObj = new GameObject("CameraControl");
        CameraControlScript cameraControl = cameraControlObj.AddComponent<CameraControlScript>();

        // Create camera GameObjects
        for (int i = 0; i < 2; i++)
        {
            GameObject cameraObj = new GameObject($"PlayerCamera{i}");
            Camera cam = cameraObj.AddComponent<Camera>();
            cameraObj.transform.SetParent(cameraControl.transform);
            playerCameras[i] = cam;

            // Add AudioListener to the camera (only for main camera)
            if (i == 0)
            {
                cameraObj.AddComponent<AudioListener>();
            }

            // Add FollowCameraScript
            FollowCameraScript followScript = cameraObj.AddComponent<FollowCameraScript>();
            followScript.carTransform = carBody;

            if (i == 0) // Main camera
            {
                followScript.offset = cameraOffset;
                followScript.rearView = true;
                cam.enabled = true; // Explicitly enable the main camera
            }
            else // Rear camera
            {
                followScript.offset = rearCameraOffset;
                cam.enabled = false;
                followScript.rearView = true;
                if (cameraObj.GetComponent<AudioListener>() != null)
                    cameraObj.GetComponent<AudioListener>().enabled = false;
            }

            followScript.followSpeed = cameraFollowSpeed;
            followScript.rotationSpeed = cameraRotationSpeed;
            followScript.lookSensitivity = cameraLookSensitivity;

            // Position the camera initially
            cameraObj.transform.position = playerCar.transform.position + (i == 0 ? cameraOffset : rearCameraOffset);
            cameraObj.transform.LookAt(playerCar.transform);
        }

        // Ensure there's at least one active camera
        if (playerCameras[0] != null)
            playerCameras[0].enabled = true;

        cameraControl.SetupCameras(playerCameras[0], playerCameras[1]);
    }

    private void ClearExisting()
    {
        if (playerCar != null)
        {
            Destroy(playerCar);
        }

        if (aiCar != null)
        {
            Destroy(aiCar);
        }

        foreach (Camera playerCamera in playerCameras)
        {
            if (playerCamera != null)
            {
                Destroy(playerCamera.gameObject);
            }
        }

        // Clean up any leftover camera control objects
        CameraControlScript[] cameraControls = GetComponents<CameraControlScript>();
        foreach (var control in cameraControls)
        {
            Destroy(control.gameObject);
        }
    }

    // Public methods to access car and camera if needed
    public GameObject GetPlayerCar()
    {
        return playerCar;
    }

    public GameObject GetAICar()
    {
        return aiCar;
    }

    public Camera[] GetPlayerCameras()
    {
        return playerCameras;
    }
}