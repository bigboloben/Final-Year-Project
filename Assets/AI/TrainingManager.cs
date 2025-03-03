using Assets.TrackGeneration;
using System.Collections.Generic;
using System.Collections;
using Unity.MLAgents;
using UnityEngine;
using System;
using Unity.MLAgents.Actuators;

public class TrainingManager : MonoBehaviour
{
    // Important: Make this a static instance so agents can find it
    public static TrainingManager Instance { get; private set; }

    [Header("Agent Setup")]
    public List<GameObject> carPrefabs;
    public int numberOfAgents = 10;
    public float agentSpacing = 5f;  // Distance between agents at start
    private List<CarAgent> agents = new List<CarAgent>();

    [Header("Track Generation")]
    public int episodesBeforeRebuild = 100;
    public float trackRebuildDelay = 1f;
    private int episodeCounter = 0;

    // Public reference to trackHandler so agents can find it
    [HideInInspector]
    public TrackHandler trackHandler;
    [HideInInspector]
    public RaceManager raceManager;

    [Header("Track Settings")]
    public Material trackMaterial;
    public Material wallMaterial;
    public PhysicsMaterial wallPhysicsMaterial;
    public GameObject startLinePrefab;
    public GameObject gridMarkerPrefab;
    public Material trackSupportMaterial;
    public float trackWidth = 12f;
    public float trackHeight = 0.1f;
    public float wallHeight = 0.5f;
    public float wallWidth = 0.5f;
    public float segments = 1f;
    public float banking = 15f;
    public int supportCount = 20;
    public int pointCount = 150;
    public int canvasSize = 1000;

    [Header("Floor Settings")]
    public GameObject floorPrefab;
    public bool setupFloor = true;

    [Header("Camera Settings")]
    public float cameraHeight = 80f;

    [Header("Race Settings")]
    public int totalLaps = 3;
    public float countdownTime = 3f;

    [Header("Layer Configuration")]
    public string agentLayerName = "Agent";
    private int agentLayer;

    // Flag to indicate when setup is complete
    private bool setupComplete = false;

    // Event that agents can subscribe to for track rebuilds
    public delegate void TrackRebuildEvent();
    public event TrackRebuildEvent OnTrackRebuilt;

    void Awake()
    {
        // Set up singleton instance
        if (Instance == null)
        {
            Instance = this;
        }
        else if (Instance != this)
        {
            Destroy(gameObject);
            return;
        }

        // Set up layer configuration
        agentLayer = LayerMask.NameToLayer(agentLayerName);
        if (agentLayer == -1)
        {
            Debug.LogError("Agent layer not found! Please create a layer named '" + agentLayerName + "' in your project settings.");
            return;
        }

        // Ignore collisions between agents
        Physics.IgnoreLayerCollision(agentLayer, agentLayer, true);
    }

    void Start()
    {
        // Set up all components in sequence
        StartCoroutine(SetupSequence());
    }

    private IEnumerator SetupSequence()
    {
        // Step 1: Set up the track system
        SetupTrackSystem();
        yield return null;

        // Step 2: Set up the floor if enabled
        if (setupFloor)
        {
            SetUpFloor();
        }
        yield return null;

        // Step 3: Set up the race manager
        SetupRaceManager();
        yield return null;

        // Step 4: Generate initial track
        GenerateTrack();
        yield return new WaitForSeconds(0.2f); // Wait for track to be fully generated

        // Step 5: Initialize race manager with checkpoints
        var checkpoints = trackHandler.GetCheckpoints();
        if (checkpoints != null && checkpoints.Count > 0)
        {
            raceManager.InitializeCheckpoints(checkpoints);
        }
        else
        {
            Debug.LogError("No checkpoints found after track generation!");
        }
        yield return null;

        // Step 6: Mark setup as complete
        setupComplete = true;
        Debug.Log("TrainingManager setup complete!");

        // Step 7: Spawn agents
        SpawnAgents();
    }

    private void SetupTrackSystem()
    {
        Debug.Log("Setting up track system...");

        // Create a new GameObject for the track system
        GameObject trackSystemObj = new GameObject("Track System");

        // Add TrackHandler component
        trackHandler = trackSystemObj.AddComponent<TrackHandler>();

        // Set up track materials
        trackHandler.trackMaterial = trackMaterial;
        trackHandler.wallMaterial = wallMaterial;
        trackHandler.wallPhysicsMaterial = wallPhysicsMaterial;
        trackHandler.startLinePrefab = startLinePrefab;
        trackHandler.gridMarkerPrefab = gridMarkerPrefab;
        trackHandler.trackSupportMaterial = trackSupportMaterial;

        // Set up track parameters
        trackHandler.trackWidth = trackWidth;
        trackHandler.trackHeight = trackHeight;
        trackHandler.wallHeight = wallHeight;
        trackHandler.wallWidth = wallWidth;
        trackHandler.segments = segments;
        trackHandler.banking = banking;
        trackHandler.supportCount = supportCount;
        trackHandler.PointCount = pointCount;
        trackHandler.CanvasSize = canvasSize;

        // Create and setup camera
        GameObject cameraObj = new GameObject("Track Camera");
        Camera trackCamera = cameraObj.AddComponent<Camera>();
        trackCamera.orthographic = true;
        trackCamera.orthographicSize = canvasSize / 2f;
        trackCamera.transform.position = Vector3.up * cameraHeight;
        trackCamera.transform.rotation = Quaternion.Euler(90f, 0f, 0f);

        // Assign camera to track handler
        trackHandler.trackCamera = trackCamera;

        Debug.Log("Track system setup complete!");
    }

    private void SetUpFloor()
    {
        Debug.Log("Setting up floor...");
        GameObject floorSystem = new GameObject("Floor System");
        FloorGenerator floorGenerator = floorSystem.AddComponent<FloorGenerator>();
        floorGenerator.floorPrefab = floorPrefab;
        floorGenerator.Initialize();
        Debug.Log("Floor setup complete!");
    }

    private void SetupRaceManager()
    {
        Debug.Log("Setting up race manager...");
        // Create race manager object
        GameObject raceManagerObj = new GameObject("Race Manager");
        raceManager = raceManagerObj.AddComponent<RaceManager>();

        // Configure race settings
        raceManager.totalLaps = totalLaps;
        raceManager.countdownTime = countdownTime;
        if (raceManager.GetType().GetField("skipCountdownForTraining") != null)
        {
            // If we added the skipCountdownForTraining field to the RaceManager, set it
            raceManager.GetType().GetField("skipCountdownForTraining").SetValue(raceManager, true);
        }
        Debug.Log("Race manager setup complete!");
    }

    // Update the SpawnAgents method in your TrainingManager.cs file

    private void SpawnAgents()
    {
        Debug.Log("Spawning agents...");
        if (carPrefabs.Count == 0)
        {
            Debug.LogError("No car prefab assigned!");
            return;
        }

        // Clear existing agents if any
        foreach (var agent in agents)
        {
            if (agent != null)
            {
                Destroy(agent.gameObject);
            }
        }
        agents.Clear();

        // Get start position from track handler
        (Vector3 startPosition1, Vector3 startPosition2, Quaternion startRotation) = trackHandler.GetTrackStartTransform();

        // Use the first start position for all agents
        Vector3 startPosition = startPosition1;

        for (int i = 0; i < numberOfAgents; i++)
        {
            try
            {
                // Instantiate the car at the start position with slight offset to prevent overlap
                Vector3 spawnPosition = startPosition + Vector3.right * (i % 3) * agentSpacing + Vector3.forward * (i / 3) * agentSpacing;
                int prefabIndex = UnityEngine.Random.Range(0, carPrefabs.Count);
                if (prefabIndex >= carPrefabs.Count) prefabIndex = 0;

                // Instantiate the car
                GameObject carObject = Instantiate(carPrefabs[prefabIndex], spawnPosition, startRotation);
                carObject.name = $"CarAgent_{i}";

                // Set the car's layer first
                carObject.layer = agentLayer;
                SetLayerRecursively(carObject, agentLayer);

                // Configure CarControlScript first so it's ready when CarAgent initializes
                CarControlScript carControl = carObject.GetComponent<CarControlScript>();
                if (carControl != null)
                {
                    carControl.trackHandler = trackHandler;

                    // Make sure it has a rigidbody
                    if (carControl.GetComponent<Rigidbody>() == null)
                    {
                        Debug.LogWarning($"Adding Rigidbody to {carObject.name} - car prefab should include a Rigidbody component");
                        Rigidbody rb = carObject.AddComponent<Rigidbody>();
                        rb.mass = 1500f;
                        rb.linearDamping = 0.2f;
                        rb.angularDamping = 0.05f;
                    }
                }

                // Wait a frame to ensure car control script is initialized
                // We'll add CarAgent in the next frame
                StartCoroutine(AddAgentAfterDelay(carObject, i));
            }
            catch (Exception e)
            {
                Debug.LogError($"Error spawning agent {i}: {e.Message}");
            }
        }
    }

    private IEnumerator AddAgentAfterDelay(GameObject carObject, int index)
    {
        // Wait one frame
        yield return null;

        try
        {
            // First add the CarAgent component
            CarAgent carAgent = carObject.GetComponent<CarAgent>();
            if (carAgent == null)
            {
                carAgent = carObject.AddComponent<CarAgent>();
            }

            // Now add the TrackSensingSystem
            TrackSensingSystem sensorSystem = carObject.GetComponent<TrackSensingSystem>();
            if (sensorSystem == null)
            {
                sensorSystem = carObject.AddComponent<TrackSensingSystem>();

                // Configure the sensor system
                sensorSystem.raycastOrigin = carObject.transform;
                sensorSystem.rayCount = 9;
                sensorSystem.rayAngle = 180f;
                sensorSystem.sensorLength = 20f;
                sensorSystem.visualizeRays = true;

                Debug.Log($"Created new TrackSensingSystem for {carObject.name}");
            }
            else
            {
                sensorSystem.raycastOrigin = carObject.transform;
                Debug.Log($"Found existing TrackSensingSystem on {carObject.name}");
            }

            // Set up all references
            carAgent.trackHandler = trackHandler;
            carAgent.raceManager = raceManager;
            carAgent.carController = carObject.GetComponent<CarControlScript>();
            carAgent.sensorSystem = sensorSystem;

            // Register agent with race manager
            raceManager.RegisterAgent(carObject, "Agent_" + index);

            // Disable independent track generation in CarAgent
            carAgent.completedCheckpointsBeforeNewTrack = int.MaxValue;

            // Calculate total observations: 
            // 9 rays + 2 direction dots + 1 distance + 1 speed + 1 drift status = 14
            int totalObservations = sensorSystem.rayCount + 5;

            // Add BehaviorParameters component
            Unity.MLAgents.Policies.BehaviorParameters behaviorParams =
                carObject.GetComponent<Unity.MLAgents.Policies.BehaviorParameters>();

            if (behaviorParams == null)
            {
                behaviorParams = carObject.AddComponent<Unity.MLAgents.Policies.BehaviorParameters>();
            }

            // Set behavior properties
            behaviorParams.BehaviorName = "CarAgent";
            behaviorParams.BrainParameters.VectorObservationSize = totalObservations;
            Debug.Log($"Setting VectorObservationSize to {totalObservations} for {carObject.name}");
            behaviorParams.BrainParameters.NumStackedVectorObservations = 1;

            // Create ActionSpec
            ActionSpec actionSpec = ActionSpec.MakeContinuous(3);

            // Add discrete branch for handbrake
            actionSpec.BranchSizes = new int[] { 2 };

            behaviorParams.BrainParameters.ActionSpec = actionSpec;
            behaviorParams.TeamId = 0;

            // Add DecisionRequester
            Unity.MLAgents.DecisionRequester decisionRequester =
                carObject.GetComponent<Unity.MLAgents.DecisionRequester>();

            if (decisionRequester == null)
            {
                decisionRequester = carObject.AddComponent<Unity.MLAgents.DecisionRequester>();
                decisionRequester.DecisionPeriod = 5;
                decisionRequester.TakeActionsBetweenDecisions = true;
            }

            // Add to our list
            agents.Add(carAgent);

            // Log success
            if (agents.Count == numberOfAgents)
            {
                Debug.Log($"Successfully spawned {numberOfAgents} agents with observation size {totalObservations}");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Error adding agent component: {e.Message}\n{e.StackTrace}");
        }
    }

    // Add this method for creating or validating a TrackSensingSystem
    private TrackSensingSystem EnsureSensorSystem(GameObject carObject)
    {
        // First check if the car already has a sensor system
        TrackSensingSystem sensorSystem = carObject.GetComponentInChildren<TrackSensingSystem>();

        if (sensorSystem == null)
        {
            // If no sensor system exists, create one
            GameObject sensorObj = new GameObject("Track Sensing System");
            sensorObj.transform.SetParent(carObject.transform);
            sensorObj.transform.localPosition = Vector3.zero; // Position at car center
            sensorSystem = sensorObj.AddComponent<TrackSensingSystem>();

            // Configure sensor rays (adjust these settings as needed)
            // If the TrackSensingSystem has public configuration properties, set them here

            Debug.Log($"Created new TrackSensingSystem for {carObject.name}");
        }

        return sensorSystem;
    }

    // This method can be called by CarAgent's Initialize method to get the TrackHandler
    public TrackHandler GetTrackHandler()
    {
        return trackHandler;
    }

    // This method can be called by CarAgent's Initialize method to get a reference to this manager
    public static TrainingManager GetInstance()
    {
        return Instance;
    }

    // Recursive method to set layer on all children
    private void SetLayerRecursively(GameObject obj, int layer)
    {
        obj.layer = layer;

        foreach (Transform child in obj.transform)
        {
            SetLayerRecursively(child.gameObject, layer);
        }
    }

    private void CountEpisodes(int academyStepCount)
    {
        // Check if it's time to rebuild based on our own counter
        if (episodeCounter > 0 && episodeCounter % episodesBeforeRebuild == 0)
        {
            CheckForRebuild();
        }
    }

    // Method to be called from CarAgent when an episode ends
    public void IncrementEpisodeCounter()
    {
        episodeCounter++;

        // Log episode count periodically
        if (episodeCounter % 10 == 0)
        {
            Debug.Log("Training progress: " + episodeCounter + " episodes completed");
        }
    }

    private void CheckForRebuild()
    {
        // Check if we've reached the rebuild threshold
        if (episodeCounter > 0 && episodeCounter % episodesBeforeRebuild == 0)
        {
            Debug.Log("Rebuilding track after " + episodeCounter + " episodes");
            StartCoroutine(RebuildTrackAfterDelay());
        }
    }

    // Central track generation method
    public void GenerateTrack()
    {
        Debug.Log("Generating track...");
        // Randomly choose between grid and circular layouts only
        int strategy = UnityEngine.Random.Range(0, 2);
        if (strategy == 0)
            trackHandler.GenerateTrack(GenerationStrategy.GridWithNoise);
        else
            trackHandler.GenerateTrack(GenerationStrategy.CircularLayout);
        Debug.Log("Track generation complete!");
    }

    private IEnumerator RebuildTrackAfterDelay()
    {
        yield return new WaitForSeconds(trackRebuildDelay);

        // Generate a new track
        GenerateTrack();

        // Wait a frame for track to be built
        yield return new WaitForSeconds(0.2f);

        // Update checkpoints in race manager
        raceManager.InitializeCheckpoints(trackHandler.GetCheckpoints());

        // Notify all agents about track rebuild
        OnTrackRebuilt?.Invoke();

        // Reset all agents
        foreach (var agent in agents)
        {
            if (agent != null)
            {
                agent.ResetForNewTrack();
                // Use EndEpisode directly - our counter will be updated by the next episode's start
                agent.EndEpisode();
            }
        }

        // Reposition agents on the new track
        RepositionAgentsOnTrack();
    }

    private void RepositionAgentsOnTrack()
    {
        // Get start position from track handler
        (Vector3 startPosition1, Vector3 startPosition2, Quaternion startRotation) = trackHandler.GetTrackStartTransform();

        // Reposition agents with a bit of spacing to prevent overlap
        for (int i = 0; i < agents.Count; i++)
        {
            if (agents[i] != null)
            {
                // Calculate position with offset to prevent overlap
                Vector3 spawnPosition = startPosition1 + Vector3.right * (i % 3) * agentSpacing + Vector3.forward * (i / 3) * agentSpacing;

                // Update transform
                agents[i].transform.position = spawnPosition;
                agents[i].transform.rotation = startRotation;
            }
        }
    }

    void OnEnable()
    {
        // Register for academy reset event to count episodes
        if (Academy.Instance != null)
        {
            Academy.Instance.AgentPreStep += CountEpisodes;
        }
    }

    void OnDisable()
    {
        // Unregister from academy
        if (Academy.Instance != null)
        {
            Academy.Instance.AgentPreStep -= CountEpisodes;
        }
    }

    void OnDestroy()
    {
        // Unregister from academy
        if (Academy.Instance != null)
        {
            Academy.Instance.AgentPreStep -= CountEpisodes;
        }
    }
}