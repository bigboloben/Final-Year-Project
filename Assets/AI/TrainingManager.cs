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

    [Header("Performance Settings")]
    public int framesToWaitDuringRebuild = 10; // Spread work across frames
    public bool disableAgentsDuringRebuild = true; // Disable agents during rebuild to reduce load
    public int maxPointCount = 150; // Maximum number of track points (reduce for performance)
    public bool useStaggeredAgentReset = true; // Reset agents over multiple frames

    [Header("Episode Synchronization")]
    public bool synchronizeEpisodes = true; // Whether to synchronize episode resets
    public float maxWaitTimeForSynchronization = 5f; // Maximum time to wait for all agents
    public float waitDelayBetweenEpisodes = 0.5f; // Short delay between episodes

    // Flag to indicate when setup is complete
    private bool setupComplete = false;

    // Flag to indicate if a track rebuild is in progress
    private bool rebuildInProgress = false;

    // Episode synchronization
    private bool episodeResetRequested = false;
    private int agentsReadyForReset = 0;
    private int totalActiveAgents = 0;
    private float episodeResetRequestTime = 5f;

    // Event that agents can subscribe to for track rebuilds
    public delegate void TrackRebuildEvent();
    public event TrackRebuildEvent OnTrackRebuilt;

    // NEW: Event for synchronized episode resets
    public delegate void EpisodeResetEvent();
    public event EpisodeResetEvent OnEpisodeReset;

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
        var episodesBeforeRebuildParam = Academy.Instance.EnvironmentParameters.GetWithDefault("track_rebuild_episodes", episodesBeforeRebuild);
        episodesBeforeRebuild = (int)episodesBeforeRebuildParam;

        // Set up all components in sequence
        StartCoroutine(SetupSequence());
    }

    void Update()
    {
        // Check for episode synchronization timeouts
        if (episodeResetRequested && synchronizeEpisodes)
        {
            float timeWaiting = Time.time - episodeResetRequestTime;
            if (timeWaiting > maxWaitTimeForSynchronization)
            {
                Debug.LogWarning($"Episode synchronization timeout reached. Forcing reset with {agentsReadyForReset}/{totalActiveAgents} agents ready.");
                StartCoroutine(ResetAllAgentsCoroutine());
            }
        }
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
        trackHandler.PointCount = Mathf.Min(pointCount, maxPointCount); // Use the minimum for better performance
        trackHandler.CanvasSize = canvasSize;

        trackHandler.trackCamEnabled = true;

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
        totalActiveAgents = 0;

        // Get start position from track handler
        (Vector3 startPosition1, Vector3 startPosition2, Quaternion startRotation) = trackHandler.GetTrackStartTransform();

        // Use the first start position for all agents
        Vector3 startPosition = startPosition1;

        // Spread agent creation across multiple frames
        StartCoroutine(SpawnAgentsOverTime(startPosition, startRotation));
    }

    private IEnumerator SpawnAgentsOverTime(Vector3 startPosition, Quaternion startRotation)
    {
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

            // Wait every 3 agents to spread creation across frames
            if (i % 3 == 2)
            {
                yield return null;
            }
        }
    }

    private IEnumerator AddAgentAfterDelay(GameObject carObject, int index)
    {
        // Wait two frames to ensure everything is initialized properly
        yield return null;
        yield return null;

        try
        {
            // First add the CarAgent component
            CarAgent carAgent = carObject.GetComponent<CarAgent>();
            if (carAgent == null)
            {
                carAgent = carObject.AddComponent<CarAgent>();
            }

            // Configure track sensing system
            TrackSensingSystem sensorSystem = carObject.GetComponent<TrackSensingSystem>();
            if (sensorSystem == null)
            {
                sensorSystem = carObject.AddComponent<TrackSensingSystem>();
                sensorSystem.raycastOrigin = carObject.transform;
                sensorSystem.rayCount = 9;
                sensorSystem.rayAngle = 180f;
                sensorSystem.sensorLength = 20f;
                sensorSystem.visualizeRays = true;
            }
            else
            {
                sensorSystem.raycastOrigin = carObject.transform;
            }

            // Set up references
            carAgent.trackHandler = trackHandler;
            carAgent.raceManager = raceManager;
            carAgent.carController = carObject.GetComponent<CarControlScript>();

            carAgent.carController.isPlayerControlled = false;

            carAgent.carController.enableVisualEffects = false; // Disable visual effects for training
            sensorSystem.visualizeRays = false; // Disable ray visualization for training
            carAgent.sensorSystem = sensorSystem;

            // NEW: Enable reset on collision
            if (carAgent.GetType().GetField("resetOnCollision") != null)
            {
                carAgent.GetType().GetField("resetOnCollision").SetValue(carAgent, true);
            }

            carAgent.visualizeTargetDirection = true;

            // Register with race manager
            raceManager.RegisterAgent(carObject, "Agent_" + index);
            carAgent.completedCheckpointsBeforeNewTrack = int.MaxValue;

            // Calculate total observations
            int totalObservations = sensorSystem.rayCount + 5; // 9 rays + 5 other values = 14

            // Configure Behavior Parameters
            Unity.MLAgents.Policies.BehaviorParameters behaviorParams =
                carObject.GetComponent<Unity.MLAgents.Policies.BehaviorParameters>();

            if (behaviorParams == null)
            {
                behaviorParams = carObject.AddComponent<Unity.MLAgents.Policies.BehaviorParameters>();
            }

            // Set behavior properties
            behaviorParams.BehaviorName = "CarAgent";

            // CRITICAL FIX: This is the correct way to set up the observation and action space
            behaviorParams.BrainParameters.VectorObservationSize = totalObservations;
            behaviorParams.BrainParameters.NumStackedVectorObservations = 1;

            // Create a properly configured ActionSpec
            var actionSpec = new ActionSpec();

            // 3 continuous actions: steering, acceleration, braking 
            actionSpec.NumContinuousActions = 3;

            // 1 discrete action branch (handbrake) with 2 possible values (on/off)
            actionSpec.BranchSizes = new int[] { 2 };

            behaviorParams.BrainParameters.ActionSpec = actionSpec;
            behaviorParams.TeamId = 0;

            // Configure decision requester
            DecisionRequester decisionRequester =
                carObject.GetComponent<Unity.MLAgents.DecisionRequester>();

            if (decisionRequester == null)
            {
                decisionRequester = carObject.AddComponent<Unity.MLAgents.DecisionRequester>();
                decisionRequester.DecisionPeriod = 5;
                decisionRequester.TakeActionsBetweenDecisions = true;
            }

            // Add to tracked agents
            agents.Add(carAgent);

            // Increment active agent count
            totalActiveAgents++;
        }
        catch (Exception e)
        {
            Debug.LogError($"Error configuring agent: {e.Message}\n{e.StackTrace}");
        }
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
        // Skip if we're currently rebuilding or resetting episodes
        if (rebuildInProgress || episodeResetRequested)
            return;

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

        Debug.Log($"Episode counter: {episodeCounter}/{episodesBeforeRebuild}");
    }

    // NEW: Method for agents to notify they are ready for episode reset
    public void NotifyAgentReadyForReset(bool forRebuild = false)
    {
        // Increment the counter of agents ready for reset
        agentsReadyForReset++;

        if (forRebuild)
        {
            Debug.Log($"Agent ready for rebuild. Ready: {agentsReadyForReset}/{totalActiveAgents}");
        }
        else
        {
            Debug.Log($"Agent ready for episode reset. Ready: {agentsReadyForReset}/{totalActiveAgents}");
        }

        // If all agents are ready, proceed with reset
        if (agentsReadyForReset >= totalActiveAgents)
        {
            if (forRebuild && episodeCounter >= episodesBeforeRebuild)
            {
                Debug.Log("All agents ready for rebuild. Starting track rebuild now.");
                StartCoroutine(RebuildTrackAfterDelay());
            }
            else if (episodeResetRequested)
            {
                Debug.Log("All agents ready for episode reset. Resetting all agents now.");
                StartCoroutine(ResetAllAgentsCoroutine());
            }
        }
    }

    // NEW: Method to initiate a synchronized episode reset
    // Add a parameter to indicate if this reset is for a rebuild
    public void RequestEpisodeReset(bool forRebuild = false)
    {
        if (!synchronizeEpisodes || episodeResetRequested || rebuildInProgress)
            return;

        Debug.Log($"Requesting synchronized episode reset for all agents (forRebuild: {forRebuild})");

        // Reset the counter and set the request flag
        episodeResetRequested = true;
        episodeResetRequestTime = Time.time;
        agentsReadyForReset = 0;

        // Notify all agents to finish their current run
        foreach (var agent in agents)
        {
            if (agent != null && agent.gameObject.activeSelf)
            {
                // Pass the forRebuild flag to the agents
                agent.RequestFinishForReset(forRebuild);
            }
        }
    }

    // NEW: Coroutine to reset all agents
    private IEnumerator ResetAllAgentsCoroutine()
    {
        bool shouldRebuildTrack = episodeCounter >= episodesBeforeRebuild;

        // Log what we're doing
        if (shouldRebuildTrack)
        {
            Debug.Log($"Resetting all agents with track rebuild. Episodes: {episodeCounter}/{episodesBeforeRebuild}");
        }

        // Wait a short delay to let physics stabilize
        yield return new WaitForSeconds(waitDelayBetweenEpisodes);

        // If we should rebuild the track, do that first
        if (shouldRebuildTrack)
        {
            StartCoroutine(RebuildTrackAfterDelay());
            yield break; // Exit this coroutine, as RebuildTrackAfterDelay will handle agent reset
        }


        // Wait a short delay to let physics stabilize
        yield return new WaitForSeconds(waitDelayBetweenEpisodes);

        // Reset episodeResetRequested flag first so agents can request new resets
        episodeResetRequested = false;

        // Reset all agents in a staggered fashion
        for (int i = 0; i < agents.Count; i++)
        {
            var agent = agents[i];
            if (agent != null && agent.gameObject.activeSelf)
            {
                // Reposition the agent
                RepositionAgent(agent, i);

                // Wait every 2 agents to give Unity time to process physics
                if (i % 2 == 1)
                {
                    yield return new WaitForFixedUpdate();
                }
            }
        }

        // Wait for physics to settle
        yield return new WaitForFixedUpdate();

        // Trigger the OnEpisodeReset event that agents are listening for
        OnEpisodeReset?.Invoke();

        // Final reset of counters
        agentsReadyForReset = 0;

        IncrementEpisodeCounter();
    }

    private void CheckForRebuild()
    {
        // Skip if already rebuilding or reset is requested
        if (rebuildInProgress || episodeResetRequested)
            return;

        // Check if we've reached the rebuild threshold
        if (episodeCounter > 0 && episodeCounter % episodesBeforeRebuild == 0)
        {
            Debug.Log($"Requesting track rebuild after {episodeCounter} episodes");

            // Request a reset SPECIFICALLY for rebuild purposes
            RequestEpisodeReset(true);  // <-- Set forRebuild to true
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
        // Mark rebuild in progress
        rebuildInProgress = true;
        episodeResetRequested = false;
        agentsReadyForReset = 0;

        yield return new WaitForSeconds(trackRebuildDelay);

        // Disable agents during rebuild
        if (disableAgentsDuringRebuild)
        {
            foreach (var agent in agents)
            {
                if (agent != null && agent.gameObject != null)
                {
                    agent.gameObject.SetActive(false);
                }
            }
        }

        yield return new WaitForSeconds(0.1f); // Let physics settle

        // Generate a new track
        GenerateTrack();

        // Wait a frame for track to be built
        yield return new WaitForSeconds(0.2f);

        // Update checkpoints in race manager
        raceManager.InitializeCheckpoints(trackHandler.GetCheckpoints());

        // Wait additional frames for Unity to process changes
        for (int i = 0; i < framesToWaitDuringRebuild; i++)
        {
            yield return null;
        }

        // Notify all agents about track rebuild
        OnTrackRebuilt?.Invoke();

        // Re-enable and reset all agents
        if (useStaggeredAgentReset)
        {
            yield return StartCoroutine(StaggeredAgentReset());
        }
        else
        {
            // Reset all agents immediately
            foreach (var agent in agents)
            {
                if (agent != null)
                {
                    if (disableAgentsDuringRebuild)
                    {
                        agent.gameObject.SetActive(true);
                    }

                    agent.ResetForNewTrack();
                    // Use EndEpisode directly - our counter will be updated by the next episode's start
                    agent.EndEpisode();
                }
            }
        }

        // Reset the episode counter
        episodeCounter = 0;

        // Clear rebuild in progress flag
        rebuildInProgress = false;
    }

    // Reset agents over multiple frames to spread out the load
    private IEnumerator StaggeredAgentReset()
    {
        // Wait a bit for physics to settle after track rebuild
        yield return new WaitForFixedUpdate();
        yield return new WaitForFixedUpdate();

        // First position all the agents at their starting positions while still inactive
        for (int i = 0; i < agents.Count; i++)
        {
            var agent = agents[i];
            if (agent != null)
            {
                // Reposition the agent first (while still inactive)
                RepositionAgent(agent, i);

                // Wait every 2 agents to give Unity time to process physics
                if (i % 2 == 1)
                {
                    yield return new WaitForFixedUpdate();
                }
            }
        }

        // Wait for physics to settle
        yield return new WaitForFixedUpdate();
        yield return new WaitForFixedUpdate();

        // Now activate and reset each agent
        for (int i = 0; i < agents.Count; i++)
        {
            var agent = agents[i];
            if (agent != null)
            {
                // Re-enable agent if it was disabled
                if (disableAgentsDuringRebuild && !agent.gameObject.activeSelf)
                {
                    agent.gameObject.SetActive(true);
                    // Wait a frame for GameObject to fully activate
                    yield return null;
                }

                // Then reset its state
                agent.ResetForNewTrack();

                // End the episode to trigger a new one
                agent.EndEpisode();

                // Wait every 2 agents to give Unity time to process physics
                if (i % 2 == 1)
                {
                    yield return new WaitForFixedUpdate();
                }
            }
        }

        // Final wait to ensure all physics is settled
        yield return new WaitForFixedUpdate();
        yield return new WaitForFixedUpdate();
    }

    // Reposition a single agent
    private void RepositionAgent(CarAgent agent, int index)
    {
        if (agent == null || agent.transform == null)
            return;

        // Get start position from track handler
        (Vector3 startPosition1, Vector3 startPosition2, Quaternion startRotation) = trackHandler.GetTrackStartTransform();

        // Calculate position with offset to prevent overlap
        Vector3 spawnPosition = startPosition1 + Vector3.right * (index % 3) * agentSpacing + Vector3.forward * (index / 3) * agentSpacing;

        // Reset physics state completely
        Rigidbody rb = agent.GetComponent<Rigidbody>();
        if (rb != null)
        {
            // First reset velocities while still non-kinematic
            bool wasKinematic = rb.isKinematic;

            // Temporarily make non-kinematic to set velocities
            if (wasKinematic)
                rb.isKinematic = false;

            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            rb.ResetInertiaTensor();
            rb.Sleep();

            // Restore original kinematic state
            if (wasKinematic)
                rb.isKinematic = true;
        }

        // Force position and rotation update
        agent.transform.position = spawnPosition;
        agent.transform.rotation = startRotation;

        // Ensure car controller is also reset
        CarControlScript carController = agent.GetComponent<CarControlScript>();
        if (carController != null)
        {
            // Check if there's a ResetVehicle method on the car controller and call it
            var resetMethod = carController.GetType().GetMethod("ResetVehicle");
            if (resetMethod != null)
            {
                resetMethod.Invoke(carController, null);
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