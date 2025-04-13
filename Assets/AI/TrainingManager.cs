using Assets.TrackGeneration;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using UnityEngine;

public class TrainingManager : MonoBehaviour
{
    // SECTION: Basic Configuration and References

    // Singleton instance
    public static TrainingManager Instance { get; private set; }

    [Header("Agent Setup")]
    public List<GameObject> carPrefabs;
    public int numberOfAgents = 10;
    public float agentSpacing = 5f;  // Distance between agents at start
    private List<CarAgent> agents = new List<CarAgent>();

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
    public float banking = 0f;
    public int supportCount = 0;
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

    // SECTION: Training Management and Curriculum

    [Header("Training Cycle")]
    public float trackRebuildDelay = 1f;
    public float maxWaitTimeForSynchronization = 20f;
    [Tooltip("If true, episode counters reset to 0 after track rebuild. If false, agents continue from their current episode count.")]
    public bool resetEpisodesAfterRebuild = true;

    [Header("Training Curriculum")]
    [Tooltip("Probability (0-1) of resetting on collision, gradually reduces during training")]
    public float collisionResetProbability = 1.0f;
    public int episodesBeforeRebuild = 1000;


    // Training progression tracking
    private int totalEpisodesCompleted = 0;  // Total individual episodes completed
    private int globalEpisodeCounter = 0;    // Global min episode counter that never resets
    private int lastGlobalCounterBeforeRebuild = 0; // Last global counter value before rebuild
    private int totalLapsCompleted = 0;
    private float bestLapTime = float.MaxValue;

    private float curriculumProgress = 0f; // 0.0 to 1.0 representing curriculum progress
    private float totalCheckpointsAcrossAgents = 0f;
    private float totalLapsAcrossAgents = 0f;
    private float totalCollisionsAcrossAgents = 0f;
    private int reportingCycle = 0;
    private float lastProgressReportTime = 0f;
    private float progressReportInterval = 20f; // Report progress every 20 seconds
    private float lastDetailedProgressTime = 0f;
    private const float DETAILED_PROGRESS_INTERVAL = 600f; // 10 minutes in seconds

    // Training State Management - using an enum for clear state tracking
    public enum TrainingState
    {
        NormalTraining,    // Regular training with no pending resets
        RebuildRequested,  // Track rebuild has been requested, waiting for agents
        RebuildInProgress  // Track rebuild is actively happening
    }

    // The current training state - single source of truth
    private TrainingState currentState = TrainingState.NormalTraining;

    // Episode tracking
    private int currentEpisode = 0;
    private int lastRebuildEpisode = 0;

    // Synchronization tracking for rebuild only
    private int agentsReadyForRebuild = 0;
    private int totalActiveAgents = 0;
    private float stateChangeTime = 0f;

    // Events for agents to subscribe to
    public delegate void TrackRebuildEvent();
    public event TrackRebuildEvent OnTrackRebuilt;

    // Performance tracking
    private int frameCount = 0;
    private float nextLogTime = 0;
    private float nextCurriculumCheckTime = 0;

    // Debug logging toggle
    [SerializeField]
    private bool isDebugLoggingEnabled = false;

    // OPTIMIZATION: Added thread safety flag
    private bool isProcessingRebuild = false;

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
        UpdateCurriculum();
        // Remove the conversion to boolean since we're keeping it as a float now
        // Initialize and set up all components in sequence
        StartCoroutine(SetupSequence());
    }

    void Update()
    {
        // Performance monitoring
        frameCount++;
        if (Time.time > nextLogTime)
        {
            float timeElapsed = Time.time - nextLogTime + 5f;
            float fps = frameCount / timeElapsed;
            frameCount = 0;
            nextLogTime = Time.time + 5f;

            // Log training progress
            if (isDebugLoggingEnabled)
            {
                Debug.Log($"[STATS] FPS: {fps:F1} | Global Episodes: {globalEpisodeCounter} | " +
                         $"Current Episode: {currentEpisode} | Laps: {totalLapsCompleted} | " +
                         $"Best Lap: {(bestLapTime < float.MaxValue ? bestLapTime.ToString("F2") + "s" : "None")}");
            }
        }

        // Update and report progress for curriculum advancement
        UpdateAndReportProgress();

        // Check for curriculum progression
        if (Time.time > nextCurriculumCheckTime)
        {
            UpdateCurriculum();
            nextCurriculumCheckTime = Time.time + 30f; // Check every 30 seconds
        }

        // Handle rebuild request timeout
        if (currentState == TrainingState.RebuildRequested)
        {
            float timeInCurrentState = Time.time - stateChangeTime;
            if (timeInCurrentState > maxWaitTimeForSynchronization)
            {
                Debug.LogWarning($"[SYNC] Timeout reached waiting for agents for rebuild. Forcing state transition. Ready: {agentsReadyForRebuild}/{totalActiveAgents}");
                StartCoroutine(PerformTrackRebuild());
            }
        }
    }

    // Update curriculum based on training progress
    private void UpdateCurriculum()
    {
        // Basic training parameters
        collisionResetProbability = Academy.Instance.EnvironmentParameters.GetWithDefault("use_strict_resets", collisionResetProbability);
        episodesBeforeRebuild = (int)Academy.Instance.EnvironmentParameters.GetWithDefault("episodes_before_rebuild", episodesBeforeRebuild);
        maxWaitTimeForSynchronization = Academy.Instance.EnvironmentParameters.GetWithDefault("maxWaitTimeForSynchronization", maxWaitTimeForSynchronization);

        // Get reward parameters and distribute to agents
        float speedReward = Academy.Instance.EnvironmentParameters.GetWithDefault("speed_reward", 0.05f);
        float checkpointReward = Academy.Instance.EnvironmentParameters.GetWithDefault("checkpoint_reward", 1f);
        float directionAlignmentReward = Academy.Instance.EnvironmentParameters.GetWithDefault("direction_alignment_reward", 0.2f);
        float lapCompletionReward = Academy.Instance.EnvironmentParameters.GetWithDefault("lapCompletionReward", 8f);
        float backwardsPenalty = Academy.Instance.EnvironmentParameters.GetWithDefault("backwards_penalty", -2f);
        float noProgressPenalty = Academy.Instance.EnvironmentParameters.GetWithDefault("no_progress_penalty", -0.2f);
        float reversePenalty = Academy.Instance.EnvironmentParameters.GetWithDefault("reverse_penalty", -0.15f);
        float collisionPenalty = Academy.Instance.EnvironmentParameters.GetWithDefault("collision_penalty", -8f);
        float longDriftReward = Academy.Instance.EnvironmentParameters.GetWithDefault("longDriftReward", 10f);
        float failedDriftPenalty = Academy.Instance.EnvironmentParameters.GetWithDefault("failedDriftPenalty", -2f);
        float noProgressThresholdTime = Academy.Instance.EnvironmentParameters.GetWithDefault("no_progress_threshold_time", 12f);
        float survivalBias = Academy.Instance.EnvironmentParameters.GetWithDefault("survival_bias", 0.002f);
        float maxTimeWithoutCheckpoint = Academy.Instance.EnvironmentParameters.GetWithDefault("max_time_without_checkpoint", 30f);


        // Update all active agents with the new parameters
        foreach (var agent in agents)
        {
            if (agent != null)
            {
                agent.UpdateParameters(
                    speedReward,
                    checkpointReward,
                    directionAlignmentReward,
                    lapCompletionReward,
                    backwardsPenalty,
                    noProgressPenalty,
                    reversePenalty,
                    collisionPenalty,
                    longDriftReward,
                    failedDriftPenalty,
                    noProgressThresholdTime,
                    survivalBias,
                    maxTimeWithoutCheckpoint
                );
            }
        }

        if (isDebugLoggingEnabled)
        {
            Debug.Log($"[CURRICULUM] Current collision reset probability: {collisionResetProbability:F2}, " +
                $"Current Track rebuild rate: {episodesBeforeRebuild}, " +
                $"Global episodes: {globalEpisodeCounter}, " +
                $"Checkpoint reward: {checkpointReward}, " +
                $"LapCompletion reward: {lapCompletionReward}");
        }
    }

    // SECTION: Setup Methods

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
        if (trackHandler.trackGenParam == null)
        {
            trackHandler.trackGenParam = new TrackGenerationParameters();
        }

        TrackGenerationParameters param = trackHandler.trackGenParam;
        // Point Generation
        param.PointCount = 50;
        // Track Geometry
        param.IdealTrackLength = 1000f;
        param.MinTrackLength = 900f;
        param.MaxTrackLength = 1100f;
        // Corner Parameters
        param.DesiredCornersCount = 5;
        param.MinCorners = 4;
        param.MaxCorners = 6;
        // Elevation Parameters
        param.MaxHeight = 10f;
        trackHandler.optimiseTrack = true;

        Debug.Log($"[REBUILD] Randomized track parameters: Corners={param.DesiredCornersCount}, " +
                  $"Length={param.IdealTrackLength}, MaxHeight={param.MaxHeight}, Optimised={trackHandler.optimiseTrack}");

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

        // Step 6: Mark setup as complete and spawn agents
        if (isDebugLoggingEnabled) Debug.Log("TrainingManager setup complete!");
        SpawnAgents();

        if (raceManager != null)
        {
            raceManager.InitiateRaceStart();
            Debug.Log("Race explicitly started by TrainingManager");
        }
    }

    private void SetupTrackSystem()
    {
        if (isDebugLoggingEnabled) Debug.Log("Setting up track system...");

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

        if (isDebugLoggingEnabled) Debug.Log("Track system setup complete!");
    }

    private void SetUpFloor()
    {
        if (isDebugLoggingEnabled) Debug.Log("Setting up floor...");
        GameObject floorSystem = new GameObject("Floor System");
        FloorGenerator floorGenerator = floorSystem.AddComponent<FloorGenerator>();
        floorGenerator.floorPrefab = floorPrefab;
        floorGenerator.Initialize();
        if (isDebugLoggingEnabled) Debug.Log("Floor setup complete!");
    }

    private void SetupRaceManager()
    {
        if (isDebugLoggingEnabled) Debug.Log("Setting up race manager...");
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
        if (isDebugLoggingEnabled) Debug.Log("Race manager setup complete!");
    }

    public void GenerateTrack()
    {
        if (isDebugLoggingEnabled) Debug.Log("Generating track...");
        // Randomly choose between grid and circular layouts only
        int strategy = UnityEngine.Random.Range(0, 2);
        if (strategy == 0)
            trackHandler.GenerateTrack(GenerationStrategy.CircularLayout);
        else
            trackHandler.GenerateTrack(GenerationStrategy.CircularLayout);
        if (isDebugLoggingEnabled) Debug.Log("Track generation complete!");
    }

    // SECTION: Agent Spawning (Following the original approach)

    private void SpawnAgents()
    {
        if (isDebugLoggingEnabled) Debug.Log("Spawning agents...");
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
        agentEpisodeCounts.Clear();
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
                carObject.tag = "AI";

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
                Transform carBodyTransform = carObject.transform.Find("CarBody");
                sensorSystem = carObject.AddComponent<TrackSensingSystem>();
                sensorSystem.raycastOrigin = carBodyTransform;
                //sensorSystem.rayCount = 9;
                //sensorSystem.rayAngle = 180f;
                //sensorSystem.sensorLength = 10f;
                sensorSystem.visualizeRays = false; // Disable for performance
            }
            else
            {
                sensorSystem.raycastOrigin = carObject.transform;
                sensorSystem.visualizeRays = false;
            }

            // Set up references
            carAgent.trackHandler = trackHandler;
            carAgent.raceManager = raceManager;
            carAgent.carController = carObject.GetComponent<CarControlScript>();

            // CRITICAL: Ensure car is not in player-controlled mode
            carAgent.carController.isPlayerControlled = false;

            carAgent.carController.enableVisualEffects = false; // Disable visual effects for training
            carAgent.sensorSystem = sensorSystem;

            // Enable reset on collision
            if (carAgent.GetType().GetField("resetOnCollision") != null)
            {
                carAgent.GetType().GetField("resetOnCollision").SetValue(carAgent, true);
            }

            carAgent.visualizeTargetDirection = false; // Disable for performance

            // Register with race manager
            raceManager.RegisterAgent(carObject, "Agent_" + index);
            carAgent.completedCheckpointsBeforeNewTrack = int.MaxValue;

            // Calculate total observations
            int totalObservations = 60;

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
                int decisionFrequency = (int)Mathf.Max(1, 5 / Mathf.Sqrt(Time.timeScale));
                decisionRequester.DecisionPeriod = decisionFrequency;
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

    private void SetLayerRecursively(GameObject obj, int layer)
    {
        obj.layer = layer;

        foreach (Transform child in obj.transform)
        {
            SetLayerRecursively(child.gameObject, layer);
        }
    }

    // SECTION: Training State Management

    // Dictionary to track each agent's current episode number
    private Dictionary<CarAgent, int> agentEpisodeCounts = new Dictionary<CarAgent, int>();

    // Called from Academy steps to check for rebuild conditions
    private void CountEpisodes(int academyStepCount)
    {
        // Only check if we're in normal training state
        if (currentState != TrainingState.NormalTraining)
            return;

        // Update global and current episode counters
        UpdateGlobalEpisodeCounter();

        // Check if it's time for a rebuild
        if (currentEpisode > 0 && currentEpisode >= episodesBeforeRebuild && currentEpisode > lastRebuildEpisode)
        {
            if (isDebugLoggingEnabled) Debug.Log($"[TRAIN] Minimum episode threshold reached: {currentEpisode}/{episodesBeforeRebuild} " +
                                           $"(Global: {globalEpisodeCounter}) - Requesting track rebuild");
            RequestTrackRebuild();
        }
    }

    // Get the minimum episode count across all agents
    private int GetMinimumEpisodeCount()
    {
        if (agentEpisodeCounts.Count == 0)
            return 0;

        return agentEpisodeCounts.Values.Min();
    }

    // Update the global episode counter based on the current minimum episode
    private void UpdateGlobalEpisodeCounter()
    {
        int minEpisodeCount = GetMinimumEpisodeCount();

        // During normal training, the global counter is the last rebuild value plus the current minimum
        globalEpisodeCounter = lastGlobalCounterBeforeRebuild + minEpisodeCount;

        // Also update the current episode tracker
        currentEpisode = minEpisodeCount;
    }

    // Report episode completion from an individual agent
    public void ReportAgentEpisodeCompleted(CarAgent agent, int agentEpisodeCount)
    {
        // Only track if we're in normal training state
        if (currentState != TrainingState.NormalTraining)
            return;

        // Update agent's episode count
        agentEpisodeCounts[agent] = agentEpisodeCount;

        // Increment total episodes counter (individual completions)
        totalEpisodesCompleted++;

        // Update the global episode counter based on minimum episode
        UpdateGlobalEpisodeCounter();

        // Log progress occasionally
        if (totalEpisodesCompleted % 10 == 0 || isDebugLoggingEnabled)
        {
            Debug.Log($"[TRAIN] Min episode across agents: {currentEpisode}/{episodesBeforeRebuild}, " +
                      $"Agent {agent.gameObject.name}: {agentEpisodeCount}, " +
                      $"Global: {globalEpisodeCounter} (Total: {totalEpisodesCompleted})");
        }
    }

    // Record a lap completion
    public void RecordLapCompletion(float lapTime)
    {
        totalLapsCompleted++;
        totalLapsAcrossAgents++;

        // Update best lap time
        if (lapTime < bestLapTime)
        {
            bestLapTime = lapTime;
            Debug.Log($"[TRAIN] New best lap time: {bestLapTime:F2}s");
        }
    }

    // Request a track rebuild
    public void RequestTrackRebuild()
    {
        // Only allow requests in normal training state and when not already processing rebuild
        if (currentState != TrainingState.NormalTraining || isProcessingRebuild)
        {
            if (isDebugLoggingEnabled) Debug.Log($"[WARN] Track rebuild requested while in {currentState} state - ignoring");
            return;
        }

        if (isDebugLoggingEnabled) Debug.Log($"[REBUILD] Track rebuild requested at episode {currentEpisode} (Global: {globalEpisodeCounter})");

        // Before changing state, save the current global episode count
        lastGlobalCounterBeforeRebuild = globalEpisodeCounter;

        // Change state and reset counters
        ChangeState(TrainingState.RebuildRequested);
        agentsReadyForRebuild = 0;

        // Notify all agents
        foreach (var agent in agents)
        {
            if (agent != null && agent.gameObject.activeSelf)
            {
                agent.RequestTrackRebuild();
            }
        }
    }

    // Agent calls this to signal it's ready for rebuild
    public void NotifyAgentReadyForRebuild()
    {
        // Make sure we're in an appropriate state to accept notifications
        if (currentState != TrainingState.RebuildRequested)
        {
            Debug.LogWarning($"[WARN] Agent notified ready for rebuild while in {currentState} state - ignoring");
            return;
        }

        // Increment the counter
        agentsReadyForRebuild++;

        if (isDebugLoggingEnabled && (agentsReadyForRebuild == 1 || agentsReadyForRebuild == totalActiveAgents))
        {
            Debug.Log($"[SYNC] {agentsReadyForRebuild}/{totalActiveAgents} agents ready for rebuild");
        }

        // Check if all agents are ready
        if (agentsReadyForRebuild >= totalActiveAgents)
        {
            if (isDebugLoggingEnabled) Debug.Log($"[SYNC] All agents ready - proceeding with track rebuild");
            StartCoroutine(PerformTrackRebuild());
        }
    }

    public float GetResetProbability()
    {
        return collisionResetProbability;
    }

    // Safely change the training state
    private void ChangeState(TrainingState newState)
    {
        if (isDebugLoggingEnabled) Debug.Log($"[STATE] {currentState} -> {newState}");
        currentState = newState;
        stateChangeTime = Time.time;
    }

    // SECTION: Training Actions Implementation

    private void RandomiseTrackParameters()
    {
        // Make sure trackHandler has TrackGenerationParameters
        if (trackHandler.trackGenParam == null)
        {
            trackHandler.trackGenParam = new TrackGenerationParameters();
        }

        TrackGenerationParameters param = trackHandler.trackGenParam;

        // Randomize track characteristics - adjust ranges as needed

        // Point Generation
        int[] pointCounts = new int[] { 50, 100, 150, 200 };
        param.PointCount = pointCounts[UnityEngine.Random.Range(0, pointCounts.Length)];

        // Track Geometry

        param.IdealTrackLength = UnityEngine.Random.Range(1000f, 1400f);
        param.MinTrackLength = UnityEngine.Random.Range(param.IdealTrackLength - 200f, param.IdealTrackLength);
        param.MaxTrackLength = UnityEngine.Random.Range(param.IdealTrackLength, param.IdealTrackLength + 200f);

        // Corner Parameters
        param.DesiredCornersCount = UnityEngine.Random.Range(6, 10);
        param.MinCorners = param.DesiredCornersCount - 2;
        param.MaxCorners = param.DesiredCornersCount + 2;


        // Quality Weights - Modify these to emphasize different aspects
        //param.CornerQualityWeight = UnityEngine.Random.Range(0.2f, 0.35f);
        //param.StraightQualityWeight = UnityEngine.Random.Range(0.05f, 0.2f);
        //param.FlowQualityWeight = UnityEngine.Random.Range(0.05f, 0.2f);
        //param.LayoutQualityWeight = UnityEngine.Random.Range(0.05f, 0.15f);
        //param.LengthQualityWeight = UnityEngine.Random.Range(0.2f, 0.3f);
        //param.ProximityQualityWeight = UnityEngine.Random.Range(0.15f, 0.25f);

        // Elevation Parameters
        param.MaxHeight = UnityEngine.Random.Range(5f, 15f);

        // Optionally enable/disable track optimization
        trackHandler.optimiseTrack = UnityEngine.Random.value > 0.5f;

        Debug.Log($"[REBUILD] Randomized track parameters: Corners={param.DesiredCornersCount}, " +
                  $"Length={param.IdealTrackLength}, MaxHeight={param.MaxHeight}, Optimised={trackHandler.optimiseTrack}");
    }

    // Perform a track rebuild and reset
    private IEnumerator PerformTrackRebuild()
    {
        // Thread safety to prevent multiple rebuilds happening at once
        if (isProcessingRebuild)
        {
            Debug.LogWarning("Track rebuild already in progress, ignoring duplicate request");
            yield break;
        }

        isProcessingRebuild = true;

        // Update state
        ChangeState(TrainingState.RebuildInProgress);

        if (isDebugLoggingEnabled) Debug.Log($"[REBUILD] Starting track rebuild at episode {currentEpisode}");

        if (raceManager != null)
        {
            raceManager.EndRace();
            if (isDebugLoggingEnabled) Debug.Log("[REBUILD] Race ended");
        }

        // Wait for a short delay
        yield return new WaitForSeconds(trackRebuildDelay);

        // Create a list of active agents to track
        List<CarAgent> agentsToProcess = new List<CarAgent>();
        foreach (var agent in agents)
        {
            if (agent != null && agent.gameObject != null)
            {
                agentsToProcess.Add(agent);
            }
        }

        // Disable agents during rebuilding if configured
        if (disableAgentsDuringRebuild)
        {
            foreach (var agent in agentsToProcess)
            {
                if (agent != null)
                {
                    agent.gameObject.SetActive(false);
                }
            }
        }

        yield return new WaitForSeconds(0.1f); // Let physics settle

        RandomiseTrackParameters();
        // Generate the new track
        if (isDebugLoggingEnabled) Debug.Log($"[REBUILD] Generating new track");
        GenerateTrack();

        // Wait for track generation to complete
        yield return new WaitForSeconds(0.2f);

        // Update checkpoints in race manager
        var checkpoints = trackHandler.GetCheckpoints();
        if (checkpoints != null && checkpoints.Count > 0)
        {
            if (isDebugLoggingEnabled) Debug.Log($"[REBUILD] Initializing race manager with {checkpoints.Count} checkpoints");
            raceManager.InitializeCheckpoints(checkpoints);
        }
        else
        {
            Debug.LogError("[REBUILD] No checkpoints found after track generation! Trying again...");
            // Try to recover by regenerating
            GenerateTrack();
            yield return new WaitForSeconds(0.2f);
            checkpoints = trackHandler.GetCheckpoints();
            if (checkpoints != null && checkpoints.Count > 0)
            {
                raceManager.InitializeCheckpoints(checkpoints);
            }
        }

        // Wait additional frames to process changes
        for (int i = 0; i < framesToWaitDuringRebuild; i++)
        {
            yield return null;
        }

        // Notify agents about the track rebuild
        if (isDebugLoggingEnabled) Debug.Log($"[REBUILD] Notifying agents about track rebuild");
        OnTrackRebuilt?.Invoke();

        // Re-enable and reset all agents
        if (useStaggeredAgentReset)
        {
            yield return StartCoroutine(StaggeredAgentReset(agentsToProcess));
        }
        else
        {
            // Reset all agents immediately
            foreach (var agent in agentsToProcess)
            {
                if (agent != null)
                {
                    if (disableAgentsDuringRebuild)
                    {
                        agent.gameObject.SetActive(true);
                    }

                    agent.ResetForNewTrack();
                    agent.EndEpisode();
                }
            }
        }

        // Reset episode counters and tracking if configured to do so
        if (resetEpisodesAfterRebuild)
        {
            currentEpisode = 0;
            lastRebuildEpisode = 0;
            agentEpisodeCounts.Clear();

            if (isDebugLoggingEnabled) Debug.Log($"[REBUILD] Episode counters reset to 0 for all agents (Global: {globalEpisodeCounter})");
        }
        else
        {
            // If not resetting, just update the last rebuild episode
            lastRebuildEpisode = currentEpisode;

            if (isDebugLoggingEnabled) Debug.Log($"[REBUILD] Episode counters maintained at min: {currentEpisode} (Global: {globalEpisodeCounter})");
        }

        if (raceManager != null)
        {
            raceManager.InitiateRaceStart();
            if (isDebugLoggingEnabled) Debug.Log("[REBUILD] Race restarted");
        }

        totalCheckpointsAcrossAgents = 0;
        totalCollisionsAcrossAgents = 0;
        totalLapsAcrossAgents = 0;

        // Return to normal training state
        ChangeState(TrainingState.NormalTraining);
        isProcessingRebuild = false;

        if (isDebugLoggingEnabled) Debug.Log("[REBUILD] Track rebuild completed successfully");
    }

    // Reset agents in a staggered manner to reduce per-frame load
    // Modify this method in TrainingManager.cs
    private IEnumerator StaggeredAgentReset(List<CarAgent> agentsToReset)
    {
        Debug.Log($"[REBUILD] Starting staggered reset for {agentsToReset.Count} agents");

        // Wait for physics to settle
        yield return new WaitForFixedUpdate();
        yield return new WaitForFixedUpdate();

        // Process agents in smaller batches (1-2 per frame) to ensure consistent behavior
        int batchSize = Mathf.Max(1, Mathf.Min(2, agentsToReset.Count / 20));

        for (int batchStart = 0; batchStart < agentsToReset.Count; batchStart += batchSize)
        {
            int batchEnd = Mathf.Min(batchStart + batchSize, agentsToReset.Count);
            Debug.Log($"[REBUILD] Resetting agents {batchStart}-{batchEnd - 1}");

            // Process each agent in the batch
            for (int i = batchStart; i < batchEnd; i++)
            {
                var agent = agentsToReset[i];
                if (agent != null)
                {
                    // First step: Re-enable gameObject if needed
                    if (disableAgentsDuringRebuild && !agent.gameObject.activeSelf)
                    {
                        agent.gameObject.SetActive(true);
                        Debug.Log($"[REBUILD] Re-enabled agent {i}");
                        // Wait a frame after enabling to let Unity initialize components
                        yield return null;
                    }

                    // Second step: Make sure critical flags are properly reset
                    ResetAgentFlags(agent, i);

                    // Third step: Reset the agent's game logic
                    ResetAgentLogic(agent, i);

                    // Fourth step: Ensure the Rigidbody is properly set
                    ResetAgentPhysics(agent, i);

                    // Fifth step: Make sure the car controller is enabled
                    ResetAgentCarController(agent, i);

                    // Sixth step: End the episode
                    yield return null; // Wait a frame before ending episode
                    agent.EndEpisode();
                    Debug.Log($"[REBUILD] EndEpisode called for agent {i}");
                }
            }

            // Wait for this batch to finish processing
            yield return new WaitForFixedUpdate();
            yield return null; // Additional frame wait to ensure stability
        }

        // Final wait to ensure all physics is settled
        yield return new WaitForFixedUpdate();
        Debug.Log("[REBUILD] Staggered agent reset completed for all agents");
    }

    // Helper methods to handle the try-catch blocks outside of yield statements
    private void ResetAgentFlags(CarAgent agent, int index)
    {
        try
        {
            var agentField = agent.GetType().GetField("isReadyForRebuild",
                System.Reflection.BindingFlags.Instance |
                System.Reflection.BindingFlags.Public |
                System.Reflection.BindingFlags.NonPublic);

            if (agentField != null)
            {
                agentField.SetValue(agent, false);
                Debug.Log($"[REBUILD] Force reset isReadyForRebuild flag for agent {index}");
            }

            var waitingField = agent.GetType().GetField("isWaitingForTrackRebuild",
                System.Reflection.BindingFlags.Instance |
                System.Reflection.BindingFlags.Public |
                System.Reflection.BindingFlags.NonPublic);

            if (waitingField != null)
            {
                waitingField.SetValue(agent, false);
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Error resetting flags for agent {index}: {e.Message}\n{e.StackTrace}");
        }
    }

    private void ResetAgentLogic(CarAgent agent, int index)
    {
        try
        {
            agent.ResetForNewTrack();
            Debug.Log($"[REBUILD] ResetForNewTrack completed for agent {index}");
        }
        catch (Exception e)
        {
            Debug.LogError($"Error in ResetForNewTrack for agent {index}: {e.Message}\n{e.StackTrace}");
        }
    }

    private void ResetAgentPhysics(CarAgent agent, int index)
    {
        try
        {
            Rigidbody rb = agent.GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.isKinematic = false;
                // Apply a small force to "wake up" the physics body
                rb.AddForce(Vector3.up * 0.1f, ForceMode.Impulse);
                Debug.Log($"[REBUILD] Reset physics for agent {index}");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Error resetting physics for agent {index}: {e.Message}");
        }
    }

    private void ResetAgentCarController(CarAgent agent, int index)
    {
        try
        {
            CarControlScript carController = agent.GetComponent<CarControlScript>();
            if (carController != null)
            {
                carController.enabled = true;
                carController.ResetPosition();
                Debug.Log($"[REBUILD] Reset position for agent {index}, rotation: {agent.transform.rotation.eulerAngles}");
            }
            else
            {
                Debug.LogError($"CarControlScript not found on agent {index}");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Error resetting car controller for agent {index}: {e.Message}");

            // Even if an exception occurs, attempt to restore critical components
            try
            {
                CarControlScript carController = agent.GetComponent<CarControlScript>();
                if (carController != null) carController.enabled = true;
            }
            catch { }
        }
    }

    // Reposition a single agent at the start position with offset
    // In TrainingManager.cs, modify RepositionAgent:
    //private void RepositionAgent(CarAgent agent, int index, bool randomize = false)
    //{
    //    if (agent == null || agent.transform == null)
    //        return;

    //    // Get the car controller component
    //    CarControlScript carController = agent.GetComponent<CarControlScript>();

    //    if (carController != null)
    //    {
    //        // Just use the car controller's reset method - this is much simpler
    //        carController.ResetPosition();

    //        if (isDebugLoggingEnabled)
    //            Debug.Log($"[REPOSITION] Agent {agent.gameObject.name} reset using car controller to rotation {agent.transform.rotation.eulerAngles}");

    //        // Optional: Apply additional randomization if needed
    //        if (randomize && startPositionRandomization > 0)
    //        {
    //            // Wait a frame for the reset to fully apply
    //            //TODO: This may not be necessary
    //        }
    //    }
    //    else
    //    {
    //        Debug.LogError($"Cannot reposition agent {agent.gameObject.name}: CarControlScript not found");
    //    }
    //}

    private void UpdateAndReportProgress()
    {
        reportingCycle++;

        if (Time.time < lastProgressReportTime + progressReportInterval)
            return; // Only report at intervals

        lastProgressReportTime = Time.time;

        // Calculate progress based on multiple factors
        float checkpointProgress = CalculateCheckpointProgress();
        float lapProgress = CalculateLapProgress();
        float reliabilityProgress = CalculateReliabilityProgress();

        // Combine different progress metrics with weights
        curriculumProgress = (checkpointProgress * 0.35f) +
                            (lapProgress * 0.35f) +
                            (reliabilityProgress * 0.3f);

        // Clamp to valid range
        curriculumProgress = Mathf.Clamp01(curriculumProgress);

        // Report progress to ML-Agents for curriculum advancement
        Academy.Instance.StatsRecorder.Add("CarAgent?team=0:progress", curriculumProgress);
        //Debug.LogWarning($"[PROGRESS] Current curriculum progress: {curriculumProgress:F3} " +
        //             $"(Checkpoints: {checkpointProgress:F2}, Laps: {lapProgress:F2}, " +
        //             $"Reliability: {reliabilityProgress:F2}");
        if (isDebugLoggingEnabled)
        {
            Debug.Log($"[PROGRESS] Current curriculum progress: {curriculumProgress:F3} " +
                     $"(Checkpoints: {checkpointProgress:F2}, Laps: {lapProgress:F2}, " +
                     $"Reliability: {reliabilityProgress:F2}");
        }

        if (Time.time >= lastDetailedProgressTime + DETAILED_PROGRESS_INTERVAL)
        {
            LogDetailedProgress();
            lastDetailedProgressTime = Time.time;
        }

    }
    private void LogDetailedProgress()
    {
        // Calculate all progress metrics
        float checkpointProgress = CalculateCheckpointProgress();
        float lapProgress = CalculateLapProgress();
        float reliabilityProgress = CalculateReliabilityProgress();

        // Format timestamp
        System.DateTime now = System.DateTime.Now;
        string timestamp = now.ToString("HH:mm:ss");

        // Build a detailed progress report
        System.Text.StringBuilder report = new System.Text.StringBuilder();
        report.AppendLine("==================================================");
        report.AppendLine($"[TRAINING PROGRESS REPORT] - {timestamp}");
        report.AppendLine("==================================================");
        report.AppendLine($"Training Duration: {Time.time / 60f:F1} minutes");
        report.AppendLine($"Global Episodes: {globalEpisodeCounter}");
        report.AppendLine($"Current Episode: {currentEpisode}");
        report.AppendLine($"Total Laps: {totalLapsCompleted}");
        report.AppendLine($"Best Lap Time: {(bestLapTime < float.MaxValue ? bestLapTime.ToString("F2") + "s" : "None")}");
        report.AppendLine($"Total Checkpoints: {totalCheckpointsAcrossAgents}");
        report.AppendLine($"Checkpoint Progress: {checkpointProgress}");
        report.AppendLine($"Lap Progress: {lapProgress}");
        report.AppendLine($"Reliability Progress: {reliabilityProgress}");
        report.AppendLine($"Overall Curriculum Progress: {curriculumProgress}");
        report.AppendLine($"Collisions: {totalCollisionsAcrossAgents}");
        report.AppendLine($"Collision Reset Probability: {collisionResetProbability}");
        report.AppendLine($"Rebuld time: {episodesBeforeRebuild}");
        report.AppendLine("==================================================");

        // Output to console - using LogWarning to make it stand out in the console
        Debug.LogWarning(report.ToString());

        // You could also write this to a file for later analysis
        // System.IO.File.AppendAllText("training_progress.log", report.ToString() + "\n");
    }

    // Calculate progress based on checkpoint completions
    private float CalculateCheckpointProgress()
    {
        if (totalActiveAgents == 0) return 0f;

        // Calculate average checkpoints per agent
        float avgCheckpoints = totalCheckpointsAcrossAgents / totalActiveAgents;


        return Mathf.Min(1f, avgCheckpoints / ((float)totalLaps * 49) * (float)numberOfAgents);
    }

    // Calculate progress based on lap completions
    private float CalculateLapProgress()
    {
        if (totalActiveAgents == 0) return 0f;

        // Calculate average laps per agent
        float avgLaps = totalLapsAcrossAgents / totalActiveAgents;

        return Mathf.Min(1f, avgLaps / (float)totalLaps * (float)numberOfAgents);
    }

    // Calculate progress based on reliability (avoiding collisions)
    private float CalculateReliabilityProgress()
    {
        if (totalActiveAgents == 0 || globalEpisodeCounter == 0) return 0f;

        // Calculate collision rate
        float collisionRate = totalCollisionsAcrossAgents / (totalActiveAgents * currentEpisode);

        // Invert to get reliability (lower collision rate = higher reliability)
        float reliability = 1f - Mathf.Min(1f, collisionRate);

        return reliability;
    }

    // Add these methods to update the metrics
    public void RecordCheckpointCompletion(CarAgent agent)
    {
        totalCheckpointsAcrossAgents++;
    }

    public void RecordCollision(CarAgent agent)
    {
        totalCollisionsAcrossAgents++;
    }

    // SECTION: Trainer API and Utility Methods

    // Get the track handler for agents to reference
    public TrackHandler GetTrackHandler()
    {
        return trackHandler;
    }

    // Get a reference to this manager through the singleton
    public static TrainingManager GetInstance()
    {
        return Instance;
    }

    void OnEnable()
    {
        // Register for academy step events
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
        // Clean up events
        if (Academy.Instance != null)
        {
            Academy.Instance.AgentPreStep -= CountEpisodes;
        }
    }
}