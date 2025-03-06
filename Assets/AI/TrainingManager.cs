using Assets.TrackGeneration;
using System.Collections.Generic;
using System.Collections;
using Unity.MLAgents;
using UnityEngine;
using System;
using Unity.MLAgents.Actuators;
using System.Linq;

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

    // SECTION: Training Management and Curriculum

    [Header("Training Cycle")]
    public int episodesBeforeRebuild = 500;
    public float trackRebuildDelay = 1f;
    public float waitDelayBetweenEpisodes = 0.5f;
    public float maxWaitTimeForSynchronization = 20f;

    [Header("Training Curriculum")]
    [Tooltip("Start with strict resets on collision, transition to penalties only for recovery skills")]
    public bool useStrictResets = true;
    [Tooltip("How many episodes before switching to penalty-only mode")]
    public int episodesToSwitchCollisionMode = 200000;
    [Tooltip("Enable/disable randomization of starting positions")]
    public bool randomizeStartPositions = true;
    [Tooltip("Maximum offset for randomized starting positions")]
    public float startPositionRandomization = 1.0f;
    [Tooltip("Occasionally apply random forces to teach recovery (0-1 chance)")]
    [Range(0, 1)]
    public float randomPerturbationChance = 0f;
    [Tooltip("Strength of random perturbations")]
    public float perturbationStrength = 0f;

    [Header("Enhanced Training Settings")]
    [Range(1, 100)]
    public int episodesToIncreaseTimeout = 50;
    public float initialTimeWithoutCheckpoint = 15f;
    public float maxTimeWithoutCheckpoint = 60f;
    [Range(0, 3)]
    public float initialStartPositionRandomization = 0.2f;
    [Range(0, 10)]
    public float maxSteeringAngleChange = 5.0f;
    public float minSpeed = 5.0f;

    // Training progression tracking
    private int totalEpisodesCompleted = 0;
    private int totalLapsCompleted = 0;
    private float bestLapTime = float.MaxValue;

    // Add this variable to track episode stats
    private List<float> episodeLengths = new List<float>();
    private List<float> episodeRewards = new List<float>();
    private float previousEpisodeEndTime = 0f;
    private int previousEpisodeCheckpoints = 0;

    // Training State Management - using an enum for clear state tracking
    public enum TrainingState
    {
        NormalTraining,    // Regular training with no pending resets
        ResetRequested,    // Reset has been requested, waiting for agents
        ResetInProgress,   // Reset is actively happening
        RebuildRequested,  // Track rebuild has been requested, waiting for agents
        RebuildInProgress  // Track rebuild is actively happening
    }

    // The current training state - single source of truth
    private TrainingState currentState = TrainingState.NormalTraining;

    // Episode tracking
    private int currentEpisode = 0;
    private int lastRebuildEpisode = 0;

    // Synchronization tracking
    private int agentsReadyForReset = 0;
    private int totalActiveAgents = 0;
    private float stateChangeTime = 0f;

    // Events for agents to subscribe to
    public delegate void TrackRebuildEvent();
    public event TrackRebuildEvent OnTrackRebuilt;

    public delegate void EpisodeResetEvent();
    public event EpisodeResetEvent OnEpisodeReset;

    // Performance tracking
    private int frameCount = 0;
    private float nextLogTime = 0;
    private float nextCurriculumCheckTime = 0;

    // Debug logging toggle
    private bool isDebugLoggingEnabled = true;

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
        var episodesParam = Academy.Instance.EnvironmentParameters.GetWithDefault("track_rebuild_episodes", episodesBeforeRebuild);
        episodesBeforeRebuild = (int)episodesParam;

        // Set initial curriculum values from EnvironmentParameters if available
        useStrictResets = Academy.Instance.EnvironmentParameters.GetWithDefault("use_strict_resets", useStrictResets ? 1.0f : 0.0f) > 0.5f;
        episodesToSwitchCollisionMode = (int)Academy.Instance.EnvironmentParameters.GetWithDefault("episodes_to_switch_collision", episodesToSwitchCollisionMode);
        randomizeStartPositions = Academy.Instance.EnvironmentParameters.GetWithDefault("randomize_starts", randomizeStartPositions ? 1.0f : 0.0f) > 0.5f;
        startPositionRandomization = Academy.Instance.EnvironmentParameters.GetWithDefault("start_randomization", startPositionRandomization);
        randomPerturbationChance = Academy.Instance.EnvironmentParameters.GetWithDefault("perturbation_chance", randomPerturbationChance);

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
                Debug.Log($"[STATS] FPS: {fps:F1} | Total Episodes: {totalEpisodesCompleted} | " +
                         $"Current Episode: {currentEpisode} | Laps: {totalLapsCompleted} | " +
                         $"Best Lap: {(bestLapTime < float.MaxValue ? bestLapTime.ToString("F2") + "s" : "None")}");
            }
        }

        // Check for curriculum progression
        if (Time.time > nextCurriculumCheckTime)
        {
            UpdateCurriculum();
            nextCurriculumCheckTime = Time.time + 30f; // Check every 30 seconds
        }

        // Handle state timeouts
        if (currentState == TrainingState.ResetRequested || currentState == TrainingState.RebuildRequested)
        {
            float timeInCurrentState = Time.time - stateChangeTime;
            if (timeInCurrentState > maxWaitTimeForSynchronization)
            {
                Debug.LogWarning($"[SYNC] Timeout reached waiting for agents. Forcing state transition. Ready: {agentsReadyForReset}/{totalActiveAgents}");
                if (currentState == TrainingState.ResetRequested)
                {
                    StartCoroutine(PerformEpisodeReset());
                }
                else if (currentState == TrainingState.RebuildRequested)
                {
                    StartCoroutine(PerformTrackRebuild());
                }
            }
        }
    }

    // Update curriculum based on training progress
    private void UpdateCurriculum()
    {
        // Switch from strict resets to penalty-only mode after threshold
        if (useStrictResets && totalEpisodesCompleted > episodesToSwitchCollisionMode)
        {
            useStrictResets = false;
            Debug.Log($"[CURRICULUM] Switching to penalty-only mode for collisions after {totalEpisodesCompleted} episodes");
        }

        // Gradually increase randomness as training progresses
        float progressFactor = Mathf.Min(1.0f, totalEpisodesCompleted / (float)(episodesToSwitchCollisionMode * 2));
        startPositionRandomization = Mathf.Lerp(initialStartPositionRandomization, 2.0f, progressFactor);
        randomPerturbationChance = Mathf.Lerp(0.01f, 0.05f, progressFactor);

        // Update timeout based on progress
        float timeoutProgressFactor = Mathf.Min(1.0f, totalEpisodesCompleted / (float)episodesToIncreaseTimeout);
        float currentTimeout = Mathf.Lerp(initialTimeWithoutCheckpoint, maxTimeWithoutCheckpoint, timeoutProgressFactor);

        // Apply the timeout to all agents
        foreach (var agent in agents)
        {
            if (agent != null)
            {
                agent.maxTimeWithoutCheckpoint = currentTimeout;
            }
        }

        // Log curriculum updates
        if (isDebugLoggingEnabled)
        {
            Debug.Log($"[CURRICULUM] Updated parameters: Start randomization = {startPositionRandomization:F2}, " +
                     $"Perturbation chance = {randomPerturbationChance:F2}, " +
                     $"Checkpoint timeout = {currentTimeout:F2}s");
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
            trackHandler.GenerateTrack(GenerationStrategy.GridWithNoise);
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
                sensorSystem = carObject.AddComponent<TrackSensingSystem>();
                sensorSystem.raycastOrigin = carObject.transform;
                sensorSystem.rayCount = 9;
                sensorSystem.rayAngle = 180f;
                sensorSystem.sensorLength = 20f;
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

            // Apply optimized parameter configuration
            ConfigureOptimizedAgentParameters(carAgent);

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

    // Configure optimized agent parameters
    private void ConfigureOptimizedAgentParameters(CarAgent agent)
    {
        if (agent == null) return;

        // Set reasonable timeouts based on current curriculum stage
        float progressFactor = Mathf.Min(1.0f, totalEpisodesCompleted / (float)episodesToIncreaseTimeout);
        agent.maxTimeWithoutCheckpoint = Mathf.Lerp(initialTimeWithoutCheckpoint, maxTimeWithoutCheckpoint, progressFactor);

        // Start with more lenient no-progress detection and gradually tighten
        agent.noProgressThreshold = Mathf.Lerp(1.0f, 0.3f, progressFactor);
        agent.noProgressThresholdTime = Mathf.Lerp(10.0f, 5.0f, progressFactor);

        // Adjust reward weightings
        agent.speedReward = Mathf.Lerp(0.03f, 0.05f, progressFactor);
        agent.checkpointReward = Mathf.Lerp(10.0f, 5.0f, progressFactor); // Start higher, then reduce
        agent.directionAlignmentReward = Mathf.Lerp(0.1f, 0.05f, progressFactor);

        // Configure car controller limits
        if (agent.carController != null)
        {
            // Prevent agents from changing steering angle too quickly
            if (agent.carController.GetType().GetField("maxSteeringAngleChange") != null)
            {
                agent.carController.GetType().GetField("maxSteeringAngleChange").SetValue(agent.carController, maxSteeringAngleChange);
            }

            // Ensure a minimum speed to prevent stopping
            if (agent.carController.GetType().GetField("minSpeedThreshold") != null)
            {
                agent.carController.GetType().GetField("minSpeedThreshold").SetValue(agent.carController, minSpeed);
            }
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

    // Called from Academy steps to check for rebuild conditions
    private void CountEpisodes(int academyStepCount)
    {
        // Only check if we're in normal training state
        if (currentState != TrainingState.NormalTraining)
            return;

        // Check if it's time for a rebuild
        if (currentEpisode > 0 && currentEpisode >= episodesBeforeRebuild && currentEpisode > lastRebuildEpisode)
        {
            if (isDebugLoggingEnabled) Debug.Log($"[TRAIN] Episode threshold reached: {currentEpisode}/{episodesBeforeRebuild} - Requesting track rebuild");
            RequestTrackRebuild();
        }
    }

    // Increment the episode counter - only called after a successful reset
    public void IncrementEpisode()
    {
        // Only increment if we're in normal training state
        if (currentState != TrainingState.NormalTraining)
            return;

        currentEpisode++;
        totalEpisodesCompleted++;

        // Log progress occasionally
        if (currentEpisode % 10 == 0 || isDebugLoggingEnabled)
        {
            Debug.Log($"[TRAIN] Episode {currentEpisode}/{episodesBeforeRebuild} completed (Total: {totalEpisodesCompleted})");
        }
    }

    // Add this method to record episode stats
    public void RecordEpisodeCompletion(float totalReward, int checkpointsPassed)
    {
        float episodeLength = Time.time - previousEpisodeEndTime;
        previousEpisodeEndTime = Time.time;

        // Only store meaningful episode data
        if (episodeLength > 1.0f)
        {
            episodeLengths.Add(episodeLength);
            episodeRewards.Add(totalReward);

            // Keep lists from growing too large
            if (episodeLengths.Count > 100)
            {
                episodeLengths.RemoveAt(0);
                episodeRewards.RemoveAt(0);
            }
        }

        // Track improvement in checkpoints passed
        int checkpointDelta = checkpointsPassed - previousEpisodeCheckpoints;
        previousEpisodeCheckpoints = checkpointsPassed;

        // Log every 10 episodes
        if (totalEpisodesCompleted % 10 == 0)
        {
            // Calculate averages
            float avgLength = 0f;
            float avgReward = 0f;

            if (episodeLengths.Count > 0)
            {
                avgLength = episodeLengths.Average();
                avgReward = episodeRewards.Average();
            }

            Debug.Log($"[TRAIN STATS] Avg Duration: {avgLength:F1}s, " +
                     $"Avg Reward: {avgReward:F1}, " +
                     $"Checkpoints: {checkpointsPassed} ({(checkpointDelta >= 0 ? "+" : "")}{checkpointDelta})");
        }
    }

    // Record a lap completion
    public void RecordLapCompletion(float lapTime)
    {
        totalLapsCompleted++;

        // Update best lap time
        if (lapTime < bestLapTime)
        {
            bestLapTime = lapTime;
            Debug.Log($"[TRAIN] New best lap time: {bestLapTime:F2}s");
        }
    }

    // Request a regular episode reset (non-rebuild)
    public void RequestEpisodeReset()
    {
        // Only allow requests in normal training state
        if (currentState != TrainingState.NormalTraining)
        {
            if (isDebugLoggingEnabled) Debug.Log($"[WARN] Episode reset requested while in {currentState} state - ignoring");
            return;
        }

        if (isDebugLoggingEnabled) Debug.Log($"[SYNC] Episode reset requested");

        // Change state and reset counters
        ChangeState(TrainingState.ResetRequested);
        agentsReadyForReset = 0;

        // Notify all agents
        foreach (var agent in agents)
        {
            if (agent != null && agent.gameObject.activeSelf)
            {
                agent.RequestFinishForReset(false);
            }
        }
    }

    // Request a track rebuild
    public void RequestTrackRebuild()
    {
        // Only allow requests in normal training state
        if (currentState != TrainingState.NormalTraining)
        {
            if (isDebugLoggingEnabled) Debug.Log($"[WARN] Track rebuild requested while in {currentState} state - ignoring");
            return;
        }

        if (isDebugLoggingEnabled) Debug.Log($"[REBUILD] Track rebuild requested at episode {currentEpisode}");

        // Change state and reset counters
        ChangeState(TrainingState.RebuildRequested);
        agentsReadyForReset = 0;

        // Notify all agents
        foreach (var agent in agents)
        {
            if (agent != null && agent.gameObject.activeSelf)
            {
                agent.RequestFinishForReset(true);
            }
        }
    }

    // Agent calls this to signal it's ready for reset
    public void NotifyAgentReadyForReset(bool forRebuild)
    {
        // Make sure we're in an appropriate state to accept notifications
        if (currentState != TrainingState.ResetRequested && currentState != TrainingState.RebuildRequested)
        {
            Debug.LogWarning($"[WARN] Agent notified ready for reset while in {currentState} state - ignoring");
            return;
        }

        // Check for state/param mismatch
        if ((currentState == TrainingState.ResetRequested && forRebuild) ||
            (currentState == TrainingState.RebuildRequested && !forRebuild))
        {
            Debug.LogWarning($"[WARN] Agent ready parameter ({forRebuild}) doesn't match current state ({currentState})");
        }

        // Increment the counter
        agentsReadyForReset++;

        if (isDebugLoggingEnabled && (agentsReadyForReset == 1 || agentsReadyForReset == totalActiveAgents))
        {
            Debug.Log($"[SYNC] {agentsReadyForReset}/{totalActiveAgents} agents ready for {(forRebuild ? "rebuild" : "reset")}");
        }

        // Check if all agents are ready
        if (agentsReadyForReset >= totalActiveAgents)
        {
            if (isDebugLoggingEnabled) Debug.Log($"[SYNC] All agents ready - proceeding with {(currentState == TrainingState.RebuildRequested ? "track rebuild" : "episode reset")}");

            // Start the appropriate action
            if (currentState == TrainingState.ResetRequested)
            {
                StartCoroutine(PerformEpisodeReset());
            }
            else if (currentState == TrainingState.RebuildRequested)
            {
                StartCoroutine(PerformTrackRebuild());
            }
        }
    }

    // Get whether we're currently using strict (reset) collision mode
    public bool IsUsingStrictResets()
    {
        return useStrictResets;
    }

    // Maybe apply a random perturbation based on curriculum
    public void MaybeApplyRandomPerturbation(CarAgent agent)
    {
        // Only apply in non-strict mode and based on random chance
        if (!useStrictResets && UnityEngine.Random.value < randomPerturbationChance)
        {
            Rigidbody rb = agent.GetComponent<Rigidbody>();
            if (rb != null)
            {
                // Random direction slightly biased toward lateral forces
                Vector3 direction = new Vector3(
                    UnityEngine.Random.Range(-1f, 1f),
                    0,
                    UnityEngine.Random.Range(-0.2f, 0.2f)
                ).normalized;

                // Apply force at agent's position
                rb.AddForce(direction * perturbationStrength, ForceMode.Impulse);

                if (isDebugLoggingEnabled) Debug.Log($"[TRAIN] Applied random perturbation to {agent.gameObject.name}");
            }
        }
    }

    // Safely change the training state
    private void ChangeState(TrainingState newState)
    {
        if (isDebugLoggingEnabled) Debug.Log($"[STATE] {currentState} -> {newState}");
        currentState = newState;
        stateChangeTime = Time.time;
    }

    // SECTION: Training Actions Implementation

    // Perform a standard episode reset
    private IEnumerator PerformEpisodeReset()
    {
        // Update state 
        ChangeState(TrainingState.ResetInProgress);

        // STOP THE RACE
        if (raceManager != null)
        {
            raceManager.EndRace();
            if (isDebugLoggingEnabled) Debug.Log("[RESET] Race ended");
        }

        // Wait for physics to stabilize
        yield return new WaitForSeconds(waitDelayBetweenEpisodes);

        // Reset all agents in a staggered fashion
        for (int i = 0; i < agents.Count; i++)
        {
            var agent = agents[i];
            if (agent != null && agent.gameObject.activeSelf)
            {
                // Reposition the agent with optional randomization
                RepositionAgent(agent, i, randomizeStartPositions);

                // Wait every 3 agents
                if (i % 3 == 2) yield return new WaitForFixedUpdate();
            }
        }

        // Wait for physics to settle
        yield return new WaitForFixedUpdate();
        yield return new WaitForFixedUpdate();

        if (raceManager != null)
        {
            raceManager.InitiateRaceStart();
            if (isDebugLoggingEnabled) Debug.Log("[RESET] Race restarted");
        }

        // Trigger the reset event for all agents to respond to
        if (isDebugLoggingEnabled) Debug.Log($"[SYNC] Triggering OnEpisodeReset event for {agents.Count} agents");
        OnEpisodeReset?.Invoke();

        // Reset counters
        agentsReadyForReset = 0;

        // Return to normal training state
        ChangeState(TrainingState.NormalTraining);

        // Increment the episode counter
        IncrementEpisode();
    }

    // Perform a track rebuild and reset
    private IEnumerator PerformTrackRebuild()
    {
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

        // Disable agents during rebuilding if configured
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
        if (isDebugLoggingEnabled) Debug.Log($"[REBUILD] Notifying {agents.Count} agents about track rebuild");
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
                    agent.EndEpisode();
                }
            }
        }

        // Reset episode counter and update last rebuild episode
        currentEpisode = 0;
        lastRebuildEpisode = 0;

        if (raceManager != null)
        {
            raceManager.InitiateRaceStart();
            if (isDebugLoggingEnabled) Debug.Log("[REBUILD] Race restarted");
        }

        // Return to normal training state
        ChangeState(TrainingState.NormalTraining);

        if (isDebugLoggingEnabled) Debug.Log("[REBUILD] Track rebuild completed successfully");
    }

    // Reset agents in a staggered manner to reduce per-frame load
    private IEnumerator StaggeredAgentReset()
    {
        // Wait for physics to settle
        yield return new WaitForFixedUpdate();
        yield return new WaitForFixedUpdate();

        // Process agents in batches
        int batchSize = 3; // Process 3 agents per frame

        for (int batchStart = 0; batchStart < agents.Count; batchStart += batchSize)
        {
            int batchEnd = Mathf.Min(batchStart + batchSize, agents.Count);

            // First reposition all agents in this batch
            for (int i = batchStart; i < batchEnd; i++)
            {
                var agent = agents[i];
                if (agent != null)
                {
                    RepositionAgent(agent, i, randomizeStartPositions);
                }
            }

            yield return new WaitForFixedUpdate();

            // Then activate and reset them
            for (int i = batchStart; i < batchEnd; i++)
            {
                var agent = agents[i];
                if (agent != null)
                {
                    // Re-enable agent if it was disabled
                    if (disableAgentsDuringRebuild && !agent.gameObject.activeSelf)
                    {
                        agent.gameObject.SetActive(true);
                    }

                    agent.ResetForNewTrack();
                    agent.EndEpisode();
                }
            }

            yield return new WaitForFixedUpdate();
        }

        // Final wait to ensure all physics is settled
        yield return new WaitForFixedUpdate();
    }

    // Reposition a single agent at the start position with offset
    private void RepositionAgent(CarAgent agent, int index, bool randomize = false)
    {
        if (agent == null || agent.transform == null)
            return;

        try
        {
            // Get start position from track handler
            (Vector3 startPosition1, Vector3 startPosition2, Quaternion startRotation) = trackHandler.GetTrackStartTransform();

            // Calculate position with offset to prevent overlap
            Vector3 spawnPosition = startPosition1 + Vector3.right * (index % 3) * agentSpacing + Vector3.forward * (index / 3) * agentSpacing;

            // Add randomization if enabled
            if (randomize)
            {
                // Add small random offset to position
                spawnPosition += new Vector3(
                    UnityEngine.Random.Range(-startPositionRandomization, startPositionRandomization),
                    0,
                    UnityEngine.Random.Range(-startPositionRandomization, startPositionRandomization));

                // Add small random rotation
                startRotation *= Quaternion.Euler(0, UnityEngine.Random.Range(-5f, 5f), 0);
            }

            // Reset physics state completely
            Rigidbody rb = agent.GetComponent<Rigidbody>();
            if (rb != null)
            {
                // First make kinematic to reliably reposition
                rb.isKinematic = true;

                // Update transform
                agent.transform.position = spawnPosition;
                agent.transform.rotation = startRotation;

                // Now reset physics state
                rb.isKinematic = false;
                rb.linearVelocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
                rb.ResetInertiaTensor();

                // Put rigidbody to sleep for stability
                rb.Sleep();
            }
            else
            {
                // No rigidbody, just set transform
                agent.transform.position = spawnPosition;
                agent.transform.rotation = startRotation;
            }

            // Reset car controller if possible
            CarControlScript carController = agent.GetComponent<CarControlScript>();
            if (carController != null)
            {
                var resetMethod = carController.GetType().GetMethod("ResetVehicle");
                if (resetMethod != null)
                {
                    resetMethod.Invoke(carController, null);
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Error repositioning agent {index}: {e.Message}");
        }
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