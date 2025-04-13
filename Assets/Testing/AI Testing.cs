using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System;
using UnityEditor;
using Assets.TrackGeneration;
using Unity.MLAgents.Policies;
using Unity.Sentis;

/// <summary>
/// Manager for systematically evaluating AI agents across multiple procedurally generated tracks.
/// Collects and reports comprehensive performance metrics.
/// </summary>
public class AIEvaluationManager : MonoBehaviour
{
    // Singleton instance
    public static AIEvaluationManager Instance { get; private set; }

    [ContextMenu("Run Evaluation")]
    public void RunEvaluationButton()
    {
        StartEvaluation();
    }

    [Header("Test Configuration")]
    [Tooltip("Number of tracks to generate for testing")]
    public int numberOfTracksToTest = 10;

    [Tooltip("Maximum time to allow for each track test in seconds")]
    public float maxTestTimePerTrack = 180f;

    [Tooltip("Target number of laps to complete on each track")]
    public int targetLapsPerTrack = 3;

    [Tooltip("Track generation strategies to test")]
    public GenerationStrategy[] generationStrategiesToTest = new GenerationStrategy[]
    {
        GenerationStrategy.GridWithNoise,
        GenerationStrategy.CircularLayout
    };

    [Header("AI Configuration")]
    [Tooltip("Model assets to test (can test multiple models)")]
    public ModelAssetConfig[] modelAssetsToTest;

    [Tooltip("Number of agents to test per model")]
    public int agentsPerModel = 3;

    [Header("Dependencies")]
    public GameObject aiCarPrefab;
    public Material trackMaterial;
    public Material wallMaterial;
    public Material trackSupportMaterial;
    public PhysicsMaterial wallPhysicsMaterial;
    public GameObject startLinePrefab;
    public GameObject gridMarkerPrefab;

    [Header("Advanced Settings")]
    [Tooltip("Time to wait between tests")]
    public float timeBetweenTests = 2f;

    [Tooltip("Layer for AI agents")]
    public string aiLayerName = "Agent";

    [Tooltip("Enable detailed logging")]
    public bool enableDetailedLogging = true;

    [Tooltip("Run tests in headless mode (no visuals)")]
    public bool headlessMode = false;

    // Internal tracking
    private int currentTrackIndex = 0;
    private int currentModelIndex = 0;
    private bool isTestRunning = false;
    private List<TrackTestResult> testResults = new List<TrackTestResult>();
    private Dictionary<string, ModelTestSummary> modelSummaries = new Dictionary<string, ModelTestSummary>();

    // Track and agent references
    private TrackHandler trackHandler;
    private RaceManager raceManager;
    private List<GameObject> activeAgents = new List<GameObject>();
    private float testStartTime;
    private int aiLayer;
    private Coroutine testCoroutine;

    // Class to represent test configuration
    [System.Serializable]
    public class ModelAssetConfig
    {
        public string modelName;
        public ModelAsset modelAsset;
    }

    // Class to store detailed results for a single track test
    [System.Serializable]
    public class TrackTestResult
    {
        public string modelName;
        public int trackNumber;
        public GenerationStrategy trackGenerationStrategy;
        public int agentId;
        public bool completedAllLaps;
        public int lapsCompleted;
        public float bestLapTime;
        public float averageLapTime;
        public float totalTestDuration;
        public int collisionCount;
        public float averageSpeed;
        public float maxSpeed;
        public float timeAtMaxSpeed;
        public int checkpointsPassed;
        public float trackLength;
        public int recoveryCount;
        public float timeSpentRecovering;
        public List<float> individualLapTimes = new List<float>();
        public List<CollisionEvent> collisions = new List<CollisionEvent>();
    }

    // Class to track collision details
    [System.Serializable]
    public class CollisionEvent
    {
        public float timeIntoTest;
        public Vector3 position;
        public float speed;
        public float recoveryTime;
    }

    // Class to aggregate metrics across all tracks for a model
    [System.Serializable]
    public class ModelTestSummary
    {
        public string modelName;
        public int tracksTested;
        public int tracksCompletedFully;
        public float completionRate;
        public float averageLapTime;
        public float bestLapTime;
        public float averageCollisionsPerTrack;
        public float averageSpeed;
        public float averageMaxSpeed;
        public float averageCompletionTime;
        public int totalCheckpointsPassed;
        public float averageRecoveryTime;
        public int totalRecoveryCount;
    }

    // Agent performance tracker component attached to each agent during testing
    private class AgentPerformanceTracker : MonoBehaviour
    {
        public string modelName;
        public int agentId;
        public int collisionCount;
        public float maxSpeed;
        public float totalSpeed;
        public int speedSamples;
        public float timeAtMaxSpeed;
        public List<float> lapTimes = new List<float>();
        public List<CollisionEvent> collisions = new List<CollisionEvent>();
        public int checkpointsPassed;
        public int recoveryCount;
        public float recoveryStartTime;
        public float timeSpentRecovering;
        public bool isRecovering;
        public float testStartTime;

        private CarControlScript carController;
        private float lastLapStartTime;
        private float recoveryThreshold = 3f; // Time threshold to consider agent in recovery mode
        private Vector3 lastPosition;
        private float positionCheckInterval = 0.5f;
        private float lastPositionCheckTime;
        private float stuckThreshold = 1.0f;
        private AIEvaluationManager evaluator;

        private void Start()
        {
            carController = GetComponent<CarControlScript>();
            lastLapStartTime = Time.time;
            testStartTime = Time.time;
            lastPosition = transform.position;
            lastPositionCheckTime = Time.time;

            evaluator = GetComponent<AIEvaluationManager>();
        }

        private void Update()
        {
            // Sample speed
            if (carController != null)
            {
                float currentSpeed = carController.GetCurrentSpeed();
                totalSpeed += currentSpeed;
                speedSamples++;

                if (currentSpeed > maxSpeed)
                {
                    maxSpeed = currentSpeed;
                }

                if (currentSpeed > maxSpeed * 0.9f)
                {
                    timeAtMaxSpeed += Time.deltaTime;
                }

                // Check if agent is stuck or making progress
                if (Time.time - lastPositionCheckTime > positionCheckInterval)
                {
                    float distance = Vector3.Distance(transform.position, lastPosition);

                    // If agent hasn't moved significantly and is not already considered recovering
                    if (distance < stuckThreshold && !isRecovering)
                    {
                        isRecovering = true;
                        recoveryStartTime = Time.time;
                        recoveryCount++;
                    }
                    // If agent has moved significantly and was in recovery
                    else if (distance >= stuckThreshold && isRecovering)
                    {
                        isRecovering = false;
                        timeSpentRecovering += (Time.time - recoveryStartTime);
                    }

                    lastPosition = transform.position;
                    lastPositionCheckTime = Time.time;
                }

                // Continue tracking recovery time if still recovering
                if (isRecovering && Time.time - recoveryStartTime > recoveryThreshold)
                {
                    // Trigger reset if stuck for too long
                    if (evaluator && Time.time - recoveryStartTime > 10f)
                    {
                        // Optionally call a reset method
                        ResetAgent();
                    }
                }
            }
        }

        public void RegisterCollision(Collision collision)
        {
            collisionCount++;

            // Record collision details
            CollisionEvent collisionEvent = new CollisionEvent
            {
                timeIntoTest = Time.time - testStartTime,
                position = transform.position,
                speed = carController ? carController.GetCurrentSpeed() : 0f,
                recoveryTime = 0f // Will be updated when recovery ends
            };

            collisions.Add(collisionEvent);

            // Start recovery tracking
            isRecovering = true;
            recoveryStartTime = Time.time;
            recoveryCount++;
        }

        public void RegisterCheckpointPassed()
        {
            checkpointsPassed++;
        }

        public void RegisterLapCompleted(float lapTime)
        {
            lapTimes.Add(lapTime);
            lastLapStartTime = Time.time;
        }

        public float GetAverageSpeed()
        {
            return speedSamples > 0 ? totalSpeed / speedSamples : 0f;
        }

        private void ResetAgent()
        {
            if (carController != null)
            {
                // End current recovery session
                if (isRecovering)
                {
                    timeSpentRecovering += (Time.time - recoveryStartTime);
                    isRecovering = false;
                }

                carController.ResetPosition();
                recoveryCount++;
            }
        }
    }

    // Helper component to detect collisions for AI agents
    private class CollisionDetector : MonoBehaviour
    {
        public AgentPerformanceTracker tracker;

        private void OnCollisionEnter(Collision collision)
        {
            if (collision.gameObject.CompareTag("Wall"))
            {
                if (tracker != null)
                {
                    tracker.RegisterCollision(collision);
                }
            }
        }
    }

    void Awake()
    {
        // Set up singleton instance
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject);
        }
        else if (Instance != this)
        {
            Destroy(gameObject);
            return;
        }
    }

    void Start()
    {
        // Get AI layer
        aiLayer = LayerMask.NameToLayer(aiLayerName);
        if (aiLayer == -1)
            Debug.LogWarning($"Layer '{aiLayerName}' not found! Using default layer.");

        Log("AI Evaluation Manager initialized. Use Editor menu to start tests.");
    }

    /// <summary>
    /// Starts a single track test with all configured models
    /// </summary>
    public void StartSingleTest()
    {
        if (isTestRunning)
        {
            Log("Test already running. Stop current test first.");
            return;
        }

        // Validate configuration
        if (!ValidateConfiguration())
            return;

        // Initialize tracking variables for a single test
        testResults = new List<TrackTestResult>();
        modelSummaries.Clear();
        currentTrackIndex = 0;
        currentModelIndex = 0;
        isTestRunning = true;

        // Create initial model summaries
        foreach (var modelConfig in modelAssetsToTest)
        {
            modelSummaries[modelConfig.modelName] = new ModelTestSummary
            {
                modelName = modelConfig.modelName,
                tracksTested = 0,
                bestLapTime = float.MaxValue
            };
        }

        // Start a single test
        testCoroutine = StartCoroutine(RunSingleTest());
    }

    /// <summary>
    /// Starts the full AI evaluation process across multiple tracks
    /// </summary>
    public void StartEvaluation()
    {
        if (isTestRunning)
        {
            Log("Test already running. Stop current test first.");
            return;
        }

        // Validate configuration
        if (!ValidateConfiguration())
            return;

        // Initialize tracking variables
        testResults = new List<TrackTestResult>();
        modelSummaries.Clear();
        currentTrackIndex = 0;
        currentModelIndex = 0;
        isTestRunning = true;

        // Create initial model summaries
        foreach (var modelConfig in modelAssetsToTest)
        {
            modelSummaries[modelConfig.modelName] = new ModelTestSummary
            {
                modelName = modelConfig.modelName,
                tracksTested = 0,
                bestLapTime = float.MaxValue
            };
        }

        // Start the test sequence
        testCoroutine = StartCoroutine(RunEvaluationSequence());
    }

    /// <summary>
    /// Stops the current evaluation
    /// </summary>
    public void StopEvaluation()
    {
        if (!isTestRunning)
            return;

        if (testCoroutine != null)
        {
            StopCoroutine(testCoroutine);
            testCoroutine = null;
        }

        CleanupScene();
        isTestRunning = false;

        Log("Evaluation stopped.");
    }

    private bool ValidateConfiguration()
    {
        if (modelAssetsToTest == null || modelAssetsToTest.Length == 0)
        {
            Log("Error: No models specified for testing!", true);
            return false;
        }

        if (aiCarPrefab == null)
        {
            Log("Error: AI car prefab not assigned!", true);
            return false;
        }

        return true;
    }

    private IEnumerator RunSingleTest()
    {
        Log("Starting single test evaluation...");

        // Generate one track
        GenerationStrategy strategy = generationStrategiesToTest[0];
        Log($"Generating track using {strategy}...");
        yield return SetupTrack(strategy);

        // For each model
        for (int modelIdx = 0; modelIdx < modelAssetsToTest.Length; modelIdx++)
        {
            currentModelIndex = modelIdx;
            var modelConfig = modelAssetsToTest[modelIdx];

            Log($"Testing model: {modelConfig.modelName}...");
            yield return TestModelOnCurrentTrack(modelConfig, 0);

            // Wait between tests
            yield return new WaitForSeconds(timeBetweenTests);
        }

        // Complete the evaluation
        FinalizeEvaluation();
        isTestRunning = false;

        Log("Single test evaluation complete!");
    }

    private IEnumerator RunEvaluationSequence()
    {
        Log("Starting full evaluation sequence...");

        // For each track
        for (int trackIdx = 0; trackIdx < numberOfTracksToTest; trackIdx++)
        {
            currentTrackIndex = trackIdx;
            GenerationStrategy strategy = GetGenerationStrategy(trackIdx);

            Log($"Generating track {trackIdx + 1}/{numberOfTracksToTest} using {strategy}...");
            yield return SetupTrack(strategy);

            // For each model
            for (int modelIdx = 0; modelIdx < modelAssetsToTest.Length; modelIdx++)
            {
                currentModelIndex = modelIdx;
                var modelConfig = modelAssetsToTest[modelIdx];

                Log($"Testing model: {modelConfig.modelName} on track {trackIdx + 1}...");
                yield return TestModelOnCurrentTrack(modelConfig, trackIdx);

                // Wait between tests
                yield return new WaitForSeconds(timeBetweenTests);
            }
        }

        // Complete the evaluation
        FinalizeEvaluation();
        isTestRunning = false;

        Log("Full evaluation complete! Use 'Export Results' to save detailed analysis.");
    }

    private GenerationStrategy GetGenerationStrategy(int trackIndex)
    {
        if (generationStrategiesToTest.Length == 0)
            return GenerationStrategy.GridWithNoise;

        return generationStrategiesToTest[trackIndex % generationStrategiesToTest.Length];
    }

    private IEnumerator SetupTrack(GenerationStrategy strategy)
    {
        // Clear any existing scene elements
        CleanupScene();

        // Set up track system
        GameObject trackSystemObj = new GameObject("Track System");
        trackHandler = trackSystemObj.AddComponent<TrackHandler>();

        // Configure track parameters
        trackHandler.trackMaterial = trackMaterial;
        trackHandler.wallMaterial = wallMaterial;
        trackHandler.wallPhysicsMaterial = wallPhysicsMaterial;
        trackHandler.startLinePrefab = startLinePrefab;
        trackHandler.gridMarkerPrefab = gridMarkerPrefab;
        trackHandler.trackSupportMaterial = trackSupportMaterial;
       

        // Create and setup camera for track generation
        GameObject cameraObj = new GameObject("Track Camera");
        Camera trackCamera = cameraObj.AddComponent<Camera>();
        trackCamera.orthographic = true;
        trackCamera.orthographicSize = trackHandler.trackGenParam.CanvasSize / 2f;
        trackCamera.transform.position = Vector3.up * 800f;
        trackCamera.transform.rotation = Quaternion.Euler(90f, 0f, 0f);

        // Assign camera to track handler
        trackHandler.trackCamera = trackCamera;


        // Set up race manager
        GameObject raceManagerObj = new GameObject("Race Manager");
        raceManager = raceManagerObj.AddComponent<RaceManager>();
        raceManager.totalLaps = targetLapsPerTrack;
        raceManager.countdownTime = 0f; // Skip countdown for testing
        raceManager.skipCountdownForTraining = true;

        // Generate track with specified strategy
        trackHandler.GenerateTrack(strategy);
        yield return new WaitForSeconds(0.5f); // Allow time for track generation

        // Initialize race manager with checkpoints
        var checkpoints = trackHandler.GetCheckpoints();
        if (checkpoints != null && checkpoints.Count > 0)
        {
            raceManager.InitializeCheckpoints(checkpoints);
        }
        else
        {
            Log("No checkpoints found after track generation!", true);
        }

        yield return null;
    }

    private IEnumerator TestModelOnCurrentTrack(ModelAssetConfig modelConfig, int trackIndex)
    {
        // Spawn agents for this model
        SpawnAgents(modelConfig);

        // Record test start time
        testStartTime = Time.time;

        // Start the race
        if (raceManager != null)
        {
            raceManager.InitiateRaceStart();
        }

        // Calculate track length (approximate)
        float trackLength = CalculateTrackLength();

        // Wait for maximum test time or until all agents complete target laps
        float elapsedTime = 0f;
        bool allAgentsFinished = false;

        while (elapsedTime < maxTestTimePerTrack && !allAgentsFinished)
        {
            elapsedTime = Time.time - testStartTime;

            allAgentsFinished = CheckIfAllAgentsFinished();

            // Log progress periodically
            if (Mathf.FloorToInt(elapsedTime) % 10 == 0 && Mathf.FloorToInt(elapsedTime) != Mathf.FloorToInt(elapsedTime - Time.deltaTime))
            {
                Log($"Test progress: {elapsedTime:F1}s / {maxTestTimePerTrack}s - Model: {modelConfig.modelName}");
            }

            yield return null;
        }

        // Collect and store results for each agent
        CollectTestResults(modelConfig.modelName, trackIndex, trackLength);

        // Clean up agents
        CleanupAgents();

        yield return null;
    }

    private void SpawnAgents(ModelAssetConfig modelConfig)
    {
        if (trackHandler == null || aiCarPrefab == null || raceManager == null)
        {
            Log("Cannot spawn agents: Missing required components", true);
            return;
        }

        // Get start positions from track
        var (startPos1, startPos2, startRot) = trackHandler.GetTrackStartTransform();
        Vector3[] startPositions = new Vector3[agentsPerModel];

        // Calculate multiple start positions with offsets if needed
        for (int i = 0; i < agentsPerModel; i++)
        {
            // Alternate between positions or add offsets
            if (i % 2 == 0)
            {
                startPositions[i] = startPos1 + Vector3.right * (i * 1.5f);
            }
            else
            {
                startPositions[i] = startPos2 + Vector3.right * ((i - 1) * 1.5f);
            }
        }

        // Spawn each agent
        for (int i = 0; i < agentsPerModel; i++)
        {
            // Instantiate the car
            GameObject carObject = Instantiate(aiCarPrefab, startPositions[i], startRot);
            carObject.name = $"AI_{modelConfig.modelName}_{i}";
            carObject.tag = "AI";

            // Set layer
            if (aiLayer != -1)
            {
                SetLayerRecursively(carObject, aiLayer);
            }

            // Configure car controller
            CarControlScript carControl = carObject.GetComponent<CarControlScript>();
            if (carControl != null)
            {
                carControl.isPlayerControlled = false;
                carControl.trackHandler = trackHandler;
                carControl.enableVisualEffects = !headlessMode;
            }

            // Configure track sensing system
            TrackSensingSystem sensorSystem = carObject.GetComponent<TrackSensingSystem>();
            if (sensorSystem == null)
            {
                Transform carBodyTransform = carObject.transform.Find("CarBody");
                sensorSystem = carObject.AddComponent<TrackSensingSystem>();
                sensorSystem.raycastOrigin = carBodyTransform != null ? carBodyTransform : carObject.transform;
                sensorSystem.visualizeRays = false;
            }

            // Set up inference agent
            StartCoroutine(SetupAgentWithModel(carObject, modelConfig, i));

            // Add performance tracker
            AgentPerformanceTracker tracker = carObject.AddComponent<AgentPerformanceTracker>();
            tracker.modelName = modelConfig.modelName;
            tracker.agentId = i;
            tracker.testStartTime = testStartTime;

            // Register with race manager
            raceManager.RegisterAgent(carObject, $"Agent_{modelConfig.modelName}_{i}");

            // Subscribe to events
            raceManager.OnLapCompleted += (agent) => {
                if (agent == carObject)
                {
                    float lapTime = raceManager.GetPlayerCurrentLapTime(agent);
                    tracker.RegisterLapCompleted(lapTime);
                }
            };

            raceManager.OnCheckpointPassed += (agent, checkpointIndex, total) => {
                if (agent == carObject)
                {
                    tracker.RegisterCheckpointPassed();
                }
            };

            // Store reference to active agent
            activeAgents.Add(carObject);
        }
    }

    private IEnumerator SetupAgentWithModel(GameObject carObject, ModelAssetConfig modelConfig, int agentId)
    {
        // Wait to ensure components are initialized
        yield return null;
        yield return null;

        CarControlScript carControl = carObject.GetComponent<CarControlScript>();
        TrackSensingSystem sensorSystem = carObject.GetComponent<TrackSensingSystem>();

        // Add or get InferenceCarAgent
        InferenceCarAgent inferenceAgent = carObject.GetComponent<InferenceCarAgent>();
        if (inferenceAgent == null)
        {
            inferenceAgent = carObject.AddComponent<InferenceCarAgent>();
        }

        // Set up references
        inferenceAgent.carController = carControl;
        inferenceAgent.sensorSystem = sensorSystem;
        inferenceAgent.trackHandler = trackHandler;

        // Set up behavior parameters
        BehaviorParameters behaviorParams = carObject.GetComponent<BehaviorParameters>();
        if (behaviorParams == null)
        {
            behaviorParams = carObject.AddComponent<BehaviorParameters>();
        }

        // Configure observation space (must match training)
        int totalObservations = 40;
        behaviorParams.BrainParameters.VectorObservationSize = totalObservations;
        behaviorParams.BrainParameters.NumStackedVectorObservations = 1;

        // Configure action space
        var actionSpec = new Unity.MLAgents.Actuators.ActionSpec();
        actionSpec.NumContinuousActions = 3;
        actionSpec.BranchSizes = new int[] { 2 };
        behaviorParams.BrainParameters.ActionSpec = actionSpec;

        // Set model asset
        behaviorParams.Model = modelConfig.modelAsset;
        behaviorParams.BehaviorName = "CarAgent";
        behaviorParams.BehaviorType = BehaviorType.InferenceOnly;

        // Add decision requester
        Unity.MLAgents.DecisionRequester decisionRequester = carObject.GetComponent<Unity.MLAgents.DecisionRequester>();
        if (decisionRequester == null)
        {
            decisionRequester = carObject.AddComponent<Unity.MLAgents.DecisionRequester>();
            decisionRequester.DecisionPeriod = 5;
            decisionRequester.TakeActionsBetweenDecisions = true;
        }

        // Initialize agent
        inferenceAgent.InitializeAgent();

        // Add collision detection
        Rigidbody rb = carObject.GetComponent<Rigidbody>();
        if (rb != null)
        {
            // Already has a rigidbody
            CollisionDetector detector = carObject.AddComponent<CollisionDetector>();
            detector.tracker = carObject.GetComponent<AgentPerformanceTracker>();
        }
    }

    private bool CheckIfAllAgentsFinished()
    {
        if (raceManager == null || activeAgents.Count == 0)
            return false;

        int agentsFinished = 0;

        foreach (var agent in activeAgents)
        {
            if (agent != null)
            {
                int lapsCompleted = raceManager.GetPlayerCurrentLap(agent);
                if (lapsCompleted >= targetLapsPerTrack)
                {
                    agentsFinished++;
                }
            }
        }

        return agentsFinished == activeAgents.Count;
    }

    private void CollectTestResults(string modelName, int trackIndex, float trackLength)
    {
        GenerationStrategy strategy = GetGenerationStrategy(trackIndex);

        foreach (var agent in activeAgents)
        {
            if (agent == null)
                continue;

            var tracker = agent.GetComponent<AgentPerformanceTracker>();
            if (tracker == null)
                continue;

            // Get lap data from agent and race manager
            int lapsCompleted = raceManager.GetPlayerCurrentLap(agent);
            float bestLapTime = float.MaxValue;
            float averageLapTime = 0f;

            if (tracker.lapTimes.Count > 0)
            {
                bestLapTime = tracker.lapTimes.Min();
                averageLapTime = tracker.lapTimes.Average();
            }
            else
            {
                // No complete laps, use current lap time as estimate
                float currentLapTime = raceManager.GetPlayerCurrentLapTime(agent);
                averageLapTime = currentLapTime;
                bestLapTime = currentLapTime;
            }

            // Create test result
            TrackTestResult result = new TrackTestResult
            {
                modelName = modelName,
                trackNumber = trackIndex,
                trackGenerationStrategy = strategy,
                agentId = tracker.agentId,
                completedAllLaps = lapsCompleted >= targetLapsPerTrack,
                lapsCompleted = lapsCompleted,
                bestLapTime = bestLapTime,
                averageLapTime = averageLapTime,
                totalTestDuration = Time.time - testStartTime,
                collisionCount = tracker.collisionCount,
                averageSpeed = tracker.GetAverageSpeed(),
                maxSpeed = tracker.maxSpeed,
                timeAtMaxSpeed = tracker.timeAtMaxSpeed,
                checkpointsPassed = tracker.checkpointsPassed,
                trackLength = trackLength,
                recoveryCount = tracker.recoveryCount,
                timeSpentRecovering = tracker.timeSpentRecovering,
                individualLapTimes = new List<float>(tracker.lapTimes),
                collisions = new List<CollisionEvent>(tracker.collisions)
            };

            // Add to results list
            testResults.Add(result);

            // Update model summary
            UpdateModelSummary(modelName, result);
        }
    }

    private void UpdateModelSummary(string modelName, TrackTestResult result)
    {
        if (!modelSummaries.ContainsKey(modelName))
        {
            modelSummaries[modelName] = new ModelTestSummary
            {
                modelName = modelName,
                tracksTested = 0,
                bestLapTime = float.MaxValue
            };
        }

        ModelTestSummary summary = modelSummaries[modelName];

        // Update summary statistics
        summary.tracksTested++;
        if (result.completedAllLaps)
            summary.tracksCompletedFully++;

        summary.completionRate = (float)summary.tracksCompletedFully / summary.tracksTested;

        // Running average for metrics
        float n = summary.tracksTested;
        summary.averageLapTime = ((n - 1) * summary.averageLapTime + result.averageLapTime) / n;
        summary.averageCollisionsPerTrack = ((n - 1) * summary.averageCollisionsPerTrack + result.collisionCount) / n;
        summary.averageSpeed = ((n - 1) * summary.averageSpeed + result.averageSpeed) / n;
        summary.averageMaxSpeed = ((n - 1) * summary.averageMaxSpeed + result.maxSpeed) / n;
        summary.averageCompletionTime = ((n - 1) * summary.averageCompletionTime + result.totalTestDuration) / n;
        summary.totalCheckpointsPassed += result.checkpointsPassed;
        summary.averageRecoveryTime = ((n - 1) * summary.averageRecoveryTime + result.timeSpentRecovering) / n;
        summary.totalRecoveryCount += result.recoveryCount;

        // Update best lap time if better
        if (result.bestLapTime < summary.bestLapTime && result.bestLapTime > 0)
            summary.bestLapTime = result.bestLapTime;
    }

    private void FinalizeEvaluation()
    {
        // Generate results text
        StringBuilder results = new StringBuilder();
        results.AppendLine("=== AI EVALUATION RESULTS ===\n");

        // Per model summaries
        results.AppendLine("MODEL SUMMARIES:");
        foreach (var summary in modelSummaries.Values)
        {
            results.AppendLine($"\n{summary.modelName}:");
            results.AppendLine($"  Tracks tested: {summary.tracksTested}");
            results.AppendLine($"  Tracks completed fully: {summary.tracksCompletedFully} ({summary.completionRate:P1})");
            results.AppendLine($"  Best lap time: {(summary.bestLapTime < float.MaxValue ? summary.bestLapTime.ToString("F2") + "s" : "N/A")}");
            results.AppendLine($"  Average lap time: {summary.averageLapTime:F2}s");
            results.AppendLine($"  Average collisions per track: {summary.averageCollisionsPerTrack:F1}");
            results.AppendLine($"  Average speed: {summary.averageSpeed:F1} units/s");
            results.AppendLine($"  Average max speed: {summary.averageMaxSpeed:F1} units/s");
            results.AppendLine($"  Average completion time: {summary.averageCompletionTime:F1}s");
            results.AppendLine($"  Total checkpoints passed: {summary.totalCheckpointsPassed}");
            results.AppendLine($"  Recovery incidents: {summary.totalRecoveryCount} (Avg {summary.averageRecoveryTime:F1}s each)");
        }

        Log(results.ToString());
    }

    private void CleanupAgents()
    {
        foreach (var agent in activeAgents)
        {
            if (agent != null)
            {
                Destroy(agent);
            }
        }

        activeAgents.Clear();
    }

    private void CleanupScene()
    {
        // Clean up any active agents
        CleanupAgents();

        // Cleanup track system
        if (trackHandler != null)
        {
            Destroy(trackHandler.gameObject);
            trackHandler = null;
        }

        // Cleanup race manager
        if (raceManager != null)
        {
            Destroy(raceManager.gameObject);
            raceManager = null;
        }

        // Clean up any remaining objects from previous tests
        GameObject[] rootObjects = UnityEngine.SceneManagement.SceneManager.GetActiveScene().GetRootGameObjects();
        foreach (var obj in rootObjects)
        {
            if (obj != gameObject &&
                (obj.name.Contains("Track") ||
                 obj.name.Contains("Race") ||
                 obj.name.Contains("AI_")))
            {
                Destroy(obj);
            }
        }
    }

    private float CalculateTrackLength()
    {
        if (trackHandler == null || trackHandler.trackSpline == null)
            return 0f;

        return trackHandler.trackSpline.Spline.GetLength();
    }

    private void Log(string message, bool isError = false)
    {
        if (isError)
        {
            Debug.LogError($"[AI Evaluation] {message}");
        }
        else
        {
            if (enableDetailedLogging || message.Contains("==="))
            {
                Debug.Log($"[AI Evaluation] {message}");
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

    /// <summary>
    /// Exports the test results to CSV files
    /// </summary>
    public void ExportResults()
    {
        if (testResults.Count == 0)
        {
            Log("No results to export!", true);
            return;
        }

        // Format results as CSV
        string path = Application.dataPath + "/AI_Evaluation_Results.csv";
        try
        {
            using (StreamWriter writer = new StreamWriter(path))
            {
                // Write header
                writer.WriteLine("Model,TrackNumber,TrackGenerationStrategy,AgentID,CompletedAllLaps,LapsCompleted," +
                              "BestLapTime,AverageLapTime,TestDuration,CollisionCount,AverageSpeed,MaxSpeed," +
                              "TimeAtMaxSpeed,CheckpointsPassed,TrackLength,RecoveryCount,TimeSpentRecovering");

                // Write data rows
                foreach (var result in testResults)
                {
                    writer.WriteLine($"{result.modelName},{result.trackNumber},{result.trackGenerationStrategy},{result.agentId}," +
                                   $"{result.completedAllLaps},{result.lapsCompleted},{result.bestLapTime},{result.averageLapTime}," +
                                   $"{result.totalTestDuration},{result.collisionCount},{result.averageSpeed},{result.maxSpeed}," +
                                   $"{result.timeAtMaxSpeed},{result.checkpointsPassed},{result.trackLength},{result.recoveryCount}," +
                                   $"{result.timeSpentRecovering}");
                }
            }

            // Also export model summaries
            string summaryPath = Application.dataPath + "/AI_Evaluation_Summary.csv";
            using (StreamWriter writer = new StreamWriter(summaryPath))
            {
                // Write header
                writer.WriteLine("Model,TracksTested,TracksCompletedFully,CompletionRate,AverageLapTime,BestLapTime," +
                              "AverageCollisionsPerTrack,AverageSpeed,AverageMaxSpeed,AverageCompletionTime," +
                              "TotalCheckpointsPassed,AverageRecoveryTime,TotalRecoveryCount");

                // Write data rows
                foreach (var summary in modelSummaries.Values)
                {
                    writer.WriteLine($"{summary.modelName},{summary.tracksTested},{summary.tracksCompletedFully}," +
                                   $"{summary.completionRate},{summary.averageLapTime},{summary.bestLapTime}," +
                                   $"{summary.averageCollisionsPerTrack},{summary.averageSpeed},{summary.averageMaxSpeed}," +
                                   $"{summary.averageCompletionTime},{summary.totalCheckpointsPassed}," +
                                   $"{summary.averageRecoveryTime},{summary.totalRecoveryCount}");
                }
            }

            Log($"Results exported to:\n{path}\n{summaryPath}");
        }
        catch (Exception e)
        {
            Log($"Error exporting results: {e.Message}", true);
        }
    }
}