using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;
using Assets.TrackGeneration;
using System;
using System.Linq;
using System.Text;

public class InferenceCarAgent : Agent
{
    [Header("References")]
    public CarControlScript carController;
    public TrackSensingSystem sensorSystem;
    public TrackHandler trackHandler;
    public RaceManager raceManager;

    [Header("Checkpoint System")]
    private List<Checkpoint> checkpoints;
    private float distanceToNextCheckpoint;
    private Vector3 directionToNextCheckpoint;
    private float lastCheckpointTime;
    private int totalCheckpointsPassed = 0;

    [Header("Checkpoint Debugging")]
    public bool enableCheckpointDebugging = false;
    private bool lastRaceActiveState = false;

    [Header("Track Memory")]
    public bool useTrackMemory = true;
    private Vector3[] trackKnots;     // Center track knots
    private Vector3[] leftWallKnots;    // Left wall knots 
    private Vector3[] rightWallKnots;   // Right wall knots
    private bool[] discoveredKnots;     // Which knots we've seen
    private int closestKnotIndex = 0;
    private int lookAheadCount = 4;     // How many points to predict ahead
    public float trackMemoryUpdateInterval = 0.2f; // Update 5 times per second
    private float lastTrackMemoryUpdateTime = 0f;
    private int cachedClosestKnotIndex = -1;

    [Header("Debug & Visualization")]
    [Tooltip("Visualize direction to next checkpoint")]
    public bool visualizeTargetDirection = true;
    [Tooltip("Show debug messages")]
    public bool isDebugLoggingEnabled = false;
    public bool visualizeRaySensors = false;
    public bool visualizeActionIntentions = false;
    public Color raySensorHitColor = Color.red;
    public Color raySensorNoHitColor = Color.green;
    public float visualizationDuration = 0.1f;

    // Safety reset
    private float safetyResetHeight = -50f;

    // Race debug timer
    private float lastRaceDebugTime = 0f;

    // Memory for observations
    private Vector3 previousPosition;
    private Vector3 previousForward;
    private float[] previousRaySensorData = new float[0];

    // For visualization of actions
    private float[] latestContinuousActions = new float[0];
    private int[] latestDiscreteActions = new int[0];

    // Action interpolation system
    private float[] previousActions = new float[3]; // [accelerate, steer, brake]
    private float[] currentActions = new float[3];
    private bool previousHandbrake = false;
    private bool currentHandbrake = false;
    private float interpolationTime = 0f;
    private float decisionInterval = 0.1f; // Will be updated dynamically

    // Drift-specific parameters (copy these from CarAgent if needed)
    [Header("Drift Parameters")]
    public float tightCornerThreshold = 35f;  // Angle threshold to consider a corner 'tight'
    private bool isInTightCorner = false;
    // Note: Additional drift parameters (e.g. minSpeedToDrift, boost thresholds, driftTime) are expected to be available on carController.

    // -------------------- Initialization --------------------

    public void InitializeAgent()
    {
        // Get or assign required components.
        if (carController == null)
            carController = GetComponent<CarControlScript>();

        if (sensorSystem == null)
            sensorSystem = GetComponentInChildren<TrackSensingSystem>();

        if (carController != null && trackHandler != null)
        {
            carController.trackHandler = trackHandler;
        }

        ResetCar();

        // Get checkpoints.
        if (trackHandler != null)
        {
            checkpoints = trackHandler.GetCheckpoints();
            if (checkpoints != null && checkpoints.Count > 0)
            {
                if (isDebugLoggingEnabled)
                    Debug.Log($"Car {gameObject.name} found {checkpoints.Count} checkpoints");
            }
            else
            {
                Debug.LogWarning($"Car {gameObject.name} couldn't find checkpoints");
            }
        }

        if (raceManager != null)
        {
            raceManager.RegisterAgent(gameObject, gameObject.name);
            Debug.Log($"[Agent {gameObject.name}] Registered with RaceManager");
            raceManager.OnCheckpointPassed += OnRaceManagerCheckpointPassed;
            raceManager.OnWrongCheckpoint += OnRaceManagerWrongCheckpoint;
        }

        // Initialize memory variables.
        previousPosition = transform.position;
        previousForward = transform.forward;

        if (sensorSystem != null)
        {
            float[] currentData = sensorSystem.GetAllSensorData();
            previousRaySensorData = new float[currentData.Length];
            Array.Copy(currentData, previousRaySensorData, currentData.Length);
        }

        // Initialize action arrays.
        for (int i = 0; i < 3; i++)
        {
            previousActions[i] = 0f;
            currentActions[i] = 0f;
        }

        var decisionRequester = GetComponent<Unity.MLAgents.DecisionRequester>();
        if (decisionRequester != null)
        {
            decisionInterval = Time.fixedDeltaTime * decisionRequester.DecisionPeriod;
        }

        InitializeTrackMemory();
    }

    protected override void Awake()
    {
        base.Awake();

        // Verify and update BehaviorParameters if necessary.
        var behaviorParams = GetComponent<Unity.MLAgents.Policies.BehaviorParameters>();
        if (behaviorParams != null)
        {
            int expectedObservations = 60;
            if (behaviorParams.BrainParameters.VectorObservationSize != expectedObservations)
            {
                Debug.Log($"Updating vector observation size from {behaviorParams.BrainParameters.VectorObservationSize} to {expectedObservations}");
                behaviorParams.BrainParameters.VectorObservationSize = expectedObservations;
            }

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

    private void InitializeTrackMemory()
    {
        if (!useTrackMemory || trackHandler == null) return;

        var centerSpline = trackHandler.trackSpline;
        var leftSpline = trackHandler.leftSpline;
        var rightSpline = trackHandler.rightSpline;

        if (centerSpline == null || leftSpline == null || rightSpline == null)
        {
            Debug.LogWarning("Could not find track splines for sampling");
            return;
        }

        int knotCount = centerSpline.Spline.Count;
        trackKnots = new Vector3[knotCount];
        leftWallKnots = new Vector3[knotCount];
        rightWallKnots = new Vector3[knotCount];
        discoveredKnots = new bool[knotCount];

        for (int i = 0; i < knotCount; i++)
        {
            trackKnots[i] = centerSpline.Spline[i].Position;
            leftWallKnots[i] = leftSpline.Spline[i].Position;
            rightWallKnots[i] = rightSpline.Spline[i].Position;
            discoveredKnots[i] = false;
        }

        Debug.Log($"Track memory initialized with {knotCount} knot points");
    }

    // -------------------- Update & FixedUpdate --------------------

    void Update()
    {
        if (raceManager != null)
        {
            bool isRaceActive = raceManager.IsRaceActive();
            if (isRaceActive != lastRaceActiveState)
            {
                Debug.Log($"[Agent {gameObject.name}] Race active state changed: {lastRaceActiveState} -> {isRaceActive}");
                lastRaceActiveState = isRaceActive;
            }

            if (isRaceActive && Time.frameCount % 30 == 0)
            {
                SyncCheckpointWithRaceManager();
            }

            if (Time.time > lastRaceDebugTime + 5f && enableCheckpointDebugging)
            {
                int nextCheckpointIndex = raceManager.GetPlayerNextCheckpoint(gameObject);
                int currentLap = raceManager.GetPlayerCurrentLap(gameObject);
                bool hasCompletedFullCircuit = raceManager.HasPlayerCompletedFullCircuit(gameObject);
                Debug.Log($"[Agent {gameObject.name}] Race active: {isRaceActive}, Next checkpoint: {nextCheckpointIndex}, Lap: {currentLap}, Full circuit: {hasCompletedFullCircuit}, Total passed: {raceManager.GetPlayerTotalCheckpointsPassed(gameObject)}");
                lastRaceDebugTime = Time.time;
            }
        }

        if (transform.position.y < safetyResetHeight)
        {
            Debug.LogWarning($"[RESET] Car {gameObject.name} fell below safety height ({safetyResetHeight}). Forcing reset.");
            ResetCar();
            EndEpisode();
            return;
        }

        VisualizeAgentPerception();
    }

    void FixedUpdate()
    {
        if (carController == null)
            return;

        interpolationTime += Time.fixedDeltaTime;
        if (decisionInterval <= 0.001f)
        {
            var decisionRequester = GetComponent<Unity.MLAgents.DecisionRequester>();
            decisionInterval = decisionRequester != null ? Time.fixedDeltaTime * decisionRequester.DecisionPeriod : 0.1f;
        }
        float t = Mathf.Clamp01(interpolationTime / decisionInterval);
        float interpolatedAccelerate = Mathf.Lerp(previousActions[0], currentActions[0], t);
        float interpolatedSteer = Mathf.Lerp(previousActions[1], currentActions[1], t);
        float interpolatedBrake = Mathf.Lerp(previousActions[2], currentActions[2], t);
        bool useHandbrake = t > 0.5f ? currentHandbrake : previousHandbrake;
        SetCarInputs(interpolatedAccelerate, interpolatedSteer, interpolatedBrake, useHandbrake);

        if (Time.time - lastTrackMemoryUpdateTime > trackMemoryUpdateInterval)
        {
            UpdateTrackMemory();
            lastTrackMemoryUpdateTime = Time.time;
        }
    }

    private void UpdateTrackMemory()
    {
        if (!useTrackMemory || trackKnots == null)
            return;

        // Find the closest knot index using an existing helper function.
        closestKnotIndex = FindClosestKnot();

        // If a valid knot is found, mark nearby knots as discovered.
        if (closestKnotIndex >= 0)
        {
            int range = 10; // Mark 10 knots ahead and behind as "discovered".
            int knotCount = trackKnots.Length;
            for (int offset = -range; offset <= range; offset++)
            {
                int index = (closestKnotIndex + offset + knotCount) % knotCount;
                discoveredKnots[index] = true;
            }
        }
    }

    private int FindClosestKnot()
    {
        if (trackKnots == null)
            return -1;

        float minDist = float.MaxValue;
        int closestIndex = -1;
        int knotCount = trackKnots.Length;

        // Optionally, if you have a cached index, you can search around it.
        if (cachedClosestKnotIndex >= 0)
        {
            int searchRange = 10;
            for (int offset = -searchRange; offset <= searchRange; offset++)
            {
                int i = (cachedClosestKnotIndex + offset + knotCount) % knotCount;
                float dist = Vector3.Distance(transform.position, trackKnots[i]);
                if (dist < minDist)
                {
                    minDist = dist;
                    closestIndex = i;
                }
            }

            // If the distance is sufficiently small or it has been checked recently,
            // return the found index; otherwise perform a full search.
            if (minDist < 5f || Time.frameCount % 120 == 0)
                return closestIndex;
        }

        // Full search over all knots if no cached or adequate candidate is found.
        for (int i = 0; i < knotCount; i++)
        {
            float dist = Vector3.Distance(transform.position, trackKnots[i]);
            if (dist < minDist)
            {
                minDist = dist;
                closestIndex = i;
            }
        }
        cachedClosestKnotIndex = closestIndex;
        return closestIndex;
    }


    // -------------------- Visualizations --------------------

    private void VisualizeAgentPerception()
    {
        if (!visualizeRaySensors && !visualizeActionIntentions)
            return;

        if (visualizeRaySensors && sensorSystem != null)
        {
            VisualizeTrackKnots();
            Vector3[] rayOrigins = sensorSystem.GetRayOrigins();
            Vector3[] rayDirections = sensorSystem.GetRayDirections();
            float[] rayDistances = sensorSystem.GetAllSensorData();

            for (int i = 0; i < rayOrigins.Length && i < rayDirections.Length && i < rayDistances.Length; i++)
            {
                float maxRayDistance = sensorSystem.GetMaxRayDistance();
                float hitDistance = rayDistances[i] * maxRayDistance;
                bool hitDetected = rayDistances[i] < 0.99f;
                Color rayColor = hitDetected ? raySensorHitColor : raySensorNoHitColor;
                Vector3 endPoint = rayOrigins[i] + rayDirections[i] * hitDistance;
                Debug.DrawLine(rayOrigins[i], endPoint, rayColor, visualizationDuration);
                if (hitDetected)
                {
                    Debug.DrawLine(endPoint, endPoint + Vector3.up * 0.5f, Color.yellow, visualizationDuration);
                }
            }
        }

        if (visualizeActionIntentions)
        {
            float accelerate = latestContinuousActions.Length > 0 ? latestContinuousActions[0] : 0f;
            float steer = latestContinuousActions.Length > 1 ? latestContinuousActions[1] : 0f;
            float brake = latestContinuousActions.Length > 2 ? latestContinuousActions[2] : 0f;
            Color accelColor = accelerate > 0 ? Color.green : Color.yellow;
            Debug.DrawLine(transform.position, transform.position + transform.forward * accelerate * 3f, accelColor, visualizationDuration);
            Color steerColor = steer > 0 ? Color.blue : Color.magenta;
            Debug.DrawLine(transform.position, transform.position + transform.right * steer * 2f, steerColor, visualizationDuration);
            if (brake > 0.1f)
            {
                Debug.DrawLine(transform.position, transform.position - transform.forward * brake * 2f, Color.red, visualizationDuration);
            }
        }
    }

    private void VisualizeTrackKnots()
    {
        if (!useTrackMemory || !visualizeRaySensors || trackKnots == null)
            return;

        int knotCount = trackKnots.Length;
        for (int i = 0; i < knotCount; i++)
        {
            if (discoveredKnots[i])
            {
                int nextIndex = (i + 1) % knotCount;
                if (discoveredKnots[nextIndex])
                {
                    Debug.DrawLine(trackKnots[i], trackKnots[nextIndex], Color.yellow, 0.1f);
                    Debug.DrawLine(leftWallKnots[i], leftWallKnots[nextIndex], Color.cyan, 0.1f);
                    Debug.DrawLine(rightWallKnots[i], rightWallKnots[nextIndex], Color.cyan, 0.1f);
                    Debug.DrawLine(leftWallKnots[i], rightWallKnots[i], Color.white, 0.1f);
                }
            }
        }

        if (closestKnotIndex >= 0)
        {
            Debug.DrawLine(transform.position, trackKnots[closestKnotIndex], Color.red, 0.1f);
            Debug.DrawLine(trackKnots[closestKnotIndex], trackKnots[closestKnotIndex] + Vector3.up * 2f, Color.red, 0.1f);
            for (int i = 1; i <= lookAheadCount; i++)
            {
                int knotIndex = (closestKnotIndex + i) % knotCount;
                if (discoveredKnots[knotIndex])
                {
                    Color observationColor = Color.Lerp(Color.green, Color.blue, i / (float)lookAheadCount);
                    Debug.DrawLine(transform.position, trackKnots[knotIndex], observationColor, 0.1f);
                    Debug.DrawLine(trackKnots[knotIndex], trackKnots[knotIndex] + Vector3.up * (3f - i * 0.5f), observationColor, 0.1f);
                    Vector3 center = trackKnots[knotIndex];
                    Vector3 leftPos = leftWallKnots[knotIndex];
                    Vector3 rightPos = rightWallKnots[knotIndex];
                    float width = Vector3.Distance(leftPos, rightPos);
                    float heightDiff = rightPos.y - leftPos.y;
                    float bankAngle = Mathf.Atan2(heightDiff, width) * Mathf.Rad2Deg;
                    Debug.DrawLine(leftPos, rightPos, observationColor, 0.1f);
                    Vector3 bankingDirection = Quaternion.Euler(0, 0, bankAngle) * Vector3.up * 3f;
                    Debug.DrawRay(center, bankingDirection, Color.magenta, 0.1f);
                    int nextIndex = (knotIndex + 1) % knotCount;
                    if (discoveredKnots[nextIndex])
                    {
                        Vector3 nextPos = trackKnots[nextIndex];
                        Vector3 trackDirection = (nextPos - center).normalized * 5f;
                        Debug.DrawRay(center, trackDirection, observationColor, 0.1f);
                    }
                }
            }
        }
    }

    // -------------------- Checkpoint & Reset --------------------

    private void SyncCheckpointWithRaceManager()
    {
        if (raceManager == null) return;
        UpdateCheckpointInfo();
    }

    private void UpdateCheckpointInfo()
    {
        try
        {
            if (checkpoints == null || checkpoints.Count == 0 || raceManager == null)
            {
                if (enableCheckpointDebugging)
                    Debug.LogWarning($"[Agent {gameObject.name}] UpdateCheckpointInfo: Checkpoints list is null or empty!");
                return;
            }
            int nextIndex = raceManager.GetPlayerNextCheckpoint(gameObject);
            if (nextIndex < checkpoints.Count)
            {
                Vector3 nextCheckpointPosition = checkpoints[nextIndex].transform.position;
                distanceToNextCheckpoint = Vector3.Distance(transform.position, nextCheckpointPosition);
                directionToNextCheckpoint = nextCheckpointPosition - transform.position;
                if (visualizeTargetDirection)
                {
                    Debug.DrawLine(transform.position, nextCheckpointPosition, Color.green, 0.1f);
                    if (enableCheckpointDebugging && Time.frameCount % 300 == 0)
                    {
                        Debug.Log($"[Agent {gameObject.name}] Drawing ray to checkpoint {nextIndex} at {nextCheckpointPosition}");
                    }
                }
            }
            else
            {
                if (enableCheckpointDebugging)
                    Debug.LogError($"[Agent {gameObject.name}] Invalid nextCheckpointIndex: {nextIndex} (max: {checkpoints.Count - 1})");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Error in UpdateCheckpointInfo: {e.Message}");
            distanceToNextCheckpoint = 50f;
            directionToNextCheckpoint = transform.forward * 50f;
        }
    }

    private void ResetCar()
    {
        if (carController == null)
        {
            Debug.LogWarning("CarController is null in ResetCar, looking for it now...");
            carController = GetComponent<CarControlScript>();
        }

        if (carController != null)
        {
            try
            {
                carController.ResetPosition();
            }
            catch (Exception e)
            {
                Debug.LogError($"Error in ResetCar: {e.Message}\n{e.StackTrace}");
            }
        }
        else
        {
            Debug.LogError($"Cannot reset car position: carController is still null");
        }
    }

    public override void OnEpisodeBegin()
    {
        ResetCar();
        ForceCheckpointReset();

        if (trackHandler != null)
        {
            checkpoints = trackHandler.GetCheckpoints();
        }

        lastCheckpointTime = Time.time;
        totalCheckpointsPassed = 0;
        lastTrackMemoryUpdateTime = Time.time;

        previousPosition = transform.position;
        previousForward = transform.forward;

        interpolationTime = 0f;
        for (int i = 0; i < 3; i++)
        {
            previousActions[i] = 0f;
            currentActions[i] = 0f;
        }
        previousHandbrake = false;
        currentHandbrake = false;

        if (carController != null)
        {
            carController.SetAccelerationInput(0f);
            carController.SetSteeringInput(0f);
            carController.SetReverseInput(0f);
            carController.SetHandbrakeInput(false);
        }

        UpdateCheckpointInfo();
    }

    private void ForceCheckpointReset()
    {
        lastCheckpointTime = Time.time;
        if (raceManager != null)
        {
            try
            {
                raceManager.ResetAgent(gameObject);
                UpdateCheckpointInfo();
            }
            catch (Exception e)
            {
                Debug.LogWarning($"Failed to reset checkpoint in RaceManager: {e.Message}");
            }
        }
    }

    // -------------------- Observations --------------------

    public override void CollectObservations(VectorSensor sensor)
    {
        if (raceManager != null)
        {
            UpdateCheckpointInfo();
        }

        if (sensorSystem == null)
        {
            Debug.LogError("SensorSystem is null in CollectObservations!");
            int observations = ((sensorSystem != null ? sensorSystem.totalRayCount * 2 : 20) + 6);
            for (int i = 0; i < observations; i++)
                sensor.AddObservation(0f);
            return;
        }

        // 1. Ray sensor data.
        float[] raySensorData = sensorSystem.GetAllSensorData();
        for (int i = 0; i < sensorSystem.totalRayCount; i++)
        {
            sensor.AddObservation(i < raySensorData.Length ? raySensorData[i] : 1.0f);
        }
        // 2. Ray velocity data.
        float[] velocityData = sensorSystem.GetVelocityData();
        for (int i = 0; i < sensorSystem.totalRayCount; i++)
        {
            float velocityValue = i < velocityData.Length ? Mathf.Clamp(velocityData[i], -10f, 10f) : 0f;
            sensor.AddObservation(velocityValue);
        }
        // 3. Normalized direction to next checkpoint.
        float forwardDot = Vector3.Dot(transform.forward, directionToNextCheckpoint.normalized);
        float rightDot = Vector3.Dot(transform.right, directionToNextCheckpoint.normalized);
        sensor.AddObservation(forwardDot);
        sensor.AddObservation(rightDot);
        // 4. Normalized distance.
        sensor.AddObservation(Mathf.Clamp01(distanceToNextCheckpoint / 100f));
        // 5. Normalized car speed.
        float speed = carController.GetCurrentSpeed();
        sensor.AddObservation(speed / carController.maxVelocity);
        // 6. Drifting flag.
        sensor.AddObservation(carController.isDrifting ? 1f : 0f);
        // 7. Time since last decision.
        float timeSinceLastDecision = Time.fixedDeltaTime * Time.timeScale * GetComponent<Unity.MLAgents.DecisionRequester>().DecisionPeriod;
        sensor.AddObservation(timeSinceLastDecision);
        // 8. Track memory observations.
        if (useTrackMemory && trackKnots != null && closestKnotIndex >= 0)
        {
            int knotCount = trackKnots.Length;
            for (int i = 1; i <= lookAheadCount; i++)
            {
                int knotIndex = (closestKnotIndex + i) % knotCount;
                if (discoveredKnots[knotIndex])
                {
                    Vector3 centerPos = trackKnots[knotIndex];
                    Vector3 leftPos = leftWallKnots[knotIndex];
                    Vector3 rightPos = rightWallKnots[knotIndex];
                    Vector3 localCenter = transform.InverseTransformPoint(centerPos);
                    float distance = Vector3.Distance(transform.position, centerPos);
                    float width = Vector3.Distance(leftPos, rightPos);
                    int nextIndex = (knotIndex + 1) % knotCount;
                    Vector3 trackDirection = (trackKnots[nextIndex] - centerPos).normalized;
                    float trackAngle = Vector3.SignedAngle(transform.forward, trackDirection, Vector3.up);
                    float heightDiff = rightPos.y - leftPos.y;
                    float bankAngle = Mathf.Atan2(heightDiff, width) * Mathf.Rad2Deg;
                    sensor.AddObservation(Mathf.Clamp01(distance / 100f));
                    sensor.AddObservation(localCenter.x / 50f);
                    sensor.AddObservation(localCenter.z / 50f);
                    sensor.AddObservation(width / 20f);
                    sensor.AddObservation(trackAngle / 180f);
                    sensor.AddObservation(bankAngle / 45f);
                }
                else
                {
                    for (int j = 0; j < 6; j++)
                        sensor.AddObservation(0f);
                }
            }
        }

        // 9. *** Add Drift-Specific Observations ***
        AddDriftObservations(sensor);
    }

    private void AddDriftObservations(VectorSensor sensor)
    {
        // 1. Is the car in a tight corner?
        sensor.AddObservation(isInTightCorner ? 1.0f : 0.0f);
        // 2. Upcoming corner angle.
        float upcomingCornerAngle = DetectUpcomingCornerAngle();
        sensor.AddObservation(Mathf.Clamp01(upcomingCornerAngle / 90f));
        // 3. Distance to next corner.
        float distanceToCorner = DetectDistanceToNextCorner();
        sensor.AddObservation(Mathf.Clamp01(distanceToCorner / 30f));
        // 4. Speed ratio relative to minimum drift speed.
        float speedRatio = carController.GetCurrentSpeed() / carController.minSpeedToDrift;
        sensor.AddObservation(Mathf.Clamp01(speedRatio));
        // 5. Boost status.
        bool hasBoost = carController.boostTimeRemaining > 0;
        sensor.AddObservation(hasBoost ? 1.0f : 0.0f);
        // 6. Normalized boost power.
        float boostPower = hasBoost ? (carController.currentBoostPower / carController.boostForce) : 0f;
        sensor.AddObservation(Mathf.Clamp01(boostPower));
        // 7. Normalized drift time.
        float maxDriftThreshold = carController.boostThresholds.Length > 0 ? carController.boostThresholds[carController.boostThresholds.Length - 1] : 3f;
        float normalizedDriftTime = carController.isDrifting ? (carController.driftTime / maxDriftThreshold) : 0f;
        sensor.AddObservation(Mathf.Clamp01(normalizedDriftTime));
        // 8. Whether current drift is at reward threshold.
        bool isAtRewardThreshold = false;
        if (carController.isDrifting && carController.boostThresholds.Length > 0)
        {
            foreach (float threshold in carController.boostThresholds)
            {
                if (Mathf.Abs(carController.driftTime - threshold) < 0.1f)
                {
                    isAtRewardThreshold = true;
                    break;
                }
            }
        }
        sensor.AddObservation(isAtRewardThreshold ? 1.0f : 0.0f);
    }

    private float DetectUpcomingCornerAngle()
    {
        if (!useTrackMemory || trackKnots == null || closestKnotIndex < 0)
            return 0f;

        int knotCount = trackKnots.Length;
        float maxAngleChange = 0f;
        for (int i = 1; i < lookAheadCount - 1; i++)
        {
            int idx1 = (closestKnotIndex + i) % knotCount;
            int idx2 = (closestKnotIndex + i + 1) % knotCount;
            int idx3 = (closestKnotIndex + i + 2) % knotCount;

            if (!discoveredKnots[idx1] || !discoveredKnots[idx2] || !discoveredKnots[idx3])
                continue;

            Vector3 segment1 = (trackKnots[idx2] - trackKnots[idx1]).normalized;
            Vector3 segment2 = (trackKnots[idx3] - trackKnots[idx2]).normalized;
            float angleChange = Vector3.Angle(segment1, segment2);
            maxAngleChange = Mathf.Max(maxAngleChange, angleChange);
        }
        return maxAngleChange;
    }

    private float DetectDistanceToNextCorner()
    {
        if (!useTrackMemory || trackKnots == null || closestKnotIndex < 0)
            return 30f;

        int knotCount = trackKnots.Length;
        float distance = 0f;
        for (int i = 1; i < lookAheadCount + 5; i++)
        {
            int idx1 = (closestKnotIndex + i - 1) % knotCount;
            int idx2 = (closestKnotIndex + i) % knotCount;
            int idx3 = (closestKnotIndex + i + 1) % knotCount;

            if (!discoveredKnots[idx1] || !discoveredKnots[idx2] || !discoveredKnots[idx3])
                continue;

            Vector3 segment1 = (trackKnots[idx2] - trackKnots[idx1]).normalized;
            Vector3 segment2 = (trackKnots[idx3] - trackKnots[idx2]).normalized;
            float angleChange = Vector3.Angle(segment1, segment2);
            if (angleChange > tightCornerThreshold)
            {
                float totalDist = 0f;
                Vector3 prevPos = transform.position;
                for (int j = 0; j <= i; j++)
                {
                    int idx = (closestKnotIndex + j) % knotCount;
                    if (!discoveredKnots[idx])
                        continue;
                    totalDist += Vector3.Distance(prevPos, trackKnots[idx]);
                    prevPos = trackKnots[idx];
                }
                return totalDist;
            }
            distance += Vector3.Distance(trackKnots[idx1], trackKnots[idx2]);
        }
        return 30f;
    }

    // -------------------- Actions & Collision --------------------

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        if (actionBuffers.ContinuousActions.Length > 0)
        {
            latestContinuousActions = new float[actionBuffers.ContinuousActions.Length];
            for (int i = 0; i < actionBuffers.ContinuousActions.Length; i++)
            {
                latestContinuousActions[i] = actionBuffers.ContinuousActions[i];
            }
        }

        if (actionBuffers.DiscreteActions.Length > 0)
        {
            latestDiscreteActions = new int[actionBuffers.DiscreteActions.Length];
            for (int i = 0; i < actionBuffers.DiscreteActions.Length; i++)
            {
                latestDiscreteActions[i] = actionBuffers.DiscreteActions[i];
            }
            previousHandbrake = currentHandbrake;
            currentHandbrake = actionBuffers.DiscreteActions[0] == 1;
        }

        try
        {
            if (actionBuffers.ContinuousActions.Length >= 3)
            {
                for (int i = 0; i < 3; i++)
                {
                    previousActions[i] = currentActions[i];
                }
                currentActions[0] = actionBuffers.ContinuousActions[0]; // accelerate
                currentActions[1] = actionBuffers.ContinuousActions[1]; // steer
                currentActions[2] = actionBuffers.ContinuousActions[2]; // brake

                if (visualizeTargetDirection)
                {
                    Debug.DrawRay(transform.position, transform.forward * currentActions[0] * 5f, Color.green);
                    Debug.DrawRay(transform.position, transform.right * currentActions[1] * 5f, Color.red);
                }
            }
            else
            {
                Debug.LogWarning($"Expected 3 continuous actions, but got {actionBuffers.ContinuousActions.Length}");
            }

            interpolationTime = 0f;
            var decisionRequester = GetComponent<Unity.MLAgents.DecisionRequester>();
            if (decisionRequester != null)
                decisionInterval = Time.fixedDeltaTime * decisionRequester.DecisionPeriod;

            if (raceManager != null && raceManager.IsRaceActive())
            {
                SyncCheckpointWithRaceManager();
            }
            UpdateCheckpointInfo();
        }
        catch (Exception ex)
        {
            Debug.LogError($"Error in OnActionReceived: {ex.Message}");
        }
    }

    public void ReportCollision()
    {
        if (isDebugLoggingEnabled)
            Debug.Log($"[Agent {gameObject.name}] Collision detected");
    }

    private void SetCarInputs(float accelerate, float steer, float brake, bool handbrake)
    {
        carController.SetAccelerationInput(accelerate);
        carController.SetSteeringInput(steer);
        carController.SetReverseInput(brake);
        carController.SetHandbrakeInput(handbrake);
    }

    // -------------------- Cleanup --------------------

    protected override void OnDisable()
    {
        base.OnDisable();
        if (raceManager != null)
        {
            raceManager.OnCheckpointPassed -= OnRaceManagerCheckpointPassed;
            raceManager.OnWrongCheckpoint -= OnRaceManagerWrongCheckpoint;
        }
    }

    void OnDestroy()
    {
        if (raceManager != null)
        {
            raceManager.OnCheckpointPassed -= OnRaceManagerCheckpointPassed;
            raceManager.OnWrongCheckpoint -= OnRaceManagerWrongCheckpoint;
        }
    }

    // -------------------- Race Manager Event Handlers --------------------

    private void OnRaceManagerCheckpointPassed(GameObject player, int checkpointIndex, int totalCheckpoints)
    {
        if (player != this.gameObject) return;

        if (enableCheckpointDebugging)
        {
            int expectedIndex = raceManager.GetPlayerNextCheckpoint(gameObject);
            int currentLap = raceManager.GetPlayerCurrentLap(gameObject);
            bool fullCircuit = raceManager.HasPlayerCompletedFullCircuit(gameObject);
            Debug.Log($"[Agent {gameObject.name}] CHECKPOINT PASSED: index={checkpointIndex}, expected={expectedIndex}, lap={currentLap}, full_circuit={fullCircuit}");
        }

        totalCheckpointsPassed++;
        lastCheckpointTime = Time.time;
        UpdateCheckpointInfo();
    }

    private void OnRaceManagerWrongCheckpoint(GameObject player, int attemptedCheckpoint, int expectedCheckpoint)
    {
        if (player != this.gameObject) return;
        if (enableCheckpointDebugging)
        {
            Debug.LogWarning($"[Agent {gameObject.name}] WRONG CHECKPOINT: Attempted={attemptedCheckpoint}, Expected={expectedCheckpoint}");
        }
    }
}
