using Assets.TrackGeneration;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Splines;

public class TrackGenerationTester : MonoBehaviour
{
    public TrackHandler trackHandler;
    public int testCount = 100;
    public bool testGridWithNoise = true;
    public bool testCircularLayout = true;
    public bool testRandom = true;

    // Settings for saving results
    public bool saveResultsToFile = true;
    public string resultsFilePath = "TrackGenerationResults.csv";

    // Improved corner detection parameters
    [Tooltip("Threshold in degrees for considering a spline segment as a corner")]
    public float cornerAngleThreshold = 30f;
    [Tooltip("Number of sample points used to analyze the track")]
    public int samplePoints = 200;
    [Tooltip("Minimum length of straight sections to count")]
    public float minStraightLength = 10f;

    // Display options
    public bool showTestTracksInScene = false;
    public float testTrackDisplayTime = 1.0f;

    private List<TestResult> results = new List<TestResult>();
    private GameObject currentTestTrack;

    [Serializable]
    public class TestResult
    {
        public GenerationStrategy strategy;
        public int seed;
        public bool success;
        public float generationTime;
        public float trackLength;
        public int cornerCount;
        public float averageCornerRadius;
        public float longestStraightLength;
        public float elevationChange;
        public float minimumTrackProximity;
        public float overallQualityScore;
        public float cornerQuality;
        public float straightQuality;
        public float flowQuality;
        public float layoutQuality;
        public float lengthQuality;
        public float proximityQuality;
        public float trackWidth;
        public float maxBankingAngle;
        public int checkpointCount;
    }

    [ContextMenu("Run Track Generation Tests")]
    public void RunTests()
    {
        StartCoroutine(RunTestsCoroutine());
    }

    private IEnumerator RunTestsCoroutine()
    {
        results.Clear();
        Debug.Log("Starting track generation tests...");

        if (testGridWithNoise)
        {
            Debug.Log("Testing GridWithNoise strategy...");
            yield return StartCoroutine(RunTestsForStrategyCoroutine(GenerationStrategy.GridWithNoise));
        }

        if (testCircularLayout)
        {
            Debug.Log("Testing CircularLayout strategy...");
            yield return StartCoroutine(RunTestsForStrategyCoroutine(GenerationStrategy.CircularLayout));
        }

        if (testRandom)
        {
            Debug.Log("Testing Random strategy...");
            yield return StartCoroutine(RunTestsForStrategyCoroutine(GenerationStrategy.Random));
        }

        AnalyzeResults();

        if (saveResultsToFile)
            SaveResultsToFile();

        Debug.Log("Track generation tests completed.");
    }

    private IEnumerator RunTestsForStrategyCoroutine(GenerationStrategy strategy)
    {
        for (int i = 0; i < testCount; i++)
        {
            // Set seed for reproducibility
            int seed = i + (int)strategy * 1000; // Ensure unique seeds for each strategy
            UnityEngine.Random.InitState(seed);

            TestResult result = new TestResult();
            result.strategy = strategy;
            result.seed = seed;

            //try
            //{
            // Time the track generation
            float startTime = Time.realtimeSinceStartup;
            trackHandler.GenerateTrack(strategy);
            result.generationTime = Time.realtimeSinceStartup - startTime;

            // Wait for mesh generation to complete
            yield return new WaitForEndOfFrame();

            // Track successfully generated
            result.success = (trackHandler.trackMesh != null);

            if (result.success)
            {
                // Collect metrics from the generated track
                result.trackLength = CalculateTrackLength(trackHandler);
                result.cornerCount = CountCorners(trackHandler);
                result.averageCornerRadius = CalculateAverageCornerRadius(trackHandler);
                result.longestStraightLength = FindLongestStraight(trackHandler);
                result.elevationChange = CalculateElevationChange(trackHandler);
                result.minimumTrackProximity = CalculateMinimumProximity(trackHandler);
                result.trackWidth = trackHandler.trackWidth;
                result.maxBankingAngle = trackHandler.banking;
                result.checkpointCount = CountCheckpoints(trackHandler);

                // Get quality scores from Voronoi evaluation
                var qualityScores = GetTrackQualityScores(trackHandler);
                result.cornerQuality = qualityScores.cornerQuality;
                result.straightQuality = qualityScores.straightQuality;
                result.flowQuality = qualityScores.flowQuality;
                result.layoutQuality = qualityScores.layoutQuality;
                result.lengthQuality = qualityScores.lengthQuality;
                result.proximityQuality = qualityScores.proximityQuality;
                result.overallQualityScore = qualityScores.overallQuality;

                // Store reference to the current track mesh for visualization if needed
                if (showTestTracksInScene)
                {
                    currentTestTrack = trackHandler.trackMesh;
                    // Allow time to view the track
                    yield return new WaitForSeconds(testTrackDisplayTime);
                }

                Debug.Log($"Test {i + 1}/{testCount} for {strategy}: Success (Time: {result.generationTime:F2}s, Corners: {result.cornerCount})");
            }
            else
            {
                Debug.LogWarning($"Test {i + 1}/{testCount} for {strategy}: Generated but no mesh created");
            }
            //}
            //catch (System.Exception e)
            //{
            //    Debug.LogError($"Test {i + 1}/{testCount} for {strategy}: Generation failed with seed {seed}: {e.Message}");
            //    result.success = false;
            //}

            results.Add(result);

            // Clean up between tests
            if (trackHandler.trackMesh != null)
            {
                DestroyImmediate(trackHandler.trackMesh);
                currentTestTrack = null;
            }

            // Wait a frame to avoid freezing the editor and allow mesh destruction
            yield return null;

            // Occasional garbage collection
            if (i % 10 == 0)
            {
                System.GC.Collect();
                yield return null;
            }
        }
    }

    private void AnalyzeResults()
    {
        Debug.Log("=== TRACK GENERATION TEST RESULTS ===");

        // Calculate statistics for each strategy
        foreach (GenerationStrategy strategy in Enum.GetValues(typeof(GenerationStrategy)))
        {
            var strategyResults = results.Where(r => r.strategy == strategy).ToList();
            if (strategyResults.Count == 0)
                continue;

            int successCount = strategyResults.Count(r => r.success);
            float successRate = (float)successCount / strategyResults.Count;

            var successfulResults = strategyResults.Where(r => r.success).ToList();
            if (successfulResults.Count == 0)
            {
                Debug.Log($"Strategy {strategy}: All {strategyResults.Count} tests failed.");
                continue;
            }

            // Calculate time statistics
            float avgTime = successfulResults.Average(r => r.generationTime);
            float minTime = successfulResults.Min(r => r.generationTime);
            float maxTime = successfulResults.Max(r => r.generationTime);
            float stdDevTime = CalculateStdDev(successfulResults.Select(r => r.generationTime));

            // Calculate track length statistics
            float avgLength = successfulResults.Average(r => r.trackLength);
            float stdDevLength = CalculateStdDev(successfulResults.Select(r => r.trackLength));

            // Calculate corner statistics
            float avgCorners = (float)successfulResults.Average(r => r.cornerCount);
            float stdDevCorners = CalculateStdDev(successfulResults.Select(r => r.cornerCount));
            int minCorners = successfulResults.Min(r => r.cornerCount);
            int maxCorners = successfulResults.Max(r => r.cornerCount);

            // Calculate corner radius statistics
            float avgRadius = successfulResults.Average(r => r.averageCornerRadius);
            float stdDevRadius = CalculateStdDev(successfulResults.Select(r => r.averageCornerRadius));

            // Calculate straight length statistics
            float avgStraight = successfulResults.Average(r => r.longestStraightLength);
            float stdDevStraight = CalculateStdDev(successfulResults.Select(r => r.longestStraightLength));

            // Calculate elevation change statistics
            float avgElevation = successfulResults.Average(r => r.elevationChange);
            float stdDevElevation = CalculateStdDev(successfulResults.Select(r => r.elevationChange));

            // Calculate quality score statistics
            float avgQuality = successfulResults.Average(r => r.overallQualityScore);
            float stdDevQuality = CalculateStdDev(successfulResults.Select(r => r.overallQualityScore));

            float avgCornerQuality = successfulResults.Average(r => r.cornerQuality);
            float avgStraightQuality = successfulResults.Average(r => r.straightQuality);
            float avgFlowQuality = successfulResults.Average(r => r.flowQuality);
            float avgLayoutQuality = successfulResults.Average(r => r.layoutQuality);
            float avgLengthQuality = successfulResults.Average(r => r.lengthQuality);
            float avgProximityQuality = successfulResults.Average(r => r.proximityQuality);

            float stdDevCornerQuality = CalculateStdDev(successfulResults.Select(r => r.cornerQuality));
            float stdDevStraightQuality = CalculateStdDev(successfulResults.Select(r => r.straightQuality));
            float stdDevFlowQuality = CalculateStdDev(successfulResults.Select(r => r.flowQuality));
            float stdDevLayoutQuality = CalculateStdDev(successfulResults.Select(r => r.layoutQuality));
            float stdDevLengthQuality = CalculateStdDev(successfulResults.Select(r => r.lengthQuality));
            float stdDevProximityQuality = CalculateStdDev(successfulResults.Select(r => r.proximityQuality));

            // Log the results
            Debug.Log($"\nStrategy: {strategy}");
            Debug.Log($"Success Rate: {successRate:P2} ({successCount}/{strategyResults.Count})");
            Debug.Log($"Generation Time: {avgTime:F2}s (std={stdDevTime:F2}, range: {minTime:F2}s-{maxTime:F2}s)");
            Debug.Log($"Track Length: {avgLength:F1} units (std={stdDevLength:F1})");
            Debug.Log($"Corner Count: {avgCorners:F1} corners (std={stdDevCorners:F1}, range: {minCorners}-{maxCorners})");
            Debug.Log($"Average Corner Radius: {avgRadius:F1} units (std={stdDevRadius:F1})");
            Debug.Log($"Longest Straight: {avgStraight:F1} units (std={stdDevStraight:F1})");
            Debug.Log($"Elevation Change: {avgElevation:F1} units (std={stdDevElevation:F1})");
            Debug.Log($"Quality Scores:");
            Debug.Log($"  - Overall: {avgQuality:F2} (std={stdDevQuality:F2})");
            Debug.Log($"  - Corner Quality: {avgCornerQuality:F2} (std={stdDevCornerQuality:F2})");
            Debug.Log($"  - Straight Quality: {avgStraightQuality:F2} (std={stdDevStraightQuality:F2})");
            Debug.Log($"  - Flow Quality: {avgFlowQuality:F2} (std={stdDevFlowQuality:F2})");
            Debug.Log($"  - Layout Quality: {avgLayoutQuality:F2} (std={stdDevLayoutQuality:F2})");
            Debug.Log($"  - Length Quality: {avgLengthQuality:F2} (std={stdDevLengthQuality:F2})");
            Debug.Log($"  - Proximity Quality: {avgProximityQuality:F2} (std={stdDevProximityQuality:F2})");
        }
    }

    private float CalculateTrackLength(TrackHandler handler)
    {
        if (handler.trackSpline == null || handler.trackSpline.Spline == null)
            return 0f;

        return handler.trackSpline.Spline.GetLength();
    }

    private int CountCorners(TrackHandler handler)
    {
        if (handler.trackSpline == null || handler.trackSpline.Spline == null)
            return 0;

        Spline spline = handler.trackSpline.Spline;
        int cornerCount = 0;
        bool inCorner = false;
        List<float> angles = new List<float>();

        // Convert threshold to radians for angle calculations
        float thresholdRadians = cornerAngleThreshold * Mathf.Deg2Rad;

        // Sample points along the spline for angle changes
        Vector3 prevDirection = Vector3.zero;
        for (int i = 0; i <= samplePoints; i++)
        {
            float t = i / (float)samplePoints;
            Vector3 currentPos = spline.EvaluatePosition(t);
            Vector3 tangent = spline.EvaluateTangent(t);

            // Skip the first point as we need two points to calculate direction
            if (i > 0)
            {
                // Calculate angle between previous and current direction
                Vector3 currentDirection = tangent.normalized;

                if (prevDirection != Vector3.zero)
                {
                    float dot = Vector3.Dot(prevDirection, currentDirection);
                    // Clamp dot product to valid range for acos
                    dot = Mathf.Clamp(dot, -1.0f, 1.0f);
                    float angle = Mathf.Acos(dot);
                    angles.Add(angle);

                    // Debug.Log($"Point {i}: Angle = {angle * Mathf.Rad2Deg} degrees");

                    // Check if this point is part of a corner
                    bool isCornerPoint = angle > thresholdRadians;

                    // Start of a new corner
                    if (isCornerPoint && !inCorner)
                    {
                        cornerCount++;
                        inCorner = true;
                    }
                    // End of current corner
                    else if (!isCornerPoint && inCorner)
                    {
                        inCorner = false;
                    }
                }

                prevDirection = currentDirection;
            }
        }

        // Ensure we detect at least 4 corners for a reasonable track
        if (cornerCount < 4 && angles.Count > 0)
        {
            // Sort angles in descending order
            angles.Sort((a, b) => b.CompareTo(a));

            // Attempt to identify top corners from angle values
            float medianAngle = angles[angles.Count / 2];
            float adjustedThreshold = medianAngle * 0.7f; // 70% of median angle

            // Count corners using adjusted threshold
            cornerCount = 0;
            inCorner = false;
            prevDirection = Vector3.zero;

            for (int i = 0; i <= samplePoints; i++)
            {
                float t = i / (float)samplePoints;
                Vector3 tangent = spline.EvaluateTangent(t);

                if (i > 0)
                {
                    Vector3 currentDirection = tangent.normalized;

                    if (prevDirection != Vector3.zero)
                    {
                        float dot = Vector3.Dot(prevDirection, currentDirection);
                        dot = Mathf.Clamp(dot, -1.0f, 1.0f);
                        float angle = Mathf.Acos(dot);

                        bool isCornerPoint = angle > adjustedThreshold;

                        if (isCornerPoint && !inCorner)
                        {
                            cornerCount++;
                            inCorner = true;
                        }
                        else if (!isCornerPoint && inCorner)
                        {
                            inCorner = false;
                        }
                    }

                    prevDirection = currentDirection;
                }
            }

            // Debug.Log($"Adjusted corner count: {cornerCount} (threshold: {adjustedThreshold * Mathf.Rad2Deg} degrees)");
        }

        return cornerCount;
    }

    private float CalculateAverageCornerRadius(TrackHandler handler)
    {
        if (handler.trackSpline == null || handler.trackSpline.Spline == null)
            return 0f;

        Spline spline = handler.trackSpline.Spline;
        List<float> cornerRadii = new List<float>();

        // Convert threshold to radians for angle calculations
        float thresholdRadians = cornerAngleThreshold * Mathf.Deg2Rad;

        // Sample points along the spline for curvature
        Vector3 prevDirection = Vector3.zero;
        for (int i = 0; i <= samplePoints; i++)
        {
            float t = i / (float)samplePoints;
            Vector3 currentPos = spline.EvaluatePosition(t);
            Vector3 tangent = spline.EvaluateTangent(t);

            // Skip the first point as we need two points to calculate direction
            if (i > 0)
            {
                // Calculate angle between previous and current direction
                Vector3 currentDirection = tangent.normalized;

                if (prevDirection != Vector3.zero)
                {
                    float dot = Vector3.Dot(prevDirection, currentDirection);
                    // Clamp dot product to valid range for acos
                    dot = Mathf.Clamp(dot, -1.0f, 1.0f);
                    float angle = Mathf.Acos(dot);

                    // If this is a corner point, calculate radius
                    if (angle > thresholdRadians)
                    {
                        float segmentLength = spline.GetLength() / samplePoints;
                        // Using the formula: radius = arc length / angle
                        float radius = segmentLength / angle;
                        cornerRadii.Add(radius);
                    }
                }

                prevDirection = currentDirection;
            }
        }

        return cornerRadii.Count > 0 ? cornerRadii.Average() : 0f;
    }

    private float FindLongestStraight(TrackHandler handler)
    {
        if (handler.trackSpline == null || handler.trackSpline.Spline == null)
            return 0f;

        Spline spline = handler.trackSpline.Spline;
        float longestStraight = 0f;
        float currentStraight = 0f;

        // Convert threshold to radians for angle calculations
        float thresholdRadians = cornerAngleThreshold * Mathf.Deg2Rad;

        // Sample points along the spline
        Vector3 prevDirection = Vector3.zero;
        Vector3 prevPos = Vector3.zero;

        for (int i = 0; i <= samplePoints; i++)
        {
            float t = i / (float)samplePoints;
            Vector3 currentPos = spline.EvaluatePosition(t);
            Vector3 tangent = spline.EvaluateTangent(t);

            // Skip the first point
            if (i > 0)
            {
                Vector3 currentDirection = tangent.normalized;
                float segmentLength = Vector3.Distance(prevPos, currentPos);

                if (prevDirection != Vector3.zero)
                {
                    float dot = Vector3.Dot(prevDirection, currentDirection);
                    dot = Mathf.Clamp(dot, -1.0f, 1.0f);
                    float angle = Mathf.Acos(dot);

                    if (angle <= thresholdRadians)
                    {
                        // We're on a straight section
                        currentStraight += segmentLength;
                    }
                    else
                    {
                        // We hit a corner
                        if (currentStraight >= minStraightLength)
                        {
                            longestStraight = Mathf.Max(longestStraight, currentStraight);
                        }
                        currentStraight = 0f;
                    }
                }

                prevDirection = currentDirection;
            }

            prevPos = currentPos;
        }

        // Check one more time at the end
        if (currentStraight >= minStraightLength)
        {
            longestStraight = Mathf.Max(longestStraight, currentStraight);
        }

        return longestStraight;
    }

    private float CalculateElevationChange(TrackHandler handler)
    {
        if (handler.trackSpline == null || handler.trackSpline.Spline == null)
            return 0f;

        Spline spline = handler.trackSpline.Spline;
        float minHeight = float.MaxValue;
        float maxHeight = float.MinValue;

        for (int i = 0; i <= samplePoints; i++)
        {
            float t = i / (float)samplePoints;
            Vector3 position = spline.EvaluatePosition(t);

            minHeight = Mathf.Min(minHeight, position.y);
            maxHeight = Mathf.Max(maxHeight, position.y);
        }

        return maxHeight - minHeight;
    }

    private float CalculateMinimumProximity(TrackHandler handler)
    {
        if (handler.trackSpline == null || handler.trackSpline.Spline == null)
            return 0f;

        Spline spline = handler.trackSpline.Spline;
        float minProximity = float.MaxValue;

        // Sample points along the spline
        Vector3[] points = new Vector3[samplePoints + 1];
        for (int i = 0; i <= samplePoints; i++)
        {
            float t = i / (float)samplePoints;
            points[i] = spline.EvaluatePosition(t);
        }

        // Check proximity between non-adjacent points
        for (int i = 0; i <= samplePoints; i++)
        {
            for (int j = i + 10; j <= samplePoints; j++)
            {
                // Skip adjacent points and near-adjacent points (10 points buffer)
                if (j - i <= 10)
                    continue;

                // Skip comparisons between end and start (first and last 10% of points)
                if ((i < samplePoints * 0.1f && j > samplePoints * 0.9f) ||
                    (j < samplePoints * 0.1f && i > samplePoints * 0.9f))
                    continue;

                float distance = Vector3.Distance(points[i], points[j]);
                minProximity = Mathf.Min(minProximity, distance);
            }
        }

        return minProximity;
    }

    private int CountCheckpoints(TrackHandler handler)
    {
        var checkpoints = handler.GetCheckpoints();
        return checkpoints != null ? checkpoints.Count : 0;
    }

    private (float cornerQuality, float straightQuality, float flowQuality, float layoutQuality, float lengthQuality, float proximityQuality, float overallQuality) GetTrackQualityScores(TrackHandler handler)
    {
        // Get the selected cycle from the handler
        var cycle = GetSelectedCycle(handler);
        if (cycle != null)
        {
            var voronoi = GetVoronoiDiagram(handler);
            if (voronoi != null)
            {
                Debug.Log($"Evaluating track with {handler.points.Count} points.");
                var qualities = voronoi.GetQualities(handler.points);
                Debug.Log($"Quality scores: {qualities}");
                return qualities;
            }
            Debug.LogWarning("Voronoi diagram is null.");
        }
        Debug.Log("Failed to get Voronoi diagram or cycle.");
        return (0, 0, 0, 0, 0, 0, 0);
    }

    private float CalculatePathLength(List<Vector2> points)
    {
        float length = 0;
        for (int i = 0; i < points.Count; i++)
        {
            Vector2 current = points[i];
            Vector2 next = points[(i + 1) % points.Count];
            length += Vector2.Distance(current, next);
        }
        return length;
    }

    private Cycle GetSelectedCycle(TrackHandler handler)
    {
        return handler.cycle;
    }

    private VoronoiDiagram GetVoronoiDiagram(TrackHandler handler)
    {
        return handler.voronoi;
    }

    private float CalculateStdDev(IEnumerable<float> values)
    {
        float avg = values.Average();
        float sumOfSquaresOfDifferences = values.Select(val => (val - avg) * (val - avg)).Sum();
        float variance = sumOfSquaresOfDifferences / values.Count();
        return Mathf.Sqrt(variance);
    }

    private float CalculateStdDev(IEnumerable<int> values)
    {
        return CalculateStdDev(values.Select(v => (float)v));
    }

    private void SaveResultsToFile()
    {
        if (results.Count == 0)
        {
            Debug.LogWarning("No results to save.");
            return;
        }

        try
        {
            string filePath = System.IO.Path.Combine(Application.persistentDataPath, resultsFilePath);
            using (System.IO.StreamWriter writer = new System.IO.StreamWriter(filePath, false))
            {
                // Write header
                writer.WriteLine("Strategy,Seed,Success,GenerationTime,TrackLength,CornerCount,AverageCornerRadius," +
                               "LongestStraightLength,ElevationChange,MinimumTrackProximity,OverallQualityScore," +
                               "CornerQuality,StraightQuality,FlowQuality,LayoutQuality,LengthQuality,ProximityQuality," +
                               "TrackWidth,MaxBankingAngle,CheckpointCount");

                // Write data
                foreach (var result in results)
                {
                    writer.WriteLine($"{result.strategy},{result.seed},{result.success},{result.generationTime:F3}," +
                                   $"{result.trackLength:F1},{result.cornerCount},{result.averageCornerRadius:F1}," +
                                   $"{result.longestStraightLength:F1},{result.elevationChange:F1},{result.minimumTrackProximity:F1}," +
                                   $"{result.overallQualityScore:F3},{result.cornerQuality:F3},{result.straightQuality:F3}," +
                                   $"{result.flowQuality:F3},{result.layoutQuality:F3},{result.lengthQuality:F3}," +
                                   $"{result.proximityQuality:F3},{result.trackWidth:F1},{result.maxBankingAngle:F1}," +
                                   $"{result.checkpointCount}");
                }
            }

            Debug.Log($"Results saved to: {filePath}");
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to save results: {e.Message}");
        }
    }

    // For testing in editor
    [ContextMenu("Generate Test Track")]
    public void GenerateTestTrack()
    {
        if (trackHandler != null)
        {
            trackHandler.GenerateTrack(GenerationStrategy.GridWithNoise);
            Debug.Log("Test track generated.");
        }
        else
        {
            Debug.LogError("TrackHandler reference is missing!");
        }
    }

    // Visualization methods
    private void OnDrawGizmos()
    {
        // Only draw if we're visualizing a test track
        if (!showTestTracksInScene || currentTestTrack == null)
            return;

        if (trackHandler == null || trackHandler.trackSpline == null || trackHandler.trackSpline.Spline == null)
            return;

        Spline spline = trackHandler.trackSpline.Spline;

        // Draw the spline path
        Gizmos.color = Color.green;
        Vector3 prevPos = spline.EvaluatePosition(0f);
        for (int i = 1; i <= 100; i++)
        {
            float t = i / 100f;
            Vector3 pos = spline.EvaluatePosition(t);
            Gizmos.DrawLine(prevPos, pos);
            prevPos = pos;
        }

        // Draw corners
        Vector3 prevDirection = Vector3.zero;
        for (int i = 0; i <= samplePoints; i++)
        {
            float t = i / (float)samplePoints;
            Vector3 pos = spline.EvaluatePosition(t);
            Vector3 tangent = spline.EvaluateTangent(t);

            if (i > 0)
            {
                Vector3 currentDirection = tangent.normalized;

                if (prevDirection != Vector3.zero)
                {
                    float dot = Vector3.Dot(prevDirection, currentDirection);
                    dot = Mathf.Clamp(dot, -1.0f, 1.0f);
                    float angle = Mathf.Acos(dot) * Mathf.Rad2Deg;

                    // If this is a corner point
                    if (angle > cornerAngleThreshold)
                    {
                        Gizmos.color = Color.red;
                        Gizmos.DrawSphere(pos, 2f);
                    }
                }

                prevDirection = currentDirection;
            }
        }
    }
}