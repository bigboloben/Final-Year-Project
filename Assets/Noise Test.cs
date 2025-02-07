using UnityEngine;
using System.Collections.Generic;
using System;

public class PerlinNoiseTrack : MonoBehaviour
{
    public int numPoints = 100;  // Number of points for the oval
    public float scale = 0.1f;     // Scale for the Perlin noise
    public float centerX = -300f;   // Center X of the oval
    public float centerY = 0f;   // Center Y of the oval
    public float radiusX = 300f;   // Radius X for the oval
    public float radiusY = 500f;   // Radius Y for the oval
    public float noiseStrength = 30f; // Strength of the Perlin noise
    public int bezierSegmentCount = 10; // Number of segments per Bezier curve
    public List<Vector3> trackPoints; // List of track points (control points for Bezier curves)

    void Start()
    {
        // Generate oval points
        List<Vector3> ovalPoints = GenerateOvalPoints(centerX, centerY, radiusX, radiusY, numPoints);

        // Generate track points with Perlin noise applied to both X and Z values
        trackPoints = GenerateTrack(ovalPoints, scale, noiseStrength);

        // Smooth the track points with Bezier curves
        List<Vector3> smoothedTrackPoints = SmoothTrackWithBezier(trackPoints);
    }

    // Draw lines between the points in the Scene view using Gizmos
    void OnDrawGizmos()
    {
        if (trackPoints != null && trackPoints.Count > 1)
        {
            // Set Gizmos color for drawing the lines
            Gizmos.color = Color.red;

            // Draw lines between consecutive track points
            for (int i = 0; i < trackPoints.Count - 1; i++)
            {
                Vector3 start = trackPoints[i];
                Vector3 end = trackPoints[i + 1];
                Gizmos.DrawLine(start, end);
            }
        }
    }

    // Generate track with Perlin noise applied to both X and Z values of the base points
    public static List<Vector3> GenerateTrack(List<Vector3> basePoints, float noiseScale, float noiseStrength)
    {
        int numPoints = basePoints.Count;
        List<Vector3> points = new List<Vector3>();
        for (int i = 0; i < numPoints; i++)
        {
            // Apply Perlin noise to both X and Z values of the track points
            // Use the base position and noiseScale for more variation
            float x = basePoints[i].x + Mathf.PerlinNoise(basePoints[i].x * noiseScale, basePoints[i].z * noiseScale) * noiseStrength;  // Add horizontal variation to X
            float z = basePoints[i].z + Mathf.PerlinNoise(basePoints[i].z * noiseScale, basePoints[i].x * noiseScale) * noiseStrength;  // Add horizontal variation to Z
            points.Add(new Vector3(x, basePoints[i].y, z)); // Keep Y constant, modify X and Z
        }
        return points;
    }

    // Smooth track points with Bezier curve interpolation
    private List<Vector3> SmoothTrackWithBezier(List<Vector3> trackPoints)
    {
        List<Vector3> smoothedPoints = new List<Vector3>();

        // Group track points into sets of four (for cubic Bezier curves)
        for (int i = 0; i < trackPoints.Count - 3; i++)
        {
            Vector3 p0 = trackPoints[i];
            Vector3 p1 = trackPoints[i + 1];
            Vector3 p2 = trackPoints[i + 2];
            Vector3 p3 = trackPoints[i + 3];

            // Create Bezier curve between control points p0, p1, p2, and p3
            for (int j = 0; j <= bezierSegmentCount; j++)
            {
                float t = j / (float)bezierSegmentCount;
                Vector3 pointOnCurve = GetBezierPoint(t, p0, p1, p2, p3);
                smoothedPoints.Add(pointOnCurve);
            }
        }
        return smoothedPoints;
    }

    // Get a point on a cubic Bezier curve
    private Vector3 GetBezierPoint(float t, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
    {
        return Mathf.Pow(1 - t, 3) * p0 +
               3 * Mathf.Pow(1 - t, 2) * t * p1 +
               3 * (1 - t) * Mathf.Pow(t, 2) * p2 +
               Mathf.Pow(t, 3) * p3;
    }

    // Generate points along an oval
    public static List<Vector3> GenerateOvalPoints(float cx, float cy, float rx, float ry, int numPoints)
    {
        List<Vector3> points = new List<Vector3>();
        for (int i = 0; i < numPoints; i++)
        {
            float theta = (float)(2 * Math.PI * i / numPoints); // Angle in radians
            float x = cx + rx * (float)Math.Cos(theta);
            float z = cy + ry * (float)Math.Sin(theta); // Use Z for vertical displacement
            points.Add(new Vector3(x, 0, z)); // Create a 2D point in the X-Z plane
        }
        return points;
    }
}
