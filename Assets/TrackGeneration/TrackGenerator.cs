using System.Collections.Generic;
using UnityEngine;

public class RacetrackGenerator : MonoBehaviour
{
    public List<Vector3> controlPoints; // Control points for the track
    public int numberOfPointsPerSegment = 10; // Number of points for each segment
    public float trackWidth = 1.0f; // Width of the track

    private void Start()
    {
        controlPoints.Add(new Vector3(0, 0, 0));    // P0: Start point
        controlPoints.Add(new Vector3(5, 0, 10));   // P1: First control point (curve)
        controlPoints.Add(new Vector3(10, 0, 10));  // P2: Second control point (curve)
        controlPoints.Add(new Vector3(15, 0, 0));   // P3: End point
    
        GenerateTrack();
    }

    private void GenerateTrack()
    {
        Debug.Log("Generate Track");
        for (int i = 0; i < controlPoints.Count - 3; i += 3)
        {
            CreateTrackSegment(controlPoints[i], controlPoints[i + 1], controlPoints[i + 2], controlPoints[i + 3]);
            
        }
    }

    private void CreateTrackSegment(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
    {
        Debug.Log("Generate Track Segment");
        for (int j = 0; j <= numberOfPointsPerSegment; j++)
        {
            float t = j / (float)numberOfPointsPerSegment;
            Vector3 pointOnCurve = GetBezierPoint(t, p0, p1, p2, p3);
            // Here you would implement your track mesh or visual representation
        }
    }

    private Vector3 GetBezierPoint(float t, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
    {
        Debug.Log("Get Bezier Point");
        return Mathf.Pow(1 - t, 3) * p0 +
               3 * Mathf.Pow(1 - t, 2) * t * p1 +
               3 * (1 - t) * Mathf.Pow(t, 2) * p2 +
               Mathf.Pow(t, 3) * p3;
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        for (int i = 0; i < controlPoints.Count; i++)
        {
            Gizmos.DrawSphere(controlPoints[i], 0.5f);
        }

        Gizmos.color = Color.green;
        for (int i = 0; i < controlPoints.Count - 3; i += 3)
        {
            Vector3 p0 = controlPoints[i];
            Vector3 p1 = controlPoints[i + 1];
            Vector3 p2 = controlPoints[i + 2];
            Vector3 p3 = controlPoints[i + 3];

            for (int j = 0; j <= numberOfPointsPerSegment; j++)
            {
                float t = j / (float)numberOfPointsPerSegment;
                Vector3 pointOnCurve = GetBezierPoint(t, p0, p1, p2, p3);
                Gizmos.DrawSphere(pointOnCurve, 0.2f);
            }
        }
    }

}
