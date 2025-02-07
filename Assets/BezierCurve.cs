using UnityEngine;

public class BezierCurve : MonoBehaviour
{
    // Control points for the Bezier curve
    public Vector3 point0; // Start point
    public Vector3 point1; // Control point 1
    public Vector3 point2; // Control point 2
    public Vector3 point3; // End point
    public int numberOfPoints = 10; // Number of points to calculate along the curve

    // Function to get a point on the Bezier curve
    public Vector3 GetBezierPoint(float t)
    {
        // Cubic Bezier formula
        return Mathf.Pow(1 - t, 3) * point0 +
               3 * Mathf.Pow(1 - t, 2) * t * point1 +
               3 * (1 - t) * Mathf.Pow(t, 2) * point2 +
               Mathf.Pow(t, 3) * point3;
    }

    // Function to visualize the Bezier curve
    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;

        // Draw the Bezier curve
        for (int i = 0; i <= numberOfPoints; i++)
        {
            float t = i / (float)numberOfPoints;
            Vector3 pointOnCurve = GetBezierPoint(t);
            Gizmos.DrawSphere(pointOnCurve, 0.5f);
        }

        // Draw lines connecting control points
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(point0, point1);
        Gizmos.DrawLine(point1, point2);
        Gizmos.DrawLine(point2, point3);
    }
}
