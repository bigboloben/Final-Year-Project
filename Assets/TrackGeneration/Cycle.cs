using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Splines;

namespace Assets.TrackGeneration
{
    public class Cycle
    {
        public List<Vector2> Points { get; private set; }
        public float Length { get; private set; }
        public float MinAngle { get; private set; }
        public int Corners { get; private set; }
        public bool Overlaps { get; private set; }
        public Vector2 OverlapsAt { get; private set; }

        public Cycle(List<Vector2> points)
        {
            Points = new List<Vector2>(points);
            CalculateLengthAndMinAngle();
            Overlaps = DoesOverlap();
        }

        private void CalculateLengthAndMinAngle()
        {
            Length = 0f;
            MinAngle = float.MaxValue;
            Corners = 0;

            for (int i = 0; i < Points.Count; i++)
            {
                Vector2 current = Points[i];
                Vector2 next = Points[(i + 1) % Points.Count];
                Length += Vector2.Distance(current, next);

                if (Points.Count >= 3)
                {
                    Vector2 prev = Points[(i - 1 + Points.Count) % Points.Count];
                    float angle = CalculateAngle(prev, current, next);
                    MinAngle = Mathf.Min(MinAngle, angle);
                    if (angle < 100)
                    {
                        Corners++;
                    }
                }
            }
        }

        private float CalculateAngle(Vector2 prev, Vector2 current, Vector2 next)
        {
            Vector2 v1 = prev - current;
            Vector2 v2 = next - current;

            v1.Normalize();  // Unity's Vector2 has Normalize() as a method
            v2.Normalize();

            float dotProduct = Mathf.Clamp(Vector2.Dot(v1, v2), -1f, 1f);
            float angle = Mathf.Acos(dotProduct) * Mathf.Rad2Deg;
            return angle;
        }

        public bool DoesOverlap()
        {
            int n = Points.Count;

            for (int i = 0; i < n; i++)
            {
                Vector2 A = Points[i];
                Vector2 B = Points[(i + 1) % n];

                for (int j = i + 2; j < n; j++)
                {
                    if (j == i + 1 || (i == 0 && j == n - 1))
                        continue;

                    Vector2 C = Points[j];
                    Vector2 D = Points[(j + 1) % n];

                    Vector2 intersect = DoSegmentsIntersect(A, B, C, D);
                    OverlapsAt = intersect;
                    if (intersect != Vector2.zero)
                        return true;
                }
            }

            return false;
        }

        private Vector2 DoSegmentsIntersect(Vector2 A, Vector2 B, Vector2 C, Vector2 D)
        {
            float dx1 = B.x - A.x;  // Using .x instead of .X for Unity
            float dy1 = B.y - A.y;  // Using .y instead of .Y for Unity
            float dx2 = D.x - C.x;
            float dy2 = D.y - C.y;

            float denom = dx1 * dy2 - dy1 * dx2;

            if (Mathf.Abs(denom) < 1e-9f)
                return Vector2.zero;

            float dx3 = C.x - A.x;
            float dy3 = C.y - A.y;

            float t = (dx3 * dy2 - dy3 * dx2) / denom;
            float u = (dx3 * dy1 - dy3 * dx1) / denom;

            if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
            {
                float intersectionX = A.x + t * dx1;
                float intersectionY = A.y + t * dy1;

                Vector2 intersect = new Vector2(intersectionX, intersectionY);
                foreach (Vector2 point in Points)
                {
                    if (Mathf.Approximately(point.x, intersect.x) && Mathf.Approximately(point.y, intersect.y))
                    {
                        return Vector2.zero;
                    }
                }
                return intersect;
            }

            return Vector2.zero;
        }

        public List<Vector2> GetSmoothedPoints(int numberOfPoints)
        {
            List<Vector2> curvePoints = new List<Vector2>();
            Points.RemoveAt(Points.Count - 1);
            //curvePoints.Capacity = Points.Count * numberOfPoints;
            for (int i = 0; i < Points.Count; i++)
            {
                //i %= Points.Count;
                List<Vector2> controlPoints = new List<Vector2>();
                int h = (int) Mathf.Repeat(i - 1, Points.Count);
                int j = (int) Mathf.Repeat(i + 1, Points.Count);
                Debug.Log($"h{h}, i{i}, j{j}");
                Vector2 v0 = Points[h];
                Vector2 v1 = Points[i];
                Vector2 v2 = Points[j];

                controlPoints = GetControlPoints(v0, v1, v2);
                Debug.Log($"v0{v0.ToString()}, v1{v1.ToString()}, v2{v2.ToString()}");
                curvePoints.AddRange(GetBezierCurve(controlPoints[0], controlPoints[1], controlPoints[2], controlPoints[3], numberOfPoints));

            }
            Debug.Log($"Points: {string.Join(", ", Points.Select(p => $"({p.x}, {p.y})"))}");
            Points = curvePoints;
            Debug.Log($"Points: {string.Join(", ", Points.Select(p => $"({p.x}, {p.y})"))}");
            return curvePoints;
        }

        public List<Vector2> GetControlPoints(Vector2 v0, Vector2 v1, Vector2 v2)
        {
            float pointRatio = 0.6f;
            float controlRatio = 0.8f;
            if (CalculateAngle(v0, v1, v2) < 50)
            {
                pointRatio = 0.5f;
                controlRatio = 0.55f;
            } 
            Vector2 p0 = Vector2.Lerp(v0, v1, pointRatio);
            Vector2 p1 = Vector2.Lerp(v0, v1, controlRatio);

            Vector2 p2 = Vector2.Lerp(v2, v1, controlRatio);
            Vector2 p3 = Vector2.Lerp(v2, v1, pointRatio);

            return new List<Vector2> { p0, p1, p2, p3 };
        }

        public List<Vector2> GetBezierCurve(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3, int numberOfPoints)
        {
            List<Vector2> curvePoints = new List<Vector2>();
            float step = (1 / (float)numberOfPoints);
            Debug.Log(step);
            for (float t = 0; t <= 1; t += step)
            {
                Vector2 A = Vector2.Lerp(p0, p1, t);
                Vector2 B = Vector2.Lerp(p1, p2, t);
                Vector2 C = Vector2.Lerp(p2, p3, t);

                Vector2 D = Vector2.Lerp(A, B, t);
                Vector2 E = Vector2.Lerp(B, C, t);

                Vector2 curvePoint = Vector2.Lerp(D, E, t);

                curvePoints.Add(curvePoint);
            }
            return curvePoints;
        }

        public SplineContainer CreateSmoothedSpline(GameObject targetObject, List<float> heights = null)
        {
            // Remove last point since we're making a closed loop
            Points.RemoveAt(Points.Count - 1);
            SplineContainer splineContainer = targetObject.GetComponent<SplineContainer>();
            if (splineContainer == null)
            {
                splineContainer = targetObject.AddComponent<SplineContainer>();
            }

            //heights = GeneratePerlinHeights(Points.Count * 2);

            // If no heights provided, default to zero
            if (heights == null)
            {
                heights = new List<float>(new float[Points.Count * 2]); // *2 because we create 2 knots per point
            }

            Spline spline = new Spline();
            for (int i = 0; i < Points.Count; i++)
            {
                List<Vector2> controlPoints = new List<Vector2>();
                int h = (int)Mathf.Repeat(i - 1, Points.Count);
                int j = (int)Mathf.Repeat(i + 1, Points.Count);
                Debug.Log($"h{h}, i{i}, j{j}");

                Vector2 v0 = Points[h];
                Vector2 v1 = Points[i];
                Vector2 v2 = Points[j];
                Vector2 knotPointA = GetKnotPoints(v0, v1);
                Vector2 knotPointB = GetKnotPoints(v2, v1);
                Vector2 outTangentA = GetTangent(v0, v1, knotPointA);
                Vector2 inTangentB = GetTangent(v2, v1, knotPointB);
                Vector2 inTangentA = -outTangentA;
                Vector2 outTangentB = -inTangentB;

                // Get heights for current knots (2 knots per point)
                float heightA = heights[i * 2];
                float heightB = heights[i * 2 + 1];

                // Create knots with varying heights
                BezierKnot knotA = new BezierKnot
                (
                    new float3(knotPointA.x, heightA, knotPointA.y),
                    new float3(inTangentA.x, heightA, inTangentA.y),
                    new float3(outTangentA.x, heightA, outTangentA.y)
                );

                BezierKnot knotB = new BezierKnot
                (
                    new float3(knotPointB.x, heightB, knotPointB.y),
                    new float3(inTangentB.x, heightB, inTangentB.y),
                    new float3(outTangentB.x, heightB, outTangentB.y)
                );

                spline.Add(knotA);
                spline.Add(knotB);
            }

            splineContainer.Spline = spline;
            splineContainer.Spline.Closed = true;
            return splineContainer;
        }

        public List<float> GeneratePerlinHeights(int numKnots, float minHeight = 0f, float maxHeight = 10f, float noiseScale = 0.3f)
        {
            List<float> heights = new List<float>();

            for (int i = 0; i < numKnots; i++)
            {
                // Use Perlin noise for smooth random variation
                float t = i * noiseScale;
                float noise = Mathf.PerlinNoise(t, 0f);

                // Scale to desired height range
                float height = Mathf.Lerp(minHeight, maxHeight, noise);
                heights.Add(height);
            }

            return heights;
        }

        public Vector2 GetKnotPoints(Vector2 a, Vector2 b)
        {
            return Vector2.Lerp(a, b, 0.6f);
        }

        public Vector2 GetTangent(Vector2 a, Vector2 b, Vector2 knot)
        {
            Vector2 controlPoint = Vector2.Lerp(a,b,0.8f);
            Vector2 tangent = controlPoint - knot;
            return tangent;
        }

        // Optional: Method to get points along the spline if needed
        public List<Vector2> GetPointsAlongSpline(SplineContainer splineContainer, int numberOfPoints)
        {
            List<Vector2> points = new List<Vector2>();
            float step = 1f / numberOfPoints;

            for (float t = 0; t <= 1; t += step)
            {
                Vector3 position = splineContainer.EvaluatePosition(t);
                points.Add(new Vector2(position.x, position.z));
            }

            return points;
        }
    }
}