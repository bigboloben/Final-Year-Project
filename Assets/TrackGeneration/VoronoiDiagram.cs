using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Assets.TrackGeneration
{
    public class VoronoiDiagram
    {
        private List<Triangle> delaunayTriangles;
        private List<VoronoiCell> cells;
        private List<Edge> voronoiEdges;
        private const float IDEAL_TRACK_LENGTH = 1200f;
        private const float MIN_STRAIGHT_LENGTH = 50f;
        private const float MAX_STRAIGHT_LENGTH = 80f;
        private const float MIN_SAFE_DISTANCE = 40f;
        private const float CRITICAL_DISTANCE = 30f;
        private const int DESIRED_CORNERS = 8;

        public VoronoiDiagram(DelaunayTriangulation delaunay, List<Triangle> triangles)
        {
            delaunayTriangles = triangles;
            cells = new List<VoronoiCell>();
            voronoiEdges = new List<Edge>();
            GenerateVoronoiCells();
        }

        private void GenerateVoronoiCells()
        {
            foreach (Triangle triangle in delaunayTriangles)
            {
                var neighbors = delaunayTriangles.Where(t => t != triangle && t.ShareEdge(triangle)).ToList();
                List<Vector2> cellVertices = new List<Vector2>();
                cellVertices.Add(triangle.CircumCenter);

                foreach (var neighbor in neighbors)
                {
                    cellVertices.Add(neighbor.CircumCenter);
                    Edge voronoiEdge = new Edge(triangle.CircumCenter, neighbor.CircumCenter);
                    if (!voronoiEdges.Any(e => e.Equals(voronoiEdge)))
                    {
                        voronoiEdges.Add(voronoiEdge);
                    }
                }

                cells.Add(new VoronoiCell(cellVertices));
            }
        }

        public float CalculateTrackQuality(List<Vector2> trackPoints)
        {
            if (trackPoints == null || trackPoints.Count < 4) return 0;

            float cornerQuality = EvaluateCorners(trackPoints);
            float straightQuality = EvaluateStraights(trackPoints);
            float flowQuality = EvaluateTrackFlow(trackPoints);
            float layoutQuality = EvaluateLayout(trackPoints);
            float lengthQuality = EvaluateLength(CalculatePathLength(trackPoints));
            float proximityQuality = EvaluateTrackProximity(trackPoints);

            return (cornerQuality * 0.25f +
                    straightQuality * 0.1f +
                    flowQuality * 0.1f +
                    layoutQuality * 0.1f +
                    lengthQuality * 0.25f +
                    proximityQuality * 0.2f);
        }

        private float EvaluateTrackProximity(List<Vector2> points)
        {
            float minDistance = float.MaxValue;

            for (int i = 0; i < points.Count; i++)
            {
                Vector2 start1 = points[i];
                Vector2 end1 = points[(i + 1) % points.Count];

                for (int j = i + 2; j < points.Count; j++)
                {
                    if (j == i + 1 || (i == 0 && j == points.Count - 1)) continue;

                    Vector2 start2 = points[j];
                    Vector2 end2 = points[(j + 1) % points.Count];

                    float distance = MinimumDistanceBetweenSegments(start1, end1, start2, end2);
                    minDistance = Mathf.Min(minDistance, distance);
                }
            }

            if (minDistance >= MIN_SAFE_DISTANCE)
                return 1.0f;

            if (minDistance <= CRITICAL_DISTANCE)
                return 0.0f;

            return (minDistance - CRITICAL_DISTANCE) / (MIN_SAFE_DISTANCE - CRITICAL_DISTANCE);
        }

        private float MinimumDistanceBetweenSegments(Vector2 start1, Vector2 end1, Vector2 start2, Vector2 end2)
        {
            float dist1 = DistanceFromPointToSegment(start1, start2, end2);
            float dist2 = DistanceFromPointToSegment(end1, start2, end2);
            float dist3 = DistanceFromPointToSegment(start2, start1, end1);
            float dist4 = DistanceFromPointToSegment(end2, start1, end1);

            return Mathf.Min(Mathf.Min(dist1, dist2), Mathf.Min(dist3, dist4));
        }

        private float EvaluateCorners(List<Vector2> points)
        {
            const float TOO_SHARP_CORNER = 25f;
            const float SHARP_CORNER = 45f;
            const float MEDIUM_CORNER = 90f;
            const float WIDE_CORNER = 120f;

            float[] desiredAngles = new float[] { SHARP_CORNER, MEDIUM_CORNER, WIDE_CORNER };
            int cornersCount = 0;
            float totalCornerQuality = 0;

            Dictionary<string, int> cornerTypes = new Dictionary<string, int>
        {
            {"too_sharp", 0},
            {"sharp", 0},
            {"medium", 0},
            {"wide", 0}
        };

            for (int i = 0; i < points.Count; i++)
            {
                Vector2 prev = points[(i - 1 + points.Count) % points.Count];
                Vector2 current = points[i];
                Vector2 next = points[(i + 1) % points.Count];
                float angle = CalculateAngle(prev, current, next);

                if (angle < 130)
                {
                    cornersCount++;

                    if (angle < TOO_SHARP_CORNER)
                    {
                        cornerTypes["too_sharp"]++;
                        totalCornerQuality += 0.2f;
                    }
                    else
                    {
                        if (angle < SHARP_CORNER) cornerTypes["sharp"]++;
                        else if (angle < MEDIUM_CORNER) cornerTypes["medium"]++;
                        else cornerTypes["wide"]++;

                        float bestAngleMatch = desiredAngles.Min(a => Mathf.Abs(angle - a));
                        float cornerQuality = 1.0f - (bestAngleMatch / 130f);
                        totalCornerQuality += cornerQuality;
                    }
                }
            }

            float tooSharpPenalty = cornerTypes["too_sharp"] > 0 ?
                Mathf.Max(0, 1 - (cornerTypes["too_sharp"]) * 2f) : 1.0f;

            float varietyScore = cornerTypes.Where(kvp => kvp.Key != "too_sharp" && kvp.Value > 0).Count() / 3.0f;
            float cornerCountQuality = 1.0f - Mathf.Abs(cornersCount - DESIRED_CORNERS) / DESIRED_CORNERS;

            return (cornerCountQuality * 0.4f +
                    (cornersCount > 0 ? totalCornerQuality / cornersCount : 0) * 0.3f +
                    varietyScore * 0.3f) * tooSharpPenalty;
        }

        private float EvaluateStraights(List<Vector2> points)
        {
            float longestStraight = 0;
            float totalStraightLength = 0;
            int straightCount = 0;
            List<float> straightLengths = new List<float>();

            for (int i = 0; i < points.Count; i++)
            {
                Vector2 start = points[i];
                Vector2 end = points[(i + 1) % points.Count];
                float length = Vector2.Distance(start, end);

                if (IsStraightSection(points, i))
                {
                    straightCount++;
                    totalStraightLength += length;
                    longestStraight = Mathf.Max(longestStraight, length);
                    straightLengths.Add(length);
                }
            }

            float lengthPenalty = 1.0f;
            if (longestStraight > MAX_STRAIGHT_LENGTH)
            {
                lengthPenalty = Mathf.Max(0, 1 - (longestStraight - MAX_STRAIGHT_LENGTH) / MAX_STRAIGHT_LENGTH);
            }

            float straightCountQuality = Mathf.Min(straightCount / 3.0f, 1.0f);

            float straightLengthQuality = 0;
            foreach (float length in straightLengths)
            {
                if (length >= MIN_STRAIGHT_LENGTH && length <= MAX_STRAIGHT_LENGTH)
                {
                    straightLengthQuality += 1.0f;
                }
                else if (length < MIN_STRAIGHT_LENGTH)
                {
                    straightLengthQuality += length / MIN_STRAIGHT_LENGTH;
                }
                else
                {
                    straightLengthQuality += MAX_STRAIGHT_LENGTH / length;
                }
            }
            straightLengthQuality = straightLengths.Count > 0 ? straightLengthQuality / straightLengths.Count : 0;

            return (straightCountQuality * 0.3f +
                    straightLengthQuality * 0.4f +
                    lengthPenalty * 0.3f);
        }

        private bool IsStraightSection(List<Vector2> points, int startIndex)
        {
            Vector2 prev = points[(startIndex - 1 + points.Count) % points.Count];
            Vector2 current = points[startIndex];
            Vector2 next = points[(startIndex + 1) % points.Count];

            float angle = CalculateAngle(prev, current, next);
            return angle > 110;
        }

        private float EvaluateTrackFlow(List<Vector2> points)
        {
            float flowQuality = 0;
            int segments = points.Count;

            for (int i = 0; i < segments; i++)
            {
                Vector2 prev = points[(i - 1 + segments) % segments];
                Vector2 current = points[i];
                Vector2 next = points[(i + 1) % segments];
                Vector2 nextNext = points[(i + 2) % segments];

                float angle1 = CalculateAngle(prev, current, next);
                float angle2 = CalculateAngle(current, next, nextNext);

                float angleChange = Mathf.Abs(angle1 - angle2);
                flowQuality += 1.0f - (angleChange / 180.0f);
            }

            return flowQuality / segments;
        }

        private float EvaluateLayout(List<Vector2> points)
        {
            float minX = points.Min(p => p.x);
            float maxX = points.Max(p => p.x);
            float minY = points.Min(p => p.y);
            float maxY = points.Max(p => p.y);

            float width = maxX - minX;
            float height = maxY - minY;
            float area = width * height;

            float trackLength = CalculatePathLength(points);
            float idealArea = trackLength * trackLength / 16;

            return Mathf.Min(area / idealArea, 1.0f);
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

        private float EvaluateLength(float length)
        {
            return 1.0f - Mathf.Abs(length - IDEAL_TRACK_LENGTH) / IDEAL_TRACK_LENGTH;
        }

        private float CalculateAngle(Vector2 p1, Vector2 p2, Vector2 p3)
        {
            Vector2 v1 = p1 - p2;
            Vector2 v2 = p3 - p2;

            v1.Normalize();
            v2.Normalize();

            float dot = Mathf.Clamp(Vector2.Dot(v1, v2), -1f, 1f);
            return Mathf.Acos(dot) * Mathf.Rad2Deg;
        }

        public List<Edge> GetVoronoiEdges()
        {
            return voronoiEdges;
        }

        public Cycle SortCycles(List<Cycle> cycles)
        {
            Cycle bestTrack = cycles
                .OrderByDescending(cycle => CalculateTrackQuality(cycle.Points))
                .FirstOrDefault();
            if (bestTrack == null)
            {
                Debug.LogError("No Tracks To Select");
            }

            return bestTrack;
        }

        private float DistanceFromPointToSegment(Vector2 point, Vector2 lineStart, Vector2 lineEnd)
        {
            Vector2 line = lineEnd - lineStart;
            Vector2 pointToStart = point - lineStart;

            float lineLengthSquared = line.sqrMagnitude;

            if (lineLengthSquared == 0)
                return Vector2.Distance(point, lineStart);

            float t = Mathf.Clamp01(Vector2.Dot(pointToStart, line) / lineLengthSquared);

            Vector2 projection = new Vector2(
                lineStart.x + t * line.x,
                lineStart.y + t * line.y
            );

            return Vector2.Distance(point, projection);
        }
    }
}