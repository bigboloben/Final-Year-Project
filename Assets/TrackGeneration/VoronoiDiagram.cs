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
        private TrackGenerationParameters parameters;
        private List<Vector2> delaunayVertices; // Store unique vertices from triangulation

        public VoronoiDiagram(DelaunayTriangulation delaunay, List<Triangle> triangles,
                             TrackGenerationParameters parameters = null)
        {
            this.delaunayTriangles = triangles;
            this.cells = new List<VoronoiCell>();
            this.voronoiEdges = new List<Edge>();
            this.parameters = parameters ?? new TrackGenerationParameters();

            // Extract unique vertices from Delaunay triangulation
            this.delaunayVertices = ExtractUniqueVertices(triangles);

            GenerateVoronoiCells();
        }

        private List<Vector2> ExtractUniqueVertices(List<Triangle> triangles)
        {
            HashSet<Vector2> uniqueVertices = new HashSet<Vector2>(new Vector2EqualityComparer());

            foreach (Triangle triangle in triangles)
            {
                uniqueVertices.Add(triangle.P1);
                uniqueVertices.Add(triangle.P2);
                uniqueVertices.Add(triangle.P3);
            }

            return uniqueVertices.ToList();
        }

        private void GenerateVoronoiCells()
        {
            // For each vertex in the Delaunay triangulation
            foreach (Vector2 vertex in delaunayVertices)
            {
                // Find all triangles that share this vertex
                List<Triangle> adjacentTriangles = delaunayTriangles.Where(t =>
                    Vector2.Distance(t.P1, vertex) < 0.001f ||
                    Vector2.Distance(t.P2, vertex) < 0.001f ||
                    Vector2.Distance(t.P3, vertex) < 0.001f).ToList();

                if (adjacentTriangles.Count < 3)
                {
                    // Skip vertices with too few adjacent triangles (likely on the boundary)
                    continue;
                }

                // Sort triangles in clockwise or counter-clockwise order around the vertex
                List<Triangle> orderedTriangles = OrderTrianglesAroundVertex(vertex, adjacentTriangles);

                // Extract circumcenters to form Voronoi cell vertices
                List<Vector2> cellVertices = orderedTriangles.Select(t => t.CircumCenter).ToList();

                // Create Voronoi cell
                VoronoiCell cell = new VoronoiCell(cellVertices);
                cells.Add(cell);

                // Add Voronoi edges
                for (int i = 0; i < cellVertices.Count; i++)
                {
                    Vector2 start = cellVertices[i];
                    Vector2 end = cellVertices[(i + 1) % cellVertices.Count];

                    Edge voronoiEdge = new Edge(start, end);
                    if (!voronoiEdges.Any(e => e.Equals(voronoiEdge)))
                    {
                        voronoiEdges.Add(voronoiEdge);
                    }
                }
            }
        }

        private List<Triangle> OrderTrianglesAroundVertex(Vector2 vertex, List<Triangle> triangles)
        {
            List<TriangleAnglePair> anglePairs = new List<TriangleAnglePair>();

            // Calculate angle for each triangle
            foreach (Triangle triangle in triangles)
            {
                // Find center of the triangle
                Vector2 center = (triangle.P1 + triangle.P2 + triangle.P3) / 3;

                // Calculate angle between reference vector (1, 0) and vector from vertex to center
                Vector2 direction = center - vertex;
                float angle = Mathf.Atan2(direction.y, direction.x);

                // Convert to degrees and normalize to [0, 360)
                angle = angle * Mathf.Rad2Deg;
                if (angle < 0) angle += 360;

                anglePairs.Add(new TriangleAnglePair(triangle, angle));
            }

            // Sort triangles by angle
            anglePairs.Sort((a, b) => a.Angle.CompareTo(b.Angle));

            // Return ordered triangles
            return anglePairs.Select(pair => pair.Triangle).ToList();
        }

        // Helper class for ordering triangles
        private class TriangleAnglePair
        {
            public Triangle Triangle { get; private set; }
            public float Angle { get; private set; }

            public TriangleAnglePair(Triangle triangle, float angle)
            {
                Triangle = triangle;
                Angle = angle;
            }
        }

        // Helper class for vector equality comparison
        private class Vector2EqualityComparer : IEqualityComparer<Vector2>
        {
            private const float Epsilon = 0.001f;

            public bool Equals(Vector2 x, Vector2 y)
            {
                return Vector2.Distance(x, y) < Epsilon;
            }

            public int GetHashCode(Vector2 obj)
            {
                return (int)(obj.x * 100000) ^ (int)(obj.y * 100000);
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


            // Apply parameterized weights
            return (cornerQuality * parameters.CornerQualityWeight +
                    straightQuality * parameters.StraightQualityWeight +
                    flowQuality * parameters.FlowQualityWeight +
                    layoutQuality * parameters.LayoutQualityWeight +
                    lengthQuality * parameters.LengthQualityWeight +
                    proximityQuality * parameters.ProximityQualityWeight);
        }

        public (float cornerQuality, float straightQuality, float flowQuality, float layoutQuality, float lengthQuality, float proximityQuality, float overallQuality) GetQualities(List<Vector2> trackPoints)
        {
            if (trackPoints == null || trackPoints.Count < 4) return (0, 0, 0, 0, 0, 0, 0);
            float cornerQuality = EvaluateCorners(trackPoints);
            float straightQuality = EvaluateStraights(trackPoints);
            float flowQuality = EvaluateTrackFlow(trackPoints);
            float layoutQuality = EvaluateLayout(trackPoints);
            float lengthQuality = EvaluateLength(CalculatePathLength(trackPoints));
            float proximityQuality = EvaluateTrackProximity(trackPoints);
            // Calculate overall quality
            float overallQuality = (cornerQuality * parameters.CornerQualityWeight +
                                    straightQuality * parameters.StraightQualityWeight +
                                    flowQuality * parameters.FlowQualityWeight +
                                    layoutQuality * parameters.LayoutQualityWeight +
                                    lengthQuality * parameters.LengthQualityWeight +
                                    proximityQuality * parameters.ProximityQualityWeight);
            return (cornerQuality, straightQuality, flowQuality, layoutQuality, lengthQuality, proximityQuality, overallQuality);
        }
        private float EvaluateCorners(List<Vector2> points)
        {
            float tooSharpCorner = parameters.TooSharpCornerAngle;
            float sharpCorner = parameters.SharpCornerAngle;
            float mediumCorner = parameters.MediumCornerAngle;
            float wideCorner = parameters.WideCornerAngle;
            int desiredCorners = parameters.DesiredCornersCount;

            float[] desiredAngles = new float[] { sharpCorner, mediumCorner, wideCorner };
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

                //Debug.Log($"Angle at point {i}: {angle}");

                if (angle < 130)
                {
                    cornersCount++;

                    if (angle < tooSharpCorner)
                    {
                        cornerTypes["too_sharp"]++;
                        totalCornerQuality += 0.2f;
                    }
                    else
                    {
                        if (angle < sharpCorner) cornerTypes["sharp"]++;
                        else if (angle < mediumCorner) cornerTypes["medium"]++;
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
            float cornerCountQuality = 1.0f - Mathf.Min(1, Mathf.Abs(cornersCount - desiredCorners) / (float)desiredCorners);

            //Debug.Log($"Corners Count: {cornersCount}");
            //Debug.Log($"Total Corner Quality: {totalCornerQuality}");
            //Debug.Log($"Too Sharp Penalty: {tooSharpPenalty}");
            //Debug.Log($"Variety Score: {varietyScore}");
            //Debug.Log($"Corner Count Quality: {cornerCountQuality}");

            float finalQuality = (cornerCountQuality * 0.4f +
                                  (cornersCount > 0 ? totalCornerQuality / cornersCount : 0) * 0.3f +
                                  varietyScore * 0.3f) * tooSharpPenalty;

            //Debug.Log($"Final corner quality: {finalQuality}");
            return finalQuality;
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
            if (longestStraight > parameters.MaxStraightLength)
            {
                lengthPenalty = Mathf.Max(0, 1 - (longestStraight - parameters.MaxStraightLength) / parameters.MaxStraightLength);
            }

            float straightCountQuality = Mathf.Min(straightCount / (float)parameters.DesiredStraightCount, 1.0f);

            float straightLengthQuality = 0;
            foreach (float length in straightLengths)
            {
                if (length >= parameters.MinStraightLength && length <= parameters.MaxStraightLength)
                {
                    straightLengthQuality += 1.0f;
                }
                else if (length < parameters.MinStraightLength)
                {
                    straightLengthQuality += length / parameters.MinStraightLength;
                }
                else
                {
                    straightLengthQuality += parameters.MaxStraightLength / length;
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
            // Use parameterized ideal length
            if (length < parameters.MinTrackLength)
                return length / parameters.MinTrackLength;
            else if (length > parameters.MaxTrackLength)
                return parameters.MaxTrackLength / length;
            else
                return 1.0f - Mathf.Abs(length - parameters.IdealTrackLength) /
                    (parameters.MaxTrackLength - parameters.MinTrackLength);
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

            if (minDistance >= parameters.MinSafeDistance)
                return 1.0f;

            if (minDistance <= parameters.CriticalDistance)
                return 0.0f;

            return (minDistance - parameters.CriticalDistance) /
                   (parameters.MinSafeDistance - parameters.CriticalDistance);
        }

        private float MinimumDistanceBetweenSegments(Vector2 start1, Vector2 end1, Vector2 start2, Vector2 end2)
        {
            float dist1 = DistanceFromPointToSegment(start1, start2, end2);
            float dist2 = DistanceFromPointToSegment(end1, start2, end2);
            float dist3 = DistanceFromPointToSegment(start2, start1, end1);
            float dist4 = DistanceFromPointToSegment(end2, start1, end1);

            return Mathf.Min(Mathf.Min(dist1, dist2), Mathf.Min(dist3, dist4));
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

        public List<Edge> GetVoronoiEdges()
        {
            return voronoiEdges;
        }

        public Cycle SortCycles(List<Cycle> cycles)
        {
            Cycle bestTrack = cycles
                .OrderByDescending(cycle => CalculateTrackQuality(cycle.Points))
                .FirstOrDefault();

            return bestTrack;
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

        // Track optimization methods
        public Cycle OptimizeTrack(Cycle track)
        {
            if (track == null || track.Points.Count < 10) return track;

            Cycle optimizedTrack = new Cycle(new List<Vector2>(track.Points));

            optimizedTrack = SmoothSharpCorners(optimizedTrack);
            optimizedTrack = FixTightSections(optimizedTrack);
            optimizedTrack = OptimizeStraights(optimizedTrack);

            return optimizedTrack;
        }

        private Cycle SmoothSharpCorners(Cycle track)
        {
            List<Vector2> points = track.Points;
            List<Vector2> smoothedPoints = new List<Vector2>(points);

            for (int i = 0; i < points.Count; i++)
            {
                Vector2 prev = points[(i - 1 + points.Count) % points.Count];
                Vector2 current = points[i];
                Vector2 next = points[(i + 1) % points.Count];

                float angle = CalculateAngle(prev, current, next);

                // Apply stronger smoothing to sharper corners
                if (angle < parameters.TooSharpCornerAngle * 1.2f) // Add a small margin
                {
                    // Calculate smoothed position
                    Vector2 target = (prev + next) / 2;
                    float smoothFactor = Mathf.Lerp(0.5f, 0.2f, angle / parameters.TooSharpCornerAngle);
                    smoothedPoints[i] = Vector2.Lerp(current, target, smoothFactor);
                }
            }

            return new Cycle(smoothedPoints);
        }

        private Cycle OptimizeStraights(Cycle track)
        {
            List<Vector2> points = track.Points;
            bool[] isStraight = new bool[points.Count];

            // Identify straight sections
            for (int i = 0; i < points.Count; i++)
            {
                Vector2 prev = points[(i - 1 + points.Count) % points.Count];
                Vector2 current = points[i];
                Vector2 next = points[(i + 1) % points.Count];

                float angle = CalculateAngle(prev, current, next);
                isStraight[i] = angle > 160f; // Very straight section
            }

            // Optimize straight sections by removing redundant points
            List<Vector2> optimized = new List<Vector2>();
            for (int i = 0; i < points.Count; i++)
            {
                if (!isStraight[i] || !isStraight[(i + 1) % points.Count] || i % 2 == 0)
                {
                    optimized.Add(points[i]);
                }
            }

            // Only use optimized points if we didn't remove too many
            if (optimized.Count >= points.Count * 0.7f)
            {
                return new Cycle(optimized);
            }

            return track;
        }

        private Cycle FixTightSections(Cycle track)
        {
            List<Vector2> points = track.Points;
            List<Vector2> fixed_points = new List<Vector2>(points);

            // Identify and fix sections that are too close to each other
            for (int i = 0; i < points.Count; i++)
            {
                for (int j = i + 2; j < points.Count; j++)
                {
                    if (j == i + 1 || (i == 0 && j == points.Count - 1)) continue;

                    Vector2 pointI = points[i];
                    Vector2 pointJ = points[j];
                    float distance = Vector2.Distance(pointI, pointJ);

                    if (distance < parameters.MinSafeDistance)
                    {
                        // Calculate repulsion vector
                        Vector2 repulsion = (pointI - pointJ).normalized;
                        float strength = (parameters.MinSafeDistance - distance) * 0.25f;

                        // Apply repulsion
                        fixed_points[i] += repulsion * strength;
                        fixed_points[j] -= repulsion * strength;
                    }
                }
            }

            return new Cycle(fixed_points);
        }

        public List<VoronoiCell> GetCells()
        {
            return cells;
        }
    }
}