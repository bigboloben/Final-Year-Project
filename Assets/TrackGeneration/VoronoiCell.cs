using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Assets.TrackGeneration
{
    public class VoronoiCell
    {
        private List<Vector2> vertices;
        private List<Edge> edges;

        public VoronoiCell(List<Vector2> cellVertices)
        {
            vertices = cellVertices;
            GenerateEdges();
        }

        private void GenerateEdges()
        {
            edges = new List<Edge>();
            for (int i = 0; i < vertices.Count; i++)
            {
                edges.Add(new Edge(vertices[i], vertices[(i + 1) % vertices.Count]));
            }
        }

        public bool ContainsPoint(Vector2 point)
        {
            bool inside = false;
            for (int i = 0, j = vertices.Count - 1; i < vertices.Count; j = i++)
            {
                if (((vertices[i].y > point.y) != (vertices[j].y > point.y)) &&
                    (point.x < (vertices[j].x - vertices[i].x) * (point.y - vertices[i].y) /
                    (vertices[j].y - vertices[i].y) + vertices[i].x))
                {
                    inside = !inside;
                }
            }
            return inside;
        }

        public float DistanceToNearestEdge(Vector2 point)
        {
            return edges.Min(edge => DistanceToEdge(point, edge));
        }

        private float DistanceToEdge(Vector2 point, Edge edge)
        {
            Vector2 edgeVector = edge.End - edge.Start;
            Vector2 pointVector = point - edge.Start;
            float edgeLengthSquared = edgeVector.sqrMagnitude;
            float dot = Vector2.Dot(pointVector, edgeVector);
            float t = Mathf.Clamp01(dot / edgeLengthSquared);

            Vector2 projection = new Vector2(
                edge.Start.x + t * edgeVector.x,
                edge.Start.y + t * edgeVector.y
            );

            return Vector2.Distance(point, projection);
        }

        public float CalculateArea()
        {
            float area = 0;
            for (int i = 0; i < vertices.Count; i++)
            {
                int j = (i + 1) % vertices.Count;
                area += vertices[i].x * vertices[j].y;
                area -= vertices[j].x * vertices[i].y;
            }
            return Mathf.Abs(area) / 2;
        }

        public bool SharesEdgeWith(VoronoiCell other)
        {
            return edges.Any(edge => other.edges.Any(otherEdge =>
                Vector2.Distance(edge.Start, otherEdge.Start) < 0.001f &&
                Vector2.Distance(edge.End, otherEdge.End) < 0.001f ||
                Vector2.Distance(edge.Start, otherEdge.End) < 0.001f &&
                Vector2.Distance(edge.End, otherEdge.Start) < 0.001f));
        }
    }
}