using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Assets.TrackGeneration
{
    public class Triangle
    {
        public Vector2 P1, P2, P3;
        public Vector2 CircumCenter { get; private set; }
        private float CircumRadius;

        public Triangle(Vector2 p1, Vector2 p2, Vector2 p3)
        {
            P1 = p1;
            P2 = p2;
            P3 = p3;
            CalculateCircumcircle();
        }

        public void CalculateCircumcircle()
        {
            // Calculate circumcenter using determinant method
            float D = 2 * (P1.x * (P2.y - P3.y) + P2.x * (P3.y - P1.y) + P3.x * (P1.y - P2.y));

            // Check for collinear points
            if (Mathf.Abs(D) < Mathf.Epsilon)
            {
                throw new ArgumentException("The points are collinear and cannot form a triangle");
            }

            float Dx = ((P1.x * P1.x + P1.y * P1.y) * (P2.y - P3.y) +
                       (P2.x * P2.x + P2.y * P2.y) * (P3.y - P1.y) +
                       (P3.x * P3.x + P3.y * P3.y) * (P1.y - P2.y));

            float Dy = ((P1.x * P1.x + P1.y * P1.y) * (P3.x - P2.x) +
                       (P2.x * P2.x + P2.y * P2.y) * (P1.x - P3.x) +
                       (P3.x * P3.x + P3.y * P3.y) * (P2.x - P1.x));

            CircumCenter = new Vector2(Dx / D, Dy / D);
            CircumRadius = Vector2.Distance(CircumCenter, P1);
        }

        public bool IsPointInCircumcircle(Vector2 point)
        {
            float distanceSquared = (point.x - CircumCenter.x) * (point.x - CircumCenter.x) +
                                  (point.y - CircumCenter.y) * (point.y - CircumCenter.y);
            return distanceSquared <= CircumRadius * CircumRadius;
        }

        public bool ContainsPoint(Vector2 point)
        {
            return Vector2.Distance(P1, point) < 0.001f ||
                   Vector2.Distance(P2, point) < 0.001f ||
                   Vector2.Distance(P3, point) < 0.001f;
        }

        public List<Edge> GetEdges()
        {
            return new List<Edge>
            {
                new Edge(P1, P2),
                new Edge(P2, P3),
                new Edge(P3, P1)
            };
        }

        public bool ContainsEdge(Edge edge) => GetEdges().Any(e => e.Equals(edge));

        public bool ShareEdge(Triangle toCompare)
        {
            var edges1 = this.GetEdges();
            var edges2 = toCompare.GetEdges();

            foreach (var edge1 in edges1)
            {
                foreach (var edge2 in edges2)
                {
                    if (edge1.Equals(edge2))
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        public bool SharesVertex(Vector2 point) => ContainsPoint(point);
    }
}