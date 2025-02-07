using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Assets.TrackGeneration
{
    public class DelaunayTriangulation
    {
        List<Vector2> points;
        Graph graph;
        List<Triangle> triangles;
        List<Edge> edges;
        Triangle superTriangle;

        public DelaunayTriangulation(List<Vector2> points)
        {
            this.points = points;
            triangles = new List<Triangle>();
            edges = new List<Edge>();
            graph = new Graph();
        }

        public Graph Compute()
        {
            Triangle superTriangle = MakeSuperTriangle();
            triangles.Add(superTriangle);

            foreach (Vector2 point in points)
            {
                AddPoint(point);
            }

            foreach (Triangle triangle in triangles)
            {
                if (triangle.SharesVertex(superTriangle.P1) ||
                    triangle.SharesVertex(superTriangle.P2) ||
                    triangle.SharesVertex(superTriangle.P3))
                {
                    continue;
                }
                else if (triangle.ShareEdge(superTriangle))
                {
                    continue;
                }

                foreach (Edge edge in triangle.GetEdges())
                {
                    graph.AddEdge(edge.Start, edge.End);
                }
            }

            return graph;
        }

        private Triangle MakeSuperTriangle()
        {
            float minX = points.Min(p => p.x);
            float minY = points.Min(p => p.y);
            float maxX = points.Max(p => p.x);
            float maxY = points.Max(p => p.y);

            float dx = (maxX - minX) * 2;
            float dy = (maxY - minY) * 2;

            Vector2 p1 = new Vector2(minX - dx, minY - dy);
            Vector2 p2 = new Vector2(maxX + dx, minY - dy);
            Vector2 p3 = new Vector2((minX + maxX) / 2, maxY + dy);

            superTriangle = new Triangle(p1, p2, p3);
            return superTriangle;
        }

        private void AddPoint(Vector2 point)
        {
            edges.Clear();
            List<Triangle> trianglesToRemove = new List<Triangle>();

            foreach (Triangle triangle in triangles)
            {
                if (triangle.IsPointInCircumcircle(point))
                {
                    trianglesToRemove.Add(triangle);
                    edges.AddRange(triangle.GetEdges());
                }
            }

            foreach (Triangle triangle in trianglesToRemove)
            {
                triangles.Remove(triangle);
            }

            edges = UniqueEdges(edges);

            foreach (Edge edge in edges)
            {
                triangles.Add(new Triangle(edge.Start, edge.End, point));
            }
        }

        private List<Edge> UniqueEdges(List<Edge> edges)
        {
            List<Edge> uniqueEdges = new List<Edge>();

            for (int i = 0; i < edges.Count; i++)
            {
                bool isUnique = true;
                for (int j = 0; j < edges.Count; j++)
                {
                    if (i != j && edges[i].Equals(edges[j]))
                    {
                        isUnique = false;
                        break;
                    }
                }
                if (isUnique)
                {
                    uniqueEdges.Add(edges[i]);
                }
            }

            return uniqueEdges;
        }

        public List<Triangle> GetTriangles()
        {
            return triangles.Where(t =>
                !t.SharesVertex(superTriangle.P1) &&
                !t.SharesVertex(superTriangle.P2) &&
                !t.SharesVertex(superTriangle.P3)).ToList();
        }
    }
}