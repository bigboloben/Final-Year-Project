using System.Collections.Generic;
using UnityEngine;

namespace Assets.TrackGeneration
{
    public class WallGenerator
    {
        private readonly TrackParameters parameters;

        public WallGenerator(TrackParameters parameters)
        {
            this.parameters = parameters;
        }

        public GameObject GenerateWall(Vector3[] edgePoints, Material material, PhysicsMaterial physics)
        {
            GameObject wallObject = new GameObject("Wall3D");
            MeshFilter meshFilter = wallObject.AddComponent<MeshFilter>();
            MeshRenderer meshRenderer = wallObject.AddComponent<MeshRenderer>();
            meshRenderer.material = TextureGenerator.CreateWallTexture(material);
            Mesh mesh = new Mesh();

            int numPoints = edgePoints.Length;
            List<Vector3> vertices = new List<Vector3>();
            List<Vector2> uvs = new List<Vector2>();
            List<Vector3> normals = new List<Vector3>();

            if (numPoints >= 16383)
            {
                Debug.LogError($"Number of points too high {numPoints}");
            }

            float totalLength = 0;
            for (int i = 1; i < numPoints; i++)
            {
                totalLength += Vector3.Distance(edgePoints[i], edgePoints[i - 1]);
            }

            float currentDistance = 0f;
            List<int> triangles = new List<int>();

            for (int i = 0; i < numPoints - 1; i++)
            {
                Vector3 right = GetRightDirection(edgePoints, i);
                Vector3 nextRight = GetRightDirection(edgePoints, i + 1);

                // Current segment vertices
                Vector3 bottomRightCurrent = edgePoints[i] + right * parameters.WallDepth - Vector3.up * parameters.WallHeight;
                Vector3 topRightCurrent = edgePoints[i] + right * parameters.WallDepth + Vector3.up * parameters.WallHeight;
                Vector3 topLeftCurrent = edgePoints[i] + right * -parameters.WallDepth + Vector3.up * parameters.WallHeight;
                Vector3 bottomLeftCurrent = edgePoints[i] + right * -parameters.WallDepth - Vector3.up * parameters.WallHeight;

                // Next segment vertices
                Vector3 bottomRightNext = edgePoints[i + 1] + nextRight * parameters.WallDepth - Vector3.up * parameters.WallHeight;
                Vector3 topRightNext = edgePoints[i + 1] + nextRight * parameters.WallDepth + Vector3.up * parameters.WallHeight;
                Vector3 topLeftNext = edgePoints[i + 1] + nextRight * -parameters.WallDepth + Vector3.up * parameters.WallHeight;
                Vector3 bottomLeftNext = edgePoints[i + 1] + nextRight * -parameters.WallDepth - Vector3.up * parameters.WallHeight;

                currentDistance += Vector3.Distance(edgePoints[i], edgePoints[i + 1]);
                float uCurrent = currentDistance / totalLength;
                float uPrev = (i > 0) ? (currentDistance - Vector3.Distance(edgePoints[i], edgePoints[i + 1])) / totalLength : 0f;

                int baseIndex = vertices.Count;

                // Right face
                AddQuad(vertices, uvs, normals, triangles, baseIndex,
                    bottomRightCurrent, topRightCurrent, bottomRightNext, topRightNext,
                    uPrev, uCurrent, right);

                // Top face
                AddQuad(vertices, uvs, normals, triangles, baseIndex + 4,
                    topRightCurrent, topLeftCurrent, topRightNext, topLeftNext,
                    uPrev, uCurrent, Vector3.up);

                // Left face
                AddQuad(vertices, uvs, normals, triangles, baseIndex + 8,
                    bottomLeftNext, topLeftNext, bottomLeftCurrent, topLeftCurrent,
                    uCurrent, uPrev, -right);

                // Bottom face
                AddQuad(vertices, uvs, normals, triangles, baseIndex + 12,
                    bottomLeftCurrent, bottomRightCurrent, bottomLeftNext, bottomRightNext,
                    uPrev, uCurrent, -Vector3.up);
            }

            mesh.SetVertices(vertices);
            mesh.SetUVs(0, uvs);
            mesh.SetNormals(normals);
            mesh.SetTriangles(triangles.ToArray(), 0);
            mesh.RecalculateBounds();

            meshFilter.mesh = mesh;

            MeshCollider collider = wallObject.AddComponent<MeshCollider>();
            collider.sharedMesh = mesh;
            collider.material = physics;

            wallObject.layer = LayerMask.NameToLayer("Outlines");

            return wallObject;
        }

        private Vector3 GetRightDirection(Vector3[] points, int currentPointIndex)
        {
            int prevIndex = (currentPointIndex - 1 + points.Length) % points.Length;
            int nextIndex = (currentPointIndex + 1) % points.Length;

            Vector3 tangent = (points[nextIndex] - points[prevIndex]).normalized;
            return Vector3.Cross(Vector3.up, tangent);
        }

        private void AddQuad(
            List<Vector3> vertices,
            List<Vector2> uvs,
            List<Vector3> normals,
            List<int> triangles,
            int baseIndex,
            Vector3 bottomLeft,
            Vector3 topLeft,
            Vector3 bottomRight,
            Vector3 topRight,
            float uMin,
            float uMax,
            Vector3 normal)
        {
            // Add vertices
            vertices.Add(bottomLeft);
            vertices.Add(topLeft);
            vertices.Add(bottomRight);
            vertices.Add(topRight);

            // Add UVs
            uvs.Add(new Vector2(uMin, 0));
            uvs.Add(new Vector2(uMin, 1));
            uvs.Add(new Vector2(uMax, 0));
            uvs.Add(new Vector2(uMax, 1));

            // Add normals
            for (int i = 0; i < 4; i++)
            {
                normals.Add(normal);
            }

            // Add triangles
            triangles.Add(baseIndex);
            triangles.Add(baseIndex + 1);
            triangles.Add(baseIndex + 2);
            triangles.Add(baseIndex + 1);
            triangles.Add(baseIndex + 3);
            triangles.Add(baseIndex + 2);
        }
    }
}