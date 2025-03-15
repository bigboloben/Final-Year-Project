using Assets.TrackGeneration;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace Assets.TrackGeneration
{
    public class SurfaceMeshBuilder
    {
        private readonly TrackParameters parameters;

        public SurfaceMeshBuilder(TrackParameters parameters)
        {
            this.parameters = parameters;
        }

        public GameObject GenerateTrackSurface(List<Vector3[]> splinePoints, Material material)
        {
            GameObject surfaceObject = new GameObject("TrackSurface");
            surfaceObject.tag = "Floor";
            surfaceObject.layer = 2;

            MeshFilter meshFilter = surfaceObject.AddComponent<MeshFilter>();
            MeshRenderer meshRenderer = surfaceObject.AddComponent<MeshRenderer>();
            meshRenderer.material = material;
            meshRenderer.renderingLayerMask = (1 << 0) | (1 << 8);

            Mesh mesh = CreateTrackMesh(splinePoints);
            meshFilter.mesh = mesh;

            MeshCollider collider = surfaceObject.AddComponent<MeshCollider>();
            collider.sharedMesh = mesh;

            return surfaceObject;
        }

        private Mesh CreateTrackMesh(List<Vector3[]> splinePoints)
        {
            Mesh mesh = new Mesh();
            Vector3[] vertices = new Vector3[splinePoints[0].Length * 2];
            int[] triangles = new int[(splinePoints[0].Length - 1) * 6];
            Vector2[] uvs = new Vector2[vertices.Length];

            GenerateVertices(splinePoints, vertices, uvs);
            GenerateTriangles(splinePoints, triangles);

            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.uv = uvs;
            var normals = CalculateSmoothNormals(vertices, triangles);
            mesh.normals = normals;
            mesh.RecalculateBounds();

            return mesh;
        }

        private void GenerateVertices(List<Vector3[]> splinePoints, Vector3[] vertices, Vector2[] uvs)
        {
            for (int i = 0; i < splinePoints[0].Length; i++)
            {
                vertices[i * 2] = splinePoints[0][i];     // Left edge
                vertices[i * 2 + 1] = splinePoints[2][i]; // Right edge

                float u = i / (float)(splinePoints[0].Length - 1);
                uvs[i * 2] = new Vector2(0, u);
                uvs[i * 2 + 1] = new Vector2(1, u);
            }
        }

        private void GenerateTriangles(List<Vector3[]> splinePoints, int[] triangles)
        {
            int triIndex = 0;
            for (int i = 0; i < splinePoints[0].Length - 1; i++)
            {
                int baseIndex = i * 2;
                int nextBaseIndex = (i + 1) * 2;

                triangles[triIndex++] = baseIndex;
                triangles[triIndex++] = baseIndex + 1;
                triangles[triIndex++] = nextBaseIndex;

                triangles[triIndex++] = baseIndex + 1;
                triangles[triIndex++] = nextBaseIndex + 1;
                triangles[triIndex++] = nextBaseIndex;
            }
        }

        private Vector3[] CalculateSmoothNormals(Vector3[] vertices, int[] triangles)
        {
            Vector3[] normals = new Vector3[vertices.Length];
            Dictionary<Vector3, List<int>> vertexGroups = new Dictionary<Vector3, List<int>>();

            // Group vertices by position using Vector3's built-in comparison
            for (int i = 0; i < vertices.Length; i++)
            {
                Vector3 vertex = vertices[i];
                bool foundGroup = false;

                // Look for existing vertex position within small threshold
                foreach (var key in vertexGroups.Keys)
                {
                    if (Vector3.Distance(key, vertex) < 0.0001f)
                    {
                        vertexGroups[key].Add(i);
                        foundGroup = true;
                        break;
                    }
                }

                if (!foundGroup)
                {
                    vertexGroups[vertex] = new List<int> { i };
                }
            }

            // Calculate face normals and contribute to vertex normals
            for (int i = 0; i < triangles.Length; i += 3)
            {
                int a = triangles[i];
                int b = triangles[i + 1];
                int c = triangles[i + 2];

                Vector3 normal = Vector3.Cross(
                    vertices[b] - vertices[a],
                    vertices[c] - vertices[a]
                ).normalized;

                normals[a] += normal;
                normals[b] += normal;
                normals[c] += normal;
            }

            // Average and normalize normals for shared vertices
            foreach (var group in vertexGroups)
            {
                Vector3 averageNormal = Vector3.zero;
                foreach (int index in group.Value)
                {
                    averageNormal += normals[index];
                }
                averageNormal.Normalize();

                foreach (int index in group.Value)
                {
                    normals[index] = averageNormal;
                }
            }

            return normals;
        }
    }
}
