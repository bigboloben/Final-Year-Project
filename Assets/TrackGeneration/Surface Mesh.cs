using Assets.TrackGeneration;
using System.Collections.Generic;
using UnityEngine;

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

            MeshFilter meshFilter = surfaceObject.AddComponent<MeshFilter>();
            MeshRenderer meshRenderer = surfaceObject.AddComponent<MeshRenderer>();
            meshRenderer.material = material;

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
            mesh.RecalculateNormals();
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
    }
}
