using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Splines;

namespace Assets.TrackGeneration
{
    public class TrackSupportGenerator
    {
        TrackParameters parameters;
        public TrackSupportGenerator(TrackParameters parameters)
        {
            this.parameters = parameters;
        }


        public GameObject GenerateTrackSupports(List<Vector3[]> splines, Material material)
        {
            float splineLength = splines[0].Length;
            int pointsPerIteration = Mathf.RoundToInt(splineLength / parameters.SupportCount);
            GameObject supports = new GameObject($"Supports");
            //List<GameObject> supports = new List<GameObject>();
            GameObject support;
            for (int i = 0; i < splineLength; i += pointsPerIteration)
            {
                support = GenerateSupport(i, splines[1][i], material);
                support.transform.SetParent(supports.transform);
                //supports.Add(support);
            }
            return supports;
        }

        public GameObject GenerateSupport(int i, Vector3 position, Material material)
        {
            GameObject support = new GameObject($"Support {i}");
            MeshFilter meshFilter = support.AddComponent<MeshFilter>();
            MeshRenderer meshRenderer = support.AddComponent<MeshRenderer>();
            meshRenderer.material = material;

            Mesh mesh = new Mesh();
            List<Vector3> vertices = new List<Vector3>();
            List<Vector2> uvs = new List<Vector2>();
            List<Vector3> normals = new List<Vector3>();
            List<int> triangles = new List<int>();

            // Top vertices - 1x1 size centered on the position
            Vector3 TTR = position + new Vector3(0.5f, -0.1f, 0.5f);
            Vector3 TTL = position + new Vector3(-0.5f, -0.1f, 0.5f);
            Vector3 TBR = position + new Vector3(0.5f, -0.1f, -0.5f);
            Vector3 TBL = position + new Vector3(-0.5f, -0.1f, -0.5f);

            // Bottom vertices - extend straight down to height 0
            Vector3 bottomPosition = new Vector3(position.x, 0, position.z);

            Vector3 BTR = bottomPosition + new Vector3(0.5f, 0, 0.5f);
            Vector3 BTL = bottomPosition + new Vector3(-0.5f, 0, 0.5f);
            Vector3 BBR = bottomPosition + new Vector3(0.5f, 0, -0.5f);
            Vector3 BBL = bottomPosition + new Vector3(-0.5f, 0, -0.5f);

            // Left face
            AddQuad(vertices, uvs, normals, triangles, BTL, TTL, BBL, TBL, 0, 1, -Vector3.right);

            //right face
            AddQuad(vertices, uvs, normals, triangles, BBR, TBR, BTR, TTR, 0, 1, Vector3.right);

            //front face
            AddQuad(vertices, uvs, normals, triangles, BBL, TBL, BBR, TBR, 0, 1, Vector3.forward);

            //back face
            AddQuad(vertices, uvs, normals, triangles, BTR, TTR, BTL, TTL, 0, 1, -Vector3.forward);

            mesh.SetVertices(vertices);
            mesh.SetUVs(0, uvs);
            mesh.SetNormals(normals);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateBounds();

            meshFilter.mesh = mesh;
            support.layer = LayerMask.NameToLayer("Outlines");

            return support;
        }

        private void AddQuad(
            List<Vector3> vertices,
            List<Vector2> uvs,
            List<Vector3> normals,
            List<int> triangles,
            Vector3 bottomLeft,
            Vector3 topLeft,
            Vector3 bottomRight,
            Vector3 topRight,
            float uMin,
            float uMax,
            Vector3 normal)
        {
            int vertCount = vertices.Count;

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

            // Add triangles - adjusting indices to account for vertex count
            triangles.Add(vertCount + 0);
            triangles.Add(vertCount + 1);
            triangles.Add(vertCount + 2);
            triangles.Add(vertCount + 1);
            triangles.Add(vertCount + 3);
            triangles.Add(vertCount + 2);
        }
    }
}