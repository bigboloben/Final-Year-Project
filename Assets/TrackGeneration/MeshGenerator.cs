using UnityEngine;
using System.Collections.Generic;

namespace Assets.TrackGeneration
{
    public class MeshGenerator : MonoBehaviour
    {
        private MeshFilter meshFilter;
        private MeshRenderer meshRenderer;
        private MeshCollider meshCollider;

        void Awake()
        {
            meshFilter = gameObject.AddComponent<MeshFilter>();
            meshRenderer = gameObject.AddComponent<MeshRenderer>();
            meshCollider = gameObject.AddComponent<MeshCollider>();
            meshRenderer.material = new Material(Shader.Find("Standard"));
        }

        public void GenerateTrackMesh(List<Vector2> centerPoints, float width = 10f, float height = 1f)
        {
            if (centerPoints.Count < 2) return;

            Mesh mesh = new Mesh();
            List<Vector3> vertices = new List<Vector3>();
            List<int> triangles = new List<int>();

            // Generate inner and outer track points
            for (int i = 0; i < centerPoints.Count; i++)
            {
                Vector2 current = centerPoints[i];
                Vector2 next = centerPoints[(i + 1) % centerPoints.Count];
                Vector2 direction = (next - current).normalized;
                Vector2 normal = new Vector2(-direction.y, direction.x);

                // Inner track points
                Vector2 innerPoint = current - normal * width;
                vertices.Add(new Vector3(innerPoint.x, 0, innerPoint.y));        // Bottom
                vertices.Add(new Vector3(innerPoint.x, height, innerPoint.y));   // Top

                // Outer track points
                Vector2 outerPoint = current + normal * width;
                vertices.Add(new Vector3(outerPoint.x, 0, outerPoint.y));       // Bottom
                vertices.Add(new Vector3(outerPoint.x, height, outerPoint.y));   // Top
            }

            // Create triangles
            for (int i = 0; i < centerPoints.Count; i++)
            {
                int currentIndex = i * 4;
                int nextIndex = ((i + 1) % centerPoints.Count) * 4;

                // Inner wall
                triangles.AddRange(new int[] {
                currentIndex, currentIndex + 1, nextIndex + 1,
                currentIndex, nextIndex + 1, nextIndex
            });

                // Outer wall
                triangles.AddRange(new int[] {
                currentIndex + 2, nextIndex + 3, currentIndex + 3,
                currentIndex + 2, nextIndex + 2, nextIndex + 3
            });

                // Top surface
                triangles.AddRange(new int[] {
                currentIndex + 1, currentIndex + 3, nextIndex + 3,
                currentIndex + 1, nextIndex + 3, nextIndex + 1
            });

                // Bottom surface
                triangles.AddRange(new int[] {
                currentIndex, nextIndex + 2, currentIndex + 2,
                currentIndex, nextIndex, nextIndex + 2
            });
            }

            mesh.vertices = vertices.ToArray();
            mesh.triangles = triangles.ToArray();
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            meshFilter.mesh = mesh;
            meshCollider.sharedMesh = mesh;
        }
    }
}