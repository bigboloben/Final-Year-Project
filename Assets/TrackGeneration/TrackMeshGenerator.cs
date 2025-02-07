using UnityEngine;
using System.Collections.Generic;
using UnityEngine.Splines;

namespace Assets.TrackGeneration
{
    public class TrackMeshGenerator
    {
        public float trackWidth;
        public float trackHeight;
        public int segmentsPerSplinePoint;
        public float banking;
        public bool generateCollider = true;

        private List<Vector3> vertices = new List<Vector3>();
        private List<int> triangles = new List<int>();
        private List<Vector2> uvs = new List<Vector2>();

        public TrackMeshGenerator(float trackWidth, float trackHeight, int segmentsPerSplinePoint, float banking)
        {
            this.trackWidth = trackWidth;
            this.trackHeight = trackHeight;
            this.segmentsPerSplinePoint = segmentsPerSplinePoint;
            this.banking = banking;
        }

        public GameObject GenerateTrackMesh(SplineContainer centerlineSpline, Material trackMaterial)
        {
            if (centerlineSpline == null || centerlineSpline.Spline == null || centerlineSpline.Spline.Count == 0)
            {
                Debug.LogError("TrackMeshGenerator: Invalid spline data!");
                return null;
            }

            try
            {
                // Create game object for the track
                GameObject trackObject = new GameObject("TrackMesh");
                MeshFilter meshFilter = trackObject.AddComponent<MeshFilter>();
                MeshRenderer meshRenderer = trackObject.AddComponent<MeshRenderer>();

                // Clear existing data
                vertices.Clear();
                triangles.Clear();
                uvs.Clear();

                // Get spline points
                var splinePoints = new List<Vector3>();
                float step = 1f / (segmentsPerSplinePoint * centerlineSpline.Spline.Count);

                for (float t = 0; t <= 1f; t += step)
                {
                    splinePoints.Add(centerlineSpline.EvaluatePosition(t));
                }

                if (splinePoints.Count < 2)
                {
                    Debug.LogError("TrackMeshGenerator: Not enough spline points generated!");
                    Object.Destroy(trackObject);
                    return null;
                }

                // Generate mesh vertices
                for (int i = 0; i < splinePoints.Count; i++)
                {
                    Vector3 forward;
                    if (i < splinePoints.Count - 1)
                        forward = (splinePoints[i + 1] - splinePoints[i]).normalized;
                    else
                        forward = (splinePoints[i] - splinePoints[i - 1]).normalized;

                    Vector3 right = Vector3.Cross(Vector3.up, forward).normalized;

                    // Apply banking
                    Quaternion bankRotation = Quaternion.AngleAxis(banking, forward);
                    right = bankRotation * right;

                    // Create vertices for track edges
                    Vector3 leftEdge = splinePoints[i] - right * (trackWidth * 0.5f);
                    Vector3 rightEdge = splinePoints[i] + right * (trackWidth * 0.5f);

                    // Add vertices for top surface
                    vertices.Add(leftEdge);
                    vertices.Add(rightEdge);

                    // Add vertices for bottom surface
                    vertices.Add(leftEdge - Vector3.up * trackHeight);
                    vertices.Add(rightEdge - Vector3.up * trackHeight);

                    // Generate UVs
                    float uvY = (float)i / (splinePoints.Count - 1);
                    uvs.Add(new Vector2(0, uvY));
                    uvs.Add(new Vector2(1, uvY));
                    uvs.Add(new Vector2(0, uvY));
                    uvs.Add(new Vector2(1, uvY));
                }

                // Generate triangles
                for (int i = 0; i < splinePoints.Count - 1; i++)
                {
                    int baseIndex = i * 4;

                    // Top face
                    triangles.Add(baseIndex);
                    triangles.Add(baseIndex + 4);
                    triangles.Add(baseIndex + 1);

                    triangles.Add(baseIndex + 4);
                    triangles.Add(baseIndex + 5);
                    triangles.Add(baseIndex + 1);

                    // Side faces
                    // Left side
                    triangles.Add(baseIndex);
                    triangles.Add(baseIndex + 2);
                    triangles.Add(baseIndex + 4);

                    triangles.Add(baseIndex + 2);
                    triangles.Add(baseIndex + 6);
                    triangles.Add(baseIndex + 4);

                    // Right side
                    triangles.Add(baseIndex + 1);
                    triangles.Add(baseIndex + 5);
                    triangles.Add(baseIndex + 3);

                    triangles.Add(baseIndex + 5);
                    triangles.Add(baseIndex + 7);
                    triangles.Add(baseIndex + 3);

                    // Bottom face
                    triangles.Add(baseIndex + 2);
                    triangles.Add(baseIndex + 3);
                    triangles.Add(baseIndex + 6);

                    triangles.Add(baseIndex + 3);
                    triangles.Add(baseIndex + 7);
                    triangles.Add(baseIndex + 6);
                }

                // Create and assign mesh
                Mesh trackMesh = new Mesh();
                trackMesh.vertices = vertices.ToArray();
                trackMesh.triangles = triangles.ToArray();
                trackMesh.uv = uvs.ToArray();
                trackMesh.RecalculateNormals();
                trackMesh.RecalculateBounds();

                // Assign mesh to components
                meshFilter.sharedMesh = trackMesh;
                meshRenderer.sharedMaterial = trackMaterial;

                if (generateCollider)
                {
                    var meshCollider = trackObject.AddComponent<MeshCollider>();
                    meshCollider.sharedMesh = trackMesh;
                }

                Debug.Log($"TrackMeshGenerator: Successfully generated mesh with {vertices.Count} vertices and {triangles.Count / 3} triangles");
                return trackObject;
            }
            catch (System.Exception e)
            {
                Debug.LogError($"TrackMeshGenerator: Error generating mesh: {e.Message}\n{e.StackTrace}");
                return null;
            }
        }
    }
}