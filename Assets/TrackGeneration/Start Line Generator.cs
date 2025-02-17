using UnityEngine;

namespace Assets.TrackGeneration {
    public class StartLineGenerator
    {
        private readonly TrackParameters parameters;

        public StartLineGenerator(TrackParameters parameters)
        {
            this.parameters = parameters;
        }

        public StartLineInfo CreateStartFinishLine(Vector3[] centerPoints, Material material)
        {
            GameObject startLine = new GameObject("StartFinishLine");
            Mesh mesh = CreateStartLineMesh(centerPoints);

            ApplyMeshComponents(startLine, mesh, material);

            // Calculate starting positions and direction
            Vector3 gridCenterPosition = centerPoints[centerPoints.Length - 20];
            Vector3 gridCenterDirection = (centerPoints[centerPoints.Length - 19] - centerPoints[centerPoints.Length - 21]).normalized;
            Vector3 gridCenterRight = Vector3.Cross(Vector3.up, gridCenterDirection).normalized;

            float carSpacing = parameters.TrackWidth * 0.25f;

            Vector3 position1 = gridCenterPosition - gridCenterRight * carSpacing + Vector3.up * 0.001f;
            Vector3 position2 = gridCenterPosition + gridCenterRight * carSpacing + Vector3.up * 0.001f;

            Quaternion startRotation = Quaternion.LookRotation(gridCenterDirection, Vector3.up);

            return new StartLineInfo(startLine, position1, position2, gridCenterDirection, startRotation);
        }

        private Mesh CreateStartLineMesh(Vector3[] centerPoints)
        {
            Mesh mesh = new Mesh();

            Vector3 position = centerPoints[0];
            Vector3 direction = (centerPoints[1] - centerPoints[0]).normalized;
            Vector3 right = Vector3.Cross(Vector3.up, direction).normalized;
            Vector3 up = Vector3.Cross(direction, right).normalized;

            float lineWidth = parameters.TrackWidth;
            float lineLength = 2f;
            float heightOffset = 0.001f;

            Vector3[] vertices = new Vector3[]
            {
                position + (heightOffset * up) - (right * lineWidth / 2),
                position + (heightOffset * up) + (right * lineWidth / 2),
                position + (heightOffset * up) + (direction * lineLength) - (right * lineWidth / 2),
                position + (heightOffset * up) + (direction * lineLength) + (right * lineWidth / 2)
            };

            int[] triangles = new int[]
            {
                0, 2, 1,
                1, 2, 3
            };

            Vector2[] uvs = new Vector2[]
            {
                new Vector2(0, 0),
                new Vector2(8, 0),
                new Vector2(0, 1),
                new Vector2(8, 1)
            };

            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.uv = uvs;
            mesh.RecalculateNormals();

            return mesh;
        }

        private void ApplyMeshComponents(GameObject startLine, Mesh mesh, Material material)
        {
            MeshFilter meshFilter = startLine.AddComponent<MeshFilter>();
            meshFilter.mesh = mesh;

            MeshRenderer meshRenderer = startLine.AddComponent<MeshRenderer>();
            material.SetTexture("_BaseMap", TextureGenerator.CreateStartLineTexture());
            meshRenderer.material = material;
        }

        public GameObject CreateStartingGridMarkers(Vector3 pos1, Vector3 pos2, Quaternion rotation, Material material)
        {
            GameObject gridMarkersContainer = new GameObject("GridMarkers");

            CreateGridMarker(pos1, rotation, gridMarkersContainer, "GridMarker1", material);
            CreateGridMarker(pos2, rotation, gridMarkersContainer, "GridMarker2", material);

            return gridMarkersContainer;
        }

        private void CreateGridMarker(Vector3 position, Quaternion rotation, GameObject parent, string name, Material material)
        {
            GameObject markerObject = new GameObject(name);
            markerObject.transform.SetParent(parent.transform);

            Mesh mesh = CreateGridMarkerMesh(position, rotation);

            MeshFilter meshFilter = markerObject.AddComponent<MeshFilter>();
            meshFilter.mesh = mesh;

            MeshRenderer meshRenderer = markerObject.AddComponent<MeshRenderer>();
            material.SetTexture("_BaseMap", TextureGenerator.CreateGridMarkerTexture());
            meshRenderer.material = material;
        }

        private Mesh CreateGridMarkerMesh(Vector3 position, Quaternion rotation)
        {
            Mesh mesh = new Mesh();

            float width = 1f;
            float length = 2f;
            float thickness = width * 0.25f;
            float heightOffset = 0f;

            Vector3 forward = rotation * Vector3.forward;
            Vector3 right = rotation * Vector3.right;
            Vector3 up = Vector3.up;

            Vector3[] vertices = CreateGridMarkerVertices(position, heightOffset, width, length, thickness, forward, right, up);
            int[] triangles = CreateGridMarkerTriangles();
            Vector2[] uvs = CreateGridMarkerUVs();

            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.uv = uvs;
            mesh.RecalculateNormals();

            return mesh;
        }

        private Vector3[] CreateGridMarkerVertices(Vector3 position, float heightOffset, float width, float length,
            float thickness, Vector3 forward, Vector3 right, Vector3 up)
        {
            return new Vector3[]
            {
                // Forward line
                position + heightOffset * up + (width + thickness) * -right + length * forward,
                position + heightOffset * up + (width + thickness) * right + length * forward,
                position + heightOffset * up + (width + thickness) * -right + (length + thickness) * forward,
                position + heightOffset * up + (width + thickness) * right + (length + thickness) * forward,

                // Left Side line
                position + heightOffset * up + (width + thickness) * -right + length/2 * -forward,
                position + heightOffset * up + width * -right + length/2 * -forward,
                position + heightOffset * up + (width + thickness) * -right + (length + thickness) * forward,
                position + heightOffset * up + width * -right + (length + thickness) * forward,
                
                // Right Side Line
                position + heightOffset * up + width * right + length/2 * -forward,
                position + heightOffset * up + (width + thickness) * right + length/2 * -forward,
                position + heightOffset * up + width * right + (length + thickness) * forward,
                position + heightOffset * up + (width + thickness) * right + (length + thickness) * forward
            };
        }

        private int[] CreateGridMarkerTriangles()
        {
            return new int[]
            {
                // Forward line
                0, 2, 1,
                1, 2, 3,
        
                // Left Side line
                4, 6, 5,
                5, 6, 7,

                // Right Side Line
                8, 10, 9,
                9, 10, 11
            };
        }

        private Vector2[] CreateGridMarkerUVs()
        {
            return new Vector2[]
            {
                new Vector2(0, 0), new Vector2(1, 0), new Vector2(0, 1), new Vector2(1, 1),
                new Vector2(0, 0), new Vector2(1, 0), new Vector2(0, 1), new Vector2(1, 1),
                new Vector2(0, 0), new Vector2(1, 0), new Vector2(0, 1), new Vector2(1, 1)
            };
        }
    }
}