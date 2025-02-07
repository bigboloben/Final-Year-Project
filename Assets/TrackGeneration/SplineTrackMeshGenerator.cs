using UnityEngine;
using UnityEngine.Splines;
using System.Collections.Generic;

namespace Assets.TrackGeneration
{
    public class SplineTrackMeshGenerator
    {
        private float trackWidth;
        private float wallHeight;
        private float wallDepth;
        private int segmentsPerUnit;
        private float bankingAngle;
        private const float TRACK_HEIGHT_OFFSET = 0.1f;  // Consistent height offset for the entire track

        public class StartLineInfo
        {
            public GameObject startLine;
            public Vector3 position1; // First car position
            public Vector3 position2; // Second car position
            public Vector3 startDirection; // Direction cars should face
            public Quaternion startRotation; // Rotation for the cars
        }
        public SplineTrackMeshGenerator(float width, float height, float depth, int segments, float banking)
        {
            trackWidth = width;
            wallHeight = height;
            wallDepth = depth;
            segmentsPerUnit = segments;
            bankingAngle = banking;
        }

        public GameObject GenerateTrackMesh(SplineContainer spline, Material trackMaterial, Material wallMaterial, PhysicsMaterial wallPhysicsMaterial, Material startLineMaterial, Material gridMarkerMaterial, out Vector3 startPos1, out Vector3 startPos2, out Quaternion startRot)
        {
            

            GameObject trackObject = new GameObject("Track");
            List<Vector3[]> splinePoints = GenerateOffsetSplines(spline);
            GameObject trackSurface = GenerateTrackSurface(splinePoints, trackMaterial);
            StartLineInfo startLineInfo = CreateStartFinishLine(splinePoints[1], startLineMaterial);
            GameObject leftWall = GenerateWall(splinePoints[0], wallMaterial, wallPhysicsMaterial);
            GameObject rightWall = GenerateWall(splinePoints[2], wallMaterial, wallPhysicsMaterial);
            GameObject gridMarkers = CreateStartingGridMarkers(startLineInfo.position1, startLineInfo.position2, startLineInfo.startRotation, gridMarkerMaterial);

            trackSurface.transform.SetParent(trackObject.transform);
            leftWall.transform.SetParent(trackObject.transform);
            rightWall.transform.SetParent(trackObject.transform);
            startLineInfo.startLine.transform.SetParent(trackObject.transform);
            gridMarkers.transform.SetParent(trackObject.transform);

            startPos1 = startLineInfo.position1;
            startPos2 = startLineInfo.position2;
            startRot = startLineInfo.startRotation;

            return trackObject;
        }

        public GameObject CreateStartingGridMarkers(Vector3 pos1, Vector3 pos2, Quaternion rotation, Material material)
        {
            // Create containers for the grid markers
            GameObject gridMarkersContainer = new GameObject("GridMarkers");
            

            CreateGridMarker(pos1, rotation, gridMarkersContainer, "GridMarker1", material);
            CreateGridMarker(pos2, rotation, gridMarkersContainer, "GridMarker2", material);

            return gridMarkersContainer;
        }

        private void CreateGridMarker(Vector3 position, Quaternion rotation, GameObject parent, string name, Material material)
        {
            GameObject markerObject = new GameObject(name);
            markerObject.transform.SetParent(parent.transform);

            // Create the mesh for the grid marker
            Mesh mesh = new Mesh();

            // Parameters for marker size
            float width = 1f;      // Width of the marker
            float length = 2f;     // Length of the marker
            float thickness = width * 0.25f;              // Thickness of the lines
            float heightOffset = 0f;          // Slight offset to prevent z-fighting

            // Get directional vectors
            Vector3 forward = rotation * Vector3.forward;
            Vector3 right = rotation * Vector3.right;
            Vector3 up = Vector3.up;

            // Create vertices for the L-shaped marker
            Vector3[] vertices = new Vector3[]
            {
                // Forward line
                position + heightOffset * up + (width + thickness) * -right + length * forward,                    // 0 bottom left   2             3
                position + heightOffset * up + (width + thickness) * right + length * forward,                    // 1 bottom right     -----------
                position + heightOffset * up + (width + thickness) * -right + (length + thickness) * forward,    // 2 top left          |         |
                position + heightOffset * up + (width + thickness) * right + (length + thickness) * forward,    // 3 top right        0 ___________ 1

                // Left Side line
                position + heightOffset * up + (width + thickness) * -right + length/2 * -forward,                 // 4 bottom left    6 ---- 7
                position + heightOffset * up + width * -right + length/2 * -forward,                              // 5 bottom right      |  |
                position + heightOffset * up + (width + thickness) * -right + (length + thickness) * forward,    // 6 top left           |  |
                position + heightOffset * up + width * -right + (length + thickness) * forward,                 // 7 top right         4 ---- 5
                
                // Right Side Line
                position + heightOffset * up + width * right + length/2 * -forward,                                // 8 bottom left   10 ---- 11
                position + heightOffset * up + (width + thickness) * right + length/2 * -forward,                 // 9 bottom right      |  |
                position + heightOffset * up + width * right + (length + thickness) * forward,                   // 10 top left          |  |
                position + heightOffset * up + (width + thickness) * right + (length + thickness) * forward     // 11 top right        8 ---- 9


            };

            // Define triangles
            int[] triangles = new int[]
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

            // Create UVs for texture mapping
            Vector2[] uvs = new Vector2[]
            {
                new Vector2(0, 0), new Vector2(1, 0), new Vector2(0, 1), new Vector2(1, 1),
                new Vector2(0, 0), new Vector2(1, 0), new Vector2(0, 1), new Vector2(1, 1),
                new Vector2(0, 0), new Vector2(1, 0), new Vector2(0, 1), new Vector2(1, 1)
            };

            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.uv = uvs;
            mesh.RecalculateNormals();

            // Add mesh components
            MeshFilter meshFilter = markerObject.AddComponent<MeshFilter>();
            meshFilter.mesh = mesh;

            MeshRenderer meshRenderer = markerObject.AddComponent<MeshRenderer>();
            //Material markerMaterial = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            material.SetTexture("_BaseMap" ,CreateGridMarkerTexture());
            meshRenderer.material = material;
        }

        private Texture2D CreateGridMarkerTexture()
        {
            int textureSize = 64;
            Texture2D texture = new Texture2D(textureSize, textureSize);
            Color gridColor = new Color(0.9f, 0.9f, 0.9f); // Slightly off-white for better visibility

            for (int y = 0; y < textureSize; y++)
            {
                for (int x = 0; x < textureSize; x++)
                {
                    texture.SetPixel(x, y, gridColor);
                }
            }

            texture.Apply();
            texture.filterMode = FilterMode.Bilinear;
            return texture;
        }


        private List<Vector3[]> GenerateOffsetSplines(SplineContainer centerSpline)
        {
            List<Vector3[]> splinePoints = new List<Vector3[]>();
            int pointCount = Mathf.Max(32, (int)(centerSpline.Spline.GetLength() * segmentsPerUnit));

            Vector3[] leftPoints = new Vector3[pointCount];
            Vector3[] centerPoints = new Vector3[pointCount];
            Vector3[] rightPoints = new Vector3[pointCount];

            // Get initial spline information
            float baseHeight = centerSpline.EvaluatePosition(0).y;

            for (int i = 0; i < pointCount; i++)
            {
                float t = i / (float)(pointCount - 1);
                Vector3 position = centerSpline.EvaluatePosition(t);
                Vector3 tangent = ((Vector3)centerSpline.EvaluateTangent(t)).normalized;
                Vector3 up = Vector3.up;
                Vector3 right = Vector3.Cross(tangent, up).normalized;

                // Calculate curvature (you'll need to implement this)
                float curvature = CalculateCurvature(centerSpline, t);

                // Apply banking rotation based on curvature
                float bankAngle = curvature * bankingAngle;
                Quaternion bankRotation = Quaternion.AngleAxis(bankAngle, tangent);

                // Rotate the right and up vectors
                right = bankRotation * right;
                up = bankRotation * up;

                // Apply the consistent height offset and banking
                Vector3 heightOffset = up * TRACK_HEIGHT_OFFSET;

                centerPoints[i] = position + heightOffset;
                leftPoints[i] = position - right * (trackWidth / 2) + heightOffset;
                rightPoints[i] = position + right * (trackWidth / 2) + heightOffset;
            }

            splinePoints.Add(leftPoints);
            splinePoints.Add(centerPoints);
            splinePoints.Add(rightPoints);

            return splinePoints;
        }

        private float CalculateCurvature(SplineContainer spline, float t)
        {
            // Get position and derivatives
            Vector3 p1 = spline.EvaluatePosition(t);
            Vector3 v1 = spline.EvaluateTangent(t);

            // Use a small delta to approximate second derivative
            float delta = 0.001f;
            Vector3 v2 = spline.EvaluateTangent(t + delta);

            // Calculate curvature 
            Vector3 acceleration = (v2 - v1) / delta;
            float curvature = Vector3.Cross(v1, acceleration).magnitude / Mathf.Pow(v1.magnitude, 3);

            return curvature;
        }


        private StartLineInfo CreateStartFinishLine(Vector3[] centerPoints, Material material)
        {
            StartLineInfo startInfo = new StartLineInfo();

            // Create the start line mesh as before
            GameObject startLine = new GameObject("StartFinishLine");
            Mesh mesh = new Mesh();

            // Calculate directions and positions
            Vector3 position = centerPoints[0];
            Vector3 direction = (centerPoints[1] - centerPoints[0]).normalized;
            Vector3 right = Vector3.Cross(Vector3.up, direction).normalized;
            Vector3 up = Vector3.Cross(direction, right).normalized;

            float lineWidth = trackWidth;
            float lineLength = 2f;
            float heightOffset = 0.001f;

            // Create start line vertices
            Vector3[] vertices = new Vector3[4];
            vertices[0] = position + (heightOffset * up) - (right * lineWidth / 2);
            vertices[1] = position + (heightOffset * up) + (right * lineWidth / 2);
            vertices[2] = position + (heightOffset * up) + (direction * lineLength) - (right * lineWidth / 2);
            vertices[3] = position + (heightOffset * up) + (direction * lineLength) + (right * lineWidth / 2);

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

            MeshFilter meshFilter = startLine.AddComponent<MeshFilter>();
            meshFilter.mesh = mesh;

            MeshRenderer meshRenderer = startLine.AddComponent<MeshRenderer>();
            //Material checkerMaterial = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            material.SetTexture("_BaseMap", CreateStartLineTexture());
            meshRenderer.material = material;

            // Calculate starting positions
            float carSpacing = trackWidth * 0.25f; // Space between cars
            
            // Calculate positions behind the start line
            Vector3 gridCenterPosition = centerPoints[centerPoints.Length - 20]; // Move back from start line
            Vector3 gridCenterDirection = (centerPoints[centerPoints.Length - 19] - centerPoints[centerPoints.Length - 21]).normalized;
            Vector3 gridCenterRight = Vector3.Cross(Vector3.up, direction).normalized;
            Vector3 gridCenterUp = Vector3.Cross(direction, right).normalized;
            startInfo.position1 = gridCenterPosition - gridCenterRight * carSpacing; // Left position
            startInfo.position2 = gridCenterPosition + gridCenterRight * carSpacing; // Right position

            // Add slight height offset to prevent cars from clipping
            startInfo.position1 += Vector3.up * 0.001f;
            startInfo.position2 += Vector3.up * 0.001f;

            // Store direction and rotation
            startInfo.startDirection = gridCenterDirection;
            startInfo.startRotation = Quaternion.LookRotation(gridCenterDirection, Vector3.up);
            startInfo.startLine = startLine;

            return startInfo;
        }

        private Texture2D CreateStartLineTexture()
        {
            int textureSize = 256;
            int squareSize = 128; // Size of each checker square
            Texture2D texture = new Texture2D(textureSize, textureSize);

            for (int y = 0; y < textureSize; y++)
            {
                for (int x = 0; x < textureSize; x++)
                {
                    bool isAlternate = ((x / squareSize) + (y / squareSize)) % 2 == 0;
                    texture.SetPixel(x, y, isAlternate ? Color.white : Color.black);
                }
            }

            texture.Apply();
            texture.filterMode = FilterMode.Point;
            texture.wrapMode = TextureWrapMode.Repeat;
            return texture;
        }

        private GameObject GenerateTrackSurface(List<Vector3[]> splinePoints, Material material)
        {

            GameObject surfaceObject = new GameObject("TrackSurface");
            surfaceObject.tag = "Floor";
            MeshFilter meshFilter = surfaceObject.AddComponent<MeshFilter>();
            MeshRenderer meshRenderer = surfaceObject.AddComponent<MeshRenderer>();
            meshRenderer.material = material;

            Mesh mesh = new Mesh();
            Vector3[] vertices = new Vector3[splinePoints[0].Length * 2];
            int[] triangles = new int[(splinePoints[0].Length - 1) * 6];
            Vector2[] uvs = new Vector2[vertices.Length];

            // Generate vertices
            for (int i = 0; i < splinePoints[0].Length; i++)
            {
                vertices[i * 2] = splinePoints[0][i];     // Left edge
                vertices[i * 2 + 1] = splinePoints[2][i]; // Right edge

                float u = i / (float)(splinePoints[0].Length - 1);
                uvs[i * 2] = new Vector2(0, u);
                uvs[i * 2 + 1] = new Vector2(1, u);
            }

            // Generate triangles with correct winding order
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

            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.uv = uvs;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            meshFilter.mesh = mesh;

            // Add MeshCollider with smooth physics settings
            MeshCollider collider = surfaceObject.AddComponent<MeshCollider>();
            collider.sharedMesh = mesh;
            //collider.convex = true;

            return surfaceObject;
        }
        private Vector3 GetRightDirection(Vector3[] points, int currentPointIndex)
        {
            // Get previous and next points (wrap around for closed loop)
            int prevIndex = (currentPointIndex - 1 + points.Length) % points.Length;
            int nextIndex = (currentPointIndex + 1) % points.Length;

            // Calculate tangent using both previous and next points
            Vector3 tangent = (points[nextIndex] - points[prevIndex]).normalized;

            // Cross product with up vector to get right direction
            Vector3 right = Vector3.Cross(Vector3.up, tangent);

            return right;
        }

        private Material CreateWallTexture(Material wallMaterial)
        {
            Color color1 = Color.red;
            Color color2 = Color.white;

            Texture2D texture = new Texture2D(512, 512);
            for (int y = 0; y < texture.height; y++)
            {
                for (int x = 0; x < texture.width; x++)
                {
                    // Changed from x to y to make stripes vertical
                    bool isFirstColor = (x/256) % 2 == 0;
                    Color color = isFirstColor ? color1 : color2;
                    texture.SetPixel(x, y, color);
                }
            }
            texture.Apply();
            texture.wrapMode = TextureWrapMode.Repeat;

            //Material wallMaterial = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            wallMaterial.SetTexture("_BaseMap", texture);
            return wallMaterial;
        }
        private GameObject GenerateWall(Vector3[] edgePoints, Material material, PhysicsMaterial physics)
        {
            GameObject wallObject = new GameObject("Wall3D");
            MeshFilter meshFilter = wallObject.AddComponent<MeshFilter>();
            MeshRenderer meshRenderer = wallObject.AddComponent<MeshRenderer>();
            meshRenderer.material = CreateWallTexture(material);
            Mesh mesh = new Mesh();

            int numPoints = edgePoints.Length;
            Vector3[] vertices = new Vector3[numPoints * 4];  // 4 vertices per point (front bottom, front top, back bottom, back top)
            Vector2[] uvs = new Vector2[vertices.Length];

            if (numPoints >= 16383)
            {
                Debug.LogError($"Number of points too high {numPoints}");
            }

            // Calculate total wall length for UV scaling
            float totalLength = 0;
            for (int i = 1; i < numPoints; i++)
            {
                totalLength += Vector3.Distance(edgePoints[i], edgePoints[i - 1]);
            }

            float currentDistance = 0f;

            for (int i = 0; i < numPoints; i++)
            {
                Vector3 right = GetRightDirection(edgePoints, i);

                // Vertex positions
                vertices[i * 4] = edgePoints[i] + right * wallDepth;   // right bottom
                vertices[i * 4 + 1] = edgePoints[i] + right * wallDepth + Vector3.up * wallHeight;  // right top
                vertices[i * 4 + 2] = edgePoints[i] + right * -wallDepth + Vector3.up * wallHeight;  // left top
                vertices[i * 4 + 3] = edgePoints[i] + right * -wallDepth; // left bottom


                if (i > 0)
                {
                    currentDistance += Vector3.Distance(edgePoints[i], edgePoints[i - 1]);
                }

                float u = currentDistance / totalLength; // Added tiling scale
                float v = 0f;  // Bottom of wall
                float v2 = wallHeight;  // Top of wall

                uvs[i * 4] = new Vector2(u, v);      // Right bottom
                uvs[i * 4 + 1] = new Vector2(u, v2); // Right top
                uvs[i * 4 + 2] = new Vector2(u, v2); // Left top
                uvs[i * 4 + 3] = new Vector2(u, v);  // Left bottom
            }


            List<int> triangles = new List<int>();

            // Generate triangles for the front and back faces based on the connection pattern
            for (int i = 0; i < numPoints - 1; i++)
            {
                int thisIndex = (i * 4);
                int nextIndex = (i + 1) * 4;
                // clockwise ordering right face
                triangles.Add(thisIndex);       // this bottom right
                triangles.Add(thisIndex + 1);   // this top right
                triangles.Add(nextIndex);       // next bottom right

                triangles.Add(thisIndex + 1);   // this top right
                triangles.Add(nextIndex + 1);   // next top right
                triangles.Add(nextIndex);       // next bottom right

                // clockwise ordering top face
                triangles.Add(thisIndex + 1);   // this top right
                triangles.Add(thisIndex + 2);   // this top left
                triangles.Add(nextIndex + 1);   // next top right

                triangles.Add(thisIndex + 2);   // this top left
                triangles.Add(nextIndex + 2);   // next top left
                triangles.Add(nextIndex + 1);   // next top right

                // clockwise ordering left face
                triangles.Add(thisIndex + 3);   // this bottom left
                triangles.Add(nextIndex + 3);   // next bottom left
                triangles.Add(thisIndex + 2);   // this top left

                triangles.Add(thisIndex + 2);   // this top left
                triangles.Add(nextIndex + 3);   // next bottom left
                triangles.Add(nextIndex + 2);   // next top left

                // clockwise ordering bottom face
                triangles.Add(thisIndex);       // this bottom right
                triangles.Add(nextIndex);       // next bottom right
                triangles.Add(thisIndex + 3);   // this bottom left

                triangles.Add(thisIndex + 3);   // this bottom left
                triangles.Add(nextIndex);       // next bottom right
                triangles.Add(nextIndex + 3);   // next bottom left
            }

            mesh.vertices = vertices;
            mesh.triangles = triangles.ToArray();
            mesh.uv = uvs;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            meshFilter.mesh = mesh;

            MeshCollider collider = wallObject.AddComponent<MeshCollider>();
            collider.sharedMesh = mesh;
            collider.material = physics;

            return wallObject;
        }
    }
}