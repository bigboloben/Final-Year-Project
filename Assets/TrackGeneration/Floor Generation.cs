using UnityEngine;
using System.Collections.Generic;

namespace Assets.TrackGeneration
{
    public class FloorGenerator : MonoBehaviour
    {
#if UNITY_EDITOR
        // Store previous values for change detection
        private int prev_octaves;
        private float prev_lacunarity;
        private float prev_persistence;
        private float prev_heightScale;
        private float prev_heightOffset;
        private float prev_horizontalScale;
        private Vector2 prev_noiseOffset;
#endif

        [Header("Floor Settings")]
        public GameObject floorPrefab;
        public int gridSize = 4;
        public float tileSize = 200f;

        [Header("Height Map Settings")]
        public float heightScale = 1f;
        public float heightOffset = 0f;

        [Header("FBM Parameters")]
        [Range(1, 8)]
        public int octaves = 4;
        [Range(1f, 4f)]
        public float lacunarity = 2f;
        [Range(0f, 1f)]
        public float persistence = 0.5f;
        public float baseFrequency = 0.02f;
        public float horizontalScale = 1f;  // Controls the width of terrain features
        public Vector2 noiseOffset = Vector2.zero;

        private List<MeshFilter> floorTiles = new List<MeshFilter>();

        public void Initialize()
        {
            GenerateFloor();
            //ApplyHeightMap();

#if UNITY_EDITOR
            StoreCurrentValues();
#endif
        }

#if UNITY_EDITOR
        private void StoreCurrentValues()
        {
            prev_octaves = octaves;
            prev_lacunarity = lacunarity;
            prev_persistence = persistence;
            prev_heightScale = heightScale;
            prev_heightOffset = heightOffset;
            prev_horizontalScale = horizontalScale;
            prev_noiseOffset = noiseOffset;
        }

        private void OnValidate()
        {
            if (floorTiles.Count > 0)
            {
                if (prev_octaves != octaves ||
                    prev_lacunarity != lacunarity ||
                    prev_persistence != persistence ||
                    prev_heightScale != heightScale ||
                    prev_heightOffset != heightOffset ||
                    prev_horizontalScale != horizontalScale ||
                    prev_noiseOffset != noiseOffset)
                {
                    ApplyHeightMap();
                    StoreCurrentValues();
                }
            }
        }
#endif

        private void GenerateFloor()
        {
            if (floorPrefab == null)
            {
                Debug.LogError("Floor prefab is not assigned!");
                return;
            }

            floorTiles.Clear();

            GameObject floorParent = new GameObject("Floor Grid");
            floorParent.transform.SetParent(transform);

            float offset = (gridSize - 1) * tileSize * 0.5f;
            Vector3 trackCenter = Vector3.zero;

            for (int x = 0; x < gridSize; x++)
            {
                for (int z = 0; z < gridSize; z++)
                {
                    Vector3 position = new Vector3(
                        x * tileSize - offset + trackCenter.x,
                        0,
                        z * tileSize - offset + trackCenter.z
                    );

                    GameObject tile = Instantiate(floorPrefab, position, Quaternion.identity, floorParent.transform);
                    tile.name = $"Floor_Tile_{x}_{z}";

                    MeshFilter meshFilter = tile.GetComponent<MeshFilter>();
                    if (meshFilter != null)
                    {
                        meshFilter.mesh = Instantiate(meshFilter.sharedMesh);
                        floorTiles.Add(meshFilter);
                    }
                }
            }
        }

        private void ApplyHeightMap()
        {
            foreach (MeshFilter meshFilter in floorTiles)
            {
                Mesh mesh = meshFilter.mesh;
                Vector3[] vertices = mesh.vertices;

                for (int i = 0; i < vertices.Length; i++)
                {
                    Vector3 worldPos = meshFilter.transform.TransformPoint(vertices[i]);
                    float height = GetHeightAtPosition(worldPos);
                    vertices[i] = meshFilter.transform.InverseTransformPoint(
                        new Vector3(worldPos.x, height, worldPos.z)
                    );
                }

                mesh.vertices = vertices;
                mesh.RecalculateNormals();
                mesh.RecalculateBounds();

                MeshCollider meshCollider = meshFilter.GetComponent<MeshCollider>();
                if (meshCollider != null)
                {
                    meshCollider.sharedMesh = mesh;
                }
            }
        }

        private float GetHeightAtPosition(Vector3 worldPos)
        {
            return FBM(worldPos) * heightScale + heightOffset;
        }

        private float FBM(Vector3 position)
        {
            float amplitude = 1f;
            float frequency = baseFrequency;
            float total = 0f;
            float maxValue = 0f;  // Used for normalization

            // Scale the position to control feature size
            Vector3 scaledPos = new Vector3(
                position.x / horizontalScale,
                position.y,
                position.z / horizontalScale
            );

            for (int i = 0; i < octaves; i++)
            {
                // Add noise at current frequency and amplitude
                float noiseValue = Mathf.PerlinNoise(
                    (scaledPos.x * frequency) + noiseOffset.x,
                    (scaledPos.z * frequency) + noiseOffset.y
                );

                total += noiseValue * amplitude;
                maxValue += amplitude;

                // Update frequency and amplitude for next octave
                frequency *= lacunarity;
                amplitude *= persistence;
            }

            // Normalize the result to keep it in the [0,1] range
            return total / maxValue;
        }

        // Optional: Method to update heights at runtime
        public void UpdateHeights()
        {
            ApplyHeightMap();
        }
    }
}