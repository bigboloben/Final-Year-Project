using UnityEngine;
using System.Collections.Generic;
using UnityEditor.ShaderGraph;

namespace Assets.TrackGeneration
{
    public class FloorGenerator : MonoBehaviour
    {
        [Header("Floor Settings")]
        public GameObject floorPrefab;
        public int gridSize = 6;
        public float tileSize = 200f;

        private List<MeshFilter> floorTiles = new List<MeshFilter>();

        private CarControls controls;

        public void Initialize()
        {
            SetRandomSeed();
            GenerateFloor();
            SetContinuousUVs();
        }
        void Awake()
        {
            controls = new CarControls();

            // Subscribe to events using the generated controls
            controls.Keyboard.Grid.performed += ctx => SetRandomSeed();
            controls.Keyboard.Circular.performed += ctx => SetRandomSeed();
            controls.Keyboard.Random.performed += ctx => SetRandomSeed();
        }
        void OnEnable()
        {
            controls.Enable();
        }

        void OnDisable()
        {
            controls.Disable();
        }

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
                    tile.tag = "Floor";

                    MeshFilter meshFilter = tile.GetComponent<MeshFilter>();
                    if (meshFilter != null)
                    {
                        meshFilter.mesh = Instantiate(meshFilter.sharedMesh);
                        floorTiles.Add(meshFilter);
                    }
                    tile.AddComponent<MeshCollider>();
                }
            }
        }
        private void SetContinuousUVs()
        {
            // Calculate the exact bounds of the grid
            float halfExtent = gridSize * tileSize * 0.5f;
            Vector3 topLeftCorner = new Vector3(-halfExtent, 0, -halfExtent);
            float gridWidth = halfExtent * 2f;
            float gridDepth = halfExtent * 2f;

            foreach (MeshFilter meshFilter in floorTiles)
            {
                Mesh mesh = meshFilter.sharedMesh;
                if (mesh == null) continue;

                Vector3[] vertices = mesh.vertices;
                Vector2[] uvs = new Vector2[vertices.Length];

                // Cache transform data for performance
                Transform transform = meshFilter.transform;
                Vector3 position = transform.position;
                Vector3 scale = transform.lossyScale;

                for (int i = 0; i < vertices.Length; i++)
                {
                    // Convert local vertex position to world position with scale applied
                    Vector3 scaledVertex = Vector3.Scale(vertices[i], scale);
                    Vector3 worldPos = position + transform.rotation * scaledVertex;

                    // Calculate UVs
                    float u = (worldPos.x - topLeftCorner.x) / gridWidth;
                    float v = (worldPos.z - topLeftCorner.z) / gridDepth;

                    uvs[i] = new Vector2(u, v);
                }

                mesh.uv = uvs;
            }
        }

        public void SetRandomSeed()
        {
            Renderer renderer = floorPrefab.GetComponent<Renderer>();
            if (renderer != null)
            {
                float randomSeed = Random.Range(0f, 1000f);
                // Set the random seed property on the material
                renderer.sharedMaterial.SetFloat("_Random_Seed", randomSeed);
            }
        }
    }
}