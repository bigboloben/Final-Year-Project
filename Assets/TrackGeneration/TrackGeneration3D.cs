using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using UnityEngine.Splines;
using UnityEngine.InputSystem;
using UnityEditor.ShaderGraph;
using Unity.VisualScripting;

namespace Assets.TrackGeneration
{
    public class Track3DVisualizer : MonoBehaviour
    {
        private CarControls controls;


        //[Header("Track Segment Settings")]
        //public GameObject trackSegmentPrefab;
        //private GameObject trackSegments;

        public int PointCount = 150;
        public int CanvasSize = 1000;

        private List<Vector2> points;
        private Graph graph;
        private DelaunayTriangulation delaunayTriangulation;
        private List<Cycle> cycles;
        private Cycle cycle;
        private VoronoiDiagram voronoi;
        private SplineContainer trackSpline;
        private GameObject splineObject;

        [Header("Track Settings")]
        public float trackWidth = 100f;
        public float trackHeight = 0.1f;
        private float wallHeight = 0.5f;
        private float wallWidth = 0.25f;
        private int segments = 2;
        private float banking = 0f;
        public Material trackMaterial;
        public Material wallMaterial;
        public PhysicsMaterial wallPhysicsMaterial;
        public Material startLineMaterial;
        public Material gridMarkerMaterial ;
        public bool showDebugLines = false;

        public Camera trackCamera;  // Reference to the camera
        public float cameraHeight = 800f;  // Height of the camera above the track

        private Vector3 startPosition1;
        private Vector3 startPosition2;
        private Quaternion startRotation;

        private GameObject trackMesh;
        private List<LineRenderer> debugLineRenderers = new List<LineRenderer>();

        void Awake()
        {
            controls = new CarControls();

            // Subscribe to events using the generated controls
            controls.Keyboard.Grid.performed += ctx => GenerateTrack(GenerationStrategy.GridWithNoise);
            controls.Keyboard.Circular.performed += ctx => GenerateTrack(GenerationStrategy.CircularLayout);
            controls.Keyboard.Random.performed += ctx => GenerateTrack(GenerationStrategy.Random);
        }

        void OnEnable()
        {
            controls.Enable();
        }

        void OnDisable()
        {
            controls.Disable();
        }

        //private void OnDestroy()
        //{
        //    controls.Dispose();
        //}

        void Start()
        {
            GenerateTrack(GenerationStrategy.GridWithNoise);
        }

        private void OnSpacePerformed(InputAction.CallbackContext context)
        {
            GenerateTrack(GenerationStrategy.GridWithNoise);
        }

        private void OnShiftPerformed(InputAction.CallbackContext context)
        {
            GenerateTrack(GenerationStrategy.CircularLayout);
        }

        private void OnEnterPerformed(InputAction.CallbackContext context)
        {
            GenerateTrack(GenerationStrategy.Random);
        }
        void Update()
        {
            UpdateCameraPosition();
        }

        private void GenerateTrack(GenerationStrategy generation)
        {
            // Clear existing objects
            if (trackMesh != null) Destroy(trackMesh);
            foreach (var line in debugLineRenderers)
            {
                Destroy(line.gameObject);
            }
            debugLineRenderers.Clear();

            if (splineObject != null)
            {
                Destroy(splineObject);
                splineObject = null;
                trackSpline = null;
            }

            points = new List<Vector2>();
            graph = new Graph();
            cycles = new List<Cycle>();

            GeneratePoints(generation);
            delaunayTriangulation = new DelaunayTriangulation(points);
            graph = delaunayTriangulation.Compute();
            graph.RemoveOutOfBounds(CanvasSize);
            cycles = graph.FindAllCycles();

            voronoi = new VoronoiDiagram(delaunayTriangulation, delaunayTriangulation.GetTriangles());
            cycle = voronoi.SortCycles(cycles);

            splineObject = new GameObject($"Track Spline");
            splineObject.transform.parent = transform;
            splineObject.SetActive(false);
            trackSpline = cycle.CreateSmoothedSpline(splineObject);
            CenterSpline();

            CreateTrackMeshFromSplines();
        }

        private void CreateTrackMeshFromSplines()
        {
            if (trackSpline == null || trackSpline.Spline == null || trackSpline.Spline.Count == 0)
            {
                Debug.LogError("Invalid spline data!");
                return;
            }

            if (trackMesh != null)
                Destroy(trackMesh);

            if (trackMaterial == null)
            {
                Debug.LogError("Track material is not assigned!");
                return;
            }

            SplineTrackMeshGenerator trackGenerator = new SplineTrackMeshGenerator(
                trackWidth,
                wallHeight, // track wall height
                wallWidth, // track wall width
                segments,    // segments per unit
                banking    // banking angle in degrees
            );

            GameObject generatedMesh = trackGenerator.GenerateTrackMesh(
                                                                        trackSpline, trackMaterial, wallMaterial, wallPhysicsMaterial, startLineMaterial, gridMarkerMaterial,
                                                                        out Vector3 startPos1, out Vector3 startPos2, out Quaternion startRot
                                                                        );
            startPosition1 = startPos1;
            startPosition2 = startPos2;
            startRotation = startRot;
            if (generatedMesh != null)
            {
                trackMesh = generatedMesh;
                trackMesh.transform.SetParent(transform);
            }
        }

        private void GeneratePoints(GenerationStrategy generationStrategy)
        {
            PointGenerator gen = new PointGenerator(CanvasSize);
            points = gen.GeneratePoints(PointCount, generationStrategy);
        }

        private void CenterSpline()
        {
            if (trackSpline == null || trackSpline.Spline == null)
                return;

            // Find bounds of the spline
            Vector3 min = Vector3.positiveInfinity;
            Vector3 max = Vector3.negativeInfinity;
            float step = 0.01f;  // Sample every 1% of the spline

            for (float t = 0; t <= 1f; t += step)
            {
                Vector3 point = trackSpline.EvaluatePosition(t);
                min = Vector3.Min(min, point);
                max = Vector3.Max(max, point);
            }

            // Calculate center of the bounds
            Vector3 center = (max + min) / 2f;
            Vector3 offset = -center; // Offset to move to world origin

            // Move the spline container
            splineObject.transform.position += offset;
        }

        public (Vector3 position1, Vector3 position2, Quaternion rotation) GetTrackStartTransform()
        {
            return (startPosition1, startPosition2, startRotation);
        }

        private bool IsPointInBounds(Vector2 point)
        {
            return point.x >= -10 && point.x <= CanvasSize + 10 &&
                   point.y >= -10 && point.y <= CanvasSize + 10;
        }

        //private void CreateTrackMesh()
        //{
        //    if (trackSpline == null || trackSpline.Spline == null || trackSpline.Spline.Count == 0)
        //    {
        //        Debug.LogError("Invalid spline data! Make sure the spline is properly created.");
        //        return;
        //    }
        //    if (trackMesh != null)
        //        Destroy(trackMesh);

        //    if (trackMaterial == null)
        //    {
        //        Debug.LogError("Track material is not assigned!");
        //        return;
        //    }

        //    TrackMeshGenerator trackGenerator = new TrackMeshGenerator(trackWidth, trackHeight, 10, 0f);
        //    GameObject generatedMesh = trackGenerator.GenerateTrackMesh(trackSpline, trackMaterial);

        //    if (generatedMesh != null)
        //    {
        //        trackMesh = generatedMesh;
        //        trackMesh.transform.SetParent(transform);
        //    }
        //    else
        //    {
        //        Debug.LogError("Failed to generate track mesh!");
        //    }
        //}

        //private void CreateTrackMeshFromSegement()
        //{
        //    if (trackMesh != null) Destroy(trackMesh);
        //    if (trackSegments != null) Destroy(trackSegments);

        //    if (trackSegmentPrefab != null)
        //    {
        //        trackSegmentPrefab.transform.localScale = new Vector3(3, 3, 3);
        //        TrackSegmentGenerator segmentGenerator = new TrackSegmentGenerator(trackSegmentPrefab);
        //        trackSegments = segmentGenerator.GenerateTrackFromSegments(trackSpline, trackSegmentPrefab, transform);
        //    }
        //}

        void UpdateCameraPosition()
        {
            if (trackCamera != null && trackSpline != null)
            {
                // Get the center of the spline
                Vector3 trackCenter = GetTrackCenter();

                // Set the camera position above the center of the track
                Vector3 cameraPosition = trackCenter + Vector3.up * cameraHeight;
                trackCamera.transform.position = cameraPosition;

                // Make the camera look down at the track
                trackCamera.transform.rotation = Quaternion.Euler(90f, 0f, 0f);  // Rotate to look straight down
            }
        }

        Vector3 GetTrackCenter()
        {
            // Find bounds of the spline to get the center
            if (trackSpline == null || trackSpline.Spline == null)
                return Vector3.zero;

            Vector3 min = Vector3.positiveInfinity;
            Vector3 max = Vector3.negativeInfinity;
            float step = 0.01f;  // Sample every 1% of the spline

            for (float t = 0; t <= 1f; t += step)
            {
                Vector3 point = trackSpline.EvaluatePosition(t);
                min = Vector3.Min(min, point);
                max = Vector3.Max(max, point);
            }

            // Calculate center of the bounds
            return (max + min) / 2f;
        }
    }
}