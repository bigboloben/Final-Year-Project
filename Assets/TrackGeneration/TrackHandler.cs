using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using UnityEngine.Splines;
using UnityEngine.InputSystem;


namespace Assets.TrackGeneration
{
    public class TrackHandler : MonoBehaviour
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
        private SplineContainer rightSpline;
        private SplineContainer leftSpline;
        private GameObject splineObject;
        private GameObject rightSplineObject;
        private GameObject leftSplineObject;

        [Header("Track Settings")]
        public float trackWidth = 100f;
        public float trackHeight = 0.1f;
        private float wallHeight = 0.5f;
        private float wallWidth = 0.25f;
        private int segments = 2;
        private float banking = 15f;
        public Material trackMaterial;
        public Material wallMaterial;
        public PhysicsMaterial wallPhysicsMaterial;
        public Material startLineMaterial;
        public Material gridMarkerMaterial ;
        public bool showDebugLines = false;

        [Header("Checkpoint Settings")]
        public float checkpointSpacing = 20f;
        public float checkpointWidth = 20f;
        public float checkpointHeight = 5f;
        //private CheckpointManager checkpointManager;
        private List<Checkpoint> checkpoints = new List<Checkpoint>();  

        public Camera trackCamera;  // Reference to the camera
        public float cameraHeight = 800f;  // Height of the camera above the track

        private Vector3 startPosition1;
        private Vector3 startPosition2;
        private Quaternion startRotation;

        private GameObject trackMesh;
        private List<LineRenderer> debugLineRenderers = new List<LineRenderer>();
        private TrackMesh trackMeshGenerator;
        private TrackParameters trackParameters;



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
                Destroy(rightSplineObject);
                rightSplineObject = null;
                rightSpline = null;
                Destroy(leftSplineObject);
                leftSplineObject = null;
                leftSpline = null;
            }

            if (checkpoints == null)
            {
                checkpoints = new List<Checkpoint>();
            }
            else
            {
                foreach (var checkpoint in checkpoints)
                {
                    if (checkpoint != null && checkpoint.gameObject != null)
                    {
                        Destroy(checkpoint.gameObject);
                    }
                }
                checkpoints.Clear();
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
            cycle.FindLongestSegmentAndSetStart();

            splineObject = new GameObject($"Track Spline");
            rightSplineObject = new GameObject($"Right Spline");
            leftSplineObject = new GameObject($"Left Spline");
            splineObject.transform.parent = transform;
            rightSplineObject.transform.parent = transform;
            leftSplineObject.transform.parent = transform;

            splineObject.SetActive(false);
            rightSplineObject.SetActive(false);
            leftSplineObject.SetActive(false);
            trackSpline = cycle.CreateSmoothedSpline(splineObject);
            (Cycle, Cycle) offsetCycles = cycle.GetOffsetCycles(trackWidth/2);
            rightSpline = offsetCycles.Item1.CreateSmoothedSpline(rightSplineObject);
            leftSpline = offsetCycles.Item2.CreateSmoothedSpline(leftSplineObject);
            //CenterSpline();

            CreateTrackMeshFromSplines();
            GenerateCheckpoints();
            
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
            trackParameters = new TrackParameters(trackWidth, wallHeight, wallWidth, segments, banking);
            trackMeshGenerator = new TrackMesh(trackParameters);

            //trackGenerator = new SplineTrackMeshGenerator(
            //    trackWidth,
            //    wallHeight, // track wall height
            //    wallWidth, // track wall width
            //    segments,    // segments per unit
            //    banking    // banking angle in degrees
            //);
            List<SplineContainer> splines = new List<SplineContainer>();
            splines.Add(leftSpline);
            splines.Add(trackSpline);
            splines.Add(rightSpline);
            GameObject generatedMesh = trackMeshGenerator.GenerateTrackMesh(
                                                                        splines, trackMaterial, wallMaterial, wallPhysicsMaterial, startLineMaterial, gridMarkerMaterial,
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
            Vector3 center = GetTrackCenter();
            

            // Move the spline container
            splineObject.transform.position -= center;
            rightSplineObject.transform.position -= center;
            leftSplineObject.transform.position -= center;
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

        private void GenerateCheckpoints()
        {
            if (trackSpline == null || trackMesh == null) return;

            // Sample points along the spline for checkpoint placement
            List<Vector3> centerPoints = new List<Vector3>();
            float step = 0.01f; // Sample every 1% of the spline

            for (float t = 0; t <= 1f; t += step)
            {
                Vector3 point = trackSpline.EvaluatePosition(t);
                centerPoints.Add(point);
            }

            // Generate checkpoints using the sampled points
            checkpoints = CheckpointGenerator.GenerateCheckpoints(
                leftPoints: trackMeshGenerator.splinePoints[0],
                rightPoints: trackMeshGenerator.splinePoints[2],
                trackObject: trackMesh
            );


            //Debug.Log($"Generated {checkpoints.Count} checkpoints");
        }

        public List<Checkpoint> GetCheckpoints()
        {
            return checkpoints;
        }


    }
}