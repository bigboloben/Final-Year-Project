using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using UnityEngine.Splines;
using UnityEngine.InputSystem;
using Unity.Mathematics;


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
        public float wallHeight = 0.5f;
        public float wallWidth = 0.25f;
        public float segments = 1/5f;
        public float banking = 15f;
        public int supportCount = 20;
        public Material trackMaterial;
        public Material wallMaterial;
        public PhysicsMaterial wallPhysicsMaterial;
        public GameObject startLinePrefab;
        public GameObject gridMarkerPrefab;
        public Material trackSupportMaterial;
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

        private float[] heights;

        public int surfaceInstanceID;


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
 

            heights = GenerateSmoothArray(points.Count);
            //heights = null;

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
            trackSpline = cycle.CreateSmoothedSpline(splineObject, heights: heights);
            (Cycle, Cycle) offsetCycles = cycle.GetOffsetCycles(trackWidth/2);
            rightSpline = offsetCycles.Item1.CreateSmoothedSpline(rightSplineObject, heights: heights);
            leftSpline = offsetCycles.Item2.CreateSmoothedSpline(leftSplineObject, heights: heights);

            CenterSplines();

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

            

            trackParameters = new TrackParameters(trackWidth, wallHeight, wallWidth, segments, banking, supportCount);
            trackMeshGenerator = new TrackMesh(trackParameters);

            List<SplineContainer> splines = new List<SplineContainer>();
            splines.Add(leftSpline);
            splines.Add(trackSpline);
            splines.Add(rightSpline);
            GameObject generatedMesh = trackMeshGenerator.GenerateTrackMesh(
                splines, trackMaterial, wallMaterial, wallPhysicsMaterial, startLinePrefab, gridMarkerPrefab, trackSupportMaterial,
                out Vector3 startPos1, out Vector3 startPos2, out Quaternion startRot
            );
            startPosition1 = startPos1;
            startPosition2 = startPos2;
            startRotation = startRot;
            if (generatedMesh != null)
            {
                trackMesh = generatedMesh;
                trackMesh.transform.SetParent(transform);
                //var track = trackMesh.transform.Find("Track").gameObject;
                var surface = trackMesh.transform.Find("TrackSurface").gameObject;
                surfaceInstanceID = surface.GetComponent<MeshCollider>().GetInstanceID();
                //Debug.Log($"Surface instance ID: {surfaceInstanceID}");
            }
        }

        private void GeneratePoints(GenerationStrategy generationStrategy)
        {
            PointGenerator gen = new PointGenerator(CanvasSize);
            points = gen.GeneratePoints(PointCount, generationStrategy);
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
               ;

                // Set the camera position above the center of the track
                Vector3 cameraPosition = Vector3.zero + Vector3.up * cameraHeight;
                trackCamera.transform.position = cameraPosition;

                // Make the camera look down at the track
                trackCamera.transform.rotation = Quaternion.Euler(90f, 0f, 0f);  // Rotate to look straight down
            }
        }

        private void CenterSplines()
        {
            // Calculate center offset from spline points
            Vector3 min = Vector3.positiveInfinity;
            Vector3 max = Vector3.negativeInfinity;

            // Get actual spline knot positions
            for (int i = 0; i < trackSpline.Spline.Count; i++)
            {
                BezierKnot knot = trackSpline.Spline[i];
                min = Vector3.Min(min, knot.Position);
                max = Vector3.Max(max, knot.Position);
            }

            // Create offset that only affects X and Z
            float3 offset = new float3(
                -(max.x + min.x) / 2f,  // Center X
                0,                      // Don't center Y
                -(max.z + min.z) / 2f   // Center Z
            );

            // Apply offset to all knots in all splines
            for (int i = 0; i < trackSpline.Spline.Count; i++)
            {
                var knot = trackSpline.Spline[i];
                knot.Position += offset;
                trackSpline.Spline[i] = knot;

                knot = rightSpline.Spline[i];
                knot.Position += offset;
                rightSpline.Spline[i] = knot;

                knot = leftSpline.Spline[i];
                knot.Position += offset;
                leftSpline.Spline[i] = knot;
            }
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
        public float[] GenerateSmoothArray(int length, float minHeight = 0f, float maxHeight = 10f)
        {
            float[] array = new float[length];
            float scale = 0.3f; // Increased scale for more frequent changes
            float offset = UnityEngine.Random.Range(0f, 1000f);

            for (int i = 0; i < length; i++)
            {
                // Using multiple frequencies of noise to add more detail
                float noise1 = Mathf.PerlinNoise(i * scale + offset, 0.5f);
                float noise2 = Mathf.PerlinNoise(i * scale * 2 + offset, 1.5f) * 0.5f; // Higher frequency, lower amplitude

                // Combine the noise values with more weight on the primary frequency
                float noiseValue = (noise1 + noise2);

                // Map to desired range
                array[i] = Mathf.Lerp(minHeight, maxHeight, noiseValue);
            }

            return array;
        }

    }
}