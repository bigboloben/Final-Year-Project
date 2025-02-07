using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;
using System.Linq;
using UnityEngine.Splines;

namespace Assets.TrackGeneration
{
    public class TrackVisualizer : MonoBehaviour
    {
        private CarControls controls;  // Using the generated class directly

        private const int PointCount = 200;
        private const int CanvasSize = 500;

        private List<Vector2> points;
        private List<Triangle> triangles;
        private Graph graph;
        private DelaunayTriangulation delaunayTriangulation;
        private List<Cycle> cycles;
        private Cycle cycle;
        private Cycle rightCycle;
        private Cycle leftCycle;
        private List<Edge> voronoiEdges;
        private VoronoiDiagram voronoi;

        public RawImage drawingSurface;

        private Texture2D drawingTexture;
        private Color[] clearColors;

        private List<SplineContainer> splineContainers = new List<SplineContainer>();

        void Awake()
        {
            controls = new CarControls();

            // Subscribe to events using the generated controls
            controls.Keyboard.Grid.performed += ctx => GenerateTrack(GenerationStrategy.GridWithNoise);
            controls.Keyboard.Circular.performed += ctx => GenerateTrack(GenerationStrategy.CircularLayout);
            controls.Keyboard.Random.performed += ctx => GenerateTrack(GenerationStrategy.Random);
            controls.Keyboard.Curve.performed += ctx => SmoothTracks();
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
            InitializeDrawingSurface();
            GenerateTrack(GenerationStrategy.GridWithNoise);
        }

        private void InitializeDrawingSurface()
        {
            drawingTexture = new Texture2D(CanvasSize, CanvasSize);
            drawingSurface.texture = drawingTexture;

            clearColors = new Color[CanvasSize * CanvasSize];
            for (int i = 0; i < clearColors.Length; i++)
                clearColors[i] = Color.white;
        }

        private void SmoothTracks()
        {
            // Create new splines and point-based curves
            //List<Cycle> smoothedCycles = new List<Cycle>();

            //foreach (var cycle in cycles)
            //{
                // Create spline GameObject for this cycle
                GameObject splineObject = new GameObject($"Track Spline");

                // Create both the point-based curve and the spline
                SplineContainer splineContainer = cycle.CreateSmoothedSpline(splineObject);
            List<Vector2> smoothedCycle = cycle.GetPointsAlongSpline(splineContainer, 200);
            // Draw the spline points
            //smoothedCycles.Add(new Cycle(cycle.GetPointsAlongSpline(splineContainer, 200)));
            //cycles = smoothedCycles;
            cycle = new Cycle(smoothedCycle);


        }

        


        private void GenerateTrack(GenerationStrategy generation)
        {
            points = new List<Vector2>();
            triangles = new List<Triangle>();
            graph = new Graph();
            voronoiEdges = new List<Edge>();
            cycles = new List<Cycle>();

            GeneratePoints(generation);
            delaunayTriangulation = new DelaunayTriangulation(points);
            graph = delaunayTriangulation.Compute();
            graph.RemoveOutOfBounds(CanvasSize);
            cycles = graph.FindAllCycles();

            voronoi = new VoronoiDiagram(delaunayTriangulation, delaunayTriangulation.GetTriangles());
            voronoiEdges = voronoi.GetVoronoiEdges();
            cycle = voronoi.SortCycles(cycles);
        }

        private void GeneratePoints(GenerationStrategy generationStratergy)
        {
            PointGenerator gen = new PointGenerator(CanvasSize);
            Debug.Log($"Generating track with strategy: {generationStratergy}");
            points = gen.GeneratePoints(PointCount, generationStratergy);
        }

        void Update()
        {
            // Clear the texture
            drawingTexture.SetPixels(clearColors);

            // Draw the graph edges
            foreach (var vertex in graph.AdjacencyList.Keys)
            {
                foreach (var neighbor in graph.AdjacencyList[vertex])
                {
                    if (IsPointInBounds(vertex) && IsPointInBounds(neighbor))
                    {
                        DrawLine(vertex, neighbor, Color.blue);
                    }
                }
            }

            // Draw Voronoi edges
            foreach (var edge in voronoiEdges)
            {
                if (IsPointInBounds(edge.Start) && IsPointInBounds(edge.End))
                {
                    DrawLine(edge.Start, edge.End, Color.magenta);
                }
            }

            // Draw points
            foreach (var point in graph.AdjacencyList.Keys)
            {
                if (IsPointInBounds(point))
                {
                    DrawPoint(point, Color.red);
                }
            }

            // Draw cycles
            //System.Random rand = new System.Random(42);
            //foreach (var cycle in cycles)
            //{
            //    Color cycleColor = new Color(
            //        rand.Next(256) / 255f,
            //        rand.Next(256) / 255f,
            //        rand.Next(256) / 255f
            //    );

            //    for (int i = 0; i < cycle.Points.Count; i++)
            //    {
            //        int nextIdx = (i + 1) % cycle.Points.Count;
            //        DrawThickLine(cycle.Points[i], cycle.Points[nextIdx], cycleColor, 5);
            //    }

            //    if (cycle.Points.Count > 0)
            //    {
            //        DrawPoint(cycle.Points[0], Color.black);
            //    }
            //}
            // Draw only the optimal cycle
            if (cycle != null && cycle.Points.Count > 0)
            {
                Color cycleColor = Color.green;
                for (int i = 0; i < cycle.Points.Count; i++)
                {
                    int nextIdx = (i + 1) % cycle.Points.Count;
                    DrawThickLine(cycle.Points[i], cycle.Points[nextIdx], cycleColor, 5);
                }
                DrawPoint(cycle.Points[0], Color.black);
            }

            drawingTexture.Apply();
        }

        private void DrawLine(Vector2 start, Vector2 end, Color color)
        {
            int x0 = Mathf.RoundToInt(start.x);
            int y0 = Mathf.RoundToInt(start.y);
            int x1 = Mathf.RoundToInt(end.x);
            int y1 = Mathf.RoundToInt(end.y);

            int dx = Mathf.Abs(x1 - x0);
            int dy = Mathf.Abs(y1 - y0);
            int sx = x0 < x1 ? 1 : -1;
            int sy = y0 < y1 ? 1 : -1;
            int err = dx - dy;

            while (true)
            {
                if (x0 >= 0 && x0 < CanvasSize && y0 >= 0 && y0 < CanvasSize)
                {
                    drawingTexture.SetPixel(x0, y0, color);
                }

                if (x0 == x1 && y0 == y1) break;
                int e2 = 2 * err;
                if (e2 > -dy)
                {
                    err -= dy;
                    x0 += sx;
                }
                if (e2 < dx)
                {
                    err += dx;
                    y0 += sy;
                }
            }
        }

        private void DrawThickLine(Vector2 start, Vector2 end, Color color, int thickness)
        {
            for (int i = -thickness / 2; i <= thickness / 2; i++)
            {
                for (int j = -thickness / 2; j <= thickness / 2; j++)
                {
                    Vector2 offset = new Vector2(i, j);
                    DrawLine(start + offset, end + offset, color);
                }
            }
        }

        private void DrawPoint(Vector2 point, Color color)
        {
            int x = Mathf.RoundToInt(point.x);
            int y = Mathf.RoundToInt(point.y);
            int radius = 3;

            for (int i = -radius; i <= radius; i++)
            {
                for (int j = -radius; j <= radius; j++)
                {
                    if (i * i + j * j <= radius * radius)
                    {
                        int px = x + i;
                        int py = y + j;
                        if (px >= 0 && px < CanvasSize && py >= 0 && py < CanvasSize)
                        {
                            drawingTexture.SetPixel(px, py, color);
                        }
                    }
                }
            }
        }

        private bool IsPointInBounds(Vector2 point)
        {
            return point.x >= -10 && point.x <= CanvasSize + 10 &&
                   point.y >= -10 && point.y <= CanvasSize + 10;
        }
    }
}