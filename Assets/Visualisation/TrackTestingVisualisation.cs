using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;
using System;
using System.Linq;

namespace Assets.TrackGeneration
{
    public class TrackTestingVisualization : MonoBehaviour
    {
        [Header("References")]
        public TrackHandler trackHandler;

        [Header("Visualization Toggles")]
        [Tooltip("Show the generated points")]
        public bool showPoints = true;
        [Tooltip("Show the Delaunay triangulation")]
        public bool showDelaunay = true;
        [Tooltip("Show the Voronoi diagram")]
        public bool showVoronoi = true;
        [Tooltip("Show all detected cycles")]
        public bool showAllCycles = true;
        [Tooltip("Show the chosen cycle")]
        public bool showChosenCycle = true;

        [Header("Visualization Settings")]
        public Color backgroundColor = Color.white; // Changed to white
        public Color pointsColor = Color.black; // Changed to black
        public Color delaunayColor = Color.cyan;
        public Color voronoiColor = Color.green;
        public Color chosenCycleColor = Color.red;
        public float pointSize = 5f;
        public float lineWidth = 2f;

        // UI Objects
        private Canvas canvas;
        private RawImage visualizationImage;
        private Texture2D texture;

        // Data
        private List<Vector2> points;
        private List<Triangle> triangles;
        private List<VoronoiCell> voronoiCells;
        private List<Cycle> cycles;
        private Cycle chosenCycle;

        // Random color generator
        private System.Random random;
        private Dictionary<Cycle, Color> cycleColors = new Dictionary<Cycle, Color>();

        void Start()
        {
            InitializeUI();
            random = new System.Random();

            if (trackHandler == null)
            {
                Debug.LogError("TrackHandler reference is missing!");
                return;
            }

            // Find and subscribe to TrackHandlerExtension events
            GameObject trackHandlerObj = trackHandler.gameObject;
            TrackHandlerExtension extension = trackHandlerObj.GetComponent<TrackHandlerExtension>();
            if (extension != null)
            {
                extension.onTrackGenerated.AddListener(RefreshData);
                Debug.Log("Successfully subscribed to TrackHandlerExtension.onTrackGenerated event");
            }
            else
            {
                Debug.LogWarning("TrackHandlerExtension not found. Auto-refresh won't work.");
            }

            // Add direct monitoring using Update for manual testing
            InvokeRepeating("CheckForTrackChanges", 1f, 0.5f);
        }

        // Previous track data for comparison
        private int previousPointCount = 0;
        private int previousCycleCount = 0;

        private void CheckForTrackChanges()
        {
            if (trackHandler == null) return;

            // Check if track data has changed
            bool hasNewPoints = trackHandler.points != null &&
                               (previousPointCount != trackHandler.points.Count);

            bool hasNewCycles = trackHandler.cycles != null &&
                               (previousCycleCount != trackHandler.cycles.Count);

            // Update our cached counts
            previousPointCount = trackHandler.points?.Count ?? 0;
            previousCycleCount = trackHandler.cycles?.Count ?? 0;

            // If something changed, refresh
            if (hasNewPoints || hasNewCycles)
            {
                Debug.Log("Track changes detected - refreshing visualization");
                RefreshData();
            }
        }

        public void InitializeUI()
        {
            // Create canvas
            GameObject canvasObj = new GameObject("VisualizationCanvas");
            canvasObj.transform.SetParent(transform);
            canvas = canvasObj.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;

            // Add canvas scaler
            CanvasScaler scaler = canvasObj.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920, 1080);

            // Add graphic raycaster
            canvasObj.AddComponent<GraphicRaycaster>();

            // Create background panel
            GameObject panelObj = new GameObject("VisualizationPanel");
            panelObj.transform.SetParent(canvas.transform, false);
            RectTransform panelRect = panelObj.AddComponent<RectTransform>();
            panelRect.anchorMin = new Vector2(0.5f, 0.5f);
            panelRect.anchorMax = new Vector2(0.5f, 0.5f);

            // Make it square
            float size = Mathf.Min(Screen.width, Screen.height) * 0.8f;
            panelRect.sizeDelta = new Vector2(size, size);
            panelRect.anchoredPosition = Vector2.zero;

            Image panelImage = panelObj.AddComponent<Image>();
            panelImage.color = backgroundColor;

            // Create image for visualization
            GameObject imageObj = new GameObject("VisualizationImage");
            imageObj.transform.SetParent(panelObj.transform, false);
            RectTransform imageRect = imageObj.AddComponent<RectTransform>();
            imageRect.anchorMin = Vector2.zero;
            imageRect.anchorMax = Vector2.one;
            imageRect.offsetMin = new Vector2(20, 20);
            imageRect.offsetMax = new Vector2(-20, -20);

            visualizationImage = imageObj.AddComponent<RawImage>();
            visualizationImage.color = Color.white;

            // Create square texture
            int texSize = 1024;
            texture = new Texture2D(texSize, texSize, TextureFormat.RGBA32, false);
            ClearTexture();
            visualizationImage.texture = texture;

            Debug.Log("UI setup complete");
        }

        private void ClearTexture()
        {
            Color[] clearColors = new Color[texture.width * texture.height];
            for (int i = 0; i < clearColors.Length; i++)
            {
                clearColors[i] = backgroundColor; // Use backgroundColor instead of transparent
            }
            texture.SetPixels(clearColors);
            texture.Apply();
        }

        public void RefreshData()
        {
            Debug.Log("Refreshing visualization data");

            if (trackHandler == null)
            {
                Debug.LogError("TrackHandler is null");
                return;
            }

            // Cache the current data from the track handler
            if (trackHandler.points != null && trackHandler.points.Count > 0)
            {
                Debug.Log($"Found {trackHandler.points.Count} points in the track handler");
                points = new List<Vector2>(trackHandler.points);

                // Get triangulation data
                if (trackHandler.delaunayTriangulation != null)
                {
                    triangles = trackHandler.delaunayTriangulation.GetTriangles();
                    Debug.Log($"Got {triangles?.Count ?? 0} triangles from Delaunay triangulation");
                }
                else
                {
                    Debug.LogWarning("Delaunay triangulation is null");
                    triangles = null;
                }

                // Get Voronoi data
                if (trackHandler.voronoi != null)
                {
                    voronoiCells = trackHandler.voronoi.GetCells();
                    Debug.Log($"Got {voronoiCells?.Count ?? 0} Voronoi cells");

                    // Validate Voronoi cells
                    if (voronoiCells != null)
                    {
                        int validCells = 0;
                        foreach (var cell in voronoiCells)
                        {
                            if (cell.GetVertices() != null && cell.GetVertices().Count >= 3)
                            {
                                validCells++;
                            }
                        }
                        Debug.Log($"Found {validCells} valid Voronoi cells out of {voronoiCells.Count}");
                    }
                }
                else
                {
                    Debug.LogWarning("Voronoi diagram is null");
                    voronoiCells = null;
                }

                // Get cycle data
                if (trackHandler.cycles != null)
                {
                    cycles = new List<Cycle>(trackHandler.cycles);
                    Debug.Log($"Got {cycles.Count} cycles");
                }
                else
                {
                    Debug.LogWarning("Cycles list is null");
                    cycles = new List<Cycle>();
                }

                chosenCycle = trackHandler.cycle;
                Debug.Log($"Chosen cycle is {(chosenCycle != null ? "present" : "null")}");

                // Generate random colors for each cycle
                cycleColors.Clear();
                if (cycles != null)
                {
                    foreach (Cycle cycle in cycles)
                    {
                        if (cycle != null)
                        {
                            cycleColors[cycle] = GetRandomColor();
                        }
                    }
                    Debug.Log($"Generated {cycleColors.Count} random colors for cycles");
                }

                UpdateVisualization();
            }
            else
            {
                Debug.LogWarning("No points available in track handler");
            }
        }

        // Add this method to manually refresh the visualization
        public void ForceRefresh()
        {
            Debug.Log("Force refreshing visualization");
            if (trackHandler != null)
            {
                RefreshData();
            }
        }

        private Color GetRandomColor()
        {
            // Generate a vibrant random color (excluding very light colors that would be hard to see on white)
            float r = 0.1f + 0.7f * (float)random.NextDouble();
            float g = 0.1f + 0.7f * (float)random.NextDouble();
            float b = 0.1f + 0.7f * (float)random.NextDouble();
            return new Color(r, g, b, 1f);
        }

        private void UpdateVisualization()
        {
            if (texture == null)
            {
                Debug.LogError("Cannot update visualization: texture is null");
                return;
            }

            if (points == null || points.Count == 0)
            {
                Debug.LogWarning("Cannot update visualization: no points available");
                return;
            }

            Debug.Log($"Updating visualization with {points.Count} points");

            // Clear texture
            ClearTexture();

            // Track bounds to determine zoom level
            float minX = float.MaxValue, maxX = float.MinValue;
            float minY = float.MaxValue, maxY = float.MinValue;

            foreach (Vector2 point in points)
            {
                minX = Mathf.Min(minX, point.x);
                maxX = Mathf.Max(maxX, point.x);
                minY = Mathf.Min(minY, point.y);
                maxY = Mathf.Max(maxY, point.y);
            }

            // Add margin
            minX -= 10; maxX += 10;
            minY -= 10; maxY += 10;

            // Calculate visualization bounds
            float dataWidth = maxX - minX;
            float dataHeight = maxY - minY;

            // Make sure we have valid dimensions
            if (dataWidth <= 0) dataWidth = 100;
            if (dataHeight <= 0) dataHeight = 100;

            Vector2 canvasSize = new Vector2(dataWidth, dataHeight);
            float minScale = Mathf.Min(texture.width / canvasSize.x, texture.height / canvasSize.y);
            float scaleFactor = minScale * 0.9f; // Slight zoom out to ensure everything is visible

            // Calculate center of data
            Vector2 dataCenter = new Vector2((minX + maxX) / 2, (minY + maxY) / 2);

            // Center the visualization
            Vector2 centerOffset = new Vector2(texture.width / 2, texture.height / 2) - dataCenter * scaleFactor;

            Debug.Log($"Visualization bounds: X=[{minX},{maxX}], Y=[{minY},{maxY}], Scale={scaleFactor}");

            // Draw elements based on toggles (draw in reverse order to ensure proper layering)
            if (showVoronoi && voronoiCells != null)
            {
                Debug.Log("Drawing Voronoi diagram...");
                DrawVoronoiDiagram(scaleFactor, centerOffset);
            }

            if (showDelaunay && triangles != null)
            {
                Debug.Log("Drawing Delaunay triangulation...");
                DrawDelaunayTriangulation(scaleFactor, centerOffset);
            }

            if (showAllCycles && cycles != null)
            {
                Debug.Log("Drawing all cycles...");
                DrawAllCycles(scaleFactor, centerOffset);
            }

            if (showChosenCycle && chosenCycle != null)
            {
                Debug.Log("Drawing chosen cycle...");
                DrawChosenCycle(scaleFactor, centerOffset);
            }

            if (showPoints && points != null)
            {
                Debug.Log("Drawing points...");
                DrawPoints(scaleFactor, centerOffset);
            }

            // Apply texture changes
            texture.Apply();

            Debug.Log("Visualization updated successfully");
        }

        private void DrawPoints(float scaleFactor, Vector2 offset)
        {
            foreach (Vector2 point in points)
            {
                Vector2 screenPoint = point * scaleFactor + offset;
                DrawCircle(Mathf.RoundToInt(screenPoint.x), Mathf.RoundToInt(screenPoint.y), Mathf.RoundToInt(pointSize), pointsColor);
            }
        }

        private void DrawDelaunayTriangulation(float scaleFactor, Vector2 offset)
        {
            if (triangles == null) return;

            foreach (Triangle triangle in triangles)
            {
                // Adjust this to match your Triangle class implementation
                Vector2 p1 = triangle.P1 * scaleFactor + offset;
                Vector2 p2 = triangle.P2 * scaleFactor + offset;
                Vector2 p3 = triangle.P3 * scaleFactor + offset;

                DrawLine(p1, p2, delaunayColor);
                DrawLine(p2, p3, delaunayColor);
                DrawLine(p3, p1, delaunayColor);
            }
        }

        private void DrawVoronoiDiagram(float scaleFactor, Vector2 offset)
        {
            if (voronoiCells == null)
            {
                Debug.LogWarning("Voronoi cells are null");
                return;
            }

            Debug.Log($"Drawing {voronoiCells.Count} Voronoi cells");

            foreach (VoronoiCell cell in voronoiCells)
            {
                List<Vector2> vertices = cell.GetVertices();

                if (vertices == null || vertices.Count < 3)
                {
                    Debug.LogWarning($"Invalid Voronoi cell with {(vertices == null ? "null" : vertices.Count.ToString())} vertices");
                    continue;
                }

                // Sort vertices clockwise around their centroid for proper rendering
                Vector2 centroid = new Vector2(0, 0);
                foreach (Vector2 vertex in vertices)
                {
                    centroid += vertex;
                }
                centroid /= vertices.Count;

                // Sort vertices by angle around centroid
                List<Vector2> sortedVertices = vertices.OrderBy(v =>
                {
                    Vector2 dir = v - centroid;
                    return Mathf.Atan2(dir.y, dir.x);
                }).ToList();

                // Draw the cell edges
                for (int i = 0; i < sortedVertices.Count; i++)
                {
                    Vector2 current = sortedVertices[i] * scaleFactor + offset;
                    Vector2 next = sortedVertices[(i + 1) % sortedVertices.Count] * scaleFactor + offset;

                    // Check for invalid coordinates
                    if (float.IsNaN(current.x) || float.IsNaN(current.y) ||
                        float.IsNaN(next.x) || float.IsNaN(next.y) ||
                        float.IsInfinity(current.x) || float.IsInfinity(current.y) ||
                        float.IsInfinity(next.x) || float.IsInfinity(next.y))
                    {
                        Debug.LogWarning($"Invalid Voronoi vertex detected: {current} -> {next}");
                        continue;
                    }

                    // Skip lines that are too long (likely boundary issues)
                    float lineLength = Vector2.Distance(current, next);
                    if (lineLength > texture.width * 0.5f)
                    {
                        continue;
                    }

                    DrawLine(current, next, voronoiColor);
                }
            }
        }

        private void DrawAllCycles(float scaleFactor, Vector2 offset)
        {
            if (cycles == null) return;

            foreach (Cycle cycle in cycles)
            {
                // Use the appropriate property/method from your Cycle class
                List<Vector2> vertices = cycle.Points;

                // Use the random color assigned to this cycle
                Color cycleColor = cycleColors.ContainsKey(cycle) ? cycleColors[cycle] : GetRandomColor();

                for (int i = 0; i < vertices.Count; i++)
                {
                    Vector2 current = vertices[i] * scaleFactor + offset;
                    Vector2 next = vertices[(i + 1) % vertices.Count] * scaleFactor + offset;

                    DrawLine(current, next, cycleColor);
                }
            }
        }

        private void DrawChosenCycle(float scaleFactor, Vector2 offset)
        {
            if (chosenCycle == null) return;

            // Use the appropriate property/method from your Cycle class
            List<Vector2> vertices = chosenCycle.Points;

            for (int i = 0; i < vertices.Count; i++)
            {
                Vector2 current = vertices[i] * scaleFactor + offset;
                Vector2 next = vertices[(i + 1) % vertices.Count] * scaleFactor + offset;

                DrawLine(current, next, chosenCycleColor, lineWidth + 1);
            }
        }

        private void DrawCircle(int centerX, int centerY, int radius, Color color)
        {
            if (texture == null) return;

            int radiusSquared = radius * radius;

            for (int y = -radius; y <= radius; y++)
            {
                for (int x = -radius; x <= radius; x++)
                {
                    if (x * x + y * y <= radiusSquared)
                    {
                        int pixelX = centerX + x;
                        int pixelY = centerY + y;

                        if (pixelX >= 0 && pixelX < texture.width && pixelY >= 0 && pixelY < texture.height)
                        {
                            texture.SetPixel(pixelX, pixelY, color);
                        }
                    }
                }
            }
        }

        private void DrawLine(Vector2 start, Vector2 end, Color color, float width = 0)
        {
            if (texture == null) return;

            // Use the default line width if not specified
            if (width <= 0) width = lineWidth;

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
                // Draw a thick point for width > 1
                if (width > 1)
                {
                    DrawCircle(x0, y0, Mathf.RoundToInt(width / 2), color);
                }
                else
                {
                    if (x0 >= 0 && x0 < texture.width && y0 >= 0 && y0 < texture.height)
                    {
                        texture.SetPixel(x0, y0, color);
                    }
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

        void OnValidate()
        {
            // Update visualization when toggling options in the editor during play mode
            if (Application.isPlaying && texture != null)
            {
                // Force a full refresh when options change
                if (trackHandler != null)
                {
                    RefreshData();
                }
                else if (points != null && points.Count > 0)
                {
                    UpdateVisualization();
                }
            }
        }

        void Update()
        {
            // Manual refresh with F5 key during runtime for testing
            if (Input.GetKeyDown(KeyCode.F5))
            {
                Debug.Log("Manual refresh triggered with F5");
                ForceRefresh();
            }
        }

        void OnDestroy()
        {
            // Clean up texture
            if (texture != null)
            {
                Destroy(texture);
            }

            // Unsubscribe from events
            if (trackHandler != null)
            {
                TrackHandlerExtension extension = trackHandler.GetComponent<TrackHandlerExtension>();
                if (extension != null)
                {
                    extension.onTrackGenerated.RemoveListener(RefreshData);
                }
            }
        }
    }
}