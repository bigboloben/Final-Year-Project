using System.Collections.Generic;
using UnityEngine;

namespace Assets.TrackGeneration
{
    public enum GenerationStrategy
    {
        GridWithNoise,
        CircularLayout,
        Random
    }

    public class PointGenerator
    {
        private System.Random random;
        private int canvasSize;
        private const int MARGIN = 40;

        public PointGenerator(int canvasSize)
        {
            this.random = new System.Random();
            this.canvasSize = canvasSize;
        }

        public List<Vector2> GeneratePoints(int pointCount, GenerationStrategy strategy = GenerationStrategy.GridWithNoise)
        {
            switch (strategy)
            {
                case GenerationStrategy.GridWithNoise:
                    return GenerateGridWithNoise(pointCount);
                case GenerationStrategy.CircularLayout:
                    return GenerateCircularLayout(pointCount);
                case GenerationStrategy.Random:
                    return GenerateRandomPoints(pointCount);
                default:
                    return GenerateGridWithNoise(pointCount);
            }
        }

        private List<Vector2> GenerateRandomPoints(int pointCount)
        {
            List<Vector2> points = new List<Vector2>();

            for (int i = 0; i < pointCount; i++)
            {
                float x = MARGIN + (float)random.NextDouble() * (canvasSize - 2 * MARGIN);
                float y = MARGIN + (float)random.NextDouble() * (canvasSize - 2 * MARGIN);

                points.Add(new Vector2(x, y));
            }

            return points;
        }

        private List<Vector2> GenerateGridWithNoise(int pointCount)
        {
            List<Vector2> points = new List<Vector2>();
            int gridSize = (int)Mathf.Sqrt(pointCount);
            float cellSize = (canvasSize - 2 * MARGIN) / (float)gridSize;
            float noiseRange = cellSize * 0.3f;

            for (int i = 0; i < gridSize; i++)
            {
                for (int j = 0; j < gridSize; j++)
                {
                    float baseX = MARGIN + i * cellSize + cellSize / 2;
                    float baseY = MARGIN + j * cellSize + cellSize / 2;

                    float noise_x = (float)(random.NextDouble() * 2 - 1) * noiseRange;
                    float noise_y = (float)(random.NextDouble() * 2 - 1) * noiseRange;

                    points.Add(new Vector2(
                        Mathf.Clamp(baseX + noise_x, MARGIN, canvasSize - MARGIN),
                        Mathf.Clamp(baseY + noise_y, MARGIN, canvasSize - MARGIN)
                    ));
                }
            }

            while (points.Count < pointCount)
            {
                points.Add(new Vector2(
                    random.Next(MARGIN, canvasSize - MARGIN),
                    random.Next(MARGIN, canvasSize - MARGIN)
                ));
            }

            return points;
        }

        private List<Vector2> GenerateCircularLayout(int pointCount)
        {
            List<Vector2> points = new List<Vector2>();
            float centerX = canvasSize / 2;
            float centerY = canvasSize / 2;
            int rings = 4;

            float maxRadius = (canvasSize - 2 * MARGIN) * 0.45f;
            float radiusStep = maxRadius / rings;

            for (int ring = 1; ring <= rings; ring++)
            {
                float radius = ring * radiusStep;
                int pointsInRing = (int)(pointCount * (ring / (float)rings) / 2);
                float angleStep = 2 * Mathf.PI / pointsInRing;

                for (int i = 0; i < pointsInRing; i++)
                {
                    float angle = i * angleStep;
                    float noise = (float)(random.NextDouble() * radiusStep);
                    float x = centerX + (radius + noise) * Mathf.Cos(angle);
                    float y = centerY + (radius + noise) * Mathf.Sin(angle);
                    points.Add(new Vector2(
                        Mathf.Clamp(x, MARGIN, canvasSize - MARGIN),
                        Mathf.Clamp(y, MARGIN, canvasSize - MARGIN)
                    ));
                }
            }

            while (points.Count < pointCount)
            {
                float x = MARGIN + (float)random.NextDouble() * (canvasSize - 2 * MARGIN);
                float y = MARGIN + (float)random.NextDouble() * (canvasSize - 2 * MARGIN);
                points.Add(new Vector2(x, y));
            }

            return points;
        }

        private List<Vector2> GenerateSpiralLayout(int pointCount)
        {
            List<Vector2> points = new List<Vector2>();
            float centerX = canvasSize / 2;
            float centerY = canvasSize / 2;

            float maxRadius = (canvasSize - 2 * MARGIN) * 0.45f;
            float spiralSpacing = maxRadius / Mathf.Sqrt(pointCount);
            float angleStep = 2 * Mathf.PI / Mathf.Sqrt(pointCount);

            for (int i = 0; i < pointCount; i++)
            {
                float angle = angleStep * i;
                float radius = spiralSpacing * angle / (2 * Mathf.PI);

                float noiseRadius = (float)(random.NextDouble() - 0.5) * spiralSpacing * 0.5f;
                float noiseAngle = (float)(random.NextDouble() - 0.5) * angleStep * 0.3f;

                float x = centerX + (radius + noiseRadius) * Mathf.Cos(angle + noiseAngle);
                float y = centerY + (radius + noiseRadius) * Mathf.Sin(angle + noiseAngle);

                points.Add(new Vector2(
                    Mathf.Clamp(x, MARGIN, canvasSize - MARGIN),
                    Mathf.Clamp(y, MARGIN, canvasSize - MARGIN)
                ));
            }

            return points;
        }

        private List<Vector2> GenerateDiagonalStrips(int pointCount)
        {
            List<Vector2> points = new List<Vector2>();
            int stripsCount = (int)Mathf.Sqrt(pointCount);
            float stripWidth = (canvasSize - 2 * MARGIN) / stripsCount;
            int pointsPerStrip = pointCount / stripsCount;

            for (int strip = 0; strip < stripsCount; strip++)
            {
                float stripStart = strip * stripWidth + MARGIN;
                for (int i = 0; i < pointsPerStrip; i++)
                {
                    float progress = (float)i / pointsPerStrip;
                    float x = stripStart + progress * stripWidth;
                    float y = MARGIN + progress * (canvasSize - 2 * MARGIN);

                    float noiseX = (float)(random.NextDouble() - 0.5) * stripWidth * 0.3f;
                    float noiseY = (float)(random.NextDouble() - 0.5) * stripWidth * 0.3f;

                    points.Add(new Vector2(
                        Mathf.Clamp(x + noiseX, MARGIN, canvasSize - MARGIN),
                        Mathf.Clamp(y + noiseY, MARGIN, canvasSize - MARGIN)
                    ));
                }
            }

            while (points.Count < pointCount)
            {
                points.Add(new Vector2(
                    random.Next(MARGIN, canvasSize - MARGIN),
                    random.Next(MARGIN, canvasSize - MARGIN)
                ));
            }

            return points;
        }

        private List<Vector2> GenerateClusteredGroups(int pointCount)
        {
            List<Vector2> points = new List<Vector2>();
            int clusterCount = (int)Mathf.Sqrt(pointCount) / 2;
            int pointsPerCluster = pointCount / clusterCount;

            for (int c = 0; c < clusterCount; c++)
            {
                float centerX = MARGIN + (float)random.NextDouble() * (canvasSize - 2 * MARGIN);
                float centerY = MARGIN + (float)random.NextDouble() * (canvasSize - 2 * MARGIN);
                float clusterRadius = (canvasSize - 2 * MARGIN) / (float)(clusterCount * 2);

                for (int i = 0; i < pointsPerCluster; i++)
                {
                    float angle = (float)(random.NextDouble() * 2 * Mathf.PI);
                    float radius = (float)(random.NextDouble() * clusterRadius);

                    float x = centerX + radius * Mathf.Cos(angle);
                    float y = centerY + radius * Mathf.Sin(angle);

                    points.Add(new Vector2(
                        Mathf.Clamp(x, MARGIN, canvasSize - MARGIN),
                        Mathf.Clamp(y, MARGIN, canvasSize - MARGIN)
                    ));
                }
            }

            while (points.Count < pointCount)
            {
                points.Add(new Vector2(
                    random.Next(MARGIN, canvasSize - MARGIN),
                    random.Next(MARGIN, canvasSize - MARGIN)
                ));
            }

            return points;
        }

        private List<Vector2> GenerateRadialSpokes(int pointCount)
        {
            List<Vector2> points = new List<Vector2>();
            float centerX = canvasSize / 2;
            float centerY = canvasSize / 2;
            int spokeCount = (int)Mathf.Sqrt(pointCount);
            int pointsPerSpoke = pointCount / spokeCount;

            float maxRadius = (canvasSize - 2 * MARGIN) * 0.45f;
            float angleStep = 2 * Mathf.PI / spokeCount;

            for (int spoke = 0; spoke < spokeCount; spoke++)
            {
                float baseAngle = spoke * angleStep;

                for (int i = 0; i < pointsPerSpoke; i++)
                {
                    float radius = (float)i / pointsPerSpoke * maxRadius;
                    float noiseRadius = (float)(random.NextDouble() - 0.5) * maxRadius * 0.1f;
                    float noiseAngle = (float)(random.NextDouble() - 0.5) * angleStep * 0.3f;

                    float x = centerX + (radius + noiseRadius) * Mathf.Cos(baseAngle + noiseAngle);
                    float y = centerY + (radius + noiseRadius) * Mathf.Sin(baseAngle + noiseAngle);

                    points.Add(new Vector2(
                        Mathf.Clamp(x, MARGIN, canvasSize - MARGIN),
                        Mathf.Clamp(y, MARGIN, canvasSize - MARGIN)
                    ));
                }
            }

            while (points.Count < pointCount)
            {
                points.Add(new Vector2(
                    random.Next(MARGIN, canvasSize - MARGIN),
                    random.Next(MARGIN, canvasSize - MARGIN)
                ));
            }

            return points;
        }
    }
}