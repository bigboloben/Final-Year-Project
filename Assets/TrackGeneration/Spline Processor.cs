using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Splines;

namespace Assets.TrackGeneration
{
    public class SplineProcessor
    {
        private readonly TrackParameters parameters;

        public SplineProcessor(TrackParameters parameters)
        {
            this.parameters = parameters;
        }

        public List<Vector3[]> GeneratePointsFromSplines(List<SplineContainer> splines)
        {
            Spline leftSpline = splines[0].Spline;
            Spline centerSpline = splines[1].Spline;
            Spline rightSpline = splines[2].Spline;

            int numberOfPoints = Mathf.Max(32, (int)(centerSpline.GetLength() * parameters.SegmentsPerUnit));
            Vector3 heightOffset = Vector3.up * TrackParameters.TRACK_HEIGHT_OFFSET;

            SplineArcLengthTable leftTable = new SplineArcLengthTable(leftSpline);
            SplineArcLengthTable centerTable = new SplineArcLengthTable(centerSpline);
            SplineArcLengthTable rightTable = new SplineArcLengthTable(rightSpline);

            Vector3[] leftPoints = GenerateSplinePoints(leftSpline, leftTable, numberOfPoints, heightOffset);
            Vector3[] centerPoints = GenerateSplinePoints(centerSpline, centerTable, numberOfPoints, heightOffset);
            Vector3[] rightPoints = GenerateSplinePoints(rightSpline, rightTable, numberOfPoints, heightOffset);

            return new List<Vector3[]> { leftPoints, centerPoints, rightPoints };
        }

        private Vector3[] GenerateSplinePoints(
            Spline spline,
            SplineArcLengthTable table,
            int numberOfPoints,
            Vector3 heightOffset)
        {
            // Create array with one extra point for the closing point
            Vector3[] points = new Vector3[numberOfPoints + 1];
            float splineLength = spline.GetLength();

            for (int i = 0; i < numberOfPoints; i++)
            {
                float normalizedArcLength = i / (float)(numberOfPoints - 1);
                float t = table.ConvertLengthToT(normalizedArcLength * splineLength);
                Vector3 position = spline.EvaluatePosition(t);
                position += heightOffset;
                points[i] = position;
            }

            // Add the first point again at the end
            points[numberOfPoints] = points[0];

            return points;
        }
    }
}
