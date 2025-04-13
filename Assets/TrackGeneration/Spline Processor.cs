using System.Collections.Generic;
using System.Linq.Expressions;
using Unity.Mathematics;
using Unity.VisualScripting;
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

            List<Spline> leftSegments = GetSplineSegments(leftSpline);
            List<Spline> centerSegments = GetSplineSegments(centerSpline);
            List<Spline> rightSegments = GetSplineSegments(rightSpline);

            List<Vector3> leftPointsList = new List<Vector3>();
            List<Vector3> centerPointsList = new List<Vector3>();
            List<Vector3> rightPointsList = new List<Vector3>();

            float3 heightOffset = TrackParameters.TRACK_HEIGHT_OFFSET * Vector3.up;

            

            for (int i = 0; i < centerSegments.Count; i++)
            {
                int numberOfPoints = Mathf.Max(5, Mathf.RoundToInt(centerSegments[i].GetLength() * parameters.SegmentsPerUnit));

                
                float lengthDiff = Mathf.Abs(leftSegments[i].GetLength() - rightSegments[i].GetLength());
                float maxHeight = 0.2f * lengthDiff;
                float3 rightHeightOffset;
                float3 leftHeightOffset;
                //Debug.Log(maxHeight);

                for (int j = 0; j < numberOfPoints - 1; j++)
                {
                    float t = j / (float)(numberOfPoints - 1);
                    float heightMultiplier = 0.5f * Mathf.Sin(((2f * t) - 0.5f) * Mathf.PI) + 0.5f;
                    float currentHeight = maxHeight * heightMultiplier;

                    //Debug.Log($"Height: {currentHeight} height multiplier {heightMultiplier} t {t}");
                    if (leftSegments[i].GetLength() < rightSegments[i].GetLength())
                    {
                        rightHeightOffset = currentHeight * Vector3.up;
                        leftHeightOffset = Vector3.zero;
                    }
                    else
                    {
                        rightHeightOffset = Vector3.zero;
                        leftHeightOffset = currentHeight * Vector3.up;
                    }
                    if (i % 2 != 0)
                    {
                        rightHeightOffset = Vector3.zero;
                        leftHeightOffset = Vector3.zero;
                    }
                    
                    leftPointsList.Add(leftSegments[i].EvaluatePosition(t) + heightOffset + leftHeightOffset);
                    centerPointsList.Add(centerSegments[i].EvaluatePosition(t) + heightOffset);
                    rightPointsList.Add(rightSegments[i].EvaluatePosition(t) + heightOffset + rightHeightOffset);
                }
            }
            leftPointsList.Add(leftPointsList[0]);
            centerPointsList.Add(centerPointsList[0]);
            rightPointsList.Add(rightPointsList[0]);



            Vector3[] leftPoints = leftPointsList.ToArray();
            Vector3[] centerPoints = centerPointsList.ToArray();
            Vector3[] rightPoints = rightPointsList.ToArray();

            //int numberOfPoints = Mathf.Max(32, (int)(centerSpline.GetLength() * parameters.SegmentsPerUnit));
            //Vector3 heightOffset = Vector3.up * TrackParameters.TRACK_HEIGHT_OFFSET;

            //SplineArcLengthTable leftTable = new SplineArcLengthTable(leftSpline);
            //SplineArcLengthTable centerTable = new SplineArcLengthTable(centerSpline);
            //SplineArcLengthTable rightTable = new SplineArcLengthTable(rightSpline);

            //Vector3[] leftPoints = GenerateSplinePoints(leftSpline, leftTable, numberOfPoints, heightOffset);
            //Vector3[] centerPoints = GenerateSplinePoints(centerSpline, centerTable, numberOfPoints, heightOffset);
            //Vector3[] rightPoints = GenerateSplinePoints(rightSpline, rightTable, numberOfPoints, heightOffset);

            return new List<Vector3[]> { leftPoints, centerPoints, rightPoints };
        }

        public List<Spline> GetSplineSegments(Spline spline)
        {
            List<Spline> segments = new List<Spline>();
            for (int i = 0; i < spline.Count; i ++)
            {
                int nextIndex = (i + 1) % spline.Count;
                Spline toAdd = new Spline();
                toAdd.Clear();
                toAdd.Add(spline[i]);
                toAdd.Add(spline[nextIndex]);
                segments.Add(toAdd);
            }
            return segments;
        }


        private List<Vector3[]> GenerateSplines(List<SplineContainer> splineContainers)
        {
            
            // Create array with one extra point for the closing point
            
            Spline leftSpline = splineContainers[0].Spline;
            Spline centerSpline = splineContainers[1].Spline;
            Spline rightSpline = splineContainers[2].Spline;
            int numberOfPoints = Mathf.Max(5, Mathf.RoundToInt(centerSpline.GetLength() * parameters.SegmentsPerUnit));
            List<Spline> splines = new(new[] { leftSpline, centerSpline, rightSpline });

            List<Vector3[]> pointList = new();

            Vector3 heightOffset = TrackParameters.TRACK_HEIGHT_OFFSET * Vector3.up;


            foreach (var spline in splines)
            {
                Vector3[] points = new Vector3[numberOfPoints + 1];

                float splineLength = spline.GetLength();

                for (int i = 0; i < numberOfPoints - 1; i++)
                {
                    float t = i / (float)(numberOfPoints - 1);
                    Vector3 position = spline.EvaluatePosition(t);
                    position += heightOffset;
                    points[i] = position;
                }

                // Add the first point again at the end
                points[numberOfPoints] = points[0];

                pointList.Add(points);

               
            }
            return pointList;
        }
    }
}
