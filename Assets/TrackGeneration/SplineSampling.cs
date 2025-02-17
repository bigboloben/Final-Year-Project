using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Splines;

namespace Assets.TrackGeneration
{
    public class SplineArcLengthTable
    {
        private List<float> arcLengths = new List<float>();
        private List<float> tValues = new List<float>();
        private Spline spline;

        public SplineArcLengthTable(Spline spline, int resolution = 1000)
        {
            this.spline = spline;
            arcLengths.Add(0);
            tValues.Add(0);

            float totalLength = 0;
            Vector3 prevPoint = spline.EvaluatePosition(0);

            for (int i = 1; i <= resolution; i++)
            {
                float t = i / (float)resolution;
                Vector3 point = spline.EvaluatePosition(t);
                totalLength += Vector3.Distance(prevPoint, point);

                arcLengths.Add(totalLength);
                tValues.Add(t);
                prevPoint = point;
            }
        }

        public float ConvertLengthToT(float targetLength)
        {
            if (targetLength <= 0) return 0;
            if (targetLength >= arcLengths[^1]) return 1;

            for (int i = 1; i < arcLengths.Count; i++)
            {
                if (arcLengths[i] >= targetLength)
                {
                    // Linear interpolation between stored points
                    float t1 = tValues[i - 1], t2 = tValues[i];
                    float l1 = arcLengths[i - 1], l2 = arcLengths[i];

                    return t1 + (targetLength - l1) / (l2 - l1) * (t2 - t1);
                }
            }

            return 1;
        }
    }
}