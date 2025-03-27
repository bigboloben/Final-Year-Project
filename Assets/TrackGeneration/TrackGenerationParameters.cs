using UnityEngine;

namespace Assets.TrackGeneration
{
    [System.Serializable]
    public class TrackGenerationParameters
    {
        [Header("Track Geometry")]
        public float MinTrackLength = 1100f;
        public float IdealTrackLength = 1200f;
        public float MaxTrackLength = 1300f;

        [Header("Point Generation")]
        public int PointCount = 150;
        public int CanvasSize = 1000;
        //public float NoiseAmount = 0.3f;

        [Header("Quality Weights")]
        [Range(0, 1)] public float CornerQualityWeight = 0.25f;
        [Range(0, 1)] public float StraightQualityWeight = 0.1f;
        [Range(0, 1)] public float FlowQualityWeight = 0.1f;
        [Range(0, 1)] public float LayoutQualityWeight = 0.1f;
        [Range(0, 1)] public float LengthQualityWeight = 0.25f;
        [Range(0, 1)] public float ProximityQualityWeight = 0.2f;

        [Header("Corner Parameters")]
        public float TooSharpCornerAngle = 30f;
        public float SharpCornerAngle = 45f;
        public float MediumCornerAngle = 90f;
        public float WideCornerAngle = 120f;
        public int DesiredCornersCount = 8;
        public int MinCorners = 6;
        public int MaxCorners = 10;

        [Header("Straight Parameters")]
        public float MinStraightLength = 50f;
        public float IdealStraightLength = 80f;
        public float MaxStraightLength = 100f;
        public int DesiredStraightCount = 3;

        [Header("Track Safety")]
        public float MinSafeDistance = 50f;
        public float CriticalDistance = 40f;

        [Header("Elevation Parameters")]
        public float MinHeight = 0f;
        public float MaxHeight = 10f;
        //public float ElevationFrequency = 0.3f;
    }
}