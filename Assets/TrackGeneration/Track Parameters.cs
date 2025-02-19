using UnityEngine;

namespace Assets.TrackGeneration
{
    public class TrackParameters
    {
        public float TrackWidth { get; }
        public float WallHeight { get; }
        public float WallDepth { get; }
        public int SegmentsPerUnit { get; }
        public float BankingAngle { get; }

        public float SupportCount { get; }
        public const float TRACK_HEIGHT_OFFSET = 0.1f;

        public TrackParameters(float width, float height, float depth, int segments, float banking, float supportCount)
        {
            TrackWidth = width;
            WallHeight = height;
            WallDepth = depth;
            SegmentsPerUnit = segments;
            BankingAngle = banking;
            SupportCount = supportCount;
        }
    }
}