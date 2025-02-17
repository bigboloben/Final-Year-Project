using UnityEngine;

namespace Assets.TrackGeneration
{
    public class StartLineInfo
    {
        public GameObject StartLine { get; }
        public Vector3 Position1 { get; }
        public Vector3 Position2 { get; }
        public Vector3 StartDirection { get; }
        public Quaternion StartRotation { get; }

        public StartLineInfo(
            GameObject startLine,
            Vector3 position1,
            Vector3 position2,
            Vector3 startDirection,
            Quaternion startRotation)
        {
            StartLine = startLine;
            Position1 = position1;
            Position2 = position2;
            StartDirection = startDirection;
            StartRotation = startRotation;
        }
    }
}
