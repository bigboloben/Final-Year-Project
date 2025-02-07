using UnityEngine;

namespace Assets.TrackGeneration
{
    public class Edge
    {
        public Vector2 Start, End;
        public Edge(Vector2 start, Vector2 end)
        {
            if (start.x < end.x || (Mathf.Approximately(start.x, end.x) && start.y < end.y))
            {
                Start = start;
                End = end;
            }
            else
            {
                Start = end;
                End = start;
            }
        }

        public bool Equals(Edge other)
        {
            return (Mathf.Approximately(Start.x, other.Start.x) && Mathf.Approximately(Start.y, other.Start.y) &&
                    Mathf.Approximately(End.x, other.End.x) && Mathf.Approximately(End.y, other.End.y)) ||
                   (Mathf.Approximately(Start.x, other.End.x) && Mathf.Approximately(Start.y, other.End.y) &&
                    Mathf.Approximately(End.x, other.Start.x) && Mathf.Approximately(End.y, other.Start.y));
        }
    }

    public class VoronoiEdge : Edge
    {
        public VoronoiEdge(Vector2 start, Vector2 end) : base(start, end) { }
    }
}