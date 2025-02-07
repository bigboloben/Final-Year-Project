using UnityEngine;
using System.Collections.Generic;
using UnityEngine.Splines;

namespace Assets.TrackGeneration
{
    public class TrackSegmentGenerator
    {
        public float segmentLength { get; private set; }
        public bool adjustScale { get; private set; }
        public float maxStretchFactor { get; private set; }
        public bool smoothTransitions { get; private set; }
        public float blendDistance { get; private set; }

        private List<GameObject> placedSegments = new List<GameObject>();

        public TrackSegmentGenerator(GameObject prefab, bool adjustScale = true, float maxStretchFactor = 1.2f,
            bool smoothTransitions = true, float blendDistance = 0.5f)
        {
            this.segmentLength = GetPrefabLength(prefab)*3f;
            this.adjustScale = adjustScale;
            this.maxStretchFactor = maxStretchFactor;
            this.smoothTransitions = smoothTransitions;
            this.blendDistance = blendDistance;
        }

        public GameObject GenerateTrackFromSegments(SplineContainer centerlineSpline, GameObject trackSegmentPrefab, Transform parent = null)
        {
            if (centerlineSpline == null || centerlineSpline.Spline == null || centerlineSpline.Spline.Count == 0)
            {
                Debug.LogError("TrackSegmentGenerator: Invalid spline data!");
                return null;
            }

            try
            {
                // Create container for all segments
                GameObject trackContainer = new GameObject("TrackSegments");
                if (parent != null)
                {
                    trackContainer.transform.SetParent(parent);
                }

                float splineLength = GetSplineLength(centerlineSpline);
                int numberOfSegments = Mathf.CeilToInt(splineLength / segmentLength);
                float step = 1f / numberOfSegments;

                for (float t = 0; t < 1f; t += step)
                {
                    PlaceSegment(centerlineSpline, trackSegmentPrefab, trackContainer.transform, t, step);
                }

                Debug.Log($"TrackSegmentGenerator: Successfully placed {placedSegments.Count} segments");
                return trackContainer;
            }
            catch (System.Exception e)
            {
                Debug.LogError($"TrackSegmentGenerator: Error generating segments: {e.Message}\n{e.StackTrace}");
                return null;
            }
        }

        private void PlaceSegment(SplineContainer spline, GameObject prefab, Transform parent, float t, float step)
        {
            Vector3 position = spline.EvaluatePosition(t);
            Vector3 tangent = spline.EvaluateTangent(t);
            Vector3 up = Vector3.up;

            GameObject segment = Object.Instantiate(prefab, parent);
            segment.transform.position = position;

            Vector3 forward = tangent.normalized;
            Vector3 right = Vector3.Cross(up, forward).normalized;
            up = Vector3.Cross(forward, right);

            segment.transform.rotation = Quaternion.LookRotation(forward, up);

            if (adjustScale)
            {
                Vector3 nextPos = spline.EvaluatePosition(Mathf.Min(t + step, 1f));
                float actualLength = Vector3.Distance(position, nextPos);
                float scaleFactor = Mathf.Min(actualLength / segmentLength, maxStretchFactor);

                segment.transform.localScale = new Vector3(3f, 3f, 3f*scaleFactor);
            }

            if (smoothTransitions)
            {
                BlendWithNeighbors(segment, t);
            }

            placedSegments.Add(segment);
        }

        private void BlendWithNeighbors(GameObject segment, float t)
        {
            // Implementation for mesh blending if needed
        }

        private float GetSplineLength(SplineContainer spline)
        {
            float length = 0f;
            int samples = 100;
            Vector3 prevPoint = spline.EvaluatePosition(0f);

            for (int i = 1; i <= samples; i++)
            {
                float t = i / (float)samples;
                Vector3 point = spline.EvaluatePosition(t);
                length += Vector3.Distance(prevPoint, point);
                prevPoint = point;
            }

            return length;
        }

        public float GetPrefabLength(GameObject prefab)
        {
            // Get the mesh from the prefab
            MeshFilter meshFilter = prefab.GetComponent<MeshFilter>();
            if (meshFilter != null && meshFilter.sharedMesh != null)
            {
                // Get the bounds and length in local space
                Bounds bounds = meshFilter.sharedMesh.bounds;
                return bounds.size.z;  // Assuming Z is your forward direction
            }

            // If no mesh, try getting renderer bounds
            Renderer renderer = prefab.GetComponent<Renderer>();
            if (renderer != null)
            {
                return renderer.bounds.size.z;
            }

            Debug.LogError("Could not determine prefab length - no Mesh or Renderer found!");
            return 1f;
        }
    }
}