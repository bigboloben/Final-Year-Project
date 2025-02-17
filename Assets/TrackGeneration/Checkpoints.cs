using UnityEngine;
using System.Collections.Generic;

namespace Assets.TrackGeneration
{
    public class Checkpoint : MonoBehaviour
    {
        private RaceManager raceManager;

        public void Initialize(RaceManager manager)
        {
            this.raceManager = manager;
        }

        private void OnTriggerEnter(Collider other)
        {
            if (other.CompareTag("Player"))
            {
                if (raceManager != null)
                {
                    raceManager.CheckpointTriggerEntered(other.gameObject, this);
                }
                else
                {
                    Debug.LogError("Race manager is null!");
                }
            }
        }
    }

    public static class CheckpointGenerator
    {
        public static List<Checkpoint> GenerateCheckpoints(
            Vector3[] centerPoints,
            GameObject trackObject,
            float checkpointSpacing = 20f,
            float checkpointWidth = 20f,
            float checkpointHeight = 5f)
        {
            List<Checkpoint> checkpoints = new List<Checkpoint>();
            GameObject checkpointsContainer = new GameObject("Checkpoints");
            checkpointsContainer.transform.SetParent(trackObject.transform);

            float currentDistance = 0f;
            int lastCheckpointIndex = 0;

            // Create checkpoints at regular intervals
            for (int i = 1; i < centerPoints.Length; i++)
            {
                currentDistance += Vector3.Distance(centerPoints[i], centerPoints[i - 1]);

                if (currentDistance >= checkpointSpacing || i == centerPoints.Length - 1)
                {
                    // Create checkpoint
                    GameObject checkpointObject = CreateCheckpointTrigger(
                        centerPoints[i],
                        GetCheckpointDirection(centerPoints, i),
                        checkpointWidth,
                        checkpointHeight);

                    checkpointObject.transform.SetParent(checkpointsContainer.transform);

                    Checkpoint checkpoint = checkpointObject.AddComponent<Checkpoint>();
                    checkpoints.Add(checkpoint);

                    currentDistance = 0f;
                    lastCheckpointIndex = i;
                }
            }

            // Initialize checkpoints with RaceManager
            RaceManager.Instance.InitializeCheckpoints(checkpoints);

            return checkpoints;
        }

        private static GameObject CreateCheckpointTrigger(Vector3 position, Vector3 direction, float width, float height)
        {
            GameObject checkpoint = new GameObject("Checkpoint");
            checkpoint.transform.position = position;

            BoxCollider triggerCollider = checkpoint.AddComponent<BoxCollider>();
            triggerCollider.isTrigger = true;
            triggerCollider.size = new Vector3(width, height, 0.5f);
            checkpoint.transform.rotation = Quaternion.LookRotation(direction, Vector3.up);

#if UNITY_EDITOR
            CreateCheckpointVisual(checkpoint, width, height);
#endif

            return checkpoint;
        }

        private static Vector3 GetCheckpointDirection(Vector3[] points, int index)
        {
            if (index >= points.Length - 1)
            {
                return (points[index] - points[index - 1]).normalized;
            }
            return (points[index + 1] - points[index]).normalized;
        }

#if UNITY_EDITOR
        private static void CreateCheckpointVisual(GameObject checkpoint, float width, float height)
        {
            LineRenderer lineRenderer = checkpoint.AddComponent<LineRenderer>();
            lineRenderer.positionCount = 5;
            lineRenderer.startWidth = 0.1f;
            lineRenderer.endWidth = 0.1f;

            Vector3 halfWidth = Vector3.right * (width / 2);
            Vector3 halfHeight = Vector3.up * (height / 2);

            lineRenderer.SetPositions(new Vector3[] {
                -halfWidth - halfHeight,
                -halfWidth + halfHeight,
                halfWidth + halfHeight,
                halfWidth - halfHeight,
                -halfWidth - halfHeight
            });
        }
#endif
    }
}