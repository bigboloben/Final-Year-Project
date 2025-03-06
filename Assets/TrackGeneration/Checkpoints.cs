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
            if (other.CompareTag("Player") || other.CompareTag("AI"))
            {
                // Existing RaceManager code
                if (raceManager != null)
                {
                    raceManager.CheckpointTriggerEntered(other.gameObject, this);
                }
            }
        }
    }

    public static class CheckpointGenerator
    {
        public static List<Checkpoint> GenerateCheckpoints(
            Vector3[] leftPoints,
            Vector3[] rightPoints,
            GameObject trackObject,
            int numberOfCheckpoints = 50,
            float checkpointHeight = 5f)
        {
            if (leftPoints == null || rightPoints == null)
            {
                Debug.LogError("Left or right points array is null!");
                return new List<Checkpoint>();
            }

            if (leftPoints.Length != rightPoints.Length)
            {
                Debug.LogError("Left and right point arrays must have the same length!");
                return new List<Checkpoint>();
            }

            if (numberOfCheckpoints < 2)
            {
                Debug.LogError("Number of checkpoints must be at least 2!");
                return new List<Checkpoint>();
            }

            if (numberOfCheckpoints > leftPoints.Length)
            {
                Debug.LogWarning($"Requested {numberOfCheckpoints} checkpoints but only {leftPoints.Length} points available. Using maximum possible checkpoints.");
                numberOfCheckpoints = leftPoints.Length;
            }

            List<Checkpoint> checkpoints = new List<Checkpoint>();
            GameObject checkpointsContainer = new GameObject("Checkpoints");
            checkpointsContainer.transform.SetParent(trackObject.transform);

            float increment = (float)(leftPoints.Length - 1) / (numberOfCheckpoints - 1);

            for (int i = 0; i < numberOfCheckpoints; i++)
            {
                int currentIndex = Mathf.Min(Mathf.RoundToInt(i * increment), leftPoints.Length - 1);

                Vector3 leftPoint = leftPoints[currentIndex];
                Vector3 rightPoint = rightPoints[currentIndex];

                GameObject checkpointObject = CreateCheckpointTrigger(leftPoint, rightPoint, checkpointHeight);
                checkpointObject.transform.SetParent(checkpointsContainer.transform);

                Checkpoint checkpoint = checkpointObject.AddComponent<Checkpoint>();
                checkpoints.Add(checkpoint);
            }

            RaceManager.Instance.InitializeCheckpoints(checkpoints);

            return checkpoints;
        }

        private static GameObject CreateCheckpointTrigger(Vector3 leftPoint, Vector3 rightPoint, float height)
        {
            GameObject checkpoint = new GameObject("Checkpoint");

            // Calculate center position between the two points
            Vector3 center = (leftPoint + rightPoint) / 2f;
            checkpoint.transform.position = center;

            // Create and setup box collider
            BoxCollider triggerCollider = checkpoint.AddComponent<BoxCollider>();
            triggerCollider.isTrigger = true;

            // Calculate width (distance between points)
            float width = Vector3.Distance(leftPoint, rightPoint);

            // Set collider size
            triggerCollider.size = new Vector3(width, height, 0.5f);

            // Calculate rotation to face along the track
            Vector3 direction = (rightPoint - leftPoint).normalized;
            checkpoint.transform.rotation = Quaternion.LookRotation(Vector3.forward, Vector3.up);
            checkpoint.transform.right = direction;

            return checkpoint;
        }
    }
}
