using UnityEngine;
using System.Collections.Generic;

namespace Assets.TrackGeneration
{
    public class Checkpoint : MonoBehaviour
    {
        private RaceManager raceManager;

        // Layer name for checkpoints
        public const string CheckpointLayerName = "Ignore Raycast";
        public const int CheckpointLayer = 2;

        public void Initialize(RaceManager manager)
        {
            this.raceManager = manager;

            // Set this object to the Checkpoint layer
            gameObject.layer = CheckpointLayer;
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

            // Create Checkpoint layer if it doesn't exist
            CreateCheckpointLayerIfNeeded();

            List<Checkpoint> checkpoints = new List<Checkpoint>();
            GameObject checkpointsContainer = new GameObject("Checkpoints");
            checkpointsContainer.transform.SetParent(trackObject.transform);

            // Set the container to the Checkpoint layer as well
            checkpointsContainer.layer = Checkpoint.CheckpointLayer;

            // Create checkpoint 0 at the start/finish line (index 0)
            GameObject startFinishCheckpoint = CreateCheckpointTrigger(leftPoints[0], rightPoints[0], checkpointHeight);
            startFinishCheckpoint.name = "Checkpoint_0_StartFinish";
            startFinishCheckpoint.transform.SetParent(checkpointsContainer.transform);
            startFinishCheckpoint.layer = Checkpoint.CheckpointLayer;
            Checkpoint checkpoint0 = startFinishCheckpoint.AddComponent<Checkpoint>();
            checkpoints.Add(checkpoint0);

            // Calculate the remaining checkpoints between point 1 and the last point
            // Ensure we don't overlap with the start/finish
            if (numberOfCheckpoints > 1)
            {
                // We want to distribute the remaining checkpoints between index 1 and index (length-1)
                // We're placing (numberOfCheckpoints-1) checkpoints in this range
                float trackLength = leftPoints.Length - 1;
                float spacing = trackLength / numberOfCheckpoints;

                // Start from 1 since we already placed checkpoint 0
                for (int i = 1; i < numberOfCheckpoints; i++)
                {
                    // Calculate position along the track, ensuring we don't place a checkpoint at position 0
                    // This formula ensures checkpoints are evenly distributed between position 1 and position (length-1)
                    float position = i * spacing;
                    int pointIndex = Mathf.RoundToInt(position);

                    // Make sure we don't exceed array bounds
                    pointIndex = Mathf.Clamp(pointIndex, 1, leftPoints.Length - 1);

                    Vector3 leftPoint = leftPoints[pointIndex];
                    Vector3 rightPoint = rightPoints[pointIndex];

                    GameObject checkpointObject = CreateCheckpointTrigger(leftPoint, rightPoint, checkpointHeight);
                    checkpointObject.name = $"Checkpoint_{i}";
                    checkpointObject.transform.SetParent(checkpointsContainer.transform);
                    checkpointObject.layer = Checkpoint.CheckpointLayer;

                    Checkpoint checkpointComponent = checkpointObject.AddComponent<Checkpoint>();
                    checkpoints.Add(checkpointComponent);
                }
            }

            // Initialize checkpoints in RaceManager
            if (RaceManager.Instance != null)
            {
                RaceManager.Instance.InitializeCheckpoints(checkpoints);
            }
            else
            {
                Debug.LogError("RaceManager.Instance is null! Cannot initialize checkpoints.");
            }

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

            // Set the checkpoint to the Checkpoint layer
            checkpoint.layer = Checkpoint.CheckpointLayer;

            return checkpoint;
        }

        private static void CreateCheckpointLayerIfNeeded()
        {
            // This should be called from Editor script ideally, but we'll add a runtime check
            // Note: This doesn't actually create the layer at runtime, just logs a warning if it doesn't exist
            if (LayerMask.NameToLayer(Checkpoint.CheckpointLayerName) == -1)
            {
                Debug.LogWarning($"Checkpoint layer '{Checkpoint.CheckpointLayerName}' doesn't exist. Please create it in Project Settings > Tags and Layers.");
                Debug.LogWarning($"Setup layer #{Checkpoint.CheckpointLayer} as '{Checkpoint.CheckpointLayerName}' in your project settings.");
            }
        }
    }
}