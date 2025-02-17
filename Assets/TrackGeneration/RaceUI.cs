using UnityEngine;
using TMPro;
using UnityEngine.UI;

namespace Assets.TrackGeneration
{
    public class RaceUIManager : MonoBehaviour
    {
        [Header("UI Text References")]
        public TextMeshProUGUI currentTimeText;
        public TextMeshProUGUI currentLapText;
        public TextMeshProUGUI bestLapText;
        public TextMeshProUGUI lapCountText;
        public TextMeshProUGUI checkpointText;
        public TextMeshProUGUI speedText;

        private RaceManager raceManager;
        private GameObject playerCar;

        public void Initialize(RaceManager raceManagerRef, GameObject playerCarRef)
        {
            raceManager = raceManagerRef;
            playerCar = playerCarRef;

            if (raceManager == null)
            {
                Debug.LogError("RaceManager reference is null in Initialize!");
                return;
            }

            if (playerCar == null)
            {
                Debug.LogError("PlayerCar reference is null in Initialize!");
                return;
            }

            // Subscribe to race events
            raceManager.OnRaceStart += HandleRaceStart;
            raceManager.OnPlayerFinish += HandlePlayerFinish;
            raceManager.OnCheckpointPassed += HandleCheckpointPassed;
            raceManager.OnLapCompleted += HandleLapCompleted;

            //Debug.Log("RaceUIManager initialized successfully with RaceManager and PlayerCar references");
        }
        void Start()
        {
            //raceManager = GetComponent<RaceManager>();
            if (raceManager != null)
            {
                // Subscribe to race events
                raceManager.OnRaceStart += HandleRaceStart;
                raceManager.OnPlayerFinish += HandlePlayerFinish;
                raceManager.OnCheckpointPassed += HandleCheckpointPassed;
                raceManager.OnLapCompleted += HandleLapCompleted;
            }

            // Find player car
            //playerCar = GameObject.FindGameObjectWithTag("Player");
        }

        void Update()
        {
            if (raceManager != null && playerCar != null)
            {
                UpdateUI();
            }
        }

        private void UpdateUI()
        {
            // Format time as minutes:seconds.milliseconds
            float currentTime = raceManager.GetPlayerCurrentLapTime(playerCar);
            float bestLap = raceManager.GetPlayerBestLapTime(playerCar);

            currentTimeText.text = $"Current Time: {FormatTime(currentTime)}";
            bestLapText.text = $"Best Lap: {(bestLap == float.MaxValue ? "--:--:---" : FormatTime(bestLap))}";

            int currentLap = raceManager.GetPlayerCurrentLap(playerCar);
            int totalLaps = raceManager.totalLaps;
            lapCountText.text = $"Lap: {currentLap + 1}/{totalLaps}";

            int nextCheckpoint = raceManager.GetPlayerNextCheckpoint(playerCar);
            int totalCheckpoints = raceManager.GetTotalCheckpoints();
            checkpointText.text = $"Checkpoint: {nextCheckpoint + 1}/{totalCheckpoints}";

            // Update speed display if the car has a Rigidbody
            Rigidbody rb = playerCar.GetComponent<Rigidbody>();
            if (rb != null)
            {
                float speedKmH = rb.linearVelocity.magnitude * 3.6f; // Convert m/s to km/h
                speedText.text = $"Speed: {speedKmH:F0} km/h";
            }
        }

        private string FormatTime(float timeInSeconds)
        {
            int minutes = (int)(timeInSeconds / 60);
            int seconds = (int)(timeInSeconds % 60);
            int milliseconds = (int)((timeInSeconds * 1000) % 1000);
            return $"{minutes:00}:{seconds:00}.{milliseconds:000}";
        }

        private void HandleRaceStart()
        {
            currentTimeText.text = "Current Time: 00:00.000";
            bestLapText.text = "Best Lap: --:--:---";
            lapCountText.text = $"Lap: 1/{raceManager.totalLaps}";
            checkpointText.text = "Checkpoint: 1/" + raceManager.GetTotalCheckpoints();
            speedText.text = "Speed: 0 km/h";
        }

        private void HandlePlayerFinish(GameObject player)
        {
            if (player == playerCar)
            {
                // Show race completion message
                currentTimeText.text = "Race Complete!";
                currentTimeText.color = Color.green;  // Change color to indicate finish

                // Display final race time
                float finalTime = raceManager.GetGlobalRaceTime();
                bestLapText.text = $"Final Time: {FormatTime(finalTime)}";
            }
        }

        private void HandleCheckpointPassed(GameObject player, int checkpointIndex, int totalCheckpoints)
        {
            if (player == playerCar)
            {
                // Update checkpoint counter
                checkpointText.text = $"Checkpoint: {checkpointIndex + 1}/{totalCheckpoints}";

                // Flash the checkpoint text briefly to indicate passing
                //StartCoroutine(FlashText(checkpointText));
            }
        }

        private void HandleLapCompleted(GameObject player)
        {
            if (player == playerCar)
            {
                int currentLap = raceManager.GetPlayerCurrentLap(playerCar);
                lapCountText.text = $"Lap: {currentLap + 1}/{raceManager.totalLaps}";

                // Flash the lap counter and display the last lap time
                //StartCoroutine(FlashText(lapCountText));

                // Update best lap if this was a better time
                float bestLap = raceManager.GetPlayerBestLapTime(playerCar);
                bestLapText.text = $"Best Lap: {FormatTime(bestLap)}";
            }
        }

        void OnDestroy()
        {
            if (raceManager != null)
            {
                // Unsubscribe from events
                raceManager.OnRaceStart -= HandleRaceStart;
                raceManager.OnPlayerFinish -= HandlePlayerFinish;
                raceManager.OnCheckpointPassed -= HandleCheckpointPassed;
                raceManager.OnLapCompleted -= HandleLapCompleted;
            }
        }
    }
}