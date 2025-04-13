using UnityEngine;
using TMPro;
using System.Collections;

namespace Assets.TrackGeneration
{
    public class RaceUIManager : MonoBehaviour
    {
        [Header("UI Text References")]
        [Tooltip("Displays the player's current lap time.")]
        public TextMeshProUGUI currentLapTimeText;

        [Tooltip("Displays the best lap time achieved.")]
        public TextMeshProUGUI bestLapText;

        [Tooltip("Displays the current lap number and total laps.")]
        public TextMeshProUGUI lapCountText;

        [Tooltip("Displays the overall race time.")]
        public TextMeshProUGUI raceTimeText;

        [Tooltip("Displays a list of completed lap times.")]
        public TextMeshProUGUI lapResultsText;

        [Tooltip("Displays the countdown before the race starts.")]
        public TextMeshProUGUI countdownText;

        private RaceManager raceManager;
        private GameObject playerCar;

        /// <summary>
        /// Initializes the RaceUIManager with the RaceManager and player's car reference.
        /// This method can be called by another manager or during setup.
        /// </summary>
        public void Initialize(RaceManager raceManagerRef, GameObject playerCarRef)
        {
            raceManager = raceManagerRef;
            playerCar = playerCarRef;

            if (raceManager == null)
            {
                Debug.LogError("RaceManager reference is null in RaceUIManager.Initialize!");
                return;
            }
            if (playerCar == null)
            {
                Debug.LogError("PlayerCar reference is null in RaceUIManager.Initialize!");
                return;
            }

            // Subscribe to RaceManager events
            raceManager.OnRaceStart += HandleRaceStart;
            raceManager.OnPlayerFinish += HandlePlayerFinish;
            raceManager.OnLapCompleted += HandleLapCompleted;

            // Subscribe to countdown tick event
            raceManager.OnCountdownTick += HandleCountdownTick;
        }


        void Start()
        {
            // Auto-assign references if they haven't been assigned by an external initializer.
            if (raceManager == null)
            {
                raceManager = GetComponent<RaceManager>();
                if (raceManager != null)
                {
                    raceManager.OnRaceStart += HandleRaceStart;
                    raceManager.OnPlayerFinish += HandlePlayerFinish;
                    raceManager.OnLapCompleted += HandleLapCompleted;
                }
            }
            if (playerCar == null)
            {
                playerCar = GameObject.FindGameObjectWithTag("Player");
            }
        }

        void Update()
        {
            if (raceManager != null && playerCar != null)
            {
                UpdateUI();
            }
        }

        /// <summary>
        /// Updates the UI elements each frame.
        /// </summary>
        private void UpdateUI()
        {
            // Update current lap time display
            float currentLapTime = raceManager.GetPlayerCurrentLapTime(playerCar);
            currentLapTimeText.text = "Current Lap: " + FormatTime(currentLapTime);

            // Update best lap time display (if a lap has been recorded, otherwise show placeholder)
            float bestLap = raceManager.GetPlayerBestLapTime(playerCar);
            bestLapText.text = "Best Lap: " + (bestLap == float.MaxValue ? "--:--:---" : FormatTime(bestLap));

            // Update lap count display (using 1-based counting for the UI)
            int lapIndex = raceManager.GetPlayerCurrentLap(playerCar);
            lapCountText.text = "Lap: " + (lapIndex + 1) + "/" + raceManager.totalLaps;

            // Update overall race time display from the global race timer
            raceTimeText.text = "Race Time: " + FormatTime(raceManager.GetGlobalRaceTime());
        }

        /// <summary>
        /// Formats the time (in seconds) into mm:ss.mmm string.
        /// </summary>
        /// <param name="timeInSeconds">Time value in seconds.</param>
        /// <returns>Formatted string representation.</returns>
        private string FormatTime(float timeInSeconds)
        {
            int minutes = (int)(timeInSeconds / 60);
            int seconds = (int)(timeInSeconds % 60);
            int milliseconds = (int)((timeInSeconds * 1000) % 1000);
            return $"{minutes:00}:{seconds:00}.{milliseconds:000}";
        }

        /// <summary>
        /// Called when the race starts. Resets UI elements.
        /// </summary>
        private void HandleRaceStart()
        {
            currentLapTimeText.text = "Current Lap: 00:00.000";
            bestLapText.text = "Best Lap: --:--:---";
            lapCountText.text = "Lap: 1/" + raceManager.totalLaps;
            raceTimeText.text = "Race Time: 00:00.000";
            lapResultsText.text = "";  // Clear any previous lap results.
        }

        private void HandleCountdownTick(float timeRemaining)
        {
            if (countdownText != null)
            {
                if (timeRemaining > 0)
                    countdownText.text = Mathf.Ceil(timeRemaining).ToString();
                else
                    countdownText.text = "GO!";
            }
        }

        /// <summary>
        /// Called each time a lap is completed.
        /// Appends the most recent lap time to the lap results display.
        /// </summary>
        private void HandleLapCompleted(GameObject player)
        {
            if (player == playerCar)
            {
                // Retrieve the player's completed lap times. (See note above on adding this getter.)
                var lapTimes = raceManager.GetPlayerLapTimes(playerCar);
                if (lapTimes != null && lapTimes.Count > 0)
                {
                    // Get the last lap time (newly recorded lap).
                    float lastLapTime = lapTimes[lapTimes.Count - 1];
                    int completedLapNumber = lapTimes.Count;  // Each recorded lap is a completed lap.
                    lapResultsText.text += $"Lap {completedLapNumber}: {FormatTime(lastLapTime)}\n";
                }

                // Update the lap count display in case it has advanced.
                int lapIndex = raceManager.GetPlayerCurrentLap(playerCar);
                lapCountText.text = "Lap: " + (lapIndex + 1) + "/" + raceManager.totalLaps;
            }
        }

        /// <summary>
        /// Called when the player's race is complete.
        /// Updates the overall race time to show the final time.
        /// </summary>
        private void HandlePlayerFinish(GameObject player)
        {
            if (player == playerCar)
            {
                raceTimeText.text = "Race Complete! Final Time: " + FormatTime(raceManager.GetGlobalRaceTime());
            }
        }

        void OnDestroy()
        {
            // Unsubscribe from events when this UI manager is destroyed.
            if (raceManager != null)
            {
                raceManager.OnRaceStart -= HandleRaceStart;
                raceManager.OnPlayerFinish -= HandlePlayerFinish;
                raceManager.OnLapCompleted -= HandleLapCompleted;
            }
        }
    }
}
