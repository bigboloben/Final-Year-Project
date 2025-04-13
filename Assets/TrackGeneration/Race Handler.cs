using UnityEngine;
using System.Collections.Generic;
using System;
using System.IO;      // Needed for file IO operations
using System.Text;    // For StringBuilder

namespace Assets.TrackGeneration
{
    public class PlayerRaceStats
    {
        public string playerName;
        public int currentLap = 0;
        public int nextCheckpointIndex = 0;
        public float currentLapTime = 0f;
        public float bestLapTime = float.MaxValue;
        public List<float> lapTimes = new List<float>();
        public bool hasFinishedRace = false;
        public float raceCompletionTime = 0f;
        public bool isAgent = false; // Flag to identify ML agents vs human players
        public bool hasCompletedFullCheckpointCircuit = false; // Flag to track if player has gone through all checkpoints
    }

    public class RaceManager : MonoBehaviour
    {
        public static RaceManager Instance { get; private set; }

        [Header("Race Settings")]
        public int totalLaps = 3;
        public float countdownTime = 3f;
        public bool skipCountdownForTraining = true; // Skip countdown for ML training

        // Race state
        private bool isRaceActive = false;
        private float raceTimer = 0f;
        private float countdownTimer;
        private Dictionary<GameObject, PlayerRaceStats> playerStats = new Dictionary<GameObject, PlayerRaceStats>();

        // Checkpoint system
        private List<Checkpoint> checkpoints = new List<Checkpoint>();

        // Events
        public event Action<float> OnCountdownTick;
        public event Action OnRaceStart;
        public event Action<GameObject> OnPlayerFinish;
        public event Action OnAllPlayersFinish;
        public event Action<GameObject, int, int> OnCheckpointPassed;  // player, checkpoint index, total checkpoints
        public event Action<GameObject> OnLapCompleted;
        public event Action<GameObject, int, int> OnWrongCheckpoint;   // player, attempted index, expected index

        void Awake()
        {
            if (Instance == null)
            {
                Instance = this;
            }
            else
            {
                Destroy(gameObject);
            }
        }

        void Update()
        {
            if (!isRaceActive && countdownTimer > 0)
            {
                countdownTimer -= Time.deltaTime;
                OnCountdownTick?.Invoke(countdownTimer);

                if (countdownTimer <= 0)
                {
                    StartRace();
                }
            }

            if (isRaceActive)
            {
                raceTimer += Time.deltaTime;

                foreach (var stats in playerStats.Values)
                {
                    if (!stats.hasFinishedRace)
                    {
                        stats.currentLapTime += Time.deltaTime;
                    }
                }
            }
        }

        public void InitializeCheckpoints(List<Checkpoint> checkpoints)
        {
            this.checkpoints = checkpoints;
            if (checkpoints != null)
            {
                foreach (var checkpoint in checkpoints)
                {
                    if (checkpoint != null)
                    {
                        checkpoint.Initialize(this);
                    }
                }
                // Auto-start race if we're in training mode with agents
                if (HasAgentsOnly() && skipCountdownForTraining)
                {
                    StartRace();
                }
            }
        }

        /// <summary>
        /// Registers a player car and immediately disables its movement.
        /// </summary>
        public void RegisterPlayer(GameObject playerCar, string playerName = "Player")
        {
            if (!playerStats.ContainsKey(playerCar))
            {
                playerStats.Add(playerCar, new PlayerRaceStats
                {
                    playerName = playerName,
                    currentLap = 0,
                    nextCheckpointIndex = 0,
                    isAgent = false,
                    hasCompletedFullCheckpointCircuit = false
                });

                // Disable movement at initialization
                CarControlScript control = playerCar.GetComponent<CarControlScript>();
                if (control != null)
                {
                    control.canMove = false;
                }
            }
        }

        /// <summary>
        /// Registers an AI car and immediately disables its movement.
        /// </summary>
        public void RegisterAgent(GameObject agentCar, string agentName = "Agent")
        {
            if (!playerStats.ContainsKey(agentCar))
            {
                playerStats.Add(agentCar, new PlayerRaceStats
                {
                    playerName = agentName,
                    currentLap = 0,
                    nextCheckpointIndex = 0,
                    isAgent = true,
                    hasCompletedFullCheckpointCircuit = false
                });

                // Disable movement at initialization
                CarControlScript control = agentCar.GetComponent<CarControlScript>();
                if (control != null)
                {
                    control.canMove = false;
                }
            }
        }

        public bool HasPlayerCompletedFullCircuit(GameObject player)
        {
            if (!playerStats.ContainsKey(player)) return false;
            return playerStats[player].hasCompletedFullCheckpointCircuit;
        }

        // Get player's total checkpoints passed
        public int GetPlayerTotalCheckpointsPassed(GameObject player)
        {
            if (!playerStats.ContainsKey(player)) return 0;

            int checkpointsPerLap = checkpoints.Count;
            int completedLaps = playerStats[player].currentLap;
            int currentLapCheckpoints = playerStats[player].nextCheckpointIndex;

            return (completedLaps * checkpointsPerLap) + currentLapCheckpoints;
        }

        // Check if we only have ML agents registered (for training)
        private bool HasAgentsOnly()
        {
            if (playerStats.Count == 0) return false;

            foreach (var stats in playerStats.Values)
            {
                if (!stats.isAgent) return false;
            }
            return true;
        }

        public void CheckpointTriggerEntered(GameObject player, Checkpoint checkpoint)
        {
            if (!playerStats.ContainsKey(player)) return;

            // Auto-start race if inactive and an agent passes a checkpoint
            if (!isRaceActive && playerStats[player].isAgent)
            {
                StartRace();
            }

            if (!isRaceActive) return;

            var stats = playerStats[player];
            int checkpointIndex = checkpoints.IndexOf(checkpoint);

            // Always notify about checkpoint passing
            OnCheckpointPassed?.Invoke(player, checkpointIndex, checkpoints.Count);

            // Check if this is the expected checkpoint
            if (checkpointIndex == stats.nextCheckpointIndex)
            {
                // Update the next expected checkpoint
                stats.nextCheckpointIndex = (checkpointIndex + 1) % checkpoints.Count;

                // Check if a full lap was completed
                if (checkpointIndex == 0 && stats.hasCompletedFullCheckpointCircuit)
                {
                    CompletePlayerLap(player);
                    stats.hasCompletedFullCheckpointCircuit = false; // Reset for next lap
                }

                // If last checkpoint before checkpoint 0, mark as completed circuit
                if (checkpointIndex == checkpoints.Count - 1)
                {
                    stats.hasCompletedFullCheckpointCircuit = true;
                }
            }
            else
            {
                // Invoke wrong checkpoint event
                OnWrongCheckpoint?.Invoke(player, checkpointIndex, stats.nextCheckpointIndex);
            }
        }

        private void CompletePlayerLap(GameObject player)
        {
            var stats = playerStats[player];

            // Record lap time
            stats.lapTimes.Add(stats.currentLapTime);
            if (stats.currentLapTime < stats.bestLapTime)
            {
                stats.bestLapTime = stats.currentLapTime;
            }

            OnLapCompleted?.Invoke(player);

            // Reset lap timing and advance lap count
            stats.currentLapTime = 0f;
            stats.currentLap++;
            stats.nextCheckpointIndex = 1;

            // Check for race completion
            if (stats.currentLap >= totalLaps)
            {
                stats.hasFinishedRace = true;
                stats.raceCompletionTime = raceTimer;
                OnPlayerFinish?.Invoke(player);

                if (AreAllPlayersFinished())
                {
                    OnAllPlayersFinish?.Invoke();
                    WriteRaceResultsToFile(); // Output results after all players finish
                }
            }
        }

        /// <summary>
        /// Initiates the race start sequence.
        /// If not skipping countdown, resets the countdown timer.
        /// </summary>
        public void InitiateRaceStart()
        {
            if (HasAgentsOnly() && skipCountdownForTraining)
            {
                StartRace();
            }
            else
            {
                countdownTimer = countdownTime;
                foreach (var stats in playerStats.Values)
                {
                    ResetPlayerStats(stats);
                }
            }
        }

        /// <summary>
        /// Starts the race by resetting race state and enabling car movement.
        /// </summary>
        private void StartRace()
        {
            isRaceActive = true;
            raceTimer = 0f;

            // Reset stats when the race begins
            foreach (var stats in playerStats.Values)
            {
                ResetPlayerStats(stats);
            }

            // Enable movement for all registered cars by setting their canMove flag to true.
            foreach (GameObject car in playerStats.Keys)
            {
                CarControlScript control = car.GetComponent<CarControlScript>();
                if (control != null)
                {
                    control.canMove = true;
                }
            }

            OnRaceStart?.Invoke();
        }

        private bool AreAllPlayersFinished()
        {
            foreach (var stats in playerStats.Values)
            {
                if (!stats.hasFinishedRace) return false;
            }
            return true;
        }

        public void ResetAgent(GameObject agent)
        {
            if (playerStats.ContainsKey(agent))
            {
                ResetPlayerStats(playerStats[agent]);
            }
        }

        private void ResetPlayerStats(PlayerRaceStats stats)
        {
            stats.currentLap = 0;
            stats.nextCheckpointIndex = 0;
            stats.currentLapTime = 0f;
            stats.bestLapTime = float.MaxValue;
            stats.lapTimes.Clear();
            stats.hasFinishedRace = false;
            stats.raceCompletionTime = 0f;
            stats.hasCompletedFullCheckpointCircuit = false;
        }

        public void EndRace()
        {
            isRaceActive = false;
            WriteRaceResultsToFile(); // Optionally call here if ending race manually
        }

        // Getter methods
        public float GetGlobalRaceTime() => raceTimer;
        public float GetPlayerCurrentLapTime(GameObject player) => playerStats.ContainsKey(player) ? playerStats[player].currentLapTime : 0f;
        public float GetPlayerBestLapTime(GameObject player) => playerStats.ContainsKey(player) ? playerStats[player].bestLapTime : 0f;
        public int GetPlayerCurrentLap(GameObject player) => playerStats.ContainsKey(player) ? playerStats[player].currentLap : 0;
        public int GetPlayerNextCheckpoint(GameObject player) => playerStats.ContainsKey(player) ? playerStats[player].nextCheckpointIndex : 0;
        public List<float> GetPlayerLapTimes(GameObject player) => playerStats.ContainsKey(player) ? playerStats[player].lapTimes : new List<float>();

        public bool IsRaceActive() => isRaceActive;
        public float GetCountdownTime() => countdownTimer;
        public Vector3 GetNextCheckpointPosition(GameObject player)
        {
            if (!playerStats.ContainsKey(player) || checkpoints.Count == 0) return Vector3.zero;
            int nextIdx = playerStats[player].nextCheckpointIndex;
            if (nextIdx < 0 || nextIdx >= checkpoints.Count) return Vector3.zero;
            return checkpoints[nextIdx].transform.position;
        }
        public int GetTotalCheckpoints() => checkpoints.Count;

        /// <summary>
        /// Helper method to format a time value (in seconds) as a string (mm:ss.mmm).
        /// </summary>
        private string FormatTime(float timeInSeconds)
        {
            int minutes = (int)(timeInSeconds / 60);
            int seconds = (int)(timeInSeconds % 60);
            int milliseconds = (int)((timeInSeconds * 1000) % 1000);
            return $"{minutes:00}:{seconds:00}.{milliseconds:000}";
        }

        /// <summary>
        /// Writes the race results (lap times and full race times) to a text file.
        /// </summary>
        private void WriteRaceResultsToFile()
        {
            StringBuilder sb = new StringBuilder();
            sb.AppendLine("Race Results:");
            sb.AppendLine($"Overall Race Time: {FormatTime(raceTimer)}");
            sb.AppendLine();

            foreach (var kvp in playerStats)
            {
                PlayerRaceStats stats = kvp.Value;
                sb.AppendLine($"Player: {stats.playerName}");
                sb.AppendLine($"Full Race Time: {(stats.hasFinishedRace ? FormatTime(stats.raceCompletionTime) : "Race Incomplete")}");
                sb.AppendLine("Lap Times:");
                if (stats.lapTimes.Count > 0)
                {
                    for (int i = 0; i < stats.lapTimes.Count; i++)
                    {
                        sb.AppendLine($"  Lap {i + 1}: {FormatTime(stats.lapTimes[i])}");
                    }
                }
                else
                {
                    sb.AppendLine("  No lap times recorded.");
                }
                sb.AppendLine(); // extra newline for clarity
            }

            string filePath = Path.Combine(Application.persistentDataPath, "RaceResults.txt");
            File.WriteAllText(filePath, sb.ToString());
            Debug.Log($"Race results written to: {filePath}");
        }
    }
}
