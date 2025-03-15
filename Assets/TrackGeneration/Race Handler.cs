using UnityEngine;
using System.Collections.Generic;
using System;

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
                //Debug.Log($"Initialized RaceManager with {checkpoints.Count} checkpoints");

                // Auto-start race if we're in training mode with agents
                if (HasAgentsOnly() && skipCountdownForTraining)
                {
                    StartRace();
                }
            }
        }

        public void RegisterPlayer(GameObject playerCar, string playerName = "Player")
        {
            if (!playerStats.ContainsKey(playerCar))
            {
                playerStats.Add(playerCar, new PlayerRaceStats
                {
                    playerName = playerName,
                    currentLap = 0,
                    nextCheckpointIndex = 0,
                    isAgent = false
                });
            }
        }

        public void RegisterAgent(GameObject agentCar, string agentName = "Agent")
        {
            if (!playerStats.ContainsKey(agentCar))
            {
                playerStats.Add(agentCar, new PlayerRaceStats
                {
                    playerName = agentName,
                    currentLap = 0,
                    nextCheckpointIndex = 0,
                    isAgent = true
                });
            }
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

            // Start race automatically if it's not active but an agent passes a checkpoint
            if (!isRaceActive && playerStats[player].isAgent)
            {
                StartRace();
            }

            if (!isRaceActive) return;

            var stats = playerStats[player];
            int checkpointIndex = checkpoints.IndexOf(checkpoint);

            // Check if this is the expected checkpoint
            if (checkpointIndex == stats.nextCheckpointIndex)
            {
                // Checkpoint passed successfully
                OnCheckpointPassed?.Invoke(player, checkpointIndex, checkpoints.Count);

                // Update next expected checkpoint
                stats.nextCheckpointIndex = (checkpointIndex + 1) % checkpoints.Count;

                // Check for lap completion
                if (checkpointIndex == checkpoints.Count - 1)
                {
                    CompletePlayerLap(player);
                }
            }
            else
            {
                // Wrong checkpoint
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

            // Reset for next lap
            stats.currentLapTime = 0f;
            stats.currentLap++;
            stats.nextCheckpointIndex = 0;

            // Check if race is complete for this player
            if (stats.currentLap >= totalLaps)
            {
                stats.hasFinishedRace = true;
                stats.raceCompletionTime = raceTimer;
                OnPlayerFinish?.Invoke(player);

                if (AreAllPlayersFinished())
                {
                    OnAllPlayersFinish?.Invoke();
                }
            }
        }

        public void InitiateRaceStart()
        {
            // If we're in training mode with just agents, skip countdown
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

        private void StartRace()
        {
            isRaceActive = true;
            raceTimer = 0f;

            // Reset all stats when the race starts
            foreach (var stats in playerStats.Values)
            {
                ResetPlayerStats(stats);
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
        }

        public void EndRace()
        {
            isRaceActive = false;
        }

        // Getter methods for UI and other systems
        public float GetGlobalRaceTime() => raceTimer;
        public float GetPlayerCurrentLapTime(GameObject player) => playerStats.ContainsKey(player) ? playerStats[player].currentLapTime : 0f;
        public float GetPlayerBestLapTime(GameObject player) => playerStats.ContainsKey(player) ? playerStats[player].bestLapTime : 0f;
        public int GetPlayerCurrentLap(GameObject player) => playerStats.ContainsKey(player) ? playerStats[player].currentLap : 0;
        public int GetPlayerNextCheckpoint(GameObject player) => playerStats.ContainsKey(player) ? playerStats[player].nextCheckpointIndex : 0;
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
    }
}