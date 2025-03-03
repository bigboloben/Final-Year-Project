using UnityEngine;
using System.Collections;
using TMPro;
using UnityEngine.UI;
using System.Collections.Generic;

namespace Assets.TrackGeneration
{
    public class SceneHandler : MonoBehaviour
    {
        TrackHandler trackHandler;
        GameObject playerCar;
        RaceManager raceSystem;


        [Header("Track Settings")]
        public Material trackMaterial;
        public Material wallMaterial;
        public PhysicsMaterial wallPhysicsMaterial;
        public GameObject startLinePrefab;
        public GameObject gridMarkerPrefab;
        public Material trackSupportMaterial;

        [Header("Floor Settings")]
        public GameObject floorPrefab;

        [Header("Track Parameters")]
        public float trackWidth = 12f;
        public float trackHeight = 0.1f;
        public float wallHeight = 0.5f;
        public float wallWidth = 0.5f;
        public float segments = 1f;
        public float banking = 15f;
        public int supportCount = 20;
        public int pointCount = 150;
        public int canvasSize = 1000;

        [Header("Camera Settings")]
        public float cameraHeight = 80f;

        [Header("Car Settings")]
        public List<GameObject> carPrefabs;
        public Vector3 cameraOffset = new Vector3(0, 3, -8);
        public Vector3 rearCameraOffset = new Vector3(0, 3, 8);
        public float cameraFollowSpeed = 6f;
        public float cameraRotationSpeed = 5f;
        public float cameraLookSensitivity = 100f;

        [Header("Race Settings")]
        public int totalLaps = 3;
        public float countdownTime = 3f;

        public bool setupCar = true;
        public bool setupTrack = true;
        public bool setupFloor = true;


        void Start()
        {
            StartCoroutine(InitializeScene());
        }

        private IEnumerator InitializeScene()
        {
            if (setupTrack) SetupTrackSystem();
            if (setupFloor) SetUpFloor();
            if (setupCar) SetUpCarSpawner();
            if (setupTrack && setupCar) SetUpRaceHandler();

            // Wait for player car to be ready
            while (playerCar == null)
            {
                yield return new WaitForSeconds(0.1f);
            }

            // Now that we have the player car, set up the UI
            if (setupTrack && setupCar) SetupRaceUI();
            //Debug.Log("Scene initialization complete!");
        }

        private void SetupTrackSystem()
        {
            // Create a new GameObject for the track system
            GameObject trackSystem = new GameObject("Track System");

            // Add TrackHandler component
            trackHandler = trackSystem.AddComponent<TrackHandler>();

            // Set up track materials
            trackHandler.trackMaterial = trackMaterial;
            trackHandler.wallMaterial = wallMaterial;
            trackHandler.wallPhysicsMaterial = wallPhysicsMaterial;
            trackHandler.startLinePrefab = startLinePrefab;
            trackHandler.gridMarkerPrefab = gridMarkerPrefab;
            trackHandler.trackSupportMaterial = trackSupportMaterial;

            // Set up track parameters
            trackHandler.trackWidth = trackWidth;
            trackHandler.trackHeight = trackHeight;
            trackHandler.wallHeight = wallHeight;
            trackHandler.wallWidth = wallWidth;
            trackHandler.segments = segments;
            trackHandler.banking = banking;
            trackHandler.supportCount = supportCount;
            trackHandler.PointCount = pointCount;
            trackHandler.CanvasSize = canvasSize;

            // Create and setup camera
            GameObject cameraObj = new GameObject("Track Camera");
            Camera trackCamera = cameraObj.AddComponent<Camera>();
            trackCamera.orthographic = true;
            trackCamera.orthographicSize = canvasSize / 2f;
            trackCamera.transform.position = Vector3.up * cameraHeight;
            trackCamera.transform.rotation = Quaternion.Euler(90f, 0f, 0f);

            // Assign camera to track handler
            trackHandler.trackCamera = trackCamera;

            //Debug.Log("Track system setup complete!");
        }

        private void SetUpFloor()
        {
            GameObject floorSystem = new GameObject("Floor System");
            FloorGenerator floorGenerator = floorSystem.AddComponent<FloorGenerator>();
            floorGenerator.floorPrefab = floorPrefab;
            floorGenerator.Initialize();
        }

        private void SetUpCarSpawner()
        {
            // Create car spawner
            GameObject carSpawnerObj = new GameObject("Car Spawner");
            CarSpawner carSpawner = carSpawnerObj.AddComponent<CarSpawner>();

            // Configure car spawner
            carSpawner.carPrefab = carPrefabs[Random.Range(0, carPrefabs.Count - 1)];
            //carSpawner.carPrefab = carPrefabs[6];
            carSpawner.trackHandler = trackHandler;
            carSpawner.cameraOffset = cameraOffset;
            carSpawner.rearCameraOffset = rearCameraOffset;
            carSpawner.cameraFollowSpeed = cameraFollowSpeed;
            carSpawner.cameraRotationSpeed = cameraRotationSpeed;
            carSpawner.cameraLookSensitivity = cameraLookSensitivity;

            //// Call SpawnPlayerCar directly
            //carSpawner.SpawnPlayerCar();

            // Wait a short moment then get the player car reference
            StartCoroutine(GetPlayerCarReference(carSpawner));
        }

        private IEnumerator GetPlayerCarReference(CarSpawner carSpawner)
        {
            // Wait a couple of frames to ensure car is spawned
            yield return new WaitForSeconds(0.1f);

            playerCar = carSpawner.GetPlayerCar();

            if (playerCar == null)
            {
                Debug.LogError("Failed to get player car reference from CarSpawner!");
            }
            else
            {
                //Debug.Log("Successfully got player car reference");
            }
        }
        private void SetUpRaceHandler()
        {
            // Create and configure the race manager
            GameObject raceManagerObj = new GameObject("Race Manager");
            raceSystem = raceManagerObj.AddComponent<RaceManager>();
            raceSystem.totalLaps = totalLaps;
            raceSystem.countdownTime = countdownTime;

            // Wait a frame to ensure track generation is complete
            StartCoroutine(InitializeRaceSystem());
        }

        private IEnumerator InitializeRaceSystem()
        {
            // Wait for a frame to ensure track and checkpoints are generated
            yield return new WaitForSeconds(0.1f);

            if (trackHandler == null)
            {
                Debug.LogError("TrackHandler is null!");
                yield break;
            }

            var checkpoints = trackHandler.GetCheckpoints();
            //Debug.Log($"Found {checkpoints.Count} checkpoints");

            if (checkpoints.Count == 0)
            {
                Debug.LogError("No checkpoints available!");
                yield break;
            }

            if (playerCar != null)
            {
                raceSystem.RegisterPlayer(playerCar, "Player 1");
                raceSystem.InitializeCheckpoints(checkpoints);
                raceSystem.InitiateRaceStart();
                //Debug.Log("Race system initialized successfully");
            }
            else
            {
                Debug.LogError("PlayerCar is null!");
            }
        }
        // Add to your SceneHandler class
        private void SetupRaceUI()
        {
            GameObject uiObject = new GameObject("UI");
            RaceUIManager uiManager = uiObject.AddComponent<RaceUIManager>();
            uiManager.Initialize(raceSystem, playerCar); // Pass both references

            // Create Canvas
            GameObject canvasObj = new GameObject("Canvas");
            canvasObj.transform.SetParent(uiObject.transform);
            Canvas canvas = canvasObj.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;

            // Add required canvas components
            canvasObj.AddComponent<CanvasScaler>();
            canvasObj.AddComponent<GraphicRaycaster>();

            // Create a panel for race info
            GameObject panelObj = new GameObject("Race Info Panel");
            panelObj.transform.SetParent(canvasObj.transform, false);



            // Add panel components
            RectTransform panelRect = panelObj.AddComponent<RectTransform>();
            Image panelImage = panelObj.AddComponent<Image>();
            panelImage.color = new Color(0, 0, 0, 0.5f); // Semi-transparent black background

            // Position panel in top-right corner
            panelRect.anchorMin = new Vector2(1, 1);
            panelRect.anchorMax = new Vector2(1, 1);
            panelRect.pivot = new Vector2(1, 1);
            panelRect.sizeDelta = new Vector2(200, 200); // Width and height of panel
            panelRect.anchoredPosition = new Vector2(-20, -20); // Offset from corner

            // Create text elements
            float yOffset = -10f; // Starting Y position
            float spacing = 30f;  // Spacing between elements

            // Current Time
            uiManager.currentTimeText = CreateTextElement(panelObj, "Current Time: 00:00.000",
                new Vector2(-10, yOffset));
            yOffset -= spacing;

            // Current Lap
            uiManager.currentLapText = CreateTextElement(panelObj, "Lap: 1/3",
                new Vector2(-10, yOffset));
            yOffset -= spacing;

            // Best Lap
            uiManager.bestLapText = CreateTextElement(panelObj, "Best Lap: --:--:---",
                new Vector2(-10, yOffset));
            yOffset -= spacing;

            // Lap Count
            uiManager.lapCountText = CreateTextElement(panelObj, "Checkpoint: 1/10",
                new Vector2(-10, yOffset));
            yOffset -= spacing;

            // Checkpoint
            uiManager.checkpointText = CreateTextElement(panelObj, "Checkpoint: 1/10",
                new Vector2(-10, yOffset));
            yOffset -= spacing;

            // Speed
            uiManager.speedText = CreateTextElement(panelObj, "Speed: 0 km/h",
                new Vector2(-10, yOffset));
        }

        private TextMeshProUGUI CreateTextElement(GameObject parent, string defaultText, Vector2 position)
        {
            GameObject textObj = new GameObject("Text");
            textObj.transform.SetParent(parent.transform, false);

            // Add TextMeshPro component
            TextMeshProUGUI tmp = textObj.AddComponent<TextMeshProUGUI>();

            // Configure the text component
            tmp.text = defaultText;
            tmp.fontSize = 16;
            tmp.color = Color.white;
            tmp.alignment = TextAlignmentOptions.Right;
            tmp.font = TMP_Settings.defaultFontAsset;

            // Set up RectTransform
            RectTransform rect = tmp.GetComponent<RectTransform>();
            rect.anchorMin = new Vector2(1, 1);
            rect.anchorMax = new Vector2(1, 1);
            rect.pivot = new Vector2(1, 1);
            rect.sizeDelta = new Vector2(180, 25); // Width and height of text
            rect.anchoredPosition = position;

            return tmp;
        }

        private void CreateTextElement(GameObject parent, string name, RaceUIManager uiManager)
        {
            GameObject textObj = new GameObject(name);
            textObj.transform.SetParent(parent.transform, false);
            TextMeshProUGUI tmp = textObj.AddComponent<TextMeshProUGUI>();
            tmp.fontSize = 16;
            tmp.color = Color.white;
            tmp.alignment = TextAlignmentOptions.Left;

            // Assign to corresponding field in RaceUIManager using reflection
            var field = typeof(RaceUIManager).GetField(name.ToLower());
            if (field != null)
                field.SetValue(uiManager, tmp);
        }
    }
}