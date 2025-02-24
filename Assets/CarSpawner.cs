using UnityEngine;
using Assets.TrackGeneration;
using UnityEditor.Rendering;

public class CarSpawner : MonoBehaviour
{
    [Header("Car Setup")]
    public GameObject carPrefab;
    public TrackHandler trackHandler;

    [Header("Camera Setup")]
    //public CameraControlScript cameraControlScript;
    public Vector3 cameraOffset = new Vector3(0, 3, -8);
    public Vector3 rearCameraOffset = new Vector3(0, 3, 8);
    public float cameraFollowSpeed = 6f;
    public float cameraRotationSpeed = 5f;
    public float cameraLookSensitivity = 100f;

    private GameObject playerCar;
    private Transform carBody;
    private Camera[] playerCameras = new Camera[2];

    void Start()
    {
        SpawnPlayerCar();
    }

    public void SpawnPlayerCar()
    {
        if (carPrefab == null || trackHandler == null)
        {
            Debug.LogError("Car prefab or track generator not assigned!");
            return;
        }

        ClearExisting();

        var (startPos, _, startRot) = trackHandler.GetTrackStartTransform();

        // Spawn the car
        playerCar = Instantiate(carPrefab, startPos, startRot);
        playerCar.tag = "Player";
        CarControlScript carControl = playerCar.GetComponent<CarControlScript>();
        if (carControl != null)
        {
            carControl.trackHandler = trackHandler;
        }
        else
        {
            Debug.LogError("CarControlScript not found on car prefab!");
        }

        carBody = playerCar.transform.Find("CarBody");
        if (carBody == null)
        {
            Debug.LogError("CarBody not found in car prefab!");
            return;
        }
        // Set up the camera
        SetupPlayerCamera();
    }

    private void SetupPlayerCamera()
    {

        GameObject cameraControlObj = new GameObject("CameraControl");
        CameraControlScript cameraControl = cameraControlObj.AddComponent<CameraControlScript>();
        // Create camera GameObject
        for (int i = 0; i < 2; i++)
        {
            GameObject cameraObj = new GameObject($"PlayerCamera{i}");
            Camera cam = cameraObj.AddComponent<Camera>();
            cameraObj.transform.SetParent(cameraControl.transform);
            playerCameras[i] = cam;

            // Add AudioListener to the camera
            cameraObj.AddComponent<AudioListener>();

            // Add FollowCameraScript
            FollowCameraScript followScript = cameraObj.AddComponent<FollowCameraScript>();
            followScript.carTransform = carBody;
            if (i == 0)
            {
                followScript.offset = cameraOffset;
                followScript.followSpeed = cameraFollowSpeed;
                followScript.rearView = true;

            }
            else 
            {
                followScript.offset = rearCameraOffset;
                cam.enabled = false;
                followScript.followSpeed = 10f;
                cameraObj.GetComponent<AudioListener>().enabled = false;
                followScript.rearView= false;

            }
            followScript.followSpeed = cameraFollowSpeed;
            followScript.rotationSpeed = cameraRotationSpeed;
            followScript.lookSensitivity = cameraLookSensitivity;

            // Position the camera initially
            cameraObj.transform.position = playerCar.transform.position + cameraOffset;
            cameraObj.transform.LookAt(playerCar.transform);
        }
        cameraControl.SetupCameras(playerCameras[0], playerCameras[1]);
    }

    private void ClearExisting()
    {
        if (playerCar != null)
        {
            Destroy(playerCar);
        }
        foreach (Camera playerCamera in playerCameras) 
        {
            if (playerCamera != null)
            {
                Destroy(playerCamera.gameObject);
            } 
        }
    }

    // Public methods to access car and camera if needed
    public GameObject GetPlayerCar()
    {
        return playerCar;
    }

    public Camera[] GetPlayerCameras()
    {
        return playerCameras;
    }
}