using UnityEngine;
using System.Collections.Generic;
using Assets.TrackGeneration;

public class TrackSensingSystem : MonoBehaviour
{
    public Transform raycastOrigin; // Position to cast rays from (car front)
    public float sensorLength = 20f; // Max length of the sensor rays
    public string trackWallTag = "Wall"; // Tag for track walls
    public bool visualizeRays = true; // Debug visualization toggle

    [Range(3, 36)]
    public int rayCount = 9; // Number of rays to cast
    public float rayAngle = 180f; // Total angle covered by rays (180 = semicircle in front)
    private float[] rayDistances; // Normalized distances to walls (1 = no obstacle, 0 = obstacle at sensor position)

    // NEW: Performance optimizations
    [Header("Performance")]
    public int raycastFrameSkip = 1; // Skip frames for raycast (0 = cast every frame)
    public LayerMask raycastLayers; // Specific layers to raycast against
    public bool enableLodSystem = true; // Enable/disable LOD for raycasts

    // NEW: LOD system
    private int frameCounter = 0;
    private bool isVisible = true;
    private Camera mainCamera;

    // Wall detection thresholds
    public float collisionWarningThreshold = 0.2f; // Threshold to detect imminent collisions

    // Add a reference to the car's rigidbody to get velocity information
    private Rigidbody carRigidbody;
    private CarAgent carAgent;

    void Awake()
    {
        // Initialize arrays for sensor readings
        rayDistances = new float[rayCount];

        // Get the car's rigidbody
        carRigidbody = GetComponent<Rigidbody>();
        if (carRigidbody == null)
            carRigidbody = GetComponentInParent<Rigidbody>();

        // Get reference to the CarAgent
        carAgent = GetComponent<CarAgent>();
        if (carAgent == null)
            carAgent = GetComponentInParent<CarAgent>();

        // Default ray origin to this transform if not set
        if (raycastOrigin == null)
            raycastOrigin = transform;

        // Find main camera
        mainCamera = Camera.main;

        // Initialize rayDistances with maximum values
        for (int i = 0; i < rayDistances.Length; i++)
        {
            rayDistances[i] = 1.0f;
        }

        // Set up raycast layers if not set
        if (raycastLayers.value == 0)
        {
            raycastLayers = Physics.DefaultRaycastLayers;
        }
    }

    void FixedUpdate()
    {
        // Increment frame counter
        frameCounter++;

        // NEW: Check if we should skip this frame
        if (raycastFrameSkip > 0 && frameCounter % (raycastFrameSkip + 1) != 0)
        {
            return;
        }

        // NEW: If LOD is enabled, check if we're visible before casting rays
        if (enableLodSystem && mainCamera != null)
        {
            // Simple LOD - only cast rays if we're in camera view or very close
            isVisible = IsVisibleToCamera() || IsCloseToCamera(50f);

            if (!isVisible)
            {
                return; // Skip raycasting when not visible
            }
        }

        // Perform sensing
        CastForwardRays();
    }

    // NEW: Check if the agent is visible to the camera
    private bool IsVisibleToCamera()
    {
        if (mainCamera == null) return true;

        Vector3 screenPoint = mainCamera.WorldToViewportPoint(transform.position);
        return screenPoint.z > 0 && screenPoint.x > -0.1f && screenPoint.x < 1.1f &&
               screenPoint.y > -0.1f && screenPoint.y < 1.1f;
    }

    // NEW: Check if the agent is close to the camera
    private bool IsCloseToCamera(float maxDistance)
    {
        if (mainCamera == null) return true;

        return Vector3.Distance(transform.position, mainCamera.transform.position) < maxDistance;
    }

    void CastForwardRays()
    {
        if (rayCount < 1) return;

        // Calculate the angle between rays
        float angleStep = rayAngle / (rayCount - 1);
        float startAngle = -rayAngle / 2f;

        // NEW: Pre-calculate raycast data for better performance
        Vector3 originPosition = raycastOrigin.position;
        Vector3 forwardDirection = raycastOrigin.forward;

        // Cast rays in a fan pattern
        for (int i = 0; i < rayCount; i++)
        {
            float currentAngle = startAngle + angleStep * i;
            // This creates a fan pattern from left to front to right
            Vector3 direction = Quaternion.Euler(0, currentAngle, 0) * forwardDirection;

            RaycastHit hit;
            if (Physics.Raycast(originPosition, direction, out hit, sensorLength, raycastLayers))
            {
                // Check if we hit something with the track wall tag
                if (hit.collider.CompareTag(trackWallTag))
                {
                    // Normalize the distance (1 = max distance, 0 = collision)
                    rayDistances[i] = hit.distance / sensorLength;

                    // Visualize the ray hit
                    if (visualizeRays)
                        Debug.DrawLine(originPosition, hit.point, Color.red);
                }
                else
                {
                    // Hit something that's not a track wall
                    rayDistances[i] = 1.0f;

                    if (visualizeRays)
                        Debug.DrawLine(originPosition, hit.point, Color.yellow);
                }
            }
            else
            {
                // No obstacle detected within range
                rayDistances[i] = 1.0f;

                // Visualize the full ray
                if (visualizeRays)
                    Debug.DrawRay(originPosition, direction * sensorLength, Color.green);
            }
        }
    }


    // Get the normalized ray distances (accessible to the AI agent)
    public float[] GetRayDistances()
    {
        return rayDistances;
    }


    // Get the combined sensor data as one array
    public float[] GetAllSensorData()
    {
        float[] combined = new float[rayDistances.Length];
        rayDistances.CopyTo(combined, 0);
        return combined;
    }

    // Helper method to check if we're about to collide with a wall
    public bool IsWallAhead()
    {
        // Check frontal rays for imminent collision
        for (int i = 0; i < rayCount; i++)
        {
            if (rayDistances[i] < collisionWarningThreshold)
                return true;
        }
        return false;
    }

    // Calculate the best direction to move based on sensor readings
    public float GetSuggestedSteeringDirection()
    {
        if (rayCount < 3) return 0f;

        // Find the ray with the largest distance value (most open space)
        float maxDistance = 0f;
        int bestRayIndex = rayCount / 2; // Default to center ray

        for (int i = 0; i < rayCount; i++)
        {
            if (rayDistances[i] > maxDistance)
            {
                maxDistance = rayDistances[i];
                bestRayIndex = i;
            }
        }

        // Convert ray index to a steering direction from -1 to 1
        float normalizedDirection = (bestRayIndex - (rayCount - 1) / 2f) / ((rayCount - 1) / 2f);
        return Mathf.Clamp(normalizedDirection, -1f, 1f);
    }

    // Additional helper methods for the AI
    public float GetAverageWallDistance()
    {
        float sum = 0f;
        for (int i = 0; i < rayDistances.Length; i++)
        {
            sum += rayDistances[i];
        }
        return sum / rayDistances.Length;
    }

    // Improved collision detection to better handle wall collisions
    private void OnCollisionEnter(Collision collision)
    {
        // Skip if carAgent is null
        if (carAgent == null)
            return;

        // Check if this is a wall collision using tag
        if (collision.gameObject.CompareTag(trackWallTag))
        {
            // Calculate collision intensity based on relative velocity
            float collisionIntensity = 0f;

            if (carRigidbody != null)
            {
                Vector3 relativeVelocity = collision.relativeVelocity;
                collisionIntensity = relativeVelocity.magnitude;

                // Only report significant collisions (reduces false positives from light touches)
                if (collisionIntensity > 1.0f)
                {
                    // Notify the agent of the collision
                    carAgent.ReportCollision();

                    // For debugging
                    if (visualizeRays)
                    {
                        Debug.DrawRay(collision.contacts[0].point, collision.contacts[0].normal * 3, Color.red, 1.0f);
                    }
                }
            }
            else
            {
                // Fallback if no rigidbody - always report the collision
                carAgent.ReportCollision();
            }
        }
    }

    // NEW: Helper to visualize when debugging
    private void OnDrawGizmos()
    {
        if (Application.isPlaying && visualizeRays && carAgent != null)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(transform.position, 1.0f);

            // Show checkpoints if available
            if (carAgent.GetType().GetField("nextCheckpointIndex") != null &&
                carAgent.GetType().GetField("checkpoints") != null)
            {
                int nextCheckpointIndex = (int)carAgent.GetType().GetField("nextCheckpointIndex").GetValue(carAgent);
                List<Checkpoint> checkpoints = carAgent.GetType().GetField("checkpoints").GetValue(carAgent) as List<Checkpoint>;

                if (checkpoints != null && checkpoints.Count > 0 && nextCheckpointIndex < checkpoints.Count)
                {
                    Gizmos.color = Color.green;
                    Vector3 checkpointPos = checkpoints[nextCheckpointIndex].transform.position;
                    Gizmos.DrawLine(transform.position, checkpointPos);
                    Gizmos.DrawWireSphere(checkpointPos, 2.0f);
                }
            }
        }
    }
}