using UnityEngine;
using System.Collections.Generic;

public class TrackSensingSystem : MonoBehaviour
{
    public Transform raycastOrigin; // Position to cast rays from (car front)
    private float sensorLength = 25f; // Max length of the sensor rays
    private float sideMultiplier = 0.8f; // Multiplier for side rays
    private float verticalMultiplier = 0.6f; // Multiplier for vertical rays
    private string trackWallTag = "Wall"; // Tag for track walls
    public bool visualizeRays = true; // Debug visualization toggle

    [Range(3, 36)]
    private int rayCount = 5; // Number of horizontal rays to cast
    private float rayAngle = 180f; // Total angle covered by rays (180 = semicircle in front)
    private float verticalRayAngle = 5f; // Angle for up/down rays - fixed at 5 degrees
    private bool useVerticalRays = true; // Toggle for vertical rays

    public int totalRayCount; // Total number of rays (horizontal + vertical)
    private float[] rayDistances; // Normalized distances to walls
    private float[] previousRayDistances; // Previous frame's distances for calculating velocity
    private float[] rayVelocities; // Velocity of ray distances (rate of change)
    private Vector3[] rayOrigins; // Array to store ray origin positions
    private Vector3[] rayDirections; // Array to store ray directions
    private LayerMask raycastLayerMask; // Layer mask for raycasting

    // Add a reference to the car's rigidbody to get velocity information
    private Rigidbody carRigidbody;
    private CarAgent carAgent;

    void Awake()
    { 
        // Calculate total ray count
        totalRayCount = rayCount + (useVerticalRays ? 6 : 0);

        // Initialize arrays for sensor readings with proper size
        rayDistances = new float[totalRayCount];
        previousRayDistances = new float[totalRayCount];
        rayVelocities = new float[totalRayCount];
        rayOrigins = new Vector3[totalRayCount];
        rayDirections = new Vector3[totalRayCount];

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

        // Initialize previous distances with maximum values
        for (int i = 0; i < totalRayCount; i++)
        {
            previousRayDistances[i] = 1.0f;
            rayVelocities[i] = 0.0f;
        }

        SetupRaycastLayerMask();
    }

    private void SetupRaycastLayerMask()
    {
        // Get the Checkpoint layer number (should match what's defined in Checkpoint class)
        int checkpointLayer = 2; // Default to layer 10

        // Check if we can get the layer by name (more robust)
        int namedLayer = LayerMask.NameToLayer("Ignore Raycast");
        if (namedLayer != -1)
        {
            checkpointLayer = namedLayer;
        }

        // Create a layer mask that includes everything
        raycastLayerMask = ~0;

        // Remove the Checkpoint layer from the mask (~ is bitwise NOT)
        raycastLayerMask &= ~(1 << checkpointLayer);

        Debug.Log($"Raycast layer mask setup to ignore layer {checkpointLayer} (Checkpoint/Floor)");
    }

    void FixedUpdate()
    {
        // Store previous ray distances before updating
        System.Array.Copy(rayDistances, previousRayDistances, totalRayCount);

        // Perform sensing
        CastForwardRays();

        // Calculate ray velocities (change in distance per second)
        CalculateRayVelocities();
    }


    void CastForwardRays()
    {
        // Calculate the angle between rays
        float angleStep = rayAngle / (rayCount - 1);
        float startAngle = -rayAngle / 2f;

        // Get indices for left, center, right rays
        int leftIndex = 0;
        int centerIndex = rayCount / 2;  // For odd numbers, this is the middle ray (4)
        int rightIndex = rayCount - 1;

        // Cast 9 straight horizontal rays
        for (int i = 0; i < rayCount; i++)
        {
            float horizontalAngle = startAngle + angleStep * i;

            // Calculate distance from center index (0 at center, 1 at edges)
            float normalizedDistanceFromCenter = Mathf.Abs(i - centerIndex) / (float)centerIndex;

            // Lerp between full size (at center) and 0.8x (at edges)
            float lerpedMultiplier = Mathf.Lerp(1.0f, sideMultiplier, normalizedDistanceFromCenter);
            float currentSensorLength = sensorLength * lerpedMultiplier;

            // Create direction for horizontal ray (straight forward from the car perspective)
            Vector3 direction = Quaternion.Euler(0, horizontalAngle, 0) * raycastOrigin.forward;

            // Store ray origin and direction
            rayOrigins[i] = raycastOrigin.position;
            rayDirections[i] = direction;

            // Cast the ray and process hit with the appropriate length
            CastRayAndProcessHit(i, direction, null, null, currentSensorLength);
        }

        // Add vertical rays if enabled
        if (useVerticalRays)
        {
            int upDownIndex = rayCount;
            float verticalSensorLength = sensorLength * verticalMultiplier; // Vertical rays are 0.6x the size

            // Array of positions to add vertical rays (left, center, right)
            int[] keyPositions = { leftIndex, centerIndex, rightIndex };

            // Add up rays at 5 degree angle at left, center, right
            for (int i = 0; i < 3; i++)
            {
                int baseIndex = keyPositions[i];
                float horizontalAngle = startAngle + angleStep * baseIndex;

                // Create up ray direction (5 degrees up)
                Vector3 upDirection = Quaternion.Euler(verticalRayAngle, horizontalAngle, 0) * raycastOrigin.forward;

                // Store ray data
                rayOrigins[upDownIndex] = raycastOrigin.position;
                rayDirections[upDownIndex] = upDirection;

                // Cast ray with special color for up rays and reduced length
                CastRayAndProcessHit(upDownIndex, upDirection, new Color(0, 1, 0.5f), new Color(1, 0, 0.5f), verticalSensorLength);

                upDownIndex++;
            }

            // Add down rays at 5 degree angle at left, center, right
            for (int i = 0; i < 3; i++)
            {
                int baseIndex = keyPositions[i];
                float horizontalAngle = startAngle + angleStep * baseIndex;

                // Create down ray direction (5 degrees down)
                Vector3 downDirection = Quaternion.Euler(-verticalRayAngle, horizontalAngle, 0) * raycastOrigin.forward;

                // Store ray data
                rayOrigins[upDownIndex] = raycastOrigin.position;
                rayDirections[upDownIndex] = downDirection;

                // Cast ray with special color for down rays and reduced length
                CastRayAndProcessHit(upDownIndex, downDirection, new Color(0.5f, 1, 0), new Color(1, 0.5f, 0), verticalSensorLength);

                upDownIndex++;
            }
        }
    }

    // Helper method to avoid code duplication
    private void CastRayAndProcessHit(int rayIndex, Vector3 direction, Color? noHitColor = null, Color? hitColor = null, float? customSensorLength = null)
    {
        // Default colors
        Color normalNoHitColor = Color.green;
        Color normalHitColor = Color.red;

        // Use custom sensor length if provided, otherwise use the default
        float currentSensorLength = customSensorLength ?? sensorLength;

        RaycastHit hit;
        if (Physics.Raycast(raycastOrigin.position, direction, out hit, currentSensorLength, raycastLayerMask))
        {
            // Check if we hit something with the track wall tag
            if (hit.collider.CompareTag(trackWallTag))
            {
                // Normalize the distance (1 = max distance, 0 = collision)
                rayDistances[rayIndex] = hit.distance / currentSensorLength;

                // Visualize the ray hit
                if (visualizeRays)
                    Debug.DrawLine(raycastOrigin.position, hit.point, hitColor ?? normalHitColor);
            }
            else
            {
                // Hit something that's not a track wall
                rayDistances[rayIndex] = 1.0f;

                if (visualizeRays)
                    Debug.DrawLine(raycastOrigin.position, hit.point, Color.yellow);
            }
        }
        else
        {
            // No obstacle detected within range
            rayDistances[rayIndex] = 1.0f;

            // Visualize the full ray
            if (visualizeRays)
                Debug.DrawRay(raycastOrigin.position, direction * currentSensorLength, noHitColor ?? normalNoHitColor);
        }
    }

    // Calculate velocities of ray distances (how quickly distances are changing)
    void CalculateRayVelocities()
    {
        float deltaTime = Time.fixedDeltaTime;
        if (deltaTime <= 0f) deltaTime = 0.02f; // Fallback value to prevent division by zero

        for (int i = 0; i < totalRayCount; i++)
        {
            // Calculate the rate of change in distance
            // Positive value = object moving away, Negative value = object approaching
            rayVelocities[i] = (rayDistances[i] - previousRayDistances[i]) / deltaTime;

            // Clamp velocity to reasonable range to avoid huge spikes
            rayVelocities[i] = Mathf.Clamp(rayVelocities[i], -10f, 10f);
        }
    }

    // Get the normalized ray distances (accessible to the AI agent)
    public float[] GetRayDistances()
    {
        return rayDistances;
    }

    // Get the ray velocity data (change in distance per second)
    public float[] GetVelocityData()
    {
        return rayVelocities;
    }

    // Get the combined sensor data as one array
    public float[] GetAllSensorData()
    {
        float[] combined = new float[rayDistances.Length];
        rayDistances.CopyTo(combined, 0);
        return combined;
    }

    // Get ray origins for visualization
    public Vector3[] GetRayOrigins()
    {
        return rayOrigins;
    }

    // Get ray directions for visualization
    public Vector3[] GetRayDirections()
    {
        return rayDirections;
    }

    // Get the maximum ray distance (sensorLength)
    public float GetMaxRayDistance()
    {
        return sensorLength;
    }

    private void OnCollisionEnter(Collision collision)
    {
        // Check if this is a wall collision using tag
        if (collision.gameObject.CompareTag(trackWallTag))
        {
            // Notify the agent of the collision
            if (carAgent != null)
            {
                carAgent.ReportCollision();
            }
        }
    }

    // Public method to properly initialize raycast origin
    public void InitializeRaycastOrigin(Transform carBodyTransform)
    {
        raycastOrigin = carBodyTransform;
        Debug.Log($"Raycast origin set to {raycastOrigin.name}");
    }
}