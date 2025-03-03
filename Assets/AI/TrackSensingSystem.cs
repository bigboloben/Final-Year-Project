using UnityEngine;
using System.Collections.Generic;

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
    }

    void FixedUpdate()
    {
        // Perform sensing
        CastForwardRays();
    }

    void CastForwardRays()
    {
        if (rayCount < 1) return;

        // Calculate the angle between rays
        float angleStep = rayAngle / (rayCount - 1);
        float startAngle = -rayAngle / 2f;

        // Cast rays in a fan pattern
        for (int i = 0; i < rayCount; i++)
        {
            float currentAngle = startAngle + angleStep * i;
            // This creates a fan pattern from left to front to right
            Vector3 direction = Quaternion.Euler(0, currentAngle, 0) * raycastOrigin.forward;

            RaycastHit hit;
            if (Physics.Raycast(raycastOrigin.position, direction, out hit, sensorLength))
            {
                // Check if we hit something with the track wall tag
                if (hit.collider.CompareTag(trackWallTag))
                {
                    // Normalize the distance (1 = max distance, 0 = collision)
                    rayDistances[i] = hit.distance / sensorLength;

                    // Visualize the ray hit
                    if (visualizeRays)
                        Debug.DrawLine(raycastOrigin.position, hit.point, Color.red);
                }
                else
                {
                    // Hit something that's not a track wall
                    rayDistances[i] = 1.0f;

                    if (visualizeRays)
                        Debug.DrawLine(raycastOrigin.position, hit.point, Color.yellow);
                }
            }
            else
            {
                // No obstacle detected within range
                rayDistances[i] = 1.0f;

                // Visualize the full ray
                if (visualizeRays)
                    Debug.DrawRay(raycastOrigin.position, direction * sensorLength, Color.green);
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
}