using UnityEngine;

public class OrbitingCamera : MonoBehaviour
{
    [Header("Orbit Settings")]
    [Tooltip("Speed of rotation in degrees per second")]
    public float rotationSpeed = 20.0f;

    [Tooltip("Distance from the center point")]
    public float radius = 10.0f;

    [Tooltip("Height above the origin")]
    public float height = 5.0f;

    [Tooltip("The point to orbit around and look at")]
    public Vector3 centerPoint = Vector3.zero;

    [Header("Additional Options")]
    [Tooltip("Reverse the direction of rotation")]
    public bool reverseRotation = false;

    [Tooltip("Start rotation from this angle (in degrees)")]
    public float startAngle = 0.0f;

    private float currentAngle;

    void Start()
    {
        // Set the initial angle
        currentAngle = startAngle;

        // Position the camera initially
        UpdateCameraPosition();
    }

    void Update()
    {
        // Calculate rotation direction
        float direction = reverseRotation ? -1.0f : 1.0f;

        // Update the current angle based on speed and time
        currentAngle += rotationSpeed * direction * Time.deltaTime;

        // Keep the angle between 0 and 360 degrees for clarity
        if (currentAngle >= 360.0f)
            currentAngle -= 360.0f;
        else if (currentAngle < 0.0f)
            currentAngle += 360.0f;

        // Update the camera position
        UpdateCameraPosition();
    }

    void UpdateCameraPosition()
    {
        // Convert angle to radians for calculation
        float angleRadians = currentAngle * Mathf.Deg2Rad;

        // Calculate the new position using trigonometry
        float x = centerPoint.x + radius * Mathf.Cos(angleRadians);
        float z = centerPoint.z + radius * Mathf.Sin(angleRadians);
        float y = centerPoint.y + height;

        // Update the camera's position
        transform.position = new Vector3(x, y, z);

        // Make the camera look at the center point
        transform.LookAt(centerPoint);
    }
}