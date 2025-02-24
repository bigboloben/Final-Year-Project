using UnityEngine;
using UnityEngine.InputSystem;

public class FollowCameraScript : MonoBehaviour
{
    public Transform carTransform; // Reference to the car's transform
    public Vector3 offset; // Position offset from the car
    public float followSpeed = 5f; // Smooth speed for following the car
    public float rotationSpeed = 5f; // Smooth speed for rotating the camera
    public float lookSensitivity = 300f; // Sensitivity of the right stick rotation
    //public bool front = false;

    private CarControls controls;
    private float horizontalRotationInput = 0f; // Input for horizontal rotation

    public bool rearView;

    void Awake()
    {
        controls = new CarControls();

        // Set up the Look action to read the Vector2 from the right stick
        controls.Camera.Look.performed += ctx => {
            Vector2 lookInput = ctx.ReadValue<Vector2>();
            horizontalRotationInput = lookInput.x; // Use the X value for horizontal camera rotation
        };
        controls.Camera.Look.canceled += ctx => horizontalRotationInput = 0f; // Reset when not pressed
    }

    private void OnEnable()
    {
        controls.Enable();
    }

    private void OnDisable()
    {
        controls.Disable();
    }

    //private void OnDestroy()
    //{
    //    controls.Dispose();
    //}

    void LateUpdate()
    {
        if (carTransform == null) return;

        if (!rearView)
        {

            // Fixed position relative to car
            transform.position = carTransform.TransformPoint(offset);

            // Look at a point behind the car
            transform.LookAt(carTransform.position);

            // Keep the camera level (no roll)
            Vector3 eulerAngles = transform.rotation.eulerAngles;
            transform.rotation = Quaternion.Euler(eulerAngles.x, eulerAngles.y, 0);
        }
        else
        {
            // Target position for the camera to follow the car
            Vector3 targetPosition = carTransform.position + carTransform.TransformVector(offset);

            // Smoothly move the camera to the target position

            transform.position = Vector3.Lerp(transform.position, targetPosition, followSpeed * Time.deltaTime);

            // Rotate the camera based on right/left input
            RotateCamera();
        }
    }

    void RotateCamera()
    {
        // Calculate the rotation based on horizontal input
        float horizontalRotation = horizontalRotationInput * lookSensitivity * Time.deltaTime;

        // Debugging line to check input values
        //Debug.Log($"Horizontal Rotation Input: {horizontalRotationInput}");

        // Rotate the camera around the car's position
        // Get the direction from the car to the camera
        Vector3 direction = transform.position - carTransform.position;

        // Create a rotation quaternion
        Quaternion rotation = Quaternion.Euler(0, horizontalRotation, 0);

        // Apply the rotation to the direction vector
        direction = rotation * direction;

        // Update the camera's position to maintain the offset while applying rotation
        transform.position = carTransform.position + direction;

        // Make the camera look at the car
        transform.LookAt(carTransform.position);
    }
}
