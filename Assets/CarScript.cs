using System.Data;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.InputSystem;
using TMPro;

public class CarScript : MonoBehaviour
{
    CarControls controls;

    public GameObject wheelObject; // Prefab for the visual wheel
    private GameObject[] wheelObjects = new GameObject[4]; // Visual wheel objects
    private Rigidbody rb;
    public TextMeshProUGUI speedText;

    // Wheel positions based on the car's transform
    private Vector3[] wheels = new Vector3[4];
    private Vector2 wheelDistance = new Vector2(0.65f, 1.2f); // Distance settings for wheel placement
    public WheelCollider[] wheelColliders; // Array for WheelColliders

    public float carTopSpeed = 30f;
    public float torqueFactor = 80000f;
    public AnimationCurve torqueCurve;

    public float maxSteering = 15f;
    public AnimationCurve steeringCurve;

    private float accelerationInput;
    private float steeringInput;
    //private float brakeInput;
    private bool handbrake;

    [Header("Suspension Settings")]
    public float spring = 36000f;
    public float damper = 6000f;
    public float targetPosition = 0.5f;

    [Header("Forward Friction Settings")]
    public float fExtremumSlip = 0.4f;
    public float fExtremumValue = 1.0f;
    public float fAsymptoteSlip = 0.8f;
    public float fAsymptoteValue = 0.5f;
    public float fStiffness = 3f;

    [Header("Sideways Friction Settings")]
    public float sExtremumSlip = 0.2f;
    public float sExtremumValue = 3f;
    public float sAsymptoteSlip = 3f;
    public float sAsymptoteValue = 3f;
    public float sStiffness = 3f;

    [Header("Stability Control Settings")]
    public float stabilityControlStrength = 0.1f; // Adjust to control the strength of stability control
    public float oversteerThreshold = 1.5f; // Threshold for applying stability control


    private bool setValues = false;

    private AntiRollBar frontAntiRollBar;
    private AntiRollBar rearAntiRollBar;


    void Awake()
    {
        controls = new CarControls();

        // Set up acceleration input callback
        controls.Car.Accelerate.performed += ctx => accelerationInput = ctx.ReadValue<float>();
        controls.Car.Accelerate.canceled += ctx => accelerationInput = 0f;

        //controls.Car.Brake.performed += ctx => brakeInput = ctx.ReadValue<float>();
        //controls.Car.Brake.canceled += ctx => brakeInput = 0f;

        controls.Car.Steer.performed += ctx =>
        {
            Vector2 steeringInputVector = ctx.ReadValue<Vector2>();
            steeringInput = steeringInputVector.x;
        };

        controls.Car.Handbrake.performed += ctx => handbrake = true;
        controls.Car.Handbrake.canceled += ctx => handbrake = false;

        controls.Car.Steer.canceled += ctx => steeringInput = 0f;

        controls.Car.SetValues.performed += ctx => setValues = true;
        controls.Car.SetValues.canceled += ctx => setValues = false;

    }

    private void OnEnable()
    {
        controls.Enable();
    }

    private void OnDisable()
    {
        controls.Disable();
    }

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = new Vector3(0, -1f, 0); // Already lowered center of mass
        rb.linearDamping = 0.1f; // Adjust as needed for desired resistance; keep low for minimal impact on acceleration
        rb.angularDamping = 1.0f;


        // Initialize wheel colliders
        wheelColliders = new WheelCollider[4];
        wheelObjects = new GameObject[4]; // Ensure wheelObjects array is initialized

        for (int i = 0; i < 4; i++)
        {
            // Create a new GameObject for each WheelCollider
            GameObject colliderObject = new GameObject("WheelCollider" + i);
            colliderObject.transform.parent = transform; // Set the car transform as the parent

            // Position the WheelCollider based on the car's transform
            colliderObject.transform.position = GetWheelPosition(i);

            // Add WheelCollider component to the collider object
            wheelColliders[i] = colliderObject.AddComponent<WheelCollider>();

            // Set WheelCollider properties
            wheelColliders[i].radius = 0.25f; // Set radius to match visual wheel
            wheelColliders[i].suspensionDistance = 0.2f; // Set suspension distance
            

            // Instantiate the visual wheel object with the correct rotation
            wheelObjects[i] = Instantiate(wheelObject, colliderObject.transform.position, colliderObject.transform.rotation);
            wheelObjects[i].transform.parent = colliderObject.transform; // Set visual wheel as a child of the collider object
        }
        SetSuspensionAndFriction();

        SetupAntiRollBars();
    }

    void FixedUpdate()
    {   
        //SetSuspensionAndFriction() ;
        // Calculate wheel positions based on the car's transform
        for (int i = 0; i < 4; i++)
        {
            wheels[i] = GetWheelPosition(i);
        }

        Suspension();
        Debug.Log("acceleration" + accelerationInput);
        Driving();
        Debug.Log("steering" + steeringInput);
        Steering();
        //Braking();
        HandleDrift();
        //StabilityControl();
        if (setValues) SetSuspensionAndFriction();
        UpdateSpeedDisplay();

    }

    void UpdateSpeedDisplay()
    {
        float speed = rb.linearVelocity.magnitude * 3.6f; // Convert m/s to km/h
        speedText.text = $"Speed: {speed:0.0} km/h \n" +
            $"{Mathf.Abs(rb.linearVelocity.magnitude)} m/s \n" +
            $"_________________________ \n"; // Format the speed
    }

    void Driving()
    {
        if (accelerationInput > 0.0f)
        {
            // Get the speed of the car in terms of forward velocity
            float carSpeed = Vector3.Dot(transform.forward, rb.linearVelocity);

            // Check if the car's speed is below the top speed
            if (Mathf.Abs(carSpeed) < carTopSpeed)
            {
                // Normalized speed to clamp between 0 and 1 based on top speed
                float normalisedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carTopSpeed);

                // Calculate torque based on the normalized speed and acceleration input
                float torque = (torqueCurve.Evaluate(normalisedSpeed) * accelerationInput * torqueFactor) * Time.fixedDeltaTime; // Scale by fixedDeltaTime

                // Apply force to the wheels to move the car
                wheelColliders[0].motorTorque = torque; // Front Right
                wheelColliders[1].motorTorque = torque; // Front Left
            }
            else
            {
                // If the top speed is reached, do not apply any torque
                wheelColliders[0].motorTorque = 0;
                wheelColliders[1].motorTorque = 0;
            }
        }
        else
        {
            // Reset motor torque when not accelerating
            wheelColliders[0].motorTorque = 0;
            wheelColliders[1].motorTorque = 0;
        }
    }


    //void Braking()
    //{
    //    // Apply brake torque to all wheel colliders
    //    float brakeTorque = 2000f; // Adjust this value based on your needs
    //    for (int i = 0; i < wheelColliders.Length; i++)
    //    {
    //        wheelColliders[i].brakeTorque = brakeTorque * brakeInput; // Apply brake torque
    //    }
    //}

    void Steering()
    {
        // Calculate the steering angle based on the steering input and maximum steering angle
        float carSpeed = Vector3.Dot(transform.forward, rb.linearVelocity);
        float normalisedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carTopSpeed);
        float steeringAngle = steeringInput * maxSteering * steeringCurve.Evaluate(normalisedSpeed);

        // Apply the steering angle to the front wheel colliders
        wheelColliders[0].steerAngle = steeringAngle; // Front Right
        wheelColliders[1].steerAngle = steeringAngle; // Front Left
    }

    void Suspension()
    {
        for (int i = 0; i < wheelColliders.Length; i++)
        {
            WheelHit hit;

            // Check if the wheel is grounded
            if (wheelColliders[i].GetGroundHit(out hit))
            {
                // Get the world position and rotation of the WheelCollider
                Vector3 wheelPosition;
                Quaternion wheelRotation;
                wheelColliders[i].GetWorldPose(out wheelPosition, out wheelRotation);

                // Update visual wheel position and rotation based on the WheelCollider
                wheelObjects[i].transform.position = wheelPosition; // Set visual wheel position
                wheelObjects[i].transform.rotation = wheelRotation; // Set visual wheel rotation
            }
            else
            {
                // If not hitting anything, position the visual wheel at its max extension
                Vector3 wheelPosition = wheelColliders[i].transform.position - Vector3.up * wheelColliders[i].suspensionDistance;
                wheelObjects[i].transform.position = wheelPosition; // Place the wheel at its max extension
                wheelObjects[i].transform.rotation = Quaternion.identity; // Maintain visual rotation
            }
        }
    }

    void HandleDrift()
    {
        // Loop only through rear wheels (index 2 and 3)
        for (int i = 2; i < 4; i++)
        {
            WheelFrictionCurve sidewaysFriction = wheelColliders[i].sidewaysFriction;

            if (handbrake)
            {
                // Lower stiffness to allow sliding
                sidewaysFriction.stiffness = sStiffness * 0.8f; // Adjust factor as needed
            }
            else
            {
                // Reset to original stiffness
                sidewaysFriction.stiffness = sStiffness;
            }

            wheelColliders[i].sidewaysFriction = sidewaysFriction;
        }
    }


    void StabilityControl()
    {
        // Get the car's local sideways velocity
        Vector3 lateralVelocity = transform.InverseTransformDirection(rb.linearVelocity).x * transform.right;

        // Check if the car is sliding over the threshold
        if (lateralVelocity.magnitude > oversteerThreshold)
        {
            // Calculate the amount of braking needed to counteract the slide
            float stabilityBrakeForce = stabilityControlStrength * lateralVelocity.magnitude;

            // Apply stability braking to wheels to reduce slide
            if (lateralVelocity.x > 0)
            {
                wheelColliders[2].brakeTorque = stabilityBrakeForce; // Apply brake to back right wheel
            }
            else
            {
                wheelColliders[3].brakeTorque = stabilityBrakeForce; // Apply brake to back left wheel
            }
        }
    }

    Vector3 GetWheelPosition(int index)
    {
        // Base offset for wheel placement
        Vector3 offset = new Vector3(0, -0.4f, 0); // Adjust this value to control how far below the car the wheels are positioned

        switch (index)
        {
            case 0: // Front Right
                return transform.position + transform.right * wheelDistance.x + transform.forward * wheelDistance.y + offset;
            case 1: // Front Left
                return transform.position + transform.right * -wheelDistance.x + transform.forward * wheelDistance.y + offset;
            case 2: // Back Right
                return transform.position + transform.right * wheelDistance.x + transform.forward * -wheelDistance.y + offset;
            case 3: // Back Left
                return transform.position + transform.right * -wheelDistance.x + transform.forward * -wheelDistance.y + offset;
            default:
                return Vector3.zero;
        }
    }

    void SetSuspensionAndFriction()
    {
        foreach (var wheel in wheelColliders)
        {
            // Suspension setup
            JointSpring suspensionSpring = new JointSpring
            {
                spring = spring,
                damper = damper,
                targetPosition = targetPosition
            };
            wheel.suspensionSpring = suspensionSpring;
            Debug.Log($"Spring Set: {spring}, Damper: {damper}, Target Position: {targetPosition}");

            // Forward friction setup
            WheelFrictionCurve forwardFriction = wheel.forwardFriction;
            forwardFriction.extremumSlip = fExtremumSlip;
            forwardFriction.extremumValue = fExtremumValue;
            forwardFriction.asymptoteSlip = fAsymptoteSlip;
            forwardFriction.asymptoteValue = fAsymptoteValue;
            forwardFriction.stiffness = fStiffness;
            wheel.forwardFriction = forwardFriction;
            Debug.Log($"Forward Friction Set: {fExtremumSlip}, {fExtremumValue}, {fAsymptoteSlip}, {fAsymptoteValue}, {fStiffness}");

            // Sideways friction setup
            WheelFrictionCurve sidewaysFriction = wheel.sidewaysFriction;
            sidewaysFriction.extremumSlip = sExtremumSlip;
            sidewaysFriction.extremumValue = sExtremumValue;
            sidewaysFriction.asymptoteSlip = sAsymptoteSlip;
            sidewaysFriction.asymptoteValue = sAsymptoteValue;
            sidewaysFriction.stiffness = sStiffness;
            wheel.sidewaysFriction = sidewaysFriction;
            Debug.Log($"Sideways Friction Set: {sExtremumSlip}, {sExtremumValue}, {sAsymptoteSlip}, {sAsymptoteValue}, {sStiffness}");
        }
    }

    void SetupAntiRollBars()
    {
        // Create and initialize AntiRollBar for the front axle
        frontAntiRollBar = gameObject.AddComponent<AntiRollBar>();
        frontAntiRollBar.Initialize(wheelColliders[1], wheelColliders[0], rb);

        // Create and initialize AntiRollBar for the rear axle
        rearAntiRollBar = gameObject.AddComponent<AntiRollBar>();
        rearAntiRollBar.Initialize(wheelColliders[3], wheelColliders[2], rb);
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.red; // Set the color for the Gizmos
        for (int i = 0; i < 4; i++)
        {
            Vector3 wheelPosition = GetWheelPosition(i);
            Gizmos.DrawSphere(wheelPosition, 0.1f); // Draw the Gizmo sphere

            // Debug logs to compare positions
            Debug.Log($"Gizmo Position: {wheelPosition}");

            // Also log the positions of the visual wheels
            if (wheelObjects[i] != null)
            {
                Debug.Log($"Visual Wheel Position {i}: {wheelObjects[i].transform.position}");
            }

            // Log the position of the WheelCollider
            if (wheelColliders[i] != null)
            {
                Vector3 colliderPos = wheelColliders[i].transform.position;
                Debug.Log($"WheelCollider Position {i}: {colliderPos}");
            }
        }
    }
}
