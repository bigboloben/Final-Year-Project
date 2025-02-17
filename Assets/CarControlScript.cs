using Assets.TrackGeneration;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UIElements;

public class CarControlScript : MonoBehaviour
{
    public TrackHandler trackHandler;
    //private CheckpointManager checkpointManager;

    CarControls controls;
    private float accelerationInput;
    private float reverseInput;
    private float steeringInput;
    //private bool handbrake;
    private Transform sphereTransform;  // Reference to the sphere
    private Transform carBodyTransform; // Reference to the car body cube
    private Rigidbody sphereRb;        // Rigidbody of the sphere

    public float accelerationForce = 40f;
    public float reverseForce = 20f;
    public float maxSteeringAngle = 70f;
    public AnimationCurve steeringCurve;
    private float heightOffset = 0.05f;   // Height of car body above sphere

    [Header("Visual Effects")]
    private float bodyRollForce = 10f;
    private float bodyRollSpeed = 10f;
    public Transform vehicleModel;  // Reference to vehicle-racer
    private float currentRoll = 0f;
    private float targetRoll = 0f;
    private float currentLean = 0f;
    private float targetLean = 0f;
    private Transform[] wheelTransforms = new Transform[4];
    private Vector3[] wheelDefaultPositions = new Vector3[4];
    private Quaternion[] wheelDefaultRotations = new Quaternion[4];
    private float currentSteerAngle = 0f;

    private ParticleSystem[] smokeParticles = new ParticleSystem[2];
    private TrailRenderer[] trailRenderers = new TrailRenderer[2];



    [Header("Grip Settings")]
    public float sidewaysGripFactor = 2f;     // Resistance to sliding sideways
    public float forwardGripFactor = 1.5f;    // Forward/backward grip
    public float maxVelocity = 30f;           // Maximum speed
    public PhysicsMaterial spherePhysicMaterial;  // Reference to physics material

    //[Header("isInContact")]
    //public bool isTouchingObstacle;
    private float reduceFactor = 1f;


    [Header("Drift Settings")]
    private bool isDrifting;
    private float driftTime;
    private float currentBoostPower;
    private float driftDirection;
    public float driftGripFactor = 0.5f;      // Reduced grip during drift
    public float minSpeedToDrift = 8f;        // Minimum speed required to initiate drift
    public float driftSteerAngle = 15f;       // Additional steering angle during drift
    public float driftLeanAngle = 15f;        // How much the car leans into drift
    public float boostForce = 30f;            // Force applied during boost
    public float maxBoostTime = 2f;           // Maximum boost duration
    public float[] boostThresholds = { 1f, 2f, 3f }; // Time thresholds for boost levels
    private float boostTimeRemaining;


    void Awake()
    {
        InitializeControls();
    }
    void OnEnable()
    {
        InitializeControls();
        controls.Enable();
    }

    private void OnDisable()
    {
        controls.Disable();
        if (controls != null)
        {
            UnsubscribeControls();
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        // If the collision is not with the floor/ground layer
        if (!collision.gameObject.CompareTag("Floor"))
        {
            reduceFactor = 0.4f;
        }
    }

    private void OnCollisionExit(Collision collision)
    {
        // If we're no longer colliding with a non-floor object
        if (!collision.gameObject.CompareTag("Floor"))
        {
            reduceFactor = 1f;
        }
    }

    

    private void InitializeControls()
    {
        controls = new CarControls();
        controls.Car.Accelerate.performed += ctx => accelerationInput = ctx.ReadValue<float>();
        controls.Car.Accelerate.canceled += ctx => accelerationInput = 0f;
        controls.Car.Reverse.performed += ctx => reverseInput = ctx.ReadValue<float>();
        controls.Car.Reverse.canceled += ctx => reverseInput = 0f;
        controls.Car.Steer.performed += ctx =>
        {
            Vector2 steeringInputVector = ctx.ReadValue<Vector2>();
            steeringInput = steeringInputVector.x;
        };
        controls.Car.Steer.canceled += ctx => steeringInput = 0f;
        controls.Car.Handbrake.performed += ctx => {
            //handbrake = true;
            TryStartDrift();
        };
        controls.Car.Handbrake.canceled += ctx => {
            //handbrake = false;
            EndDrift();
        };

        controls.Car.Reset.performed += ctx => { ResetPosition(); };
        controls.Car.Reset.canceled += ctx => { };
    }

    private void UnsubscribeControls()
    {
        controls = new CarControls();
        controls.Car.Accelerate.performed -= ctx => accelerationInput = ctx.ReadValue<float>();
        controls.Car.Accelerate.canceled -= ctx => accelerationInput = 0f;
        controls.Car.Reverse.performed -= ctx => reverseInput = ctx.ReadValue<float>();
        controls.Car.Reverse.canceled -= ctx => reverseInput = 0f;
        controls.Car.Steer.performed -= ctx =>
        {
            Vector2 steeringInputVector = ctx.ReadValue<Vector2>();
            steeringInput = steeringInputVector.x;
        };
        controls.Car.Steer.canceled -= ctx => steeringInput = 0f;
        controls.Car.Handbrake.performed -= ctx => {
            //handbrake = true;
            TryStartDrift();
        };
        controls.Car.Handbrake.canceled -= ctx => {
            //handbrake = false;
            EndDrift();
        };

        controls.Car.Reset.performed -= ctx => { ResetPosition(); };
        controls.Car.Reset.canceled -= ctx => { };
    }
    void Start()
    {
        sphereTransform = transform;
        carBodyTransform = transform.Find("CarBody");
        vehicleModel = carBodyTransform.Find("CarModel");
        //Debug.LogError($"{vehicleModel.name}");
        wheelTransforms[0] = carBodyTransform.Find("frontLeft");
        wheelTransforms[1] = carBodyTransform.Find("frontRight");
        wheelTransforms[2] = carBodyTransform.Find("backLeft");
        wheelTransforms[3] = carBodyTransform.Find("backRight");

        smokeParticles[0] = carBodyTransform.Find("leftSmoke").GetComponent<ParticleSystem>();
        smokeParticles[1] = carBodyTransform.Find("rightSmoke").GetComponent<ParticleSystem>();

        foreach (var particle in smokeParticles)
        {
            particle.Stop(true); // Clear and stop
            particle.Play(true); // Initialize
            particle.Stop(true); // Stop but keep initialized
        }

        trailRenderers[0] = carBodyTransform.Find("leftLine").GetComponent<TrailRenderer>();
        trailRenderers[1] = carBodyTransform.Find("rightLine").GetComponent<TrailRenderer>();

        for (int i = 0; i < 4; i++)
        {
            wheelDefaultPositions[i] = wheelTransforms[i].localPosition;
            wheelDefaultRotations[i] = wheelTransforms[i].localRotation;
        }

        sphereRb = sphereTransform.GetComponent<Rigidbody>();

        // Configure the sphere's rigidbody
        // Configure the sphere's rigidbody to prevent CCD bouncing
        sphereRb.interpolation = RigidbodyInterpolation.Interpolate;
        sphereRb.collisionDetectionMode = CollisionDetectionMode.Discrete; // Changed from Continuous
        //sphereRb.isKinematic = true;  // Just for testing
        //sphereRb.interpolation = RigidbodyInterpolation.None;
        sphereRb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

        // Ensure the car body has no rigidbody component
        Rigidbody carBodyRb = carBodyTransform.GetComponent<Rigidbody>();


        if (carBodyRb != null)
        {
            Destroy(carBodyRb);
        }
        if (spherePhysicMaterial != null)
        {
            spherePhysicMaterial.bounciness = 0f;        // No bounce
            spherePhysicMaterial.dynamicFriction = 0.3f; // Lower friction for smoother movement
            spherePhysicMaterial.staticFriction = 0.5f;  // Moderate static friction
            spherePhysicMaterial.frictionCombine = PhysicsMaterialCombine.Minimum;
            spherePhysicMaterial.bounceCombine = PhysicsMaterialCombine.Minimum;
        }

        if (trackHandler != null)
        {
            var (startPos1, startPos2, startRot) = trackHandler.GetTrackStartTransform();
            transform.position = startPos1;
            transform.rotation = startRot;
        }
    }


    void FixedUpdate()
    {
        ApplyGrip();
        Driving();
        Reversing();
        //Braking();
        Steering();
        //UpdateCarBodyPosition();
        UpdateDriftAndBoost();
    }
    void Update()
    {
        // Put visual updates in Update
        UpdateCarBodyPosition();
        UpdateBodyRoll();
        UpdateWheelTurn();
        ApplyEffects();
    }

    private void ApplyEffects()
    {
        if (isDrifting)
        {
            trailRenderers[0].emitting = true;
            trailRenderers[1].emitting = true;
            foreach (var particle in smokeParticles)
            {
                float newSize = 75f;
                if (driftTime >= boostThresholds[1]) newSize = 300f;
                else if (driftTime >= boostThresholds[0]) newSize = 150f;

                var main = particle.main;
                main.startSize = newSize;
                if (!particle.isPlaying)
                {
                    particle.Play(true);
                }
            }
        }
        else
        {
            trailRenderers[0].emitting = false;
            trailRenderers[1].emitting = false;
            foreach (var particle in smokeParticles)
            {
                if (particle.isPlaying)
                {
                    particle.Stop(true);
                }
            }
        }
    }

    private void UpdateWheelTurn()
    {
        float carVelocity = sphereRb.linearVelocity.magnitude;  
        float normalisedSpeed = Mathf.Clamp01(Mathf.Abs(carVelocity) / maxVelocity);
        float steerRotation = steeringInput * maxSteeringAngle * steeringCurve.Evaluate(normalisedSpeed) / 3;

        if (isDrifting)
        {
            float steeringAmount = steeringInput + (1 * driftDirection);
            steerRotation = -steeringAmount * maxSteeringAngle * steeringCurve.Evaluate(normalisedSpeed) / 3;

        }

        currentSteerAngle = Mathf.Lerp(currentSteerAngle, steerRotation, Time.deltaTime * 5f);

        wheelTransforms[0].localRotation *= Quaternion.Euler(0, currentSteerAngle, 0);
        wheelTransforms[1].localRotation *= Quaternion.Euler(0, currentSteerAngle, 0);


    }

    private void UpdateBodyRoll()
    {
        if (vehicleModel == null) return;

        float lateralVelocity = Vector3.Dot(sphereRb.linearVelocity, carBodyTransform.right);

        targetRoll = lateralVelocity * bodyRollForce;
        targetRoll += steeringInput * bodyRollForce * (sphereRb.linearVelocity.magnitude / maxVelocity);
        targetRoll = Mathf.Clamp(targetRoll, -8, 8);
        currentRoll = Mathf.Lerp(currentRoll, targetRoll, Time.deltaTime * bodyRollSpeed);

        float forwardVelocity = Vector3.Dot(sphereRb.linearVelocity, carBodyTransform.forward);
        float speedRatio = 1 - (sphereRb.linearVelocity.magnitude / maxVelocity);

        // Calculate base lean from acceleration/reverse
        targetLean = (accelerationInput - reverseInput) * bodyRollForce * speedRatio;

        // Add extra backward lean during boost
        if (boostTimeRemaining > 0 && !isDrifting && accelerationInput > 0.01f)
        {
            float boostLeanIntensity = 15f; // Negative value for backward lean
            float boostRatio = boostTimeRemaining / maxBoostTime; // Gradually reduce lean as boost runs out
            targetLean += boostLeanIntensity * boostRatio;
        }

        targetLean = Mathf.Clamp(targetLean, -15, 8); // Increased negative range for backward lean
        currentLean = Mathf.Lerp(currentLean, targetLean, Time.deltaTime * bodyRollSpeed);

        Vector3 currentRotation = vehicleModel.localEulerAngles;

        Quaternion targetRotation = Quaternion.Euler(
            currentLean,
            currentRotation.y,
            currentRoll
        );

        Quaternion smoothRotation = Quaternion.Lerp(vehicleModel.localRotation, targetRotation, Time.deltaTime * bodyRollSpeed);
        vehicleModel.localRotation = smoothRotation;

        float width = 0.3f;
        float lateralOffset = width * (Mathf.Sin(Mathf.Deg2Rad * currentRoll));

        for (int i = 0; i < 4; i++)
        {
            float tempLat = lateralOffset;
            Vector3 wheelPosition = wheelDefaultPositions[i];
            if (currentRoll > 0 && (i == 1 || i == 3))
            {
                tempLat *= 2f;
            }
            if (currentRoll < 0 && (i == 0 || i == 2))
            {
                tempLat *= 2f;
            }

            wheelTransforms[i].localPosition = new Vector3(
                wheelDefaultPositions[i].x + tempLat,
                wheelDefaultPositions[i].y,
                wheelDefaultPositions[i].z
            );

            wheelTransforms[i].localRotation = wheelDefaultRotations[i];
        }
    }



    private void TryStartDrift()
    {
        float forwardSpeed = Vector3.Dot(sphereRb.linearVelocity, carBodyTransform.forward);
        if (forwardSpeed > minSpeedToDrift && Mathf.Abs(steeringInput) > 0.1f)
        {
            isDrifting = true;
            driftDirection = Mathf.Sign(steeringInput);
            driftTime = 0f;
        }
    }

    private void EndDrift()
    {
        if (isDrifting)
        {
            isDrifting = false;
            ApplyBoostFromDrift();
            // Reset car rotation
            carBodyTransform.localRotation = Quaternion.Euler(0, carBodyTransform.localEulerAngles.y, 0);
        }
    }
    private void ApplyBoostFromDrift()
    {
        // Determine boost level based on drift time
        float boostPower = 0f;
        for (int i = 0; i < boostThresholds.Length; i++)
        {
            if (driftTime >= boostThresholds[i])
            {
                boostPower = boostForce * (i + 1);
            }
        }
        if (accelerationInput < 0.1)
        {
            boostTimeRemaining = 0f;

        }

        if (boostPower > 0)
        {
            // Apply the boost and set remaining boost time
            currentBoostPower = boostPower;
            boostTimeRemaining = maxBoostTime;
        }
    }
    void UpdateDriftAndBoost()
    {
        if (isDrifting&&accelerationInput>0.01f)
        {
            driftTime += Time.fixedDeltaTime;
        }

        // Apply boost if we have any remaining
        if (boostTimeRemaining > 0 && !isDrifting && accelerationInput > 0.01f)
        {
            Vector3 boostDirection = carBodyTransform.forward;
            sphereRb.AddForce(boostDirection * currentBoostPower, ForceMode.Force);
            boostTimeRemaining -= Time.fixedDeltaTime;
        }
    }

    void ApplyGrip()
    {
        // Get the velocity relative to the car body's orientation instead of the sphere
        Vector3 localVelocity = carBodyTransform.InverseTransformDirection(sphereRb.linearVelocity);

        // Calculate desired velocity (what we want) vs current velocity (what we have)
        float desiredSidewaysVelocity = 0f; // We want zero sideways velocity for grip
        float sidewaysVelocityDifference = desiredSidewaysVelocity - localVelocity.x;

        // Calculate the current speed and direction
        float currentSpeed = sphereRb.linearVelocity.magnitude;

        // Calculate grip forces
        // Sideways force is stronger at higher speeds for better stability
        float speedFactor = Mathf.Clamp01(currentSpeed / 10f);
        float currentSidewaysGrip = sidewaysGripFactor * (1f + speedFactor * 2f);

        // Calculate forces with smoothing to prevent overcorrection
        Vector3 sidewaysForce = carBodyTransform.right * sidewaysVelocityDifference * currentSidewaysGrip;

        // Apply a downward force to improve grip
        //Vector3 downwardForce = -carBodyTransform.up * currentSpeed * 0.5f;

        // Apply forces to the sphere's rigidbody
        sphereRb.AddForce(sidewaysForce, ForceMode.Force);
        //sphereRb.AddForce(downwardForce, ForceMode.Acceleration);

        // Limit maximum velocity with smooth clamping
        if (sphereRb.linearVelocity.magnitude > maxVelocity*reduceFactor)
        {
            float slowDownFactor = Mathf.Lerp(1f, (maxVelocity*reduceFactor) / sphereRb.linearVelocity.magnitude, Time.fixedDeltaTime * 5f);
            sphereRb.linearVelocity *= slowDownFactor;
        }
    }

    void Driving()
    {
        Vector3 forwardDirection = carBodyTransform.forward;
        float currentForce = accelerationInput * accelerationForce * reduceFactor;

        // Reduce speed if touching obstacles
        //if (isTouchingObstacle)
        //{
        //    currentForce *= reduceFactor; // 70% of normal speed
        //}

        sphereRb.AddForce(forwardDirection * currentForce, ForceMode.Force);
    }

    void Reversing()
    {
        Vector3 backwardsDirection = -carBodyTransform.forward;
        float currentForce = reverseInput * reverseForce * reduceFactor;

        //// Reduce speed if touching obstacles
        //if (isTouchingObstacle)
        //{
        //    currentForce *= reduceFactor; // 70% of normal speed
        //}

        sphereRb.AddForce(backwardsDirection * currentForce, ForceMode.Force);
    }

    //void Braking()
    //{
    //    if (brakeInput>0 && isReversing == false)
    //    {
    //        sphereRb.linearDamping = brakeInput * 3f;  // Adjust this value to change braking strength
    //    }
    //    else
    //    {
    //        sphereRb.linearDamping = 0.1f;  // Normal rolling resistance
    //    }
    //}

    void Steering()
    {
        float carVelocity = sphereRb.linearVelocity.magnitude;

        if (accelerationInput > 0 || carVelocity > 1)
        {
            // Check if we're moving backwards
            float forwardSpeed = Vector3.Dot(sphereRb.linearVelocity, carBodyTransform.forward);
            float steeringDirection = Mathf.Sign(forwardSpeed); // Will be -1 when moving backwards

            if (isDrifting)
            {
                float steeringAmount = steeringInput + (1 * driftDirection);
                // Flip steering when moving backwards
                float rotationAmount = 80 * steeringAmount * steeringDirection * Time.fixedDeltaTime;
                carBodyTransform.Rotate(0, rotationAmount, 0, Space.World);
            }
            else
            {
                float normalisedSpeed = Mathf.Clamp01(Mathf.Abs(carVelocity) / maxVelocity);
                // Flip steering when moving backwards
                float rotationAmount = steeringInput * maxSteeringAngle * steeringDirection *
                                     steeringCurve.Evaluate(normalisedSpeed) * Time.fixedDeltaTime;
                carBodyTransform.Rotate(0, rotationAmount, 0, Space.World);
            }
        }
    }

    void UpdateCarBodyPosition()
    {
        // Smoothly interpolate the car body position
        Vector3 targetPosition = sphereTransform.position + Vector3.up * heightOffset;
        Vector3 currentPosition = carBodyTransform.position;
        carBodyTransform.position = Vector3.Lerp(currentPosition, targetPosition, Time.fixedDeltaTime * 10f);

        // Keep the original rotation for the Y axis (steering) but zero out other rotations
        Vector3 currentRotation = carBodyTransform.eulerAngles;
        carBodyTransform.eulerAngles = new Vector3(0, currentRotation.y, 0);
    }

    void ResetPosition()
    {
        if (trackHandler != null)
        {
            var (startPos1, startPos2, startRot) = trackHandler.GetTrackStartTransform();
            transform.position = startPos1;
            transform.rotation = startRot;
        }
    }

    public float GetCurrentSpeed()
    {
        if (sphereRb != null)
        {
            return sphereRb.linearVelocity.magnitude;
        }
        return 0f;
    }
}