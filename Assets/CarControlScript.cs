using Assets.TrackGeneration;
using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UIElements;

public class CarControlScript : MonoBehaviour
{
    public TrackHandler trackHandler;

    // Add a flag to determine if this car is player controlled or AI controlled
    [Header("Control Settings")]
    public bool isPlayerControlled = true;
    public bool canMove = true;

    // Input values that both player and AI will use
    private float accelerationInput;
    private float reverseInput;
    private float steeringInput;
    private bool handbrakeInput;

    // Input system for player control
    private CarControls controls;
    private Transform sphereTransform;  // Reference to the sphere
    private Transform carBodyTransform; // Reference to the car body cube
    private Rigidbody sphereRb;        // Rigidbody of the sphere
    private Collider sphereCollider;
    private int colliderInstanceId;
    private int trackColliderInstanceId;

    public float accelerationForce = 40f;
    public float reverseForce = 20f;
    public float maxSteeringAngle = 70f;
    public AnimationCurve steeringCurve;
    private float heightOffset = 0.05f;   // Height of car body above sphere

    [Header("Visual Effects")]
    public bool enableVisualEffects = true;
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
    private float downforce = 60f;
    public float sidewaysGripFactor = 2f;     // Resistance to sliding sideways
    public float forwardGripFactor = 1.5f;    // Forward/backward grip
    public float maxVelocity = 30f;           // Maximum speed
    public PhysicsMaterial spherePhysicMaterial;  // Reference to physics material

    private float reduceFactor = 1f;

    [Header("Drift Settings")]
    public bool isDrifting;
    public float driftTime;
    public float currentBoostPower;
    private float driftDirection;
    public float driftGripFactor = 0.5f;      // Reduced grip during drift
    public float minSpeedToDrift = 8f;        // Minimum speed required to initiate drift
    public float driftSteerAngle = 15f;       // Additional steering angle during drift
    public float driftLeanAngle = 15f;        // How much the car leans into drift
    public float boostForce = 30f;            // Force applied during boost
    public float maxBoostTime = 2f;           // Maximum boost duration
    public float[] boostThresholds = { 1f, 2f, 3f }; // Time thresholds for boost levels
    public float boostTimeRemaining;

    bool reset = false;

    private float groundRayLength = 0.7f;   // Length of ground detection ray
    private float groundRotationSpeed = 1f; // How quickly the car adjusts to ground normal
    private RaycastHit groundHit;           // Store raycast hit information

    public bool removeBounces = true;
    private float smoothingFactor = 0.5f; // Adjust this value between 0 and 1
    private Vector3 normalAverage = Vector3.zero;

    void Awake()
    {
        // Only initialize controls if this is player controlled
        if (isPlayerControlled)
        {
            InitializeControls();
        }
    }

    void OnEnable()
    {
        if (isPlayerControlled && controls != null)
        {
            controls.Enable();
        }
        Physics.ContactModifyEventCCD += PreventGhostBumps;
        Physics.ContactModifyEvent += PreventGhostBumps;
    }

    private void OnDisable()
    {
        if (isPlayerControlled && controls != null)
        {
            controls.Disable();
            UnsubscribeControls();
        }
        Physics.ContactModifyEventCCD -= PreventGhostBumps;
        Physics.ContactModifyEvent -= PreventGhostBumps;
    }

    private void OnDestroy()
    {
        Physics.ContactModifyEventCCD -= PreventGhostBumps;
        Physics.ContactModifyEvent -= PreventGhostBumps;
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
        // Only initialize controls for player-controlled cars
        if (!isPlayerControlled) return;

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
            handbrakeInput = true;
            TryStartDrift();
        };
        controls.Car.Handbrake.canceled += ctx => {
            handbrakeInput = false;
            EndDrift();
        };

        controls.Car.Reset.performed += ctx => reset = ctx.ReadValue<float>() > 0;
        controls.Car.Reset.canceled += ctx => reset = ctx.ReadValue<float>() > 0;

        controls.Keyboard.Grid.performed += ctx => TrackReset();
        controls.Keyboard.Circular.performed += ctx => TrackReset();
        controls.Keyboard.Random.performed += ctx => TrackReset();
    }

    private void UnsubscribeControls()
    {
        if (!isPlayerControlled || controls == null) return;

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
            handbrakeInput = true;
            TryStartDrift();
        };
        controls.Car.Handbrake.canceled -= ctx => {
            handbrakeInput = false;
            EndDrift();
        };

        controls.Car.Reset.performed -= ctx => reset = ctx.ReadValue<float>() > 0;
        controls.Car.Reset.canceled -= ctx => reset = ctx.ReadValue<float>() > 0;

        controls.Keyboard.Grid.performed -= ctx => TrackReset();
        controls.Keyboard.Circular.performed -= ctx => TrackReset();
        controls.Keyboard.Random.performed -= ctx => TrackReset();
    }

    void Start()
    {
        sphereTransform = transform;
        carBodyTransform = transform.Find("CarBody");
        vehicleModel = carBodyTransform.Find("CarModel");

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
        sphereCollider = sphereTransform.GetComponent<SphereCollider>();
        sphereCollider.hasModifiableContacts = true;
        sphereCollider.providesContacts = true;
        colliderInstanceId = sphereCollider.GetInstanceID();

        if (trackHandler == null)
        {
            Debug.LogError("Track handler not assigned!");
        }
        else
        {
            trackColliderInstanceId = trackHandler.surfaceInstanceID;
        }

        sphereRb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
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

            // Validate position before applying
            if (!float.IsInfinity(startPos1.x) && !float.IsNaN(startPos1.x) &&
                !float.IsInfinity(startPos1.y) && !float.IsNaN(startPos1.y) &&
                !float.IsInfinity(startPos1.z) && !float.IsNaN(startPos1.z))
            {
                transform.position = startPos1;
                transform.rotation = startRot;
            }
            else
            {
                Debug.LogError("Invalid starting position from track handler!");
                transform.position = new Vector3(0, 1, 0); // Fallback position
            }
        }


        // Log control mode
        Debug.Log($"Car {gameObject.name} initialized in {(isPlayerControlled ? "PLAYER" : "AI")} control mode");
    }

    void TrackReset()
    {
        trackColliderInstanceId = trackHandler.surfaceInstanceID;
    }

    void FixedUpdate()
    {
        if (!canMove) return;
        if (reset)
        {
            ResetPosition();
        }
        else
        {
            ApplyGrip();
            Driving();
            Reversing();
            Steering();
            UpdateDriftAndBoost();
        }
    }

    void Update()
    {
        // Put visual updates in Update
        if (!enableVisualEffects) return;
        UpdateCarBodyPosition();
        UpdateBodyRoll();
        UpdateWheelTurn();
        ApplyEffects();
    }

    private void PreventGhostBumps(PhysicsScene scene, NativeArray<ModifiableContactPair> contactPairs)
    {
        if (!removeBounces) return; // Early exit if not needed

        Vector3 groundNormal = groundHit.normal;
        if (groundNormal == Vector3.zero) groundNormal = Vector3.up;

        for (int pairIndex = 0; pairIndex < contactPairs.Length; pairIndex++)
        {
            ModifiableContactPair pair = contactPairs[pairIndex];

            if (pair.colliderInstanceID != colliderInstanceId) continue;

            if (pair.otherColliderInstanceID != trackColliderInstanceId) continue;

            for (int i = 0; i < pair.contactCount; i++)
            {
                Vector3 contactNormal = pair.GetNormal(i);

                Vector3 smoothedNormal = FastSmoothNormal(contactNormal);

                pair.SetNormal(i, smoothedNormal);
            }
        }
    }

    private Vector3 FastSmoothNormal(Vector3 currentNormal)
    {
        if (normalAverage == Vector3.zero)
        {
            normalAverage = currentNormal;
            return currentNormal;
        }

        normalAverage = Vector3.Lerp(normalAverage, currentNormal, 1.0f - smoothingFactor);

        if (normalAverage.sqrMagnitude < 0.001f)
            return Vector3.up;

        return normalAverage.normalized;
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
        if (isDrifting && accelerationInput > 0.01f)
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

        if (localVelocity.magnitude > 1)
        {
            Vector3 downwardForce = -transform.up * downforce;
            sphereRb.AddForce(downwardForce, ForceMode.Force);
        }

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

        // Apply forces to the sphere's rigidbody
        sphereRb.AddForce(sidewaysForce, ForceMode.Force);

        // Limit maximum velocity with smooth clamping
        if (sphereRb.linearVelocity.magnitude > maxVelocity * reduceFactor)
        {
            float slowDownFactor = Mathf.Lerp(1f, (maxVelocity * reduceFactor) / sphereRb.linearVelocity.magnitude, Time.fixedDeltaTime * 5f);
            sphereRb.linearVelocity *= slowDownFactor;
        }
    }

    void Driving()
    {
        Vector3 forwardDirection = carBodyTransform.forward;
        float currentForce = accelerationInput * accelerationForce * reduceFactor;
        sphereRb.AddForce(forwardDirection * currentForce, ForceMode.Force);
    }

    void Reversing()
    {
        Vector3 backwardsDirection = -carBodyTransform.forward;
        float currentForce = reverseInput * reverseForce * reduceFactor;
        sphereRb.AddForce(backwardsDirection * currentForce, ForceMode.Force);
    }

    void Steering()
    {
        float carVelocity = sphereRb.linearVelocity.magnitude;

        if (accelerationInput > 0 || carVelocity > 1)
        {
            // Check if we're moving backwards
            float forwardSpeed = Vector3.Dot(sphereRb.linearVelocity, carBodyTransform.forward);
            float steeringDirection = Mathf.Sign(forwardSpeed); 

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
        Vector3 rayStart = sphereTransform.position;
        Vector3 rayDirection = Vector3.down;

        if (Physics.Raycast(rayStart, rayDirection, out groundHit, groundRayLength))
        {
            // Get the ground normal
            Vector3 groundNormal = groundHit.normal;

            // Keep the current Y rotation (steering)
            float currentYRotation = carBodyTransform.eulerAngles.y;

            // Create rotation from ground normal
            Quaternion targetGroundRotation = Quaternion.FromToRotation(Vector3.up, groundNormal);

            // Combine ground rotation with steering
            Quaternion targetRotation = targetGroundRotation * Quaternion.Euler(0, currentYRotation, 0);

            // Smoothly interpolate the rotation
            carBodyTransform.rotation = Quaternion.Lerp(
                carBodyTransform.rotation,
                targetRotation,
                Time.fixedDeltaTime * groundRotationSpeed
            );
        }
        else
        {
            // If no ground is detected, gradually return to neutral rotation while preserving Y
            float currentYRotation = carBodyTransform.eulerAngles.y;
            Quaternion targetRotation = Quaternion.Euler(0, currentYRotation, 0);
            carBodyTransform.rotation = Quaternion.Lerp(
                carBodyTransform.rotation,
                targetRotation,
                Time.fixedDeltaTime * groundRotationSpeed
            );
        }

        // Update position
        Vector3 targetPosition = sphereTransform.position + Vector3.up * heightOffset;
        Vector3 currentPosition = carBodyTransform.position;
        carBodyTransform.position = Vector3.Lerp(currentPosition, targetPosition, Time.fixedDeltaTime * 10f);
    }

    public void ResetVehicle()
    {
        if (trackHandler != null)
        {
            var (startPos1, startPos2, startRot) = trackHandler.GetTrackStartTransform();
            Vector3 resetPosition;
            if (isPlayerControlled)
            {
                resetPosition = startPos1;
            }
            else
            {
                resetPosition = startPos2;
            }

            transform.position = resetPosition + Vector3.up * 0.25f;
            transform.rotation = startRot;

            // Reset the physics of the sphere Rigidbody
            if (sphereRb != null)
            {
                // Check if the rigidbody is kinematic before setting velocities
                bool wasKinematic = sphereRb.isKinematic;

                // Temporarily make non-kinematic to set velocities if needed
                if (wasKinematic)
                    sphereRb.isKinematic = false;

                // Now safe to set velocities
                sphereRb.linearVelocity = Vector3.zero;
                sphereRb.angularVelocity = Vector3.zero;

                // Restore original kinematic state
                if (wasKinematic)
                    sphereRb.isKinematic = true;
            }

            // Reset input values
            accelerationInput = 0f;
            reverseInput = 0f;
            steeringInput = 0f;
            handbrakeInput = false;

            // Reset car body position to match sphere (if applicable)
            if (carBodyTransform != null)
            {
                carBodyTransform.position = resetPosition + Vector3.up * (0.25f + heightOffset);
                carBodyTransform.rotation = startRot;
            }

            // Reset drift & boost states
            isDrifting = false;
            boostTimeRemaining = 0f;
            currentBoostPower = 0f;
        }
    }

    public void ResetPosition()
    {
        ResetVehicle();
    }

    public float GetCurrentSpeed()
    {
        if (sphereRb != null)
        {
            return sphereRb.linearVelocity.magnitude;
        }
        return 0f;
    }

    // Methods for external control (for AI)
    public void SetAccelerationInput(float value)
    {
        // Only allow AI to control if not player controlled
        if (!isPlayerControlled || !Application.isPlaying)
        {
            accelerationInput = Mathf.Clamp01(value);
        }
    }

    public void SetSteeringInput(float value)
    {
        // Only allow AI to control if not player controlled
        if (!isPlayerControlled || !Application.isPlaying)
        {
            steeringInput = Mathf.Clamp(value, -1f, 1f);
        }
    }

    public void SetReverseInput(float value)
    {
        // Only allow AI to control if not player controlled
        if (!isPlayerControlled || !Application.isPlaying)
        {
            reverseInput = Mathf.Clamp01(value);
        }
    }

    public void SetHandbrakeInput(bool value)
    {
        // Only allow AI to control if not player controlled
        if (!isPlayerControlled || !Application.isPlaying)
        {
            handbrakeInput = value;
            if (value)
            {
                TryStartDrift();
            }
            else
            {
                if (isDrifting)
                {
                    EndDrift();
                }
            }
        }
    }
}