using UnityEngine;

public class AntiRollBar : MonoBehaviour
{
    public WheelCollider wheelLeft;
    public WheelCollider wheelRight;
    public float antiRollForce = 5000f;

    private Rigidbody rb;

    public void Initialize(WheelCollider leftWheel, WheelCollider rightWheel, Rigidbody carRigidbody)
    {
        wheelLeft = leftWheel;
        wheelRight = rightWheel;
        rb = carRigidbody;
    }

    void FixedUpdate()
    {
        ApplyAntiRoll();
    }

    void ApplyAntiRoll()
    {
        WheelHit hit;
        float travelLeft = 1.0f;
        float travelRight = 1.0f;

        bool groundedLeft = wheelLeft.GetGroundHit(out hit);
        if (groundedLeft)
        {
            travelLeft = (-wheelLeft.transform.InverseTransformPoint(hit.point).y - wheelLeft.radius) / wheelLeft.suspensionDistance;
        }

        bool groundedRight = wheelRight.GetGroundHit(out hit);
        if (groundedRight)
        {
            travelRight = (-wheelRight.transform.InverseTransformPoint(hit.point).y - wheelRight.radius) / wheelRight.suspensionDistance;
        }

        float antiRollForceApplied = (travelLeft - travelRight) * antiRollForce;

        if (groundedLeft)
        {
            rb.AddForceAtPosition(wheelLeft.transform.up * -antiRollForceApplied, wheelLeft.transform.position);
        }
        if (groundedRight)
        {
            rb.AddForceAtPosition(wheelRight.transform.up * antiRollForceApplied, wheelRight.transform.position);
        }
    }
}
