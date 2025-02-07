using UnityEngine;

public class CameraControlScript : MonoBehaviour

{
    private Camera[] cameras = new Camera[2];
    private CarControls controls;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Awake()
    {

        controls = new CarControls();

        //cameras[0] = transform.Find("PlayerCamera0").gameObject;
        //cameras[1] = transform.Find("PlayerCamera1").gameObject;

        //controls.Camera.LookBehind.performed += ctx =>
        //{
        //    cameras[0].SetActive(false);
        //    cameras[1].SetActive(true);
        //};
        //controls.Camera.LookBehind.canceled += ctx =>
        //{
        //    cameras[0].SetActive(true);
        //    cameras[1].SetActive(false);
        //};
    }
    public void SetupCameras(Camera camera0, Camera camera1)
    {
        cameras[0] = camera0;
        cameras[1] = camera1;

        controls.Camera.LookBehind.performed += ctx =>
        {
            cameras[0].enabled = false;
            cameras[1].enabled = true;
        };

        controls.Camera.LookBehind.canceled += ctx =>
        {
            cameras[0].enabled = true;
            cameras[1].enabled = false;
        };
    }

    private void OnEnable()
    {
        controls.Enable();
    }

    private void OnDisable()
    {
        controls.Disable();
    }
}
