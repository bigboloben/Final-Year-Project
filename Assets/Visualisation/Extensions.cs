using UnityEngine;
using UnityEngine.Events;

namespace Assets.TrackGeneration
{
    // First, let's create the extension class for the track handler
    public class TrackHandlerExtension : MonoBehaviour
    {
        public UnityEvent onTrackGenerated = new UnityEvent();
        private TrackHandler trackHandler;
        private bool trackWasGenerated = false;

        void Start()
        {
            trackHandler = GetComponent<TrackHandler>();
        }

        void Update()
        {
            // Check if track was just generated
            if (trackHandler.trackSpline != null && !trackWasGenerated)
            {
                trackWasGenerated = true;
                onTrackGenerated.Invoke();
            }

            // Reset flag if track is destroyed
            if (trackHandler.trackSpline == null && trackWasGenerated)
            {
                trackWasGenerated = false;
            }
        }
    }

    [RequireComponent(typeof(TrackHandler))]
    public class TrackVisualizationSetup : MonoBehaviour
    {
        private TrackHandler trackHandler;
        private TrackTestingVisualization visualization;

        void Start()
        {
            // Get the track handler reference
            trackHandler = GetComponent<TrackHandler>();

            // Create the visualization component
            GameObject visualizationObj = new GameObject("TrackVisualization");
            visualization = visualizationObj.AddComponent<TrackTestingVisualization>();
            visualization.trackHandler = trackHandler;

            // Initialize UI (ensure this happens before we try to use RefreshData)
            visualization.InitializeUI();

            // Add the extension component if it doesn't exist
            TrackHandlerExtension extension = gameObject.GetComponent<TrackHandlerExtension>();
            if (extension == null)
            {
                extension = gameObject.AddComponent<TrackHandlerExtension>();
            }

            // Subscribe to the track generated event
            extension.onTrackGenerated.AddListener(OnTrackGenerated);
        }

        private void OnTrackGenerated()
        {
            if (visualization != null)
            {
                // Use RefreshData instead of OnTrackGenerated
                visualization.RefreshData();
            }
        }

        private void OnDestroy()
        {
            // Clean up event subscription if needed
            TrackHandlerExtension extension = gameObject.GetComponent<TrackHandlerExtension>();
            if (extension != null)
            {
                extension.onTrackGenerated.RemoveListener(OnTrackGenerated);
            }
        }
    }
}