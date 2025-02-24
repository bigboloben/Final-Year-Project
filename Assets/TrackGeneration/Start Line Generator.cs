using UnityEngine;

namespace Assets.TrackGeneration {
    public class StartLineGenerator
    {
        private readonly TrackParameters parameters;

        public StartLineGenerator(TrackParameters parameters)
        {
            this.parameters = parameters;
        }

        public StartLineInfo CreateStartFinishLine(Vector3[] centerPoints, GameObject startLinePrefab, GameObject startGridPrefab)
        {
            GameObject startLine = new GameObject("StartFinishLine");

            // Calculate starting positions and direction
            Vector3 gridCenterPosition = centerPoints[0];
            Vector3 gridCenterDirection = (centerPoints[1] - centerPoints[centerPoints.Length - 1]).normalized;
            Vector3 gridCenterRight = Vector3.Cross(Vector3.up, gridCenterDirection).normalized;

            float carSpacing = parameters.TrackWidth * 0.25f;

            Vector3 position1 = gridCenterPosition - gridCenterRight * carSpacing + Vector3.up * 0.1f - gridCenterDirection * 4f;
            Vector3 position2 = gridCenterPosition + gridCenterRight * carSpacing + Vector3.up * 0.1f - gridCenterDirection * 4f;

            Quaternion startRotation = Quaternion.LookRotation(gridCenterDirection, Vector3.up);

            GameObject line = CreateDecal(startLinePrefab, centerPoints[0] + gridCenterDirection * 4f, startRotation, "Start Line");
            GameObject grid1 = CreateDecal(startGridPrefab, position1, startRotation, "Start Grid 1");
            GameObject grid2 = CreateDecal(startGridPrefab, position2, startRotation, "Start Grid 2");

            line.transform.SetParent(startLine.transform);
            grid1.transform.SetParent(startLine.transform);
            grid2.transform.SetParent(startLine.transform);

            return new StartLineInfo(startLine, position1, position2, gridCenterDirection, startRotation);
        }

        public GameObject CreateDecal(GameObject prefab, Vector3 position, Quaternion rotation, string name)
        {
            // Rotate 90 degrees around X axis
            Quaternion xRotation = Quaternion.Euler(90, 0, 0);
            Quaternion finalRotation = rotation * xRotation;

            GameObject decal = Object.Instantiate(
                prefab,
                position + Vector3.up * 1.5f,
                finalRotation
            );
            decal.name = name;
            return decal;
        }

    }
}