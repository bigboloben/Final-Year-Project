using UnityEngine;
using UnityEngine.Splines;
using System.Collections.Generic;

namespace Assets.TrackGeneration
{
    public class TrackMesh
    {
        private readonly TrackParameters parameters;
        private readonly SurfaceMeshBuilder meshBuilder;
        private readonly SplineProcessor splineProcessor;
        private readonly StartLineGenerator startLineGenerator;
        private readonly WallGenerator wallGenerator;

        public TrackMesh(TrackParameters parameters)
        {
            this.parameters = parameters;
            this.meshBuilder = new SurfaceMeshBuilder(parameters);
            this.splineProcessor = new SplineProcessor(parameters);
            this.startLineGenerator = new StartLineGenerator(parameters);
            this.wallGenerator = new WallGenerator(parameters);
        }

        public GameObject GenerateTrackMesh(
            List<SplineContainer> splines,
            Material trackMaterial,
            Material wallMaterial,
            PhysicsMaterial wallPhysicsMaterial,
            Material startLineMaterial,
            Material gridMarkerMaterial,
            out Vector3 startPos1,
            out Vector3 startPos2,
            out Quaternion startRot)
        {
            GameObject trackObject = new GameObject("Track");

            List<Vector3[]> splinePoints = splineProcessor.GeneratePointsFromSplines(splines);

            GameObject trackSurface = meshBuilder.GenerateTrackSurface(splinePoints, trackMaterial);
            //List<Checkpoint> checkpoints = CheckpointGenerator.GenerateCheckpoints(splinePoints[1], trackObject);

            var startLineInfo = startLineGenerator.CreateStartFinishLine(splinePoints[1], startLineMaterial);
            GameObject leftWall = wallGenerator.GenerateWall(splinePoints[0], wallMaterial, wallPhysicsMaterial);
            GameObject rightWall = wallGenerator.GenerateWall(splinePoints[2], wallMaterial, wallPhysicsMaterial);
            GameObject gridMarkers = startLineGenerator.CreateStartingGridMarkers(
                startLineInfo.Position1,
                startLineInfo.Position2,
                startLineInfo.StartRotation,
                gridMarkerMaterial);

            // Parent all objects to track
            trackSurface.transform.SetParent(trackObject.transform);
            leftWall.transform.SetParent(trackObject.transform);
            rightWall.transform.SetParent(trackObject.transform);
            startLineInfo.StartLine.transform.SetParent(trackObject.transform);
            gridMarkers.transform.SetParent(trackObject.transform);

            startPos1 = startLineInfo.Position1;
            startPos2 = startLineInfo.Position2;
            startRot = startLineInfo.StartRotation;

            return trackObject;
        }
    }
}
