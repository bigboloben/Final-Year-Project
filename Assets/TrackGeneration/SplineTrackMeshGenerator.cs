using UnityEngine;
using UnityEngine.Splines;
using System.Collections.Generic;
using Unity.Mathematics;
using System;
using System.Diagnostics.CodeAnalysis;
//using UnityEngine.Rendering.PostProcessing;


namespace Assets.TrackGeneration
{
    public class SplineTrackMeshGenerator
    {
        private float trackWidth;
        private float wallHeight;
        private float wallDepth;
        private int segmentsPerUnit;
        private float bankingAngle;
        private const float TRACK_HEIGHT_OFFSET = 0.1f;  // Consistent height offset for the entire track
        List<Checkpoint> checkpoints;

        public class StartLineInfo
        {
            public GameObject startLine;
            public Vector3 position1; // First car position
            public Vector3 position2; // Second car position
            public Vector3 startDirection; // Direction cars should face
            public Quaternion startRotation; // Rotation for the cars
        }
        public SplineTrackMeshGenerator(float width, float height, float depth, int segments, float banking)
        {
            trackWidth = width;
            wallHeight = height;
            wallDepth = depth;
            segmentsPerUnit = segments;
            bankingAngle = banking;
        }

        public GameObject GenerateTrackMesh(List<SplineContainer> splines, Material trackMaterial, Material wallMaterial, PhysicsMaterial wallPhysicsMaterial, Material startLineMaterial, Material gridMarkerMaterial, out Vector3 startPos1, out Vector3 startPos2, out Quaternion startRot)
        {
            

            GameObject trackObject = new GameObject("Track");
            //List<Vector3[]> splinePoints = GenerateOffsetSplinesFromSplines(spline);
            List<Vector3[]> splinePoints = GeneratePointsFromSplines(splines); 
            GameObject trackSurface = GenerateTrackSurface(splinePoints, trackMaterial);
            //List<Checkpoint> checkpoints = CheckpointGenerator.GenerateCheckpoints(splinePoints[1], trackObject);
            StartLineInfo startLineInfo = CreateStartFinishLine(splinePoints[1], startLineMaterial);
            GameObject leftWall = GenerateWall(splinePoints[0], wallMaterial, wallPhysicsMaterial);
            GameObject rightWall = GenerateWall(splinePoints[2], wallMaterial, wallPhysicsMaterial);
            GameObject gridMarkers = CreateStartingGridMarkers(startLineInfo.position1, startLineInfo.position2, startLineInfo.startRotation, gridMarkerMaterial);

            trackSurface.transform.SetParent(trackObject.transform);
            leftWall.transform.SetParent(trackObject.transform);
            rightWall.transform.SetParent(trackObject.transform);
            startLineInfo.startLine.transform.SetParent(trackObject.transform);
            gridMarkers.transform.SetParent(trackObject.transform);

            startPos1 = startLineInfo.position1;
            startPos2 = startLineInfo.position2;
            startRot = startLineInfo.startRotation;

            return trackObject;
        }

        public List<Vector3[]> GeneratePointsFromSplines(List<SplineContainer> splines)
        {
            Spline leftSpline = splines[0].Spline;
            Spline centerSpline = splines[1].Spline;
            Spline rightSpline = splines[2].Spline;

            List<Vector3[]> splinePoints = new List<Vector3[]>();

            int numberOfPoints = Mathf.Max(32, (int)(centerSpline.GetLength() * segmentsPerUnit));
            Vector3 heightOffset = -Vector3.down * TRACK_HEIGHT_OFFSET;

            //Vector3[] leftPoints = GetPoints(leftSpline, numberOfPoints, heightOffset);
            //Vector3[] centerPoints = GetPoints(centerSpline, numberOfPoints, heightOffset);
            //Vector3[] rightPoints = GetPoints(rightSpline, numberOfPoints, heightOffset);

            Vector3[] leftPoints = new Vector3[numberOfPoints];
            Vector3[] centerPoints = new Vector3[numberOfPoints];
            Vector3[] rightPoints = new Vector3[numberOfPoints];






            float leftLength = leftSpline.GetLength();
            float centerLength = centerSpline.GetLength();
            float rightLength = rightSpline.GetLength();

            SplineArcLengthTable leftTable = new SplineArcLengthTable(leftSpline);
            SplineArcLengthTable centerTable = new SplineArcLengthTable(centerSpline);
            SplineArcLengthTable rightTable = new SplineArcLengthTable(rightSpline);

            for (int i = 0; i < numberOfPoints; i++)
            {
                float normalizedArcLength = i / (float)(numberOfPoints - 1); // 0 to 1

                float tLeft = leftTable.ConvertLengthToT(normalizedArcLength * leftLength);
                float tCenter = centerTable.ConvertLengthToT(normalizedArcLength * centerLength);
                float tRight = rightTable.ConvertLengthToT(normalizedArcLength * rightLength);


                float t = i / (float)(numberOfPoints - 1);  // This gives us values from 0 to 1
                Vector3 leftPosition = leftSpline.EvaluatePosition(tLeft);
                leftPosition += heightOffset;
                leftPoints[i] = leftPosition;

                Vector3 centerPosition = centerSpline.EvaluatePosition(tCenter);
                centerPosition += heightOffset;
                centerPoints[i] = centerPosition;

                Vector3 rightPosition = rightSpline.EvaluatePosition(tRight);
                rightPosition += heightOffset;
                rightPoints[i] = rightPosition;
            }

            splinePoints.Add(leftPoints);
            splinePoints.Add(centerPoints);
            splinePoints.Add(rightPoints);


            return splinePoints;
        }

        public Vector3[] GetPoints(Spline spline, int numberOfPoints, Vector3 heightOffset)
        {
            Vector3[] points = new Vector3[numberOfPoints];
            float step = 1f / numberOfPoints;
            int i = 0;
            for (float t = 0; t <= 1; t += step, i++)
            {
                Vector3 position = spline.EvaluatePosition(t);
                position += heightOffset;
                points[i] = position;
            }
            return points;
        }



        public GameObject CreateStartingGridMarkers(Vector3 pos1, Vector3 pos2, Quaternion rotation, Material material)
        {
            // Create containers for the grid markers
            GameObject gridMarkersContainer = new GameObject("GridMarkers");
            

            CreateGridMarker(pos1, rotation, gridMarkersContainer, "GridMarker1", material);
            CreateGridMarker(pos2, rotation, gridMarkersContainer, "GridMarker2", material);

            return gridMarkersContainer;
        }

        private void CreateGridMarker(Vector3 position, Quaternion rotation, GameObject parent, string name, Material material)
        {
            GameObject markerObject = new GameObject(name);
            markerObject.transform.SetParent(parent.transform);

            // Create the mesh for the grid marker
            Mesh mesh = new Mesh();

            // Parameters for marker size
            float width = 1f;      // Width of the marker
            float length = 2f;     // Length of the marker
            float thickness = width * 0.25f;              // Thickness of the lines
            float heightOffset = 0f;          // Slight offset to prevent z-fighting

            // Get directional vectors
            Vector3 forward = rotation * Vector3.forward;
            Vector3 right = rotation * Vector3.right;
            Vector3 up = Vector3.up;

            // Create vertices for the L-shaped marker
            Vector3[] vertices = new Vector3[]
            {
                // Forward line
                position + heightOffset * up + (width + thickness) * -right + length * forward,                    // 0 bottom left   2             3
                position + heightOffset * up + (width + thickness) * right + length * forward,                    // 1 bottom right     -----------
                position + heightOffset * up + (width + thickness) * -right + (length + thickness) * forward,    // 2 top left          |         |
                position + heightOffset * up + (width + thickness) * right + (length + thickness) * forward,    // 3 top right        0 ___________ 1

                // Left Side line
                position + heightOffset * up + (width + thickness) * -right + length/2 * -forward,                 // 4 bottom left    6 ---- 7
                position + heightOffset * up + width * -right + length/2 * -forward,                              // 5 bottom right      |  |
                position + heightOffset * up + (width + thickness) * -right + (length + thickness) * forward,    // 6 top left           |  |
                position + heightOffset * up + width * -right + (length + thickness) * forward,                 // 7 top right         4 ---- 5
                
                // Right Side Line
                position + heightOffset * up + width * right + length/2 * -forward,                                // 8 bottom left   10 ---- 11
                position + heightOffset * up + (width + thickness) * right + length/2 * -forward,                 // 9 bottom right      |  |
                position + heightOffset * up + width * right + (length + thickness) * forward,                   // 10 top left          |  |
                position + heightOffset * up + (width + thickness) * right + (length + thickness) * forward     // 11 top right        8 ---- 9


            };

            // Define triangles
            int[] triangles = new int[]
            {
            // Forward line
                0, 2, 1,
                1, 2, 3,
        
            // Left Side line
                4, 6, 5,
                5, 6, 7,

            // Right Side Line
                8, 10, 9,
                9, 10, 11
            };

            // Create UVs for texture mapping
            Vector2[] uvs = new Vector2[]
            {
                new Vector2(0, 0), new Vector2(1, 0), new Vector2(0, 1), new Vector2(1, 1),
                new Vector2(0, 0), new Vector2(1, 0), new Vector2(0, 1), new Vector2(1, 1),
                new Vector2(0, 0), new Vector2(1, 0), new Vector2(0, 1), new Vector2(1, 1)
            };

            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.uv = uvs;
            mesh.RecalculateNormals();

            // Add mesh components
            MeshFilter meshFilter = markerObject.AddComponent<MeshFilter>();
            meshFilter.mesh = mesh;

            MeshRenderer meshRenderer = markerObject.AddComponent<MeshRenderer>();
            //Material markerMaterial = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            material.SetTexture("_BaseMap" ,CreateGridMarkerTexture());
            meshRenderer.material = material;
        }

        private Texture2D CreateGridMarkerTexture()
        {
            int textureSize = 64;
            Texture2D texture = new Texture2D(textureSize, textureSize);
            Color gridColor = new Color(0.9f, 0.9f, 0.9f); // Slightly off-white for better visibility

            for (int y = 0; y < textureSize; y++)
            {
                for (int x = 0; x < textureSize; x++)
                {
                    texture.SetPixel(x, y, gridColor);
                }
            }

            texture.Apply();
            texture.filterMode = FilterMode.Bilinear;
            return texture;
        }

        private List<Vector3[]> GenerateOffsetSplinesFromSplines(SplineContainer centerSplineContainer)
        {
            Spline centerSpline = centerSplineContainer.Spline;

            GameObject leftSplineObject = new GameObject("LeftSpline");
            GameObject rightSplineObject = new GameObject("RightSpline");
            GameObject centerSplineObject = new GameObject("CenterSpline");
            SplineContainer leftSplineContainer = leftSplineObject.AddComponent<SplineContainer>();
            SplineContainer splineContainer = centerSplineObject.AddComponent<SplineContainer>();
            SplineContainer rightSplineContainer = rightSplineObject.AddComponent<SplineContainer>();


            //Get references to the new splines
            Spline leftSpline = leftSplineContainer.Spline;
            Spline rightSpline = rightSplineContainer.Spline;
            Spline spline = splineContainer.Spline;

            //Spline leftSpline = new Spline();
            //Spline rightSpline = new Spline();

            // Clear any default knots
            leftSpline.Clear();
            rightSpline.Clear();
            spline.Clear();

            List<int> leftDeleted = new List<int>();
            List<int> rightDeleted = new List<int>();

            float curveThreshold = 0.24f;

            for (int i = 0; i < centerSpline.Count; i++)
            {
                // Create knots for both sides
                BezierKnot leftKnot = centerSpline[i];
                BezierKnot nextLeftKnot = centerSpline[(i + 1) % centerSpline.Count];
                BezierKnot prevLeftKnot = centerSpline[(i - 1 + centerSpline.Count) % centerSpline.Count];
                BezierKnot rightKnot = centerSpline[i];
                BezierKnot nextRightKnot = centerSpline[(i + 1) % centerSpline.Count];
                BezierKnot prevRightKnot = centerSpline[(i - 1 + centerSpline.Count) % centerSpline.Count];


                // Calculate offset directions
                Vector3 tangent = ((Vector3)leftKnot.TangentOut).normalized;
                Vector3 up = Vector3.up;
                float3 rightVector = Vector3.Cross(up, tangent).normalized;

                Vector3 nextTangent = ((Vector3)nextLeftKnot.TangentOut).normalized;
                float3 nextRightVector = Vector3.Cross(up, nextTangent).normalized;

                Vector3 prevTangent = ((Vector3)prevLeftKnot.TangentOut).normalized;
                float3 prevRightVector = Vector3.Cross(up, prevTangent).normalized;

                // Apply offsets
                leftKnot.Position -= rightVector * trackWidth / 2;
                rightKnot.Position += rightVector * trackWidth / 2;

                nextLeftKnot.Position -= nextRightVector * trackWidth / 2;
                nextRightKnot.Position += nextRightVector * trackWidth / 2;

                prevLeftKnot.Position -= prevRightVector * trackWidth / 2;
                prevRightKnot.Position += prevRightVector * trackWidth / 2;

                Vector3 averageTangent = GetTangent(centerSpline, i);
                Vector3 curvature = GetCurvature(centerSpline, i);

                // Cross product to determine turning direction
                float direction = Vector3.Cross(averageTangent, curvature).y;

                float nextLeftDistance = Vector3.Distance(leftKnot.Position, nextLeftKnot.Position);
                float nextRightDistance = Vector3.Distance(rightKnot.Position, nextRightKnot.Position);
                float nextOriginalDistance = Vector3.Distance(centerSpline[i].Position, centerSpline[(i+1)%centerSpline.Count].Position);

                float outLeftScale = nextLeftDistance / nextOriginalDistance;
                float outRightScale = nextRightDistance / nextOriginalDistance;

                float prevLeftDistance = Vector3.Distance(leftKnot.Position, prevLeftKnot.Position);
                float prevRightDistance = Vector3.Distance(rightKnot.Position, prevRightKnot.Position);
                float prevOriginalDistance = Vector3.Distance(centerSpline[i].Position, centerSpline[(i - 1 + centerSpline.Count) % centerSpline.Count].Position);

                float inLeftScale = prevLeftDistance / prevOriginalDistance;
                float inRightScale = prevRightDistance / prevOriginalDistance;

                Debug.LogError($"knot {i} next distances {nextLeftDistance}, {nextRightDistance}, {nextOriginalDistance}");
                Debug.LogError($"knot {i} prev distances {prevLeftDistance}, {prevRightDistance}, {prevOriginalDistance}");
                Debug.LogError($"knot {i} curvature {curvature} magnitude {curvature.magnitude}");

                leftKnot.TangentIn *= inLeftScale;
                leftKnot.TangentOut *= outLeftScale;

                rightKnot.TangentIn *= inRightScale;
                rightKnot.TangentOut *= inRightScale;

                bool rightAuto = false;
                bool leftAuto = false;

                if (direction < 0 && curvature.magnitude > curveThreshold)
                    {
                        float inLength = ((Vector3)rightKnot.TangentIn).magnitude;
                        float outLength = ((Vector3)rightKnot.TangentOut).magnitude;

                        float3 shiftDirection;
                        float shiftDistance;

                        if (inLength < outLength)
                        {
                            // Shift towards previous point
                            shiftDirection = ((Vector3)prevRightKnot.Position - (Vector3)rightKnot.Position).normalized;
                            shiftDistance = inLength * 0.1f;
                            rightKnot.Position += shiftDirection * shiftDistance;
                        

                            
                        }
                        else
                        {
                            // Shift towards next point
                            shiftDirection = ((Vector3)nextRightKnot.Position - (Vector3)rightKnot.Position).normalized;
                            shiftDistance = outLength * 0.1f;
                            rightKnot.Position += shiftDirection * shiftDistance;

                           
                        }
                    rightKnot.TangentIn *= 0.5f;
                    rightKnot.TangentOut *= 0.5f;

                    rightAuto = true;

                    leftKnot.TangentIn *= 1.2f;
                    leftKnot.TangentOut *= 1.2f;

                    LimitTangentLength(ref rightKnot, prevRightKnot.Position, nextRightKnot.Position);
                    PreventTangentCrossover(ref rightKnot);
                }
                else if (direction > 0 && curvature.magnitude > curveThreshold)
                    {
                        float inLength = ((Vector3)leftKnot.TangentIn).magnitude;
                        float outLength = ((Vector3)leftKnot.TangentOut).magnitude;

                        float3 shiftDirection;
                        float shiftDistance;

                        if (inLength < outLength)
                        {
                            // Shift towards previous point
                            shiftDirection = ((Vector3)prevLeftKnot.Position - (Vector3)leftKnot.Position).normalized;
                            shiftDistance = inLength * 0.2f; // Adjust factor as needed
                            leftKnot.Position += shiftDirection * shiftDistance;

                            
                        }
                        else
                        {
                            // Shift towards next point
                            shiftDirection = ((Vector3)nextLeftKnot.Position - (Vector3)leftKnot.Position).normalized;
                            shiftDistance = outLength * 0.2f;
                            leftKnot.Position += shiftDirection * shiftDistance;

                           
                        }
                    leftKnot.TangentIn *= 0.5f;
                    leftKnot.TangentOut *= 0.5f;

                    leftAuto = true;

                    rightKnot.TangentIn *= 1.2f;
                    rightKnot.TangentOut *= 1.2f;

                    LimitTangentLength(ref leftKnot, prevLeftKnot.Position, nextLeftKnot.Position);
                    PreventTangentCrossover(ref leftKnot);
                }
                else if (direction < 0 && curvature.magnitude < curveThreshold)
                {
                    rightKnot.TangentIn *= 0.9f;
                    rightKnot.TangentOut *= 0.9f;

                    leftKnot.TangentIn += 1.1f;
                    leftKnot.TangentOut += 1.1f;

                }
                else if (direction > 0 && curvature.magnitude < curveThreshold)
                {
                    rightKnot.TangentIn *= 1.1f;
                    rightKnot.TangentOut *= 1.1f;

                    leftKnot.TangentIn *= 0.9f;
                    leftKnot.TangentOut *= 0.9f;
                    
                }



                // Add knots to the new splines

                spline.Add(centerSpline[i]);


                if (rightAuto) 
                {
                    leftSpline.Add(leftKnot);
                    rightDeleted.Add(i);

                    


                }
                else if (leftAuto) 
                {
                    rightSpline.Add(rightKnot);
                    leftDeleted.Add(i);

                    
                }
                else
                {
                    rightSpline.Add(rightKnot);
                    leftSpline.Add(leftKnot);
                }

            }
            leftSpline.Closed = true;
            rightSpline.Closed = true;
            centerSpline.Closed = true;

            foreach (int i in leftDeleted)
            {
                Debug.LogError($"deleted {i}");
                BezierKnot tempPrevLeft = leftSpline[(i - 1 + leftSpline.Count) % leftSpline.Count];
                tempPrevLeft.TangentOut *= 1.7f;
                leftSpline[(i - 1 + leftSpline.Count) % leftSpline.Count] = tempPrevLeft;
                BezierKnot tempNextLeft = leftSpline[(i)%leftSpline.Count];
                tempNextLeft.TangentIn *= 1.7f;
                leftSpline[(i) % leftSpline.Count] = tempNextLeft;
            }
            foreach (int i in rightDeleted)
            {
                Debug.LogError($"deleted {i}");
                BezierKnot tempPrevRight = rightSpline[(i - 1 + rightSpline.Count) % rightSpline.Count];
                tempPrevRight.TangentOut *= 1.7f;
                rightSpline[(i - 1 + rightSpline.Count) % rightSpline.Count] = tempPrevRight;
                BezierKnot tempNextRight = rightSpline[(i) % rightSpline.Count];
                tempNextRight.TangentIn *= 1.7f;
                rightSpline[(i) % rightSpline.Count] = tempNextRight;
            }



            List<Vector3[]> splinePoints = new List<Vector3[]>();

            int numberOfPoints = Mathf.Max(32, (int)(centerSpline.GetLength() * segmentsPerUnit));
            Vector3[] leftPoints = new Vector3[numberOfPoints];
            Vector3[] centerPoints = new Vector3[numberOfPoints];
            Vector3[] rightPoints = new Vector3[numberOfPoints];

            Vector3 heightOffset = -Vector3.down * TRACK_HEIGHT_OFFSET;
            for (int i = 0; i < numberOfPoints; i++)
            {
                float t = i / (float)(numberOfPoints - 1);  // This gives us values from 0 to 1
                Vector3 leftPosition = leftSpline.EvaluatePosition(t);
                leftPosition += heightOffset;
                leftPoints[i] = leftPosition;

                Vector3 centerPosition = centerSpline.EvaluatePosition(t);
                centerPosition += heightOffset;
                centerPoints[i] = centerPosition;

                Vector3 rightPosition = rightSpline.EvaluatePosition(t);
                rightPosition += heightOffset;
                rightPoints[i] = rightPosition;
            }

            splinePoints.Add(leftPoints);
            splinePoints.Add(centerPoints);
            splinePoints.Add(rightPoints);


            return splinePoints;
        }

        void LimitTangentLength(ref BezierKnot knot, Vector3 prevPos, Vector3 nextPos)
        {
            float distToPrev = Vector3.Distance(knot.Position, prevPos);
            float distToNext = Vector3.Distance(knot.Position, nextPos);

            float maxInLength = distToPrev * 0.33f;  // Use 1/3 of distance as max length
            float maxOutLength = distToNext * 0.33f;

            if (((Vector3)knot.TangentIn).magnitude > maxInLength)
            {
                knot.TangentIn = ((Vector3)knot.TangentIn).normalized * maxInLength;
            }

            if (((Vector3)knot.TangentOut).magnitude > maxOutLength)
            {
                knot.TangentOut = ((Vector3)knot.TangentOut).normalized * maxOutLength;
            }
        }

        void PreventTangentCrossover(ref BezierKnot knot)
        {
            Vector3 inDir = ((Vector3)knot.TangentIn).normalized;
            Vector3 outDir = ((Vector3)knot.TangentOut).normalized;

            float dot = Vector3.Dot(inDir, outDir);
            if (dot < -0.7f)  // Tangents are pointing in nearly opposite directions
            {
                // Reduce both tangents to prevent crossover
                float reduction = 0.5f;
                knot.TangentIn *= reduction;
                knot.TangentOut *= reduction;
            }
        }

        Vector3 GetTangent(Spline spline, int index)
        {
            int prevIndex = (index - 1 + spline.Count) % spline.Count;
            int nextIndex = (index + 1) % spline.Count;

            Vector3 prevPos = spline[prevIndex].Position;
            Vector3 nextPos = spline[nextIndex].Position;

            return (nextPos - prevPos).normalized; // Approximate tangent
        }

        Vector3 GetCurvature(Spline spline, int index)
        {
            BezierKnot currentKnot = spline[index];
            BezierKnot nextKnot = spline[(index + 1) % spline.Count];
            BezierKnot prevKnot = spline[(index - 1 + spline.Count) % spline.Count]; // Handle looping

            // Compute direct vectors
            Vector3 directNext = nextKnot.Position - currentKnot.Position;
            Vector3 directPrev = currentKnot.Position - prevKnot.Position;

            float directNextDist = directNext.magnitude;
            float directPrevDist = directPrev.magnitude;

            // Get control point deviations (outgoing and incoming)
            float3 outControl = currentKnot.Position + currentKnot.TangentOut;
            float3 inControlNext = nextKnot.Position + nextKnot.TangentIn;
            float3 inControlCurrent = currentKnot.Position + currentKnot.TangentIn;
            float3 outControlPrev = prevKnot.Position + prevKnot.TangentOut;

            // Compute deviations from the direct paths
            Vector3 normalizedNextPath = directNext.normalized;
            Vector3 normalizedPrevPath = directPrev.normalized;

            float outDeviation = Vector3.Cross(outControl - currentKnot.Position, normalizedNextPath).magnitude;
            float inDeviationNext = Vector3.Cross(inControlNext - nextKnot.Position, normalizedNextPath).magnitude;
            float inDeviationPrev = Vector3.Cross(inControlCurrent - currentKnot.Position, normalizedPrevPath).magnitude;
            float outDeviationPrev = Vector3.Cross(outControlPrev - prevKnot.Position, normalizedPrevPath).magnitude;

            // Average the curvature contribution from both sides
            float curvatureMagnitude =
                ((outDeviation + inDeviationNext) / directNextDist + (outDeviationPrev + inDeviationPrev) / directPrevDist) * 0.5f;

            // Direction from the midpoint to the control point averages
            Vector3 midPoint = (prevKnot.Position + nextKnot.Position) * 0.5f;
            Vector3 controlMidPoint = (outControl + inControlNext + inControlCurrent + outControlPrev) * 0.25f;

            return (controlMidPoint - midPoint).normalized * curvatureMagnitude;
        }

        private List<Vector3[]> GenerateOffsetSplines(SplineContainer centerSpline)
        {
            List<Vector3[]> splinePoints = new List<Vector3[]>();
            int pointCount = Mathf.Max(32, (int)(centerSpline.Spline.GetLength() * segmentsPerUnit));
            Vector3[] leftPoints = new Vector3[pointCount];
            Vector3[] centerPoints = new Vector3[pointCount];
            Vector3[] rightPoints = new Vector3[pointCount];
            // Get initial spline information
            float baseHeight = centerSpline.EvaluatePosition(0).y;
            float leftWidth = trackWidth;
            float rightWidth = trackWidth;
            for (int i = 0; i < pointCount; i++)
            {
                float t = i / (float)(pointCount - 1);
                Vector3 position = centerSpline.EvaluatePosition(t);
                Vector3 tangent = ((Vector3)centerSpline.EvaluateTangent(t)).normalized;
                Vector3 up = Vector3.up;
                Vector3 right = Vector3.Cross(tangent, up).normalized;

                float curvature = CalculateCurvature(centerSpline, t);
                //// Apply banking rotation based on curvature
                //float bankAngle = curvature * bankingAngle;
                //Quaternion bankRotation = Quaternion.AngleAxis(bankAngle, tangent);
                //// Rotate the right and up vectors
                //right = bankRotation * right;
                //up = bankRotation * up;
                //if (curvature < 0)
                //{
                //    leftWidth = trackWidth + (Mathf.Abs(curvature) * 0.5f);
                //    rightWidth = trackWidth;
                //}
                //else if (curvature > 0)
                //{
                //    rightWidth = trackWidth + (Mathf.Abs(curvature) * 0.5f);
                //    leftWidth = trackWidth;
                //}
                //else
                //{
                //    rightWidth = trackWidth;
                //    leftWidth = trackWidth;
                //}
                // Apply the consistent height offset and banking
                Vector3 heightOffset = up * TRACK_HEIGHT_OFFSET;
                centerPoints[i] = position + heightOffset;
                leftPoints[i] = position - right * (leftWidth / 2) + heightOffset;
                rightPoints[i] = position + right * (rightWidth / 2) + heightOffset;
            }
            splinePoints.Add(leftPoints);
            splinePoints.Add(centerPoints);
            splinePoints.Add(rightPoints);
            return splinePoints;
        }


        //private float CalculateCurvature(SplineContainer spline, float t)
        //{
        //    // Use three points to calculate curvature
        //    float delta = 0.01f;  // Larger delta for better stability

        //    // Get three points along the curve
        //    Vector3 p0 = spline.EvaluatePosition(Mathf.Max(0, t - delta));
        //    Vector3 p1 = spline.EvaluatePosition(t);
        //    Vector3 p2 = spline.EvaluatePosition(Mathf.Min(1, t + delta));

        //    // Calculate vectors between points
        //    Vector3 v1 = p1 - p0;
        //    Vector3 v2 = p2 - p1;

        //    // Calculate the change in direction
        //    float angle = Vector3.SignedAngle(v1, v2, Vector3.up);

        //    // Convert angle to curvature (sign indicates left/right turn)
        //    float curvature = angle / (delta * spline.Spline.GetLength());

        //    return curvature; // Convert to radians
        //}

        private float CalculateCurvature(SplineContainer spline, float t)
        {
            // Calculate the first derivative (tangent)
            Vector3 firstDerivative = ((Vector3)spline.EvaluateTangent(t)).normalized;

            // Calculate a small step for numerical differentiation
            float deltaT = 0.01f;

            // Calculate the second derivative
            Vector3 tangentBefore = ((Vector3)spline.EvaluateTangent(t - deltaT)).normalized;
            Vector3 tangentAfter = ((Vector3)spline.EvaluateTangent(t + deltaT)).normalized;
            Vector3 secondDerivative = (tangentAfter - tangentBefore) / (2 * deltaT);

           
            float curvature = secondDerivative.magnitude / Mathf.Pow(firstDerivative.magnitude, 3);

            return curvature;
        }
        private float[] SmoothCurvatureValues(float[] curvatures, int smoothingRadius = 2)
        {
            float[] smoothedCurvatures = new float[curvatures.Length];
            for (int i = 0; i < curvatures.Length; i++)
            {
                float sum = 0f;
                int count = 0;
                for (int j = -smoothingRadius; j <= smoothingRadius; j++)
                {
                    int index = i + j;
                    if (index >= 0 && index < curvatures.Length)
                    {
                        sum += curvatures[index];
                        count++;
                    }
                }
                smoothedCurvatures[i] = sum / count;
            }
            return smoothedCurvatures;
        }
       
        //private float CalculateCurvature(SplineContainer spline, float t)
        //{
        //    // Get position and derivatives
        //    Vector3 p1 = spline.EvaluatePosition(t);
        //    Vector3 v1 = spline.EvaluateTangent(t);

        //    // Use a small delta to approximate second derivative
        //    float delta = 0.001f;
        //    Vector3 v2 = spline.EvaluateTangent(t + delta);

        //    // Calculate curvature 
        //    Vector3 acceleration = (v2 - v1) / delta;
        //    float curvature = Vector3.Cross(v1, acceleration).magnitude / Mathf.Pow(v1.magnitude, 3);

        //    return curvature;
        //}


        private StartLineInfo CreateStartFinishLine(Vector3[] centerPoints, Material material)
        {
            StartLineInfo startInfo = new StartLineInfo();

            // Create the start line mesh as before
            GameObject startLine = new GameObject("StartFinishLine");
            Mesh mesh = new Mesh();

            // Calculate directions and positions
            Vector3 position = centerPoints[0];
            Vector3 direction = (centerPoints[1] - centerPoints[0]).normalized;
            Vector3 right = Vector3.Cross(Vector3.up, direction).normalized;
            Vector3 up = Vector3.Cross(direction, right).normalized;

            float lineWidth = trackWidth;
            float lineLength = 2f;
            float heightOffset = 0.001f;

            // Create start line vertices
            Vector3[] vertices = new Vector3[4];
            vertices[0] = position + (heightOffset * up) - (right * lineWidth / 2);
            vertices[1] = position + (heightOffset * up) + (right * lineWidth / 2);
            vertices[2] = position + (heightOffset * up) + (direction * lineLength) - (right * lineWidth / 2);
            vertices[3] = position + (heightOffset * up) + (direction * lineLength) + (right * lineWidth / 2);

            int[] triangles = new int[]
            {
                0, 2, 1,
                1, 2, 3
            };

            Vector2[] uvs = new Vector2[]
            {
                new Vector2(0, 0),
                new Vector2(8, 0),
                new Vector2(0, 1),
                new Vector2(8, 1)
            };

            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.uv = uvs;
            mesh.RecalculateNormals();

            MeshFilter meshFilter = startLine.AddComponent<MeshFilter>();
            meshFilter.mesh = mesh;

            MeshRenderer meshRenderer = startLine.AddComponent<MeshRenderer>();
            //Material checkerMaterial = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            material.SetTexture("_BaseMap", CreateStartLineTexture());
            meshRenderer.material = material;

            // Calculate starting positions
            float carSpacing = trackWidth * 0.25f; // Space between cars
            
            // Calculate positions behind the start line
            Vector3 gridCenterPosition = centerPoints[centerPoints.Length - 20]; // Move back from start line
            Vector3 gridCenterDirection = (centerPoints[centerPoints.Length - 19] - centerPoints[centerPoints.Length - 21]).normalized;
            Vector3 gridCenterRight = Vector3.Cross(Vector3.up, direction).normalized;
            Vector3 gridCenterUp = Vector3.Cross(direction, right).normalized;
            startInfo.position1 = gridCenterPosition - gridCenterRight * carSpacing; // Left position
            startInfo.position2 = gridCenterPosition + gridCenterRight * carSpacing; // Right position

            // Add slight height offset to prevent cars from clipping
            startInfo.position1 += Vector3.up * 0.001f;
            startInfo.position2 += Vector3.up * 0.001f;

            // Store direction and rotation
            startInfo.startDirection = gridCenterDirection;
            startInfo.startRotation = Quaternion.LookRotation(gridCenterDirection, Vector3.up);
            startInfo.startLine = startLine;

            return startInfo;
        }

        private Texture2D CreateStartLineTexture()
        {
            int textureSize = 256;
            int squareSize = 128; // Size of each checker square
            Texture2D texture = new Texture2D(textureSize, textureSize);

            for (int y = 0; y < textureSize; y++)
            {
                for (int x = 0; x < textureSize; x++)
                {
                    bool isAlternate = ((x / squareSize) + (y / squareSize)) % 2 == 0;
                    texture.SetPixel(x, y, isAlternate ? Color.white : Color.black);
                }
            }

            texture.Apply();
            texture.filterMode = FilterMode.Point;
            texture.wrapMode = TextureWrapMode.Repeat;
            return texture;
        }

        private GameObject GenerateTrackSurface(List<Vector3[]> splinePoints, Material material)
        {

            GameObject surfaceObject = new GameObject("TrackSurface");
            surfaceObject.tag = "Floor";
            MeshFilter meshFilter = surfaceObject.AddComponent<MeshFilter>();
            MeshRenderer meshRenderer = surfaceObject.AddComponent<MeshRenderer>();
            meshRenderer.material = material;

            Mesh mesh = new Mesh();
            Vector3[] vertices = new Vector3[splinePoints[0].Length * 2];
            int[] triangles = new int[(splinePoints[0].Length - 1) * 6];
            Vector2[] uvs = new Vector2[vertices.Length];

            // Generate vertices
            for (int i = 0; i < splinePoints[0].Length; i++)
            {
                vertices[i * 2] = splinePoints[0][i];     // Left edge
                vertices[i * 2 + 1] = splinePoints[2][i]; // Right edge

                float u = i / (float)(splinePoints[0].Length - 1);
                uvs[i * 2] = new Vector2(0, u);
                uvs[i * 2 + 1] = new Vector2(1, u);
            }

            // Generate triangles with correct winding order
            int triIndex = 0;
            for (int i = 0; i < splinePoints[0].Length - 1; i++)
            {
                int baseIndex = i * 2;
                int nextBaseIndex = (i + 1) * 2;

                triangles[triIndex++] = baseIndex;
                triangles[triIndex++] = baseIndex + 1;
                triangles[triIndex++] = nextBaseIndex;

                triangles[triIndex++] = baseIndex + 1;
                triangles[triIndex++] = nextBaseIndex + 1;
                triangles[triIndex++] = nextBaseIndex;
            }

            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.uv = uvs;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            meshFilter.mesh = mesh;

            // Add MeshCollider with smooth physics settings
            MeshCollider collider = surfaceObject.AddComponent<MeshCollider>();
            collider.sharedMesh = mesh;
            //collider.convex = true;

            return surfaceObject;
        }
        private Vector3 GetRightDirection(Vector3[] points, int currentPointIndex)
        {
            // Get previous and next points (wrap around for closed loop)
            int prevIndex = (currentPointIndex - 1 + points.Length) % points.Length;
            int nextIndex = (currentPointIndex + 1) % points.Length;

            // Calculate tangent using both previous and next points
            Vector3 tangent = (points[nextIndex] - points[prevIndex]).normalized;

            // Cross product with up vector to get right direction
            Vector3 right = Vector3.Cross(Vector3.up, tangent);

            return right;
        }

        private Material CreateWallTexture(Material wallMaterial)
        {
            Color color1 = Color.red;
            Color color2 = Color.white;

            Texture2D texture = new Texture2D(512, 512);
            for (int y = 0; y < texture.height; y++)
            {
                for (int x = 0; x < texture.width; x++)
                {
                    // Changed from x to y to make stripes vertical
                    bool isFirstColor = (x/256) % 2 == 0;
                    Color color = isFirstColor ? color1 : color2;
                    texture.SetPixel(x, y, color);
                }
            }
            texture.Apply();
            texture.wrapMode = TextureWrapMode.Repeat;

            //Material wallMaterial = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            wallMaterial.SetTexture("_BaseMap", texture);
            return wallMaterial;
        }
        private GameObject GenerateWall(Vector3[] edgePoints, Material material, PhysicsMaterial physics)
        {
            GameObject wallObject = new GameObject("Wall3D");
            MeshFilter meshFilter = wallObject.AddComponent<MeshFilter>();
            MeshRenderer meshRenderer = wallObject.AddComponent<MeshRenderer>();
            meshRenderer.material = CreateWallTexture(material);
            Mesh mesh = new Mesh();

            int numPoints = edgePoints.Length;
            // We need 16 vertices per segment (4 vertices per face * 4 faces)
            List<Vector3> vertices = new List<Vector3>();
            List<Vector2> uvs = new List<Vector2>();
            List<Vector3> normals = new List<Vector3>();

            if (numPoints >= 16383)
            {
                Debug.LogError($"Number of points too high {numPoints}");
            }

            // Calculate total wall length for UV scaling
            float totalLength = 0;
            for (int i = 1; i < numPoints; i++)
            {
                totalLength += Vector3.Distance(edgePoints[i], edgePoints[i - 1]);
            }

            float currentDistance = 0f;
            List<int> triangles = new List<int>();

            for (int i = 0; i < numPoints - 1; i++)
            {
                Vector3 right = GetRightDirection(edgePoints, i);
                Vector3 nextRight = GetRightDirection(edgePoints, i + 1);

                // Current segment vertices
                Vector3 bottomRightCurrent = edgePoints[i] + right * wallDepth;
                Vector3 topRightCurrent = edgePoints[i] + right * wallDepth + Vector3.up * wallHeight;
                Vector3 topLeftCurrent = edgePoints[i] + right * -wallDepth + Vector3.up * wallHeight;
                Vector3 bottomLeftCurrent = edgePoints[i] + right * -wallDepth;

                // Next segment vertices
                Vector3 bottomRightNext = edgePoints[i + 1] + nextRight * wallDepth;
                Vector3 topRightNext = edgePoints[i + 1] + nextRight * wallDepth + Vector3.up * wallHeight;
                Vector3 topLeftNext = edgePoints[i + 1] + nextRight * -wallDepth + Vector3.up * wallHeight;
                Vector3 bottomLeftNext = edgePoints[i + 1] + nextRight * -wallDepth;

                currentDistance += Vector3.Distance(edgePoints[i], edgePoints[i + 1]);
                float uCurrent = currentDistance / totalLength;
                float uPrev = (i > 0) ? (currentDistance - Vector3.Distance(edgePoints[i], edgePoints[i + 1])) / totalLength : 0f;

                int baseIndex = vertices.Count;

                // Right face
                AddQuad(vertices, uvs, normals, triangles, baseIndex,
                    bottomRightCurrent, topRightCurrent, bottomRightNext, topRightNext,
                    uPrev, uCurrent, right);

                // Top face
                AddQuad(vertices, uvs, normals, triangles, baseIndex + 4,
                    topRightCurrent, topLeftCurrent, topRightNext, topLeftNext,
                    uPrev, uCurrent, Vector3.up);

                // Left face
                AddQuad(vertices, uvs, normals, triangles, baseIndex + 8,
                    bottomLeftNext, topLeftNext, bottomLeftCurrent, topLeftCurrent,
                    uCurrent, uPrev, -right);

                // Bottom face
                AddQuad(vertices, uvs, normals, triangles, baseIndex + 12,
                    bottomRightCurrent, bottomLeftCurrent, bottomRightNext, bottomLeftNext,
                    uPrev, uCurrent, -Vector3.up);
            }

            mesh.SetVertices(vertices);
            mesh.SetUVs(0, uvs);
            mesh.SetNormals(normals);
            mesh.SetTriangles(triangles.ToArray(), 0);
            mesh.RecalculateBounds();

            meshFilter.mesh = mesh;

            MeshCollider collider = wallObject.AddComponent<MeshCollider>();
            collider.sharedMesh = mesh;
            collider.material = physics;

            wallObject.layer = LayerMask.NameToLayer("Outlines");

            return wallObject;
        }

        private void AddQuad(
            List<Vector3> vertices,
            List<Vector2> uvs,
            List<Vector3> normals,
            List<int> triangles,
            int baseIndex,
            Vector3 bottomLeft,
            Vector3 topLeft,
            Vector3 bottomRight,
            Vector3 topRight,
            float uMin,
            float uMax,
            Vector3 normal)
        {
            // Add vertices
            vertices.Add(bottomLeft);
            vertices.Add(topLeft);
            vertices.Add(bottomRight);
            vertices.Add(topRight);

            // Add UVs
            uvs.Add(new Vector2(uMin, 0));
            uvs.Add(new Vector2(uMin, 1));
            uvs.Add(new Vector2(uMax, 0));
            uvs.Add(new Vector2(uMax, 1));

            // Add normals
            for (int i = 0; i < 4; i++)
            {
                normals.Add(normal);
            }

            // Add triangles
            triangles.Add(baseIndex);
            triangles.Add(baseIndex + 1);
            triangles.Add(baseIndex + 2);
            triangles.Add(baseIndex + 1);
            triangles.Add(baseIndex + 3);
            triangles.Add(baseIndex + 2);
        }
    }
}