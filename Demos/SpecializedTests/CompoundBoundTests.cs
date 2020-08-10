using BepuUtilities;
using DemoRenderer;
using DemoUtilities;
using BepuPhysics;
using BepuPhysics.Collidables;
using System;
using System.Numerics;
using System.Diagnostics;
using DemoContentLoader;
using DemoRenderer.UI;
using DemoRenderer.Constraints;

namespace Demos.Demos
{
    public class CompoundBoundTests : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-10, 0, -10);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new PositionFirstTimestepper());


        }

        void GetArcExpansion(in Vector3 offset, in Vector3 angularVelocity, float dt, out Vector3 minExpansion, out Vector3 maxExpansion)
        {
            //minExpansion = default;
            //maxExpansion = default;
            //var angularSpeed = angularVelocity.Length();
            //if (angularSpeed == 0)
            //{
            //    return;
            //}
            //var angularDirection = angularVelocity / angularSpeed;
            //var theta = angularSpeed * dt;
            //Matrix3x3.Transform(offset, Matrix3x3.CreateFromAxisAngle(angularDirection, theta), out var endpoint);
            //var startToEnd = endpoint - offset;
            //var distance = startToEnd.Length();
            //if (distance == 0)
            //{
            //    return;
            //}
            //var arcX = startToEnd / distance;
            //var radius = offset.Length();

            //Vector3x.Cross(arcX, angularDirection, out var arcY);
            //var halfTheta = theta * 0.5f;
            //var expansionMagnitudeX = MathHelper.Sin(MathHelper.Min(MathHelper.PiOver2, halfTheta)) * radius - distance * 0.5f;
            //var expansionMagnitudeY = radius - radius * MathHelper.Cos(halfTheta);

            //var expansionX = expansionMagnitudeX * arcX;
            //BoundingBoxHelpers.ExpandBoundingBox(expansionX, ref minExpansion, ref maxExpansion);
            //BoundingBoxHelpers.ExpandBoundingBox(-expansionX, ref minExpansion, ref maxExpansion);
            //BoundingBoxHelpers.ExpandBoundingBox(expansionMagnitudeY * arcY, ref minExpansion, ref maxExpansion);



            //var angularSpeedSquared = angularVelocity.LengthSquared();
            //if (angularSpeedSquared == 0)
            //{
            //    minExpansion = default;
            //    maxExpansion = default;
            //    return;
            //}
            //var inverseAngularSpeedSquared = 1f / angularSpeedSquared;

            ////x - angularVelocity * dot(x, angularVelocity)

            //var angularSpeed = MathF.Sqrt(angularSpeedSquared);
            //var angularDirection = angularVelocity / angularSpeed;
            //var planeDot = Vector3.Dot(angularDirection, offset);
            //var planeOffset = planeDot * angularDirection;
            //var x = new Vector3(1, 0, 0) - angularDirection.X * angularDirection;
            //var y = new Vector3(0, 1, 0) - angularDirection.Y * angularDirection;
            //var z = new Vector3(0, 0, 1) - angularDirection.Z * angularDirection;
            //x = x / x.Length();
            //y = y / y.Length();
            //z = z / z.Length();
            //var radius = offset.Length();
            //var circleMax = radius * new Vector3(x.X, y.Y, z.Z);
            //var circleMin = -circleMax;
            //circleMax += planeOffset;
            //circleMin += planeOffset;

            //var theta = angularSpeed * dt;
            //Matrix3x3.Transform(offset, Matrix3x3.CreateFromAxisAngle(angularDirection, theta), out var endpoint);

            //var min = Vector3.Min(endpoint, offset);
            //var max = Vector3.Max(endpoint, offset);

            //minExpansion = circleMin - min;
            //maxExpansion = circleMax - max;



            //var angularSpeed = angularVelocity.Length();
            //var angularDirection = angularVelocity / angularSpeed;
            //var radius = offset.Length();
            //maxExpansion = new Vector3(radius - radius * MathHelper.Cos(MathHelper.Min(MathHelper.Pi, angularSpeed * dt / 2)));
            //minExpansion = -maxExpansion;


            //var theta = angularSpeed * dt;
            //Matrix3x3.Transform(offset, Matrix3x3.CreateFromAxisAngle(angularDirection, theta), out var endpoint);

            //var min = Vector3.Min(endpoint, offset);
            //var max = Vector3.Max(endpoint, offset);
            //minExpansion = minExpansion - min;
            //maxExpansion = maxExpansion - max;




            var angularSpeed = angularVelocity.Length();
            var angularDirection = angularVelocity / angularSpeed;
            var theta = angularSpeed * dt;
            Matrix3x3.Transform(offset, Matrix3x3.CreateFromAxisAngle(angularDirection, MathHelper.Min(theta, MathHelper.Pi)), out var endpoint);
            var distance = Vector3.Distance(endpoint, offset);

            maxExpansion = new Vector3(distance);
            minExpansion = -maxExpansion;

            var min = Vector3.Min(endpoint, offset);
            var max = Vector3.Max(endpoint, offset);
            minExpansion = minExpansion - min;
            maxExpansion = maxExpansion - max;
        }

        void GetEstimatedExpansion(in Vector3 localPoseA, in Vector3 angularVelocityA, in Vector3 offsetB, in Vector3 angularVelocityB, float dt, out Vector3 minExpansion, out Vector3 maxExpansion)
        {
            GetArcExpansion(localPoseA, angularVelocityA, dt, out var minExpansionA, out var maxExpansionA);
            GetArcExpansion(-offsetB, -angularVelocityB, dt, out var minExpansionB, out var maxExpansionB);
            minExpansion = minExpansionA + minExpansionB;
            maxExpansion = maxExpansionA + maxExpansionB;
        }

        Vector3 GetRandomVector(float width, Random random)
        {
            return new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) * width - new Vector3(width * 0.5f);
        }
        Random random = new Random(5);
        public unsafe override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            Vector3 basePosition = new Vector3();
            for (int testIndex = 0; testIndex < 16; ++testIndex)
            {
                var testShape = new Sphere(0);
                var orientationA = Quaternion.Identity;
                var orientationB = Quaternion.Identity;

                var velocityA = new BodyVelocity(GetRandomVector(2, random), GetRandomVector(1, random));
                var velocityB = new BodyVelocity(GetRandomVector(2, random), GetRandomVector(1, random));
                var offsetB = GetRandomVector(2, random);
                var localPoseA = new RigidPose(GetRandomVector(2, random), Quaternion.Identity);
                float dt = 10f;

                const int pathPointCount = 512;
                var localPathPoints = new Vector3[pathPointCount];

                for (int i = 0; i < pathPointCount; ++i)
                {
                    var t = (dt * i) / (pathPointCount - 1);
                    //local point = (aPosition + aLinear * t - bPosition - bLinear * t + localOffsetA * (orientationA * rotate(angularA * t)) * inverse(orientationB * rotate(angularB * t))

                    PoseIntegration.Integrate(orientationA, velocityA.Angular, t, out var integratedA);
                    PoseIntegration.Integrate(orientationB, velocityB.Angular, t, out var integratedB);
                    var worldRotatedPoint = velocityA.Linear * t - velocityB.Linear * t - offsetB + QuaternionEx.Transform(localPoseA.Position, integratedA);
                    localPathPoints[i] = QuaternionEx.Transform(worldRotatedPoint, QuaternionEx.Conjugate(integratedB));
                }
                var referenceSweep = localPathPoints[pathPointCount - 1] - localPathPoints[0];
                var sweepMin = Vector3.Min(localPathPoints[pathPointCount - 1], localPathPoints[0]);
                var sweepMax = Vector3.Max(localPathPoints[pathPointCount - 1], localPathPoints[0]);
                Vector3 referenceMin = new Vector3(float.MaxValue);
                Vector3 referenceMax = new Vector3(float.MinValue);
                for (int i = 0; i < pathPointCount; ++i)
                {
                    referenceMin = Vector3.Min(referenceMin, localPathPoints[i]);
                    referenceMax = Vector3.Max(referenceMax, localPathPoints[i]);
                }
                var referenceMinExpansion = referenceMin - sweepMin;
                var referenceMaxExpansion = referenceMax - sweepMax;

                var shapeIndex = Simulation.Shapes.Add(testShape);
                BoundingBoxHelpers.GetLocalBoundingBoxForSweep(shapeIndex, Simulation.Shapes, localPoseA, orientationA, velocityA, offsetB, orientationB, velocityB, dt, out var naiveSweep, out var naiveMin, out var naiveMax);
                naiveMin += Vector3.Min(naiveSweep, default);
                naiveMax += Vector3.Max(naiveSweep, default);
                Simulation.Shapes.Remove(shapeIndex);
                BoundingBox.CreateMerged(naiveMin, naiveMax, referenceMin, referenceMax, out var combinedMin, out var combinedMax);
                if ((combinedMin - naiveMin).LengthSquared() > 1e-5f || (combinedMax - naiveMax).LengthSquared() > 1e-5f)
                {
                    Console.WriteLine($"Naive fails to contain reference: min offset {combinedMin - naiveMin}, max offset {combinedMax - naiveMax}");
                }

                basePosition.X += 32 + MathF.Max(0, -combinedMin.X) + combinedMax.X - combinedMin.X;

                for (int i = 0; i < localPathPoints.Length - 1; ++i)
                {
                    renderer.Lines.Allocate() = new LineInstance(basePosition + localPathPoints[i], basePosition + localPathPoints[i + 1], new Vector3(1, 0, 0), new Vector3());
                }

                BoundingBoxLineExtractor.WriteBoundsLines(basePosition + referenceMin, basePosition + referenceMax, new Vector3(0, 1, 0), new Vector3(), ref renderer.Lines.Allocate(12));
                BoundingBoxLineExtractor.WriteBoundsLines(basePosition + sweepMin, basePosition + sweepMax, new Vector3(0, 0, 1), new Vector3(), ref renderer.Lines.Allocate(12));
                var expansionOffset = new Vector3(0, 0, 65);
                BoundingBoxLineExtractor.WriteBoundsLines(basePosition + expansionOffset + new Vector3(-0.01f), basePosition + expansionOffset + new Vector3(0.01f), new Vector3(0, 0, 0), new Vector3(), ref renderer.Lines.Allocate(12));
                BoundingBoxLineExtractor.WriteBoundsLines(basePosition + expansionOffset + referenceMinExpansion, basePosition + expansionOffset + referenceMaxExpansion, new Vector3(1, 0, 1), new Vector3(), ref renderer.Lines.Allocate(12));

                BoundingBoxLineExtractor.WriteBoundsLines(basePosition + naiveMin, basePosition + naiveMax, new Vector3(1, 1, 1), new Vector3(), ref renderer.Lines.Allocate(12));
                BoundingBoxLineExtractor.WriteBoundsLines(basePosition + expansionOffset + naiveMin - sweepMin, basePosition + expansionOffset + naiveMax - sweepMax, new Vector3(0, 1, 1), new Vector3(), ref renderer.Lines.Allocate(12));


                //{
                //    QuaternionWide.Broadcast(Quaternion.Identity, out var wideOrientation);
                //    Vector3Wide.Broadcast(new Vector3(1, 1, 1), out var wideVelocity);
                //    var halfDt = new Vector<float>(0.5f);
                //    const int testCount = 1024;
                //    var resultsSweep = stackalloc Vector3[testCount];
                //    var resultsMin = stackalloc Vector3[testCount];
                //    var resultsMax = stackalloc Vector3[testCount];
                //    Box box = new Box(1, 1, 1);
                //    var start = Stopwatch.GetTimestamp();
                //    for (int i = 0; i < testCount; ++i)
                //    {
                //        BoundingBoxHelpers.GetLocalBoundingBoxForSweep(ref box, localPoseA, orientationA, velocityA, offsetB, orientationB, velocityB, dt, out resultsSweep[i], out resultsMin[i], out resultsMax[i]);


                //    }
                //    var end = Stopwatch.GetTimestamp();
                //    Console.WriteLine($"Time per sweep bound test (ns): {(end - start) * (1e9 / (testCount * Stopwatch.Frequency))}");
                //}
            }


            base.Render(renderer, camera, input, text, font);
        }
    }
}


