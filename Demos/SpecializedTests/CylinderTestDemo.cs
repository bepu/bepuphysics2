using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace Demos.SpecializedTests
{
    public class CylinderTestDemo : Demo
    {
        private static void BruteForceSearch(in Vector3 lineOrigin, in Vector3 lineDirection, float halfLength, in Cylinder cylinder, out float closestT, out float closestDistanceSquared, out float errorMargin)
        {
            const int sampleCount = 1 << 20;
            var inverseSampleCount = 1.0 / (sampleCount - 1);
            errorMargin = (float)inverseSampleCount;
            var radiusSquared = cylinder.Radius * cylinder.Radius;
            closestDistanceSquared = float.MaxValue;
            closestT = float.MaxValue;
            for (int i = 0; i < sampleCount; ++i)
            {
                var t = (float)(halfLength * (i * inverseSampleCount * 2 - 1));
                var point = lineOrigin + lineDirection * t;
                var horizontalLengthSquared = point.X * point.X + point.Z * point.Z;
                Vector3 clamped;
                if (horizontalLengthSquared > radiusSquared)
                {
                    var scale = cylinder.Radius / MathF.Sqrt(horizontalLengthSquared);
                    clamped.X = scale * point.X;
                    clamped.Z = scale * point.Z;
                }
                else
                {
                    clamped.X = point.X;
                    clamped.Z = point.Z;
                }
                clamped.Y = MathF.Max(-cylinder.HalfLength, MathF.Min(cylinder.HalfLength, point.Y));
                var distanceSquared = Vector3.DistanceSquared(clamped, point);
                if (distanceSquared < closestDistanceSquared)
                {
                    closestDistanceSquared = distanceSquared;
                    closestT = t;
                }
            }

        }
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 4, -6);
            camera.Yaw = MathHelper.Pi;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, 0, 0)));

            Simulation.Bodies.Add(BodyDescription.CreateConvexDynamic(new Vector3(), 1f, Simulation.Shapes, new Cylinder(3, 4)));
            Simulation.Bodies.Add(BodyDescription.CreateConvexKinematic(new Vector3(0, 0, 0), Simulation.Shapes, new Sphere(2)));

            //for (int i = 0; i < 4; ++i)
            //    for (int j = 0; j < 4; ++j)
            //        Simulation.Bodies.Add(BodyDescription.CreateConvexDynamic(new Vector3(i * 11, (i + j) / 100f, j * 11), 1, Simulation.Shapes, new Cylinder(5, 1)));



            //Simulation.Statics.Add(new StaticDescription(new Vector3(0, -2, 0), new CollidableDescription(Simulation.Shapes.Add(new Cylinder(5, 1)), 0.1f)));

            {
                Vector3Wide.Broadcast(new Vector3(2, 0, 0), out var capsuleOrigin);
                Vector3Wide.Broadcast(Vector3.Normalize(new Vector3(1, 0, 1)), out var capsuleDirection);
                var cylinder = new Cylinder(1, 1);
                CylinderWide cylinderWide = default;
                cylinderWide.Broadcast(cylinder);
                CapsuleCylinderTester.GetClosestPointBetweenLineSegmentAndCylinder(capsuleOrigin, capsuleDirection, new Vector<float>(2), cylinderWide, out var t, out var min, out var max, out var offsetFromCylindertoLineSegment);
            }
            {
                var cylinder = new Cylinder(1, 1);
                CylinderWide cylinderWide = default;
                cylinderWide.Broadcast(cylinder);
                Random random = new Random(5);
                float totalIntervalError = 0;
                float sumOfSquaredIntervalError = 0;
                float totalBruteError = 0;
                float sumOfSquaredBruteError = 0;
                float totalBruteDistanceError = 0;
                float sumOfSquaredBruteDistanceError = 0;
                var capsuleTests = 100;
                for (int i = 0; i < capsuleTests; ++i)
                {
                    Vector3 randomPointNearCylinder;
                    var capsule = new Capsule(0.2f + 2.8f * (float)random.NextDouble(), 0.2f + 3.8f * (float)random.NextDouble());
                    var minimumDistance = cylinder.Radius + capsule.HalfLength + 0.01f;
                    var minimumDistanceSquared = minimumDistance * minimumDistance;
                    while (true)
                    {
                        randomPointNearCylinder = new Vector3((cylinder.Radius + capsule.HalfLength) * 2, (cylinder.HalfLength + capsule.HalfLength) * 2, (cylinder.Radius + capsule.HalfLength) * 2) *
                            (new Vector3(2) * new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) - Vector3.One);
                        var pointOnCylinderAxis = new Vector3(0, MathF.Max(-cylinder.HalfLength, MathF.Min(cylinder.HalfLength, randomPointNearCylinder.Y)), 0);
                        var offset = randomPointNearCylinder - pointOnCylinderAxis;
                        var lengthSquared = offset.LengthSquared();
                        if (lengthSquared > minimumDistanceSquared)
                            break;
                    }

                    Vector3 direction;
                    float directionLengthSquared;
                    do
                    {
                        direction = new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) * new Vector3(2) - Vector3.One;
                        directionLengthSquared = direction.LengthSquared();
                    } while (directionLengthSquared < 1e-8f);
                    direction /= MathF.Sqrt(directionLengthSquared);


                    Vector3Wide.Broadcast(randomPointNearCylinder, out var capsuleOrigin);
                    Vector3Wide.Broadcast(direction, out var capsuleY);

                    CapsuleCylinderTester.GetClosestPointBetweenLineSegmentAndCylinder(capsuleOrigin, capsuleY, new Vector<float>(capsule.HalfLength), cylinderWide, out var t, out var min, out var max, out var offsetFromCylindertoLineSegment);
                    Vector3Wide.LengthSquared(offsetFromCylindertoLineSegment, out var distanceSquaredWide);
                    var distanceSquared = distanceSquaredWide[0];

                    BruteForceSearch(randomPointNearCylinder, direction, capsule.HalfLength, cylinder, out var bruteT, out var bruteDistanceSquared, out var errorMargin);
                    var errorRelativeToBrute = MathF.Max(MathF.Abs(bruteT - t[0]), errorMargin) - errorMargin;
                    sumOfSquaredBruteError += errorRelativeToBrute * errorRelativeToBrute;
                    totalBruteError += errorRelativeToBrute;

                    if ((bruteDistanceSquared == 0) != (bruteDistanceSquared == 0))
                    {
                        Console.WriteLine($"Search and brute force disagree on intersection state.");
                    }

                    var bruteDistanceError = MathF.Abs(MathF.Sqrt(distanceSquared) - MathF.Sqrt(bruteDistanceSquared));
                    sumOfSquaredBruteDistanceError += bruteDistanceError * bruteDistanceError;
                    totalBruteDistanceError += bruteDistanceError;

                    var intervalSpan = Vector.Abs(max - min)[0];
                    sumOfSquaredIntervalError += intervalSpan * intervalSpan;
                    totalIntervalError += intervalSpan;


                }
                var averageIntervalSpan = totalIntervalError / capsuleTests;
                var averageIntervalSquaredSpan = sumOfSquaredIntervalError / capsuleTests;
                var intervalStandardDeviation = MathF.Sqrt(MathF.Max(0, averageIntervalSquaredSpan - averageIntervalSpan * averageIntervalSpan));
                Console.WriteLine($"Average interval span: {averageIntervalSpan}, stddev {intervalStandardDeviation}");

                var averageBruteError = totalBruteError / capsuleTests;
                var averageBruteSquaredError = sumOfSquaredBruteError / capsuleTests;
                var bruteStandardDeviation = MathF.Sqrt(MathF.Max(0, averageBruteSquaredError - averageBruteError * averageBruteError));
                Console.WriteLine($"Average brute T error: {averageBruteError}, stddev {bruteStandardDeviation}");

                var averageBruteDistanceError = totalBruteDistanceError / capsuleTests;
                var averageBruteDistanceSquaredError = sumOfSquaredBruteDistanceError / capsuleTests;
                var bruteDistanceStandardDeviation = MathF.Sqrt(MathF.Max(0, averageBruteSquaredError - averageBruteError * averageBruteError));
                Console.WriteLine($"Average brute distance error: {averageBruteDistanceError}, stddev {bruteDistanceStandardDeviation}");
            }
        }
    }
}
