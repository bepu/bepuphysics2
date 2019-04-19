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
using System.Diagnostics;
using Quaternion = BepuUtilities.Quaternion;
using BepuPhysics.CollisionDetection.SweepTasks;
using Demos.Demos;

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

        private static void TestSegmentCylinder()
        {
            var cylinder = new Cylinder(0.5f, 1);
            CylinderWide cylinderWide = default;
            cylinderWide.Broadcast(cylinder);
            Random random = new Random(5);
            //double totalIntervalError = 0;
            //double sumOfSquaredIntervalError = 0;

            double totalBruteError = 0;
            double sumOfSquaredBruteError = 0;

            double totalBruteDistanceError = 0;
            double sumOfSquaredBruteDistanceError = 0;

            //long iterationsSum = 0;
            //long iterationsSquaredSum = 0;
            var capsuleTests = 1000;

            int warmupCount = 32;
            int innerIterations = 128;
            long testTicks = 0;
            for (int i = 0; i < warmupCount + capsuleTests; ++i)
            {
                Vector3 randomPointNearCylinder;
                var capsule = new Capsule(0.2f + .8f * (float)random.NextDouble(), 0.2f + 0.8f * (float)random.NextDouble());
                var minimumDistance = 1f * (cylinder.Radius + cylinder.HalfLength);
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

                //CapsuleCylinderTester.GetClosestPointBetweenLineSegmentAndCylinder(capsuleOrigin, capsuleY, new Vector<float>(capsule.HalfLength), cylinderWide, out var t, out var min, out var max, out var offsetFromCylindertoLineSegment, out var iterationsRequired);

                //CapsuleCylinderTester.GetClosestPointBetweenLineSegmentAndCylinder(capsuleOrigin, capsuleY, new Vector<float>(capsule.HalfLength), cylinderWide, out var t, out var offsetFromCylindertoLineSegment);
                Vector<float> t = default;
                Vector3Wide offsetFromCylinderToLineSegment = default;
                var innerStart = Stopwatch.GetTimestamp();
                for (int j = 0; j < innerIterations; ++j)
                {
                    CapsuleCylinderTester.GetClosestPointBetweenLineSegmentAndCylinder(capsuleOrigin, capsuleY, new Vector<float>(capsule.HalfLength), cylinderWide, Vector<int>.Zero, out t, out offsetFromCylinderToLineSegment);
                }
                var innerStop = Stopwatch.GetTimestamp();
                if (i > warmupCount)
                {
                    testTicks += innerStop - innerStart;
                }
                Vector3Wide.LengthSquared(offsetFromCylinderToLineSegment, out var distanceSquaredWide);
                var distanceSquared = distanceSquaredWide[0];

                //iterationsSum += iterationsRequired[0];
                //iterationsSquaredSum += iterationsRequired[0] * iterationsRequired[0];

                BruteForceSearch(randomPointNearCylinder, direction, capsule.HalfLength, cylinder, out var bruteT, out var bruteDistanceSquared, out var errorMargin);
                var errorRelativeToBrute = MathF.Max(MathF.Abs(bruteT - t[0]), errorMargin) - errorMargin;
                sumOfSquaredBruteError += errorRelativeToBrute * errorRelativeToBrute;
                totalBruteError += errorRelativeToBrute;

                if ((distanceSquared == 0) != (bruteDistanceSquared == 0))
                {
                    Console.WriteLine($"Search and brute force disagree on intersecting distance; search found {distanceSquared}, brute found {bruteDistanceSquared}");
                }

                var bruteDistanceError = MathF.Abs(MathF.Sqrt(distanceSquared) - MathF.Sqrt(bruteDistanceSquared));
                sumOfSquaredBruteDistanceError += bruteDistanceError * bruteDistanceError;
                totalBruteDistanceError += bruteDistanceError;

                //var intervalSpan = Vector.Abs(max - min)[0];
                //sumOfSquaredIntervalError += intervalSpan * intervalSpan;
                //totalIntervalError += intervalSpan;


            }
            Console.WriteLine($"Average time per test (ns): {1e9 * testTicks / (innerIterations * capsuleTests * Stopwatch.Frequency)}");

            //var averageIntervalSpan = totalIntervalError / capsuleTests;
            //var averageIntervalSquaredSpan = sumOfSquaredIntervalError / capsuleTests;
            //var intervalStandardDeviation = Math.Sqrt(Math.Max(0, averageIntervalSquaredSpan - averageIntervalSpan * averageIntervalSpan));
            //Console.WriteLine($"Average interval span: {averageIntervalSpan}, stddev {intervalStandardDeviation}");

            var averageBruteError = totalBruteError / capsuleTests;
            var averageBruteSquaredError = sumOfSquaredBruteError / capsuleTests;
            var bruteStandardDeviation = Math.Sqrt(Math.Max(0, averageBruteSquaredError - averageBruteError * averageBruteError));
            Console.WriteLine($"Average brute T error: {averageBruteError}, stddev {bruteStandardDeviation}");

            var averageBruteDistanceError = totalBruteDistanceError / capsuleTests;
            var averageBruteDistanceSquaredError = sumOfSquaredBruteDistanceError / capsuleTests;
            var bruteDistanceStandardDeviation = Math.Sqrt(Math.Max(0, averageBruteSquaredError - averageBruteError * averageBruteError));
            Console.WriteLine($"Average brute distance error: {averageBruteDistanceError}, stddev {bruteDistanceStandardDeviation}");

            //var averageIterations = (double)iterationsSum / capsuleTests;
            //var averageIterationSquared = (double)iterationsSquaredSum / capsuleTests;
            //var iterationStandardDeviation = Math.Sqrt(Math.Max(0, averageIterationSquared - averageIterations * averageIterations));
            //Console.WriteLine($"Average iteration count: {averageIterations}, stddev {iterationStandardDeviation}");
        }

        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(10, 0, 6);
            camera.Pitch = 0;
            camera.Yaw = 0;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, 0f, 0)));

            var cylinderShape = new Cylinder(1f, .2f);
            cylinderShape.ComputeInertia(1, out var cylinderInertia);
            var cylinder = BodyDescription.CreateDynamic(new Vector3(10f, 3, 0), cylinderInertia, new CollidableDescription(Simulation.Shapes.Add(cylinderShape), 1000), new BodyActivityDescription(0.01f));
            Simulation.Bodies.Add(cylinder);
            Simulation.Bodies.Add(BodyDescription.CreateConvexKinematic(new RigidPose(new Vector3(0, -6, 0), Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), MathHelper.PiOver4)), Simulation.Shapes, new Sphere(2)));
            Simulation.Bodies.Add(BodyDescription.CreateConvexKinematic(new RigidPose(new Vector3(7, -6, 0), Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), MathHelper.PiOver4)), Simulation.Shapes, new Capsule(0.5f, 1f)));
            Simulation.Bodies.Add(BodyDescription.CreateConvexKinematic(new RigidPose(new Vector3(21, -3, 0), Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), 0)), Simulation.Shapes, new Box(3f, 1f, 3f)));
            Simulation.Bodies.Add(BodyDescription.CreateConvexKinematic(new RigidPose(new Vector3(28, -6, 0), Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), 0)), Simulation.Shapes,
                new Triangle(new Vector3(10f, 0, 10f), new Vector3(14f, 0, 10f), new Vector3(10f, 0, 14f))));
            Simulation.Bodies.Add(BodyDescription.CreateConvexKinematic(new RigidPose(new Vector3(14, -6, 0), Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), 0)), Simulation.Shapes, new Cylinder(3f, .2f)));


            cylinderShape = new Cylinder(1f, 3);
            var cylinderShapeIndex = Simulation.Shapes.Add(cylinderShape);
            cylinderShape.ComputeInertia(1, out cylinderInertia);
            //const int rowCount = 15;
            //for (int rowIndex = 0; rowIndex < rowCount; ++rowIndex)
            //{
            //    int columnCount = rowCount - rowIndex;
            //    for (int columnIndex = 0; columnIndex < columnCount; ++columnIndex)
            //    {
            //        Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(
            //            (-columnCount * 0.5f + columnIndex) * cylinderShape.Radius * 2f,
            //            (rowIndex + 0.5f) * cylinderShape.Length - 9.5f, -10),
            //            cylinderInertia,
            //            new CollidableDescription(cylinderShapeIndex, 0.1f),
            //            new BodyActivityDescription(0.01f)));
            //    }
            //}

            var box = new Box(1f, 3f, 2f);
            var capsule = new Capsule(1f, 1f);
            var sphere = new Sphere(1.5f);
            box.ComputeInertia(1, out var boxInertia);
            capsule.ComputeInertia(1, out var capsuleInertia);
            sphere.ComputeInertia(1, out var sphereInertia);
            var boxIndex = Simulation.Shapes.Add(box);
            var capsuleIndex = Simulation.Shapes.Add(capsule);
            var sphereIndex = Simulation.Shapes.Add(sphere);
            const int width = 2;
            const int height = 1;
            const int length = 2;
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        var location = new Vector3(5, 3,5) * new Vector3(i, j, k) + new Vector3(-width * 1.5f, 2.5f, -30 - length * 1.5f);
                        var bodyDescription = new BodyDescription
                        {
                            Activity = new BodyActivityDescription(-0.01f),
                            Pose = new RigidPose
                            {
                                Orientation = Quaternion.Identity,// Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 1, 1)), MathF.PI * 0.79813f),
                                Position = location
                            },
                            Collidable = new CollidableDescription
                            {
                                Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                                SpeculativeMargin = 0.1f
                            }
                        };
                        switch (j % 4)
                        {
                            case 0:
                            //    bodyDescription.Collidable.Shape = boxIndex;
                            //    bodyDescription.LocalInertia = boxInertia;
                            //    break;
                            case 1:
                            //    bodyDescription.Collidable.Shape = capsuleIndex;
                            //    bodyDescription.LocalInertia = capsuleInertia;
                            //    break;
                            case 2:
                            //    bodyDescription.Collidable.Shape = sphereIndex;
                            //    bodyDescription.LocalInertia = sphereInertia;
                            //    break;
                            case 3:
                                bodyDescription.Collidable.Shape = cylinderShapeIndex;
                                bodyDescription.LocalInertia = cylinderInertia;
                                break;
                        }
                        Simulation.Bodies.Add(bodyDescription);

                    }
                }
            }

            const int planeWidth = 50;
            const int planeHeight = 50;
            DemoMeshHelper.CreateDeformedPlane(planeWidth, planeHeight,
                (int x, int y) =>
                {
                    var octave0 = (MathF.Sin((x + 5f) * 0.05f) + MathF.Sin((y + 11) * 0.05f)) * 3f;
                    var octave1 = (MathF.Sin((x + 17) * 0.15f) + MathF.Sin((y + 19) * 0.15f)) * 2f;
                    var octave2 = (MathF.Sin((x + 37) * 0.35f) + MathF.Sin((y + 93) * 0.35f)) * 1f;
                    var octave3 = (MathF.Sin((x + 53) * 0.65f) + MathF.Sin((y + 47) * 0.65f)) * 0.5f;
                    var octave4 = (MathF.Sin((x + 67) * 1.50f) + MathF.Sin((y + 13) * 1.5f)) * 0.25f;
                    return new Vector3(x, octave0 + octave1 + octave2 + octave3 + octave4, y);
                }, new Vector3(4, 1, 4), BufferPool, out var planeMesh);
            Simulation.Statics.Add(new StaticDescription(new Vector3(-100, -15, 100), Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), MathF.PI / 2),
                new CollidableDescription(Simulation.Shapes.Add(planeMesh), 0.1f)));

            //Simulation.Statics.Add(new StaticDescription(new Vector3(0, -10, 0), Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), 0), Simulation.Shapes.Add(new Cylinder(100, 1f)), 0.1f));

            //{
            //    CapsuleCylinderTester tester = default;
            //    CapsuleWide a = default;
            //    a.Broadcast(new Capsule(0.5f, 1));
            //    CylinderWide b = default;
            //    b.Broadcast(new Cylinder(0.5f, 1));
            //    var speculativeMargin = new Vector<float>(2f);
            //    Vector3Wide.Broadcast(new Vector3(0, -0.4f, 0), out var offsetB);
            //    QuaternionWide.Broadcast(Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathHelper.PiOver2), out var orientationA);
            //    QuaternionWide.Broadcast(Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), 0), out var orientationB);
            //    tester.Test(ref a, ref b, ref speculativeMargin, ref offsetB, ref orientationA, ref orientationB, Vector<float>.Count, out var manifold);
            //}
            //{
            //    CylinderWide a = default;
            //    a.Broadcast(new Cylinder(0.5f, 1f));
            //    CylinderWide b = default;
            //    b.Broadcast(new Cylinder(0.5f, 1f));
            //    var supportFinderA = new CylinderSupportFinder();
            //    var supportFinderB = new CylinderSupportFinder();
            //    Vector3Wide.Broadcast(new Vector3(-0.8f, 0.01f, 0.71f), out var localOffsetB);
            //    Matrix3x3Wide.Broadcast(Matrix3x3.CreateFromAxisAngle(new Vector3(1, 0, 0), 0.1f), out var localOrientationB);
            //    Vector3Wide.Normalize(localOffsetB, out var initialGuess);

            //    GradientDescent<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>.Refine(a, b, localOffsetB, localOrientationB,
            //        ref supportFinderA, ref supportFinderB, initialGuess, new Vector<float>(-0.1f), new Vector<float>(1e-4f), 1500, Vector<int>.Zero, out var localNormal, out var depthBelowThreshold);

            //    GJKDistanceTester<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder> gjk = default;
            //    QuaternionWide.Broadcast(Quaternion.Identity, out var localOrientationQuaternionA);
            //    QuaternionWide.CreateFromRotationMatrix(localOrientationB, out var localOrientationQuaternionB);
            //    gjk.Test(ref a, ref b, ref localOffsetB, ref localOrientationQuaternionA, ref localOrientationQuaternionB, out var intersected, out var distance, out var closestA, out var gjkNormal);
            //    //TimeGradientDescent(32);
            //    //TimeGradientDescent(1000000);
            //}
            //{
            //    CylinderWide a = default;
            //    a.Broadcast(new Cylinder(0.5f, 1f));
            //    CylinderWide b = default;
            //    b.Broadcast(new Cylinder(0.5f, 1f));
            //    var supportFinderA = new CylinderSupportFinder();
            //    var supportFinderB = new CylinderSupportFinder();
            //    Vector3Wide.Broadcast(new Vector3(0.5f, 0.5f, 0.5f), out var localOffsetB);
            //    Vector3Wide.Broadcast(Vector3.Normalize(new Vector3(1, 0, 1)), out var localCastDirection);
            //    Matrix3x3Wide.Broadcast(Matrix3x3.CreateFromAxisAngle(new Vector3(1, 0, 0), 0), out var localOrientationB);
            //    MPR<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>.Test(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, new Vector<float>(1e-5f), Vector<int>.Zero, out var intersecting, out var localNormal);
            //    for (int i = 0; i < 5; ++i)
            //    {
            //        MPR<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>.LocalSurfaceCast(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, localNormal, new Vector<float>(1e-3f), Vector<int>.Zero,
            //            out var t, out localNormal);
            //    }
            //    Vector3Wide.Normalize(localNormal, out var test);

            //    GJKDistanceTester<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder> gjk = default;
            //    QuaternionWide.Broadcast(Quaternion.Identity, out var localOrientationQuaternionA);
            //    QuaternionWide.CreateFromRotationMatrix(localOrientationB, out var localOrientationQuaternionB);
            //    gjk.Test(ref a, ref b, ref localOffsetB, ref localOrientationQuaternionA, ref localOrientationQuaternionB, out var intersected, out var distance, out var closestA, out var gjkNormal);
            //    //TimeMPRSurfaceCast(32);
            //    //TimeMPRSurfaceCast(1000000);
            //}
            //{
            //    CylinderWide a = default;
            //    a.Broadcast(new Cylinder(0.5f, 1f));
            //    CylinderWide b = default;
            //    b.Broadcast(new Cylinder(0.5f, 1f));
            //    var supportFinderA = new CylinderSupportFinder();
            //    var supportFinderB = new CylinderSupportFinder();
            //    Vector3Wide.Broadcast(new Vector3(-0.335f, -0.0f, 1.207f), out var localOffsetB);
            //    Matrix3x3Wide.Broadcast(Matrix3x3.CreateFromAxisAngle(new Vector3(1, 0, 0), 10.0f), out var localOrientationB);
            //    MPR<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>.Test(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, new Vector<float>(1e-5f), Vector<int>.Zero, out var intersecting, out var localNormal);
            //    Vector3Wide.Normalize(localNormal, out var test);
            //    GJKDistanceTester<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder> gjk = default;
            //    QuaternionWide.Broadcast(Quaternion.Identity, out var localOrientationQuaternionA);
            //    QuaternionWide.CreateFromRotationMatrix(localOrientationB, out var localOrientationQuaternionB);
            //    gjk.Test(ref a, ref b, ref localOffsetB, ref localOrientationQuaternionA, ref localOrientationQuaternionB, out var intersected, out var distance, out var closestA, out var gjkNormal);
            //    //TimeMPR(32);
            //    //TimeMPR(1000000);
            //}

        }



    }
}
