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
                    CapsuleCylinderTester.GetClosestPointBetweenLineSegmentAndCylinder(capsuleOrigin, capsuleY, new Vector<float>(capsule.HalfLength), cylinderWide, out t, out offsetFromCylinderToLineSegment);
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
            camera.Position = new Vector3(0, 4, -6);
            camera.Yaw = MathHelper.Pi;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, 0, 0)));

            var cylinder = BodyDescription.CreateConvexDynamic(new Vector3(5f, -3, 0), 1f, Simulation.Shapes, new Cylinder(1, 2));
            cylinder.Collidable.SpeculativeMargin = float.MaxValue;
            Simulation.Bodies.Add(cylinder);
            Simulation.Bodies.Add(BodyDescription.CreateConvexKinematic(new RigidPose(new Vector3(0, -3, 0), Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), MathHelper.PiOver4)), Simulation.Shapes, new Sphere(2)));
            Simulation.Bodies.Add(BodyDescription.CreateConvexKinematic(new RigidPose(new Vector3(5, -3, 0), Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), MathHelper.PiOver4)), Simulation.Shapes, new Capsule(0.5f, 1f)));

            {
                CapsuleCylinderTester tester = default;
                CapsuleWide a = default;
                a.Broadcast(new Capsule(0.5f, 1));
                CylinderWide b = default;
                b.Broadcast(new Cylinder(0.5f, 1));
                var speculativeMargin = new Vector<float>(2f);
                Vector3Wide.Broadcast(new Vector3(0, -0.4f, 0), out var offsetB);
                QuaternionWide.Broadcast(Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathHelper.PiOver2), out var orientationA);
                QuaternionWide.Broadcast(Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), 0), out var orientationB);
                tester.Test(ref a, ref b, ref speculativeMargin, ref offsetB, ref orientationA, ref orientationB, Vector<float>.Count, out var manifold);
            }
            {
                CylinderWide a = default;
                a.Broadcast(new Cylinder(0.5f, 1f));
                CylinderWide b = default;
                b.Broadcast(new Cylinder(0.5f, 1f));
                var supportFinderA = new CylinderSupportFinder();
                var supportFinderB = new CylinderSupportFinder();
                Vector3Wide.Broadcast(new Vector3(-0.8f, 0.01f, 0.71f), out var localOffsetB);
                Matrix3x3Wide.Broadcast(Matrix3x3.CreateFromAxisAngle(new Vector3(1, 0, 0), 0.1f), out var localOrientationB);
                Vector3Wide.Normalize(localOffsetB, out var initialGuess);

                GradientDescent<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>.Refine(a, b, localOffsetB, localOrientationB,
                    ref supportFinderA, ref supportFinderB, initialGuess, new Vector<float>(-0.1f), new Vector<float>(1e-4f), 1500, Vector<int>.Zero, out var localNormal, out var depthBelowThreshold);

                GJKDistanceTester<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder> gjk = default;
                QuaternionWide.Broadcast(Quaternion.Identity, out var localOrientationQuaternionA);
                QuaternionWide.CreateFromRotationMatrix(localOrientationB, out var localOrientationQuaternionB);
                gjk.Test(ref a, ref b, ref localOffsetB, ref localOrientationQuaternionA, ref localOrientationQuaternionB, out var intersected, out var distance, out var closestA, out var gjkNormal);
                TimeGradientDescent(32);
                TimeGradientDescent(1000000);
            }
            {
                CylinderWide a = default;
                a.Broadcast(new Cylinder(0.5f, 1f));
                CylinderWide b = default;
                b.Broadcast(new Cylinder(0.5f, 1f));
                var supportFinderA = new CylinderSupportFinder();
                var supportFinderB = new CylinderSupportFinder();
                Vector3Wide.Broadcast(new Vector3(0.5f, 0.5f, 0.5f), out var localOffsetB);
                Vector3Wide.Broadcast(Vector3.Normalize(new Vector3(1, 0, 1)), out var localCastDirection);
                Matrix3x3Wide.Broadcast(Matrix3x3.CreateFromAxisAngle(new Vector3(1, 0, 0), 0), out var localOrientationB);
                MPR<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>.Test(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, new Vector<float>(1e-5f), Vector<int>.Zero, out var intersecting, out var localNormal);
                for (int i = 0; i < 5; ++i)
                {
                    MPR<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>.LocalSurfaceCast(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, localNormal, new Vector<float>(1e-3f), Vector<int>.Zero,
                        out var t, out localNormal);
                }
                Vector3Wide.Normalize(localNormal, out var test);

                GJKDistanceTester<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder> gjk = default;
                QuaternionWide.Broadcast(Quaternion.Identity, out var localOrientationQuaternionA);
                QuaternionWide.CreateFromRotationMatrix(localOrientationB, out var localOrientationQuaternionB);
                gjk.Test(ref a, ref b, ref localOffsetB, ref localOrientationQuaternionA, ref localOrientationQuaternionB, out var intersected, out var distance, out var closestA, out var gjkNormal);
                TimeMPRSurfaceCast(32);
                TimeMPRSurfaceCast(1000000);
            }
            {
                CylinderWide a = default;
                a.Broadcast(new Cylinder(0.5f, 1f));
                CylinderWide b = default;
                b.Broadcast(new Cylinder(0.5f, 1f));
                var supportFinderA = new CylinderSupportFinder();
                var supportFinderB = new CylinderSupportFinder();
                Vector3Wide.Broadcast(new Vector3(-0.335f, -0.0f, 1.207f), out var localOffsetB);
                Matrix3x3Wide.Broadcast(Matrix3x3.CreateFromAxisAngle(new Vector3(1, 0, 0), 10.0f), out var localOrientationB);
                MPR<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>.Test(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, new Vector<float>(1e-5f), Vector<int>.Zero, out var intersecting, out var localNormal);
                Vector3Wide.Normalize(localNormal, out var test);
                GJKDistanceTester<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder> gjk = default;
                QuaternionWide.Broadcast(Quaternion.Identity, out var localOrientationQuaternionA);
                QuaternionWide.CreateFromRotationMatrix(localOrientationB, out var localOrientationQuaternionB);
                gjk.Test(ref a, ref b, ref localOffsetB, ref localOrientationQuaternionA, ref localOrientationQuaternionB, out var intersected, out var distance, out var closestA, out var gjkNormal);
                TimeMPR(32);
                TimeMPR(1000000);
            }

        }

        void TimeGradientDescent(int iterationCount)
        {
            CylinderWide a = default;
            a.Broadcast(new Cylinder(0.5f, 1f));
            CylinderWide b = default;
            b.Broadcast(new Cylinder(0.5f, 1f));
            var supportFinderA = new CylinderSupportFinder();
            var supportFinderB = new CylinderSupportFinder();
            Vector3Wide.Broadcast(new Vector3(1, 0, 0), out var localOffsetB);
            Matrix3x3Wide.Broadcast(Matrix3x3.CreateFromAxisAngle(new Vector3(1, 1, 1), 2.0f), out var localOrientationB);
            Vector3Wide.Normalize(localOffsetB, out var initialGuess);

            var start = Stopwatch.GetTimestamp();
            for (int i = 0; i < iterationCount; ++i)
            {
                GradientDescent<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>.Refine(a, b, localOffsetB, localOrientationB,
                    ref supportFinderA, ref supportFinderB, initialGuess, new Vector<float>(-0.1f), new Vector<float>(1e-4f), 1500, Vector<int>.Zero, out var localNormal, out var depthBelowThreshold);
            }
            var end = Stopwatch.GetTimestamp();
            Console.WriteLine($"Gradient descent time (ns) per iteration (iteration count {iterationCount}): {1e9 * (end - start) / (iterationCount * (double)Stopwatch.Frequency)}");
        }

        void TimeMPR(int iterationCount)
        {
            CylinderWide a = default;
            a.Broadcast(new Cylinder(0.5f, 1f));
            CylinderWide b = default;
            b.Broadcast(new Cylinder(0.5f, 1f));
            var supportFinderA = new CylinderSupportFinder();
            var supportFinderB = new CylinderSupportFinder();
            Vector3Wide.Broadcast(new Vector3(-0.335f, -1.1f, 0.707f), out var localOffsetB);
            Matrix3x3Wide.Broadcast(Matrix3x3.CreateFromAxisAngle(new Vector3(1, 0, 0), 10.0f), out var localOrientationB);
            var start = Stopwatch.GetTimestamp();
            for (int i = 0; i < iterationCount; ++i)
            {
                MPR<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>.Test(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, new Vector<float>(1e-3f), Vector<int>.Zero, out var intersecting, out var localNormal);
            }
            var end = Stopwatch.GetTimestamp();
            Console.WriteLine($"MPR time (ns) per iteration (iteration count {iterationCount}): {1e9 * (end - start) / (iterationCount * (double)Stopwatch.Frequency)}");
        }

        void TimeMPRSurfaceCast(int iterationCount)
        {
            CylinderWide a = default;
            a.Broadcast(new Cylinder(0.5f, 1f));
            CylinderWide b = default;
            b.Broadcast(new Cylinder(0.5f, 1f));
            var supportFinderA = new CylinderSupportFinder();
            var supportFinderB = new CylinderSupportFinder();
            Vector3Wide.Broadcast(new Vector3(.5f, 0.5f, 0.5f), out var localOffsetB);
            Vector3Wide.Broadcast(Vector3.Normalize(new Vector3(1, 0.7f, 1)), out var localCastDirection);
            Matrix3x3Wide.Broadcast(Matrix3x3.CreateFromAxisAngle(new Vector3(1, 0, 0), 0), out var localOrientationB);
            var start = Stopwatch.GetTimestamp();
            for (int i = 0; i < iterationCount; ++i)
            {
                MPR<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>.LocalSurfaceCast(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, localCastDirection, new Vector<float>(1e-2f), Vector<int>.Zero,
                    out var t, out var localNormal);
            }
            var end = Stopwatch.GetTimestamp();
            Console.WriteLine($"MPR Surface Cast time (ns) per iteration (iteration count {iterationCount}): {1e9 * (end - start) / (iterationCount * (double)Stopwatch.Frequency)}");
        }

    }
}
