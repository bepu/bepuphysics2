using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct BoxPairDistanceTester : IPairDistanceTester<BoxWide, BoxWide>
    {
        //TODO: This is far from an optimal implementation. Sweeps aren't ultra-critical, so I just skipped spending too much time on this. May want to revisit later if perf concerns arise.
        //(Options include SPMD-style iterative algorithms (GJK and friends), or just making this brute force approach a little less dumb.)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestEdgeEdge(
               ref Vector<float> halfExtentA, ref Vector3Wide edgeCenterA, ref Vector3Wide edgeDirectionA,
               ref Vector<float> halfExtentB, ref Vector3Wide edgeCenterB, ref Vector3Wide edgeDirectionB,
               ref Vector3Wide offsetB,
               out Vector<float> distance, out Vector3Wide normal, out Vector3Wide closestA)
        {
            Vector3Wide.Subtract(ref edgeCenterA, ref edgeCenterB, out var edgeCentersOffsetBA);
            Vector3Wide.CrossWithoutOverlap(ref edgeDirectionA, ref edgeDirectionB, out normal);
            Vector3Wide.Length(ref normal, out var length);
            var inverseLength = Vector<float>.One / length;
            Vector3Wide.Scale(ref normal, ref inverseLength, out normal);
            Vector3Wide.Dot(ref normal, ref offsetB, out var calibrationDot);
            var shouldNegate = Vector.LessThan(calibrationDot, Vector<float>.Zero);
            normal.X = Vector.ConditionalSelect(shouldNegate, -normal.X, normal.X);
            normal.Y = Vector.ConditionalSelect(shouldNegate, -normal.Y, normal.Y);
            normal.Z = Vector.ConditionalSelect(shouldNegate, -normal.Z, normal.Z);
            Vector3Wide.Dot(ref normal, ref edgeCentersOffsetBA, out distance);


            //The edge distance is only meaningful if the normal is well defined and the edges projected intervals overlap.
            //(Parallel edges and edge endpoint cases are handled by the vertex-box cases.)
            Vector3Wide.Dot(ref edgeDirectionA, ref edgeDirectionB, out var dadb);
            Vector3Wide.Dot(ref edgeDirectionA, ref edgeCentersOffsetBA, out var daOffsetBA);
            Vector3Wide.Dot(ref edgeDirectionB, ref edgeCentersOffsetBA, out var dbOffsetBA);
            //Note potential division by zero when the axes are parallel. Later validation catches it.
            var ta = (dbOffsetBA * dadb - daOffsetBA) / (Vector<float>.One - dadb * dadb);
            Vector3Wide.Scale(ref normal, ref ta, out closestA);
            Vector3Wide.Add(ref closestA, ref edgeCenterA, out closestA);
            var bExtentOnA = halfExtentB * dadb;
            var ta0 = -bExtentOnA - daOffsetBA;
            var ta1 = bExtentOnA - daOffsetBA;
            var taMin = Vector.Min(ta0, ta1);
            var taMax = Vector.Max(ta0, ta1);
            var aExtentOnB = halfExtentA * dadb;
            var tb0 = dbOffsetBA - aExtentOnB;
            var tb1 = dbOffsetBA + aExtentOnB;
            var tbMin = Vector.Min(tb0, tb1);
            var tbMax = Vector.Max(tb0, tb1);

            var bOutsideA = Vector.BitwiseOr(Vector.GreaterThan(taMin, halfExtentA), Vector.LessThan(taMax, -halfExtentA));
            var aOutsideB = Vector.BitwiseOr(Vector.GreaterThan(tbMin, halfExtentB), Vector.LessThan(tbMax, -halfExtentB));
            var intervalsDontOverlap = Vector.BitwiseOr(bOutsideA, aOutsideB);
            var badNormal = Vector.LessThan(length, new Vector<float>(1e-15f));
            var edgeDistanceInvalid = Vector.BitwiseOr(badNormal, intervalsDontOverlap);
            distance = Vector.ConditionalSelect(edgeDistanceInvalid, new Vector<float>(float.MaxValue), distance);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Select(
            ref Vector<float> distance, ref Vector3Wide normal, ref Vector3Wide closestA,
            ref Vector<float> distanceCandidate, ref Vector3Wide normalCandidate, ref Vector3Wide closestACandidate)
        {
            var useCandidate = Vector.LessThan(distanceCandidate, distance);
            distance = Vector.Min(distance, distanceCandidate);
            Vector3Wide.ConditionalSelect(ref useCandidate, ref normalCandidate, ref normal, out normal);
            Vector3Wide.ConditionalSelect(ref useCandidate, ref closestACandidate, ref closestA, out closestA);
        }

        public void Test(ref BoxWide a, ref BoxWide b, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Vector<int> intersected, out Vector<float> distance, out Vector3Wide closestA, out Vector3Wide normal)
        {
            Matrix3x3Wide.CreateFromQuaternion(ref orientationA, out var rA);
            Matrix3x3Wide.CreateFromQuaternion(ref orientationB, out var rB);
            var negateX = Vector.LessThan(offsetB.X, Vector<float>.Zero);
            var negateY = Vector.LessThan(offsetB.Y, Vector<float>.Zero);
            var negateZ = Vector.LessThan(offsetB.Z, Vector<float>.Zero);
            Vector3Wide signedHalfExtentsA;
            signedHalfExtentsA.X = Vector.ConditionalSelect(negateX, -a.HalfWidth, a.HalfWidth);
            signedHalfExtentsA.Y = Vector.ConditionalSelect(negateY, -a.HalfHeight, a.HalfHeight);
            signedHalfExtentsA.Z = Vector.ConditionalSelect(negateZ, -a.HalfLength, a.HalfLength);
            Vector3Wide signedHalfExtentsB;
            signedHalfExtentsB.X = Vector.ConditionalSelect(negateX, b.HalfWidth, -b.HalfWidth);
            signedHalfExtentsB.Y = Vector.ConditionalSelect(negateY, b.HalfHeight, -b.HalfHeight);
            signedHalfExtentsB.Z = Vector.ConditionalSelect(negateZ, b.HalfLength, -b.HalfLength);

            Vector3Wide.Scale(ref rA.X, ref signedHalfExtentsA.X, out Vector3Wide xa);
            Vector3Wide.Scale(ref rA.Y, ref signedHalfExtentsA.Y, out Vector3Wide ya);
            Vector3Wide.Scale(ref rA.Z, ref signedHalfExtentsA.Z, out Vector3Wide za);
            Vector3Wide.Scale(ref rB.X, ref signedHalfExtentsB.X, out Vector3Wide xb);
            Vector3Wide.Scale(ref rB.Y, ref signedHalfExtentsB.Y, out Vector3Wide yb);
            Vector3Wide.Scale(ref rB.Z, ref signedHalfExtentsB.Z, out Vector3Wide zb);
            //aX x bX
            Vector3Wide.Add(ref ya, ref za, out var edgeCenterA);
            Vector3Wide.Add(ref yb, ref zb, out var edgeCenterB);
            TestEdgeEdge(ref a.HalfWidth, ref edgeCenterA, ref rA.X, ref b.HalfWidth, ref edgeCenterB, ref rB.X, ref offsetB,
                out distance, out normal, out closestA);
            //aX x bY
            Vector3Wide.Add(ref xb, ref zb, out edgeCenterB);
            TestEdgeEdge(ref a.HalfWidth, ref edgeCenterA, ref rA.X, ref b.HalfHeight, ref edgeCenterB, ref rB.Y, ref offsetB, 
                out var distanceCandidate, out var normalCandidate, out var closestACandidate);
            Select(ref distance, ref normal, ref closestA, ref distanceCandidate, ref normalCandidate, ref closestACandidate);
            //aX x bZ
            Vector3Wide.Add(ref xb, ref yb, out edgeCenterB);
            TestEdgeEdge(ref a.HalfWidth, ref edgeCenterA, ref rA.X, ref b.HalfLength, ref edgeCenterB, ref rB.Z, ref offsetB,
                out distanceCandidate, out normalCandidate, out closestACandidate);
            Select(ref distance, ref normal, ref closestA, ref distanceCandidate, ref normalCandidate, ref closestACandidate);
            //aY x bX
            Vector3Wide.Add(ref xa, ref za, out edgeCenterA);
            Vector3Wide.Add(ref yb, ref zb, out edgeCenterB);
            TestEdgeEdge(ref a.HalfHeight, ref edgeCenterA, ref rA.Y, ref b.HalfWidth, ref edgeCenterB, ref rB.X, ref offsetB,
                out distanceCandidate, out normalCandidate, out closestACandidate);
            Select(ref distance, ref normal, ref closestA, ref distanceCandidate, ref normalCandidate, ref closestACandidate);
            //aY x bY
            Vector3Wide.Add(ref xb, ref zb, out edgeCenterB);
            TestEdgeEdge(ref a.HalfHeight, ref edgeCenterA, ref rA.Y, ref b.HalfHeight, ref edgeCenterB, ref rB.Y, ref offsetB,
                out distanceCandidate, out normalCandidate, out closestACandidate);
            Select(ref distance, ref normal, ref closestA, ref distanceCandidate, ref normalCandidate, ref closestACandidate);
            //aY x bZ
            Vector3Wide.Add(ref xb, ref yb, out edgeCenterB);
            TestEdgeEdge(ref a.HalfHeight, ref edgeCenterA, ref rA.Y, ref b.HalfLength, ref edgeCenterB, ref rB.Z, ref offsetB,
                out distanceCandidate, out normalCandidate, out closestACandidate);
            Select(ref distance, ref normal, ref closestA, ref distanceCandidate, ref normalCandidate, ref closestACandidate);
            //aZ x bX
            Vector3Wide.Add(ref xa, ref ya, out edgeCenterA);
            Vector3Wide.Add(ref yb, ref zb, out edgeCenterB);
            TestEdgeEdge(ref a.HalfLength, ref edgeCenterA, ref rA.Z, ref b.HalfWidth, ref edgeCenterB, ref rB.X, ref offsetB,
                out distanceCandidate, out normalCandidate, out closestACandidate);
            Select(ref distance, ref normal, ref closestA, ref distanceCandidate, ref normalCandidate, ref closestACandidate);
            //aZ x bY
            Vector3Wide.Add(ref xb, ref zb, out edgeCenterB);
            TestEdgeEdge(ref a.HalfLength, ref edgeCenterA, ref rA.Z, ref b.HalfHeight, ref edgeCenterB, ref rB.Y, ref offsetB,
                out distanceCandidate, out normalCandidate, out closestACandidate);
            Select(ref distance, ref normal, ref closestA, ref distanceCandidate, ref normalCandidate, ref closestACandidate);
            //aZ x bZ
            Vector3Wide.Add(ref xb, ref yb, out edgeCenterB);
            TestEdgeEdge(ref a.HalfLength, ref edgeCenterA, ref rA.Z, ref b.HalfLength, ref edgeCenterB, ref rB.Z, ref offsetB,
                out distanceCandidate, out normalCandidate, out closestACandidate);
            Select(ref distance, ref normal, ref closestA, ref distanceCandidate, ref normalCandidate, ref closestACandidate);


        }

    }
}
