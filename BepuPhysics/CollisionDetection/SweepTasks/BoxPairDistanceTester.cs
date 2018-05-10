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
            //While we based the edgeCenterB on the rotation, we deferred the translation since it's common to all cases.
            Vector3Wide.Add(ref edgeCenterB, ref offsetB, out edgeCenterB);
            Vector3Wide.Subtract(ref edgeCenterA, ref edgeCenterB, out var edgeCentersOffsetBA);
            Vector3Wide.CrossWithoutOverlap(ref edgeDirectionA, ref edgeDirectionB, out normal);
            Vector3Wide.Length(ref normal, out var length);
            var inverseLength = Vector<float>.One / length;
            Vector3Wide.Scale(ref normal, ref inverseLength, out normal);
            Vector3Wide.Dot(ref normal, ref offsetB, out var calibrationDot);
            var shouldNegate = Vector.GreaterThan(calibrationDot, Vector<float>.Zero);
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
            //tb = ta * (da * db) - db * (b - a)
            var tb = ta * dadb + dbOffsetBA;
            Vector3Wide.Scale(ref edgeDirectionA, ref ta, out closestA);
            Vector3Wide.Add(ref closestA, ref edgeCenterA, out closestA);

            var outsideA = Vector.BitwiseOr(Vector.GreaterThan(ta, halfExtentA), Vector.LessThan(ta, -halfExtentA));
            var outsideB = Vector.BitwiseOr(Vector.GreaterThan(tb, halfExtentB), Vector.LessThan(tb, -halfExtentB));
            var closestPointOutsideOfEdges = Vector.BitwiseOr(outsideA, outsideB);
            var badNormal = Vector.LessThan(length, new Vector<float>(1e-15f));
            var edgeDistanceInvalid = Vector.BitwiseOr(badNormal, closestPointOutsideOfEdges);
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void SelectForVertex(
            ref Vector<float> distanceSquared, ref Vector3Wide offset, ref Vector3Wide closestA,
            ref Vector<float> distanceSquaredCandidate, ref Vector3Wide offsetCandidate, ref Vector3Wide closestACandidate)
        {
            var useCandidate = Vector.LessThanOrEqual(distanceSquaredCandidate, distanceSquared);
            distanceSquared = Vector.Min(distanceSquared, distanceSquaredCandidate);
            Vector3Wide.ConditionalSelect(ref useCandidate, ref closestACandidate, ref closestA, out closestA);
            Vector3Wide.ConditionalSelect(ref useCandidate, ref offsetCandidate, ref offset, out offset);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void VertexBox(ref BoxWide a, ref Vector3Wide x, ref Vector3Wide y, ref Vector3Wide z, ref Vector3Wide offsetB,
            out Vector<float> distanceSquared, out Vector3Wide vertexOffset, out Vector3Wide closestA)
        {
            Vector3Wide.Add(ref x, ref y, out var vertexB);
            Vector3Wide.Add(ref z, ref offsetB, out var add);
            Vector3Wide.Add(ref add, ref vertexB, out vertexB);
            closestA.X = Vector.Min(a.HalfWidth, Vector.Max(-a.HalfWidth, vertexB.X));
            closestA.Y = Vector.Min(a.HalfHeight, Vector.Max(-a.HalfHeight, vertexB.Y));
            closestA.Z = Vector.Min(a.HalfLength, Vector.Max(-a.HalfLength, vertexB.Z));
            Vector3Wide.Subtract(ref closestA, ref vertexB, out vertexOffset);
            Vector3Wide.Dot(ref vertexOffset, ref vertexOffset, out distanceSquared);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void VerticesBox(ref BoxWide a, ref BoxWide b, ref Matrix3x3Wide rA, ref Matrix3x3Wide rB, ref Vector3Wide offsetB,
            out Vector<float> distanceSquared, out Vector3Wide offset, out Vector3Wide closestA)
        {
            Matrix3x3Wide.MultiplyByTransposeWithoutOverlap(ref rB, ref rA, out var rBInA);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ref offsetB, ref rA, out var localOffsetB);
            Vector3Wide.Scale(ref rBInA.X, ref b.HalfWidth, out var x);
            Vector3Wide.Scale(ref rBInA.Y, ref b.HalfHeight, out var y);
            Vector3Wide.Scale(ref rBInA.Z, ref b.HalfLength, out var z);
            Vector3Wide.Negate(ref x, out var negativeX);
            Vector3Wide.Negate(ref y, out var negativeY);
            Vector3Wide.Negate(ref z, out var negativeZ);
            VertexBox(ref a, ref x, ref y, ref z, ref localOffsetB, out distanceSquared, out var localOffset, out var localClosestA);
            VertexBox(ref a, ref x, ref y, ref negativeZ, ref localOffsetB, out var distanceSquaredCandidate, out var offsetCandidate, out var closestACandidate);
            SelectForVertex(ref distanceSquared, ref localOffset, ref localClosestA, ref distanceSquaredCandidate, ref offsetCandidate, ref closestACandidate);
            VertexBox(ref a, ref x, ref negativeY, ref z, ref localOffsetB, out distanceSquaredCandidate, out offsetCandidate, out closestACandidate);
            SelectForVertex(ref distanceSquared, ref localOffset, ref localClosestA, ref distanceSquaredCandidate, ref offsetCandidate, ref closestACandidate);
            VertexBox(ref a, ref x, ref negativeY, ref negativeZ, ref localOffsetB, out distanceSquaredCandidate, out offsetCandidate, out closestACandidate);
            SelectForVertex(ref distanceSquared, ref localOffset, ref localClosestA, ref distanceSquaredCandidate, ref offsetCandidate, ref closestACandidate);
            VertexBox(ref a, ref negativeX, ref y, ref z, ref localOffsetB, out distanceSquaredCandidate, out offsetCandidate, out closestACandidate);
            SelectForVertex(ref distanceSquared, ref localOffset, ref localClosestA, ref distanceSquaredCandidate, ref offsetCandidate, ref closestACandidate);
            VertexBox(ref a, ref negativeX, ref y, ref negativeZ, ref localOffsetB, out distanceSquaredCandidate, out offsetCandidate, out closestACandidate);
            SelectForVertex(ref distanceSquared, ref localOffset, ref localClosestA, ref distanceSquaredCandidate, ref offsetCandidate, ref closestACandidate);
            VertexBox(ref a, ref negativeX, ref negativeY, ref z, ref localOffsetB, out distanceSquaredCandidate, out offsetCandidate, out closestACandidate);
            SelectForVertex(ref distanceSquared, ref localOffset, ref localClosestA, ref distanceSquaredCandidate, ref offsetCandidate, ref closestACandidate);
            VertexBox(ref a, ref negativeX, ref negativeY, ref negativeZ, ref localOffsetB, out distanceSquaredCandidate, out offsetCandidate, out closestACandidate);
            SelectForVertex(ref distanceSquared, ref localOffset, ref localClosestA, ref distanceSquaredCandidate, ref offsetCandidate, ref closestACandidate);

            //We were working in rA's local space; push it back into world.
            Matrix3x3Wide.TransformWithoutOverlap(ref localOffset, ref rA, out offset);
            Matrix3x3Wide.TransformWithoutOverlap(ref localClosestA, ref rA, out closestA);

        }

        public void Test(ref BoxWide a, ref BoxWide b, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Vector<int> intersected, out Vector<float> distance, out Vector3Wide closestA, out Vector3Wide normal)
        {
            Matrix3x3Wide.CreateFromQuaternion(ref orientationA, out var rA);
            Matrix3x3Wide.CreateFromQuaternion(ref orientationB, out var rB);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ref offsetB, ref rA, out var offsetBLocalA);
            Vector3Wide signedHalfExtentsA;
            signedHalfExtentsA.X = Vector.ConditionalSelect(Vector.LessThan(offsetBLocalA.X, Vector<float>.Zero), -a.HalfWidth, a.HalfWidth);
            signedHalfExtentsA.Y = Vector.ConditionalSelect(Vector.LessThan(offsetBLocalA.Y, Vector<float>.Zero), -a.HalfHeight, a.HalfHeight);
            signedHalfExtentsA.Z = Vector.ConditionalSelect(Vector.LessThan(offsetBLocalA.Z, Vector<float>.Zero), -a.HalfLength, a.HalfLength);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ref offsetB, ref rB, out var offsetBLocalB);
            Vector3Wide signedHalfExtentsB;
            signedHalfExtentsB.X = Vector.ConditionalSelect(Vector.LessThan(offsetBLocalB.X, Vector<float>.Zero), b.HalfWidth, -b.HalfWidth);
            signedHalfExtentsB.Y = Vector.ConditionalSelect(Vector.LessThan(offsetBLocalB.Y, Vector<float>.Zero), b.HalfHeight, -b.HalfHeight);
            signedHalfExtentsB.Z = Vector.ConditionalSelect(Vector.LessThan(offsetBLocalB.Z, Vector<float>.Zero), b.HalfLength, -b.HalfLength);

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
            
            //Periodically branching can be worth it. Sweeps are relatively coherent.
            if(Vector.LessThanOrEqualAll(distance, Vector<float>.Zero))
            {
                intersected = new Vector<int>(-1);
                return;
            }

            VerticesBox(ref a, ref b, ref rA, ref rB, ref offsetB, out var verticesDistanceSquared, out var verticesOffset, out var verticesClosest);
            if (Vector.EqualsAll(verticesDistanceSquared, Vector<float>.Zero))
            {
                intersected = new Vector<int>(-1);
                return;
            }
            Vector3Wide.Negate(ref offsetB, out var offsetA);
            VerticesBox(ref b, ref a, ref rB, ref rA, ref offsetA, out var verticesBDistanceSquared, out var verticesBOffset, out var verticesBClosest);
            //Note that the test of the vertices of B output the closest point as belonging to B, so we apply the offset to get the other point.
            Vector3Wide.Negate(ref verticesBOffset, out verticesBOffset);
            Vector3Wide.Add(ref verticesBClosest, ref verticesBOffset, out verticesBClosest);
            Vector3Wide.Add(ref verticesBClosest, ref offsetB, out verticesBClosest);
            SelectForVertex(ref verticesDistanceSquared, ref verticesOffset, ref verticesClosest, ref verticesBDistanceSquared, ref verticesBOffset, ref verticesBClosest);

            var verticesDistance = Vector.SquareRoot(verticesDistanceSquared);
            var verticesInverseDistance = Vector<float>.One / verticesDistance;
            Vector3Wide.Scale(ref verticesOffset, ref verticesInverseDistance, out var verticesNormal);

            Select(ref distance, ref normal, ref closestA, ref verticesDistance, ref verticesNormal, ref verticesClosest);
            intersected = Vector.LessThanOrEqual(distance, Vector<float>.Zero);

        }

    }
}
