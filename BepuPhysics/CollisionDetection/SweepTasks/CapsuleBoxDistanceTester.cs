using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    public struct CapsuleBoxDistanceTester : IPairDistanceTester<CapsuleWide, BoxWide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestBoxEdge(
            in Vector<float> offsetAX, in Vector<float> offsetAY, in Vector<float> offsetAZ,
            in Vector<float> capsuleAxisX, in Vector<float> capsuleAxisY, in Vector<float> capsuleAxisZ,
            in Vector<float> boxHalfWidth, in Vector<float> boxHalfHeight, in Vector<float> boxHalfLength,
            out Vector<float> depth, out Vector<float> nX, out Vector<float> nY, out Vector<float> nZ)
        {
            //Assume edge Z is being tested. Input will be swizzled to match.
            var length = Vector.SquareRoot(capsuleAxisX * capsuleAxisX + capsuleAxisY * capsuleAxisY);
            var inverseLength = Vector<float>.One / length;
            var useFallbackNormal = Vector.LessThan(length, new Vector<float>(1e-7f));
            nX = Vector.ConditionalSelect(useFallbackNormal, Vector<float>.One, -capsuleAxisY * inverseLength);
            nY = Vector.ConditionalSelect(useFallbackNormal, Vector<float>.Zero, capsuleAxisX * inverseLength);
            nZ = Vector<float>.Zero;

            //The normal is perpendicular to the capsule axis, so the capsule's extent is 0 centered on offsetA * N.
            depth = Vector.Abs(nX) * boxHalfWidth + Vector.Abs(nY) * boxHalfHeight + Vector.Abs(nZ) * boxHalfLength - Vector.Abs(offsetAX * nX + offsetAY * nY + offsetAZ * nZ);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void GetEdgeClosestPoint(ref Vector3Wide normal, in Vector<int> edgeDirectionIndex,
            in BoxWide box,
            in Vector3Wide offsetA, in Vector3Wide capsuleAxis, in Vector<float> capsuleHalfLength, out Vector3Wide closestPointFromEdge)
        {
            Vector3Wide.Dot(normal, offsetA, out var calibrationDot);
            var flipNormal = Vector.LessThan(calibrationDot, Vector<float>.Zero);
            normal.X = Vector.ConditionalSelect(flipNormal, -normal.X, normal.X);
            normal.Y = Vector.ConditionalSelect(flipNormal, -normal.Y, normal.Y);
            normal.Z = Vector.ConditionalSelect(flipNormal, -normal.Z, normal.Z);
            Vector3Wide boxEdgeCenter;
            //Note that this can result in a closest point in the middle of the box when the capsule is parallel to a face. That's fine.
            boxEdgeCenter.X = Vector.ConditionalSelect(Vector.Equals(normal.X, Vector<float>.Zero), Vector<float>.Zero,
                Vector.ConditionalSelect(Vector.GreaterThan(normal.X, Vector<float>.Zero), box.HalfWidth, -box.HalfWidth));
            boxEdgeCenter.Y = Vector.ConditionalSelect(Vector.Equals(normal.Y, Vector<float>.Zero), Vector<float>.Zero,
                Vector.ConditionalSelect(Vector.GreaterThan(normal.Y, Vector<float>.Zero), box.HalfHeight, -box.HalfHeight));
            boxEdgeCenter.Z = Vector.ConditionalSelect(Vector.Equals(normal.Z, Vector<float>.Zero), Vector<float>.Zero,
                Vector.ConditionalSelect(Vector.GreaterThan(normal.Z, Vector<float>.Zero), box.HalfLength, -box.HalfLength));

            Vector3Wide boxEdgeDirection;
            var useEdgeX = Vector.Equals(edgeDirectionIndex, Vector<int>.Zero);
            var useEdgeY = Vector.Equals(edgeDirectionIndex, Vector<int>.One);
            var useEdgeZ = Vector.Equals(edgeDirectionIndex, new Vector<int>(2));
            boxEdgeDirection.X = Vector.ConditionalSelect(useEdgeX, Vector<float>.One, Vector<float>.Zero);
            boxEdgeDirection.Y = Vector.ConditionalSelect(useEdgeY, Vector<float>.One, Vector<float>.Zero);
            boxEdgeDirection.Z = Vector.ConditionalSelect(useEdgeZ, Vector<float>.One, Vector<float>.Zero);


            //From CapsulePairCollisionTask, point of closest approach along the capsule axis, unbounded:
            //ta = (da * (b - a) + (db * (a - b)) * (da * db)) / (1 - ((da * db) * (da * db))  
            //where da = capsuleAxis, db = boxEdgeDirection, a = offsetA, b = boxEdgeCenter
            Vector3Wide.Subtract(boxEdgeCenter, offsetA, out var ab);
            Vector3Wide.Dot(ab, capsuleAxis, out var abda);
            Vector3Wide.Dot(ab, boxEdgeDirection, out var abdb);
            Vector3Wide.Dot(capsuleAxis, boxEdgeDirection, out var dadb);

            //Note division by zero guard.
            var ta = (abda - abdb * dadb) / Vector.Max(new Vector<float>(1e-15f), (Vector<float>.One - dadb * dadb));

            //In some cases, ta won't be in a useful location. Need to constrain it to the projection of the box edge onto the capsule edge.
            //B onto A: +-BHalfExtent * (da * db) + da * ab
            var bHalfExtent = Vector.ConditionalSelect(useEdgeX, box.HalfWidth, Vector.ConditionalSelect(useEdgeY, box.HalfHeight, box.HalfLength));
            var bOntoAOffset = bHalfExtent * Vector.Abs(dadb);
            var taMin = Vector.Max(-capsuleHalfLength, Vector.Min(capsuleHalfLength, abda - bOntoAOffset));
            var taMax = Vector.Min(capsuleHalfLength, Vector.Max(-capsuleHalfLength, abda + bOntoAOffset));
            ta = Vector.Min(Vector.Max(ta, taMin), taMax);

            Vector3Wide.Scale(capsuleAxis, ta, out var offsetAlongCapsule);
            Vector3Wide.Add(offsetA, offsetAlongCapsule, out closestPointFromEdge);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestEndpointNormal(in Vector3Wide offsetA, in Vector3Wide capsuleAxis, in Vector<float> capsuleHalfLength, in Vector3Wide endpoint,
            in BoxWide box, out Vector<float> depth, out Vector3Wide normal)
        {
            Vector3Wide clamped;
            clamped.X = Vector.Min(box.HalfWidth, Vector.Max(-box.HalfWidth, endpoint.X));
            clamped.Y = Vector.Min(box.HalfHeight, Vector.Max(-box.HalfHeight, endpoint.Y));
            clamped.Z = Vector.Min(box.HalfLength, Vector.Max(-box.HalfLength, endpoint.Z));
            Vector3Wide.Subtract(endpoint, clamped, out normal);

            Vector3Wide.Length(normal, out var length);
            var inverseLength = Vector<float>.One / length;
            Vector3Wide.Scale(normal, inverseLength, out normal);
            //The dot between the offset from B to A and the normal gives us the center offset. 
            //The dot between the capsule axis and normal gives us the (unscaled) extent of the capsule along the normal.
            //The depth is (boxExtentAlongNormal + capsuleExtentAlongNormal) - separationAlongNormal.
            Vector3Wide.Dot(offsetA, normal, out var baN);
            Vector3Wide.Dot(capsuleAxis, normal, out var daN);
            depth =
                Vector.Abs(normal.X) * box.HalfWidth + Vector.Abs(normal.Y) * box.HalfHeight + Vector.Abs(normal.Z) * box.HalfLength +
                Vector.Abs(daN * capsuleHalfLength) -
                Vector.Abs(baN);
            //If the endpoint doesn't generate a valid normal due to containment, ignore the depth result.
            depth = Vector.ConditionalSelect(Vector.GreaterThan(length, new Vector<float>(1e-10f)), depth, new Vector<float>(float.MaxValue));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestVertexAxis(in BoxWide box, in Vector3Wide offsetA, in Vector3Wide capsuleAxis, in Vector<float> capsuleHalfLength,
            out Vector<float> depth, out Vector3Wide normal, out Vector3Wide closestA)
        {
            //The available feature pairs between the capsule axis and box are:
            //Box edge - capsule endpoints: handled by capsule endpoint clamping 
            //Box edge - capsule axis: handled explicitly by the edge cases
            //Box face - capsule endpoints: handled by capsule endpoint clamping
            //Box face - capsule axis: redundant with edge-axis or face-endpoint
            //Box vertex - capsule endpoints: handled by capsule endpoint clamping
            //Box vertex - capsule axis: unhandled
            //So here, we need to identify the maximum separating axis caused by vertices. 
            //We can safely ignore cases that are handled by the endpoint clamp, too. 
            //A brute force approach could test every vertex-axis offset as a normal, but more cleverness is possible.
            //Note that this case requires that the capsule axis be in the voronoi region of a vertex. That is only possible if the offset from the box origin to the capsule axis
            //supports an extreme point at the vertex. 
            //(That extreme point relationship may also be met in cases of intersection, but that's fine- this distance tester is not concerned with intersection beyond a boolean result.)

            //closest point on axis to origin = offsetA - (offsetA * capsuleAxis) * capsuleAxis
            Vector3Wide.Dot(offsetA, capsuleAxis, out var dot);
            var clampedDot = Vector.Min(capsuleHalfLength, Vector.Max(-capsuleHalfLength, dot));
            Vector3Wide.Scale(capsuleAxis, clampedDot, out var axisOffset);
            Vector3Wide.Subtract(offsetA, axisOffset, out var closestOnAxis);

            Vector3Wide vertex;
            vertex.X = Vector.ConditionalSelect(Vector.LessThan(closestOnAxis.X, Vector<float>.Zero), -box.HalfWidth, box.HalfWidth);
            vertex.Y = Vector.ConditionalSelect(Vector.LessThan(closestOnAxis.Y, Vector<float>.Zero), -box.HalfHeight, box.HalfHeight);
            vertex.Z = Vector.ConditionalSelect(Vector.LessThan(closestOnAxis.Z, Vector<float>.Zero), -box.HalfLength, box.HalfLength);

            //closest point on axis to vertex: ((vertex - offsetA) * capsuleAxis) * capsuleAxis + offsetA - vertex
            Vector3Wide.Subtract(vertex, offsetA, out var capsuleCenterToVertex);
            Vector3Wide.Dot(capsuleCenterToVertex, capsuleAxis, out var vertexDot);
            Vector3Wide.Scale(capsuleAxis, vertexDot, out var vertexAxisOffset);
            Vector3Wide.Add(vertexAxisOffset, offsetA, out closestA);
            Vector3Wide.Subtract(closestA, vertex, out var vertexToClosestOnCapsule);

            Vector3Wide.Length(vertexToClosestOnCapsule, out var length);
            var inverseLength = Vector<float>.One / length;
            Vector3Wide.Scale(vertexToClosestOnCapsule, inverseLength, out normal);
            //The normal is perpendicular to the capsule axis by construction, so no need to include the capsule length extent.
            depth = Vector.Abs(normal.X) * box.HalfWidth + Vector.Abs(normal.Y) * box.HalfHeight + Vector.Abs(normal.Z) * box.HalfLength -
                Vector.Abs(offsetA.X * normal.X + offsetA.Y * normal.Y + offsetA.Z * normal.Z);
            //Ignore degenerate cases. Worst outcome is that it reports intersection, which is pretty reasonable.
            depth = Vector.ConditionalSelect(Vector.LessThan(length, new Vector<float>(1e-10f)), new Vector<float>(float.MaxValue), depth);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void SelectForEdge(
            ref Vector<float> edgeDepth, ref Vector3Wide edgeLocalNormal, ref Vector<int> edgeDirectionIndex,
            ref Vector<float> edgeDepthCandidate, ref Vector3Wide edgeLocalNormalCandidate, Vector<int> edgeDirectionIndexCandidate)
        {
            var useCandidate = Vector.LessThan(edgeDepthCandidate, edgeDepth);
            edgeDepth = Vector.Min(edgeDepth, edgeDepthCandidate);
            Vector3Wide.ConditionalSelect(useCandidate, edgeLocalNormalCandidate, edgeLocalNormal, out edgeLocalNormal);
            edgeDirectionIndex = Vector.ConditionalSelect(useCandidate, edgeDirectionIndexCandidate, edgeDirectionIndex);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void Select(
            ref Vector<float> depth, ref Vector3Wide localNormal, ref Vector3Wide localClosest,
            ref Vector<float> depthCandidate, ref Vector3Wide localNormalCandidate, ref Vector3Wide localClosestCandidate)
        {
            var useCandidate = Vector.LessThan(depthCandidate, depth);
            depth = Vector.Min(depth, depthCandidate);
            Vector3Wide.ConditionalSelect(useCandidate, localNormalCandidate, localNormal, out localNormal);
            Vector3Wide.ConditionalSelect(useCandidate, localClosestCandidate, localClosest, out localClosest);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void Select(
            ref Vector<float> depth, ref Vector3Wide localNormal,
            ref Vector<float> depthCandidate, ref Vector3Wide localNormalCandidate)
        {
            var useCandidate = Vector.LessThan(depthCandidate, depth);
            depth = Vector.Min(depth, depthCandidate);
            Vector3Wide.ConditionalSelect(useCandidate, localNormalCandidate, localNormal, out localNormal);
        }

        public void Test(in CapsuleWide a, in BoxWide b, in Vector3Wide offsetB, in QuaternionWide orientationA, in QuaternionWide orientationB, in Vector<int> inactiveLanes,
            out Vector<int> intersected, out Vector<float> distance, out Vector3Wide closestA, out Vector3Wide normal)
        {
            //Bring the capsule into the box's local space.
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var rB);
            var capsuleAxis = QuaternionWide.TransformUnitY(orientationA);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(capsuleAxis, rB, out var localCapsuleAxis);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, rB, out var localOffsetB);
            Vector3Wide.Negate(localOffsetB, out var localOffsetA);

            Vector3Wide.Scale(localCapsuleAxis, a.HalfLength, out var endpointOffset);
            Vector3Wide.Subtract(localOffsetA, endpointOffset, out var endpoint0);
            TestEndpointNormal(localOffsetA, localCapsuleAxis, a.HalfLength, endpoint0, b, out var depth, out var localNormal);
            Vector3Wide.Add(localOffsetA, endpointOffset, out var endpoint1);
            TestEndpointNormal(localOffsetA, localCapsuleAxis, a.HalfLength, endpoint1, b, out var depthCandidate, out var localNormalCandidate);
            Select(ref depth, ref localNormal, ref depthCandidate, ref localNormalCandidate);
            //Note that we did not yet pick a closest point for endpoint cases. That's because each case only generates a normal and interval test, not a minimal distance test.
            //The choice of which endpoint is actually closer is deferred until now.
            Vector3Wide.Dot(localCapsuleAxis, localNormal, out var endpointChoiceDot);
            Vector3Wide.ConditionalSelect(Vector.LessThan(endpointChoiceDot, Vector<float>.Zero), endpoint1, endpoint0, out var localClosest);

            Vector3Wide edgeLocalNormal, edgeLocalNormalCandidate;
            //Swizzle XYZ -> YZX
            TestBoxEdge(localOffsetA.Y, localOffsetA.Z, localOffsetA.X,
                localCapsuleAxis.Y, localCapsuleAxis.Z, localCapsuleAxis.X,
                b.HalfHeight, b.HalfLength, b.HalfWidth,
                out var edgeDepth, out edgeLocalNormal.Y, out edgeLocalNormal.Z, out edgeLocalNormal.X);
            var edgeDirectionIndex = Vector<int>.Zero;
            //Swizzle XYZ -> ZXY
            TestBoxEdge(localOffsetA.Z, localOffsetA.X, localOffsetA.Y,
                localCapsuleAxis.Z, localCapsuleAxis.X, localCapsuleAxis.Y,
                b.HalfLength, b.HalfWidth, b.HalfHeight,
                out var edgeDepthCandidate, out edgeLocalNormalCandidate.Z, out edgeLocalNormalCandidate.X, out edgeLocalNormalCandidate.Y);
            SelectForEdge(ref edgeDepth, ref edgeLocalNormal, ref edgeDirectionIndex, ref edgeDepthCandidate, ref edgeLocalNormalCandidate, Vector<int>.One);
            //Swizzle XYZ -> XYZ
            TestBoxEdge(localOffsetA.X, localOffsetA.Y, localOffsetA.Z,
                localCapsuleAxis.X, localCapsuleAxis.Y, localCapsuleAxis.Z,
                b.HalfWidth, b.HalfHeight, b.HalfLength,
                out edgeDepthCandidate, out edgeLocalNormalCandidate.X, out edgeLocalNormalCandidate.Y, out edgeLocalNormalCandidate.Z);
            SelectForEdge(ref edgeDepth, ref edgeLocalNormal, ref edgeDirectionIndex, ref edgeDepthCandidate, ref edgeLocalNormalCandidate, new Vector<int>(2));

            //We can skip the edge finalization if they aren't ever used.
            if (Vector.LessThanAny(edgeDepth, depth))
            {
                GetEdgeClosestPoint(ref edgeLocalNormal, edgeDirectionIndex, b, localOffsetA, localCapsuleAxis, a.HalfLength, out var edgeLocalClosest);
                Select(ref depth, ref localNormal, ref localClosest, ref edgeDepth, ref edgeLocalNormal, ref edgeLocalClosest);
            }

            TestVertexAxis(b, localOffsetA, localCapsuleAxis, a.HalfLength, out depthCandidate, out localNormalCandidate, out var localClosestCandidate);
            Select(ref depth, ref localNormal, ref localClosest, ref depthCandidate, ref localNormalCandidate, ref localClosestCandidate);

            //Transform normal and closest point back into world space.
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, rB, out normal);
            Matrix3x3Wide.TransformWithoutOverlap(localClosest, rB, out closestA);
            Vector3Wide.Add(closestA, offsetB, out closestA);
            Vector3Wide.Scale(normal, a.Radius, out var closestOffset);
            Vector3Wide.Subtract(closestA, closestOffset, out closestA);
            distance = -depth - a.Radius;
            intersected = Vector.LessThan(distance, Vector<float>.Zero);

        }


    }


}
