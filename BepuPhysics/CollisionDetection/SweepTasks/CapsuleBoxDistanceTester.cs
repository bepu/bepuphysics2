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
            ref Vector<float> offsetAX, ref Vector<float> offsetAY, ref Vector<float> offsetAZ,
            ref Vector<float> capsuleAxisX, ref Vector<float> capsuleAxisY, ref Vector<float> capsuleAxisZ,
            ref Vector<float> boxHalfWidth, ref Vector<float> boxHalfHeight, ref Vector<float> boxHalfLength,
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
        static void GetEdgeClosestPoint(ref Vector3Wide normal, ref Vector<int> edgeDirectionIndex,
            ref BoxWide box,
            ref Vector3Wide offsetA, ref Vector3Wide capsuleAxis, ref Vector<float> capsuleHalfLength, out Vector3Wide closestPointFromEdge)
        {
            Vector3Wide.Dot(ref normal, ref offsetA, out var calibrationDot);
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
            Vector3Wide.Subtract(ref boxEdgeCenter, ref offsetA, out var ab);
            Vector3Wide.Dot(ref ab, ref capsuleAxis, out var abda);
            Vector3Wide.Dot(ref ab, ref boxEdgeDirection, out var abdb);
            Vector3Wide.Dot(ref capsuleAxis, ref boxEdgeDirection, out var dadb);

            //Note division by zero guard.
            var ta = (abda - abdb * dadb) / Vector.Max(new Vector<float>(1e-15f), (Vector<float>.One - dadb * dadb));

            //In some cases, ta won't be in a useful location. Need to constrain it to the projection of the box edge onto the capsule edge.
            //B onto A: +-BHalfExtent * (da * db) + da * ab
            var bHalfExtent = Vector.ConditionalSelect(useEdgeX, box.HalfWidth, Vector.ConditionalSelect(useEdgeY, box.HalfHeight, box.HalfLength));
            var bOntoAOffset = bHalfExtent * Vector.Abs(dadb);
            var taMin = Vector.Max(-capsuleHalfLength, Vector.Min(capsuleHalfLength, abda - bOntoAOffset));
            var taMax = Vector.Min(capsuleHalfLength, Vector.Max(-capsuleHalfLength, abda + bOntoAOffset));
            ta = Vector.Min(Vector.Max(ta, taMin), taMax);

            Vector3Wide.Scale(ref capsuleAxis, ref ta, out var offsetAlongCapsule);
            Vector3Wide.Add(ref offsetA, ref offsetAlongCapsule, out closestPointFromEdge);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestEndpointNormal(ref Vector3Wide offsetA, ref Vector3Wide capsuleAxis, ref Vector<float> capsuleHalfLength, ref Vector3Wide endpoint,
            ref BoxWide box, out Vector<float> depth, out Vector3Wide normal)
        {
            Vector3Wide clamped;
            clamped.X = Vector.Min(box.HalfWidth, Vector.Max(-box.HalfWidth, endpoint.X));
            clamped.Y = Vector.Min(box.HalfHeight, Vector.Max(-box.HalfHeight, endpoint.Y));
            clamped.Z = Vector.Min(box.HalfLength, Vector.Max(-box.HalfLength, endpoint.Z));
            Vector3Wide.Subtract(ref endpoint, ref clamped, out normal);

            Vector3Wide.Length(ref normal, out var length);
            var inverseLength = Vector<float>.One / length;
            Vector3Wide.Scale(ref normal, ref inverseLength, out normal);
            //The dot between the offset from B to A and the normal gives us the center offset. 
            //The dot between the capsule axis and normal gives us the (unscaled) extent of the capsule along the normal.
            //The depth is (boxExtentAlongNormal + capsuleExtentAlongNormal) - separationAlongNormal.
            Vector3Wide.Dot(ref offsetA, ref normal, out var baN);
            Vector3Wide.Dot(ref capsuleAxis, ref normal, out var daN);
            depth = 
                Vector.Abs(normal.X) * box.HalfWidth + Vector.Abs(normal.Y) * box.HalfHeight + Vector.Abs(normal.Z) * box.HalfLength + 
                Vector.Abs(daN * capsuleHalfLength) - 
                Vector.Abs(baN);
            //If the endpoint doesn't generate a valid normal due to containment, ignore the depth result.
            depth = Vector.ConditionalSelect(Vector.GreaterThan(length, new Vector<float>(1e-10f)), depth, new Vector<float>(float.MaxValue));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void SelectForEdge(
            ref Vector<float> edgeDepth, ref Vector3Wide edgeLocalNormal, ref Vector<int> edgeDirectionIndex,
            ref Vector<float> edgeDepthCandidate, ref Vector3Wide edgeLocalNormalCandidate, Vector<int> edgeDirectionIndexCandidate)
        {
            var useCandidate = Vector.LessThan(edgeDepthCandidate, edgeDepth);
            edgeDepth = Vector.Min(edgeDepth, edgeDepthCandidate);
            Vector3Wide.ConditionalSelect(useCandidate, edgeLocalNormalCandidate, edgeLocalNormal, out edgeLocalNormal);
            edgeDirectionIndex = Vector.ConditionalSelect(useCandidate, edgeDirectionIndexCandidate, edgeDirectionIndex);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void Select(
            ref Vector<float> depth, ref Vector3Wide localNormal, ref Vector3Wide localClosest,
            ref Vector<float> depthCandidate, ref Vector3Wide localNormalCandidate, ref Vector3Wide localClosestCandidate)
        {
            var useCandidate = Vector.LessThan(depthCandidate, depth);
            depth = Vector.Min(depth, depthCandidate);
            Vector3Wide.ConditionalSelect(useCandidate, localNormalCandidate, localNormal, out localNormal);
            Vector3Wide.ConditionalSelect(useCandidate, localClosestCandidate, localClosest, out localClosest);
        }

        public void Test(ref CapsuleWide a, ref BoxWide b, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Vector<int> intersected, out Vector<float> distance, out Vector3Wide closestA, out Vector3Wide normal)
        {
            //Bring the capsule into the box's local space.
            Matrix3x3Wide.CreateFromQuaternion(ref orientationB, out var rB);
            QuaternionWide.TransformUnitY(ref orientationA, out var capsuleAxis);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ref capsuleAxis, ref rB, out var localCapsuleAxis);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ref offsetB, ref rB, out var localOffsetB);
            Vector3Wide.Negate(ref localOffsetB, out var localOffsetA);

            Vector3Wide.Scale(ref localCapsuleAxis, ref a.HalfLength, out var endpointOffset);
            Vector3Wide.Subtract(ref localOffsetA, ref endpointOffset, out var localClosest);
            TestEndpointNormal(ref localOffsetA, ref localCapsuleAxis, ref a.HalfLength, ref localClosest, ref b, out var depth, out var localNormal);
            Vector3Wide.Add(ref localOffsetA, ref endpointOffset, out var endpoint1);
            TestEndpointNormal(ref localOffsetA, ref localCapsuleAxis, ref a.HalfLength, ref endpoint1, ref b, out var depthCandidate, out var localNormalCandidate);
            Select(ref depth, ref localNormal, ref localClosest, ref depthCandidate, ref localNormalCandidate, ref endpoint1);

            Vector3Wide edgeLocalNormal, edgeLocalNormalCandidate;
            //Swizzle XYZ -> YZX
            TestBoxEdge(ref localOffsetA.Y, ref localOffsetA.Z, ref localOffsetA.X,
                ref localCapsuleAxis.Y, ref localCapsuleAxis.Z, ref localCapsuleAxis.X,
                ref b.HalfHeight, ref b.HalfLength, ref b.HalfWidth,
                out var edgeDepth, out edgeLocalNormal.Y, out edgeLocalNormal.Z, out edgeLocalNormal.X);
            var edgeDirectionIndex = Vector<int>.Zero;
            //Swizzle XYZ -> ZXY
            TestBoxEdge(ref localOffsetA.Z, ref localOffsetA.X, ref localOffsetA.Y,
                ref localCapsuleAxis.Z, ref localCapsuleAxis.X, ref localCapsuleAxis.Y,
                ref b.HalfLength, ref b.HalfWidth, ref b.HalfHeight,
                out var edgeDepthCandidate, out edgeLocalNormalCandidate.Z, out edgeLocalNormalCandidate.X, out edgeLocalNormalCandidate.Y);
            SelectForEdge(ref edgeDepth, ref edgeLocalNormal, ref edgeDirectionIndex, ref edgeDepthCandidate, ref edgeLocalNormalCandidate, Vector<int>.One);
            //Swizzle XYZ -> XYZ
            TestBoxEdge(ref localOffsetA.X, ref localOffsetA.Y, ref localOffsetA.Z,
                ref localCapsuleAxis.X, ref localCapsuleAxis.Y, ref localCapsuleAxis.Z,
                ref b.HalfWidth, ref b.HalfHeight, ref b.HalfLength,
                out edgeDepthCandidate, out edgeLocalNormalCandidate.X, out edgeLocalNormalCandidate.Y, out edgeLocalNormalCandidate.Z);
            SelectForEdge(ref edgeDepth, ref edgeLocalNormal, ref edgeDirectionIndex, ref edgeDepthCandidate, ref edgeLocalNormalCandidate, new Vector<int>(2));

            //We can skip the edge finalization if they aren't ever used.
            if (Vector.LessThanAny(edgeDepth, depth))
            {
                GetEdgeClosestPoint(ref edgeLocalNormal, ref edgeDirectionIndex, ref b, ref localOffsetA, ref localCapsuleAxis, ref a.HalfLength, out var edgeLocalClosest);
                Select(ref depth, ref localNormal, ref localClosest, ref edgeDepth, ref edgeLocalNormal, ref edgeLocalClosest);
            }

            //Transform normal and closest point back into world space.
            Matrix3x3Wide.TransformWithoutOverlap(ref localNormal, ref rB, out normal);
            Matrix3x3Wide.TransformWithoutOverlap(ref localClosest, ref rB, out closestA);
            Vector3Wide.Add(ref closestA, ref offsetB, out closestA);
            Vector3Wide.Scale(ref normal, ref a.Radius, out var closestOffset);
            Vector3Wide.Subtract(ref closestA, ref closestOffset, out closestA);
            distance = -depth - a.Radius;
            intersected = Vector.LessThan(distance, Vector<float>.Zero);

        }

        
    }


}
