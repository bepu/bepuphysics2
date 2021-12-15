using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using static BepuUtilities.GatherScatter;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct BoxPairTester : IPairTester<BoxWide, BoxWide, Convex4ContactManifoldWide>
    {
        public int BatchSize => 32;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestEdgeEdge(
            ref Vector<float> halfWidthA, ref Vector<float> halfHeightA, ref Vector<float> halfLengthA,
            ref Vector<float> halfWidthB, ref Vector<float> halfHeightB, ref Vector<float> halfLengthB,
            ref Vector<float> offsetBX, ref Vector<float> offsetBY, ref Vector<float> offsetBZ,
            ref Vector3Wide rBX, ref Vector3Wide rBY, ref Vector3Wide rBZ,
            ref Vector3Wide edgeBDirection,
            out Vector<float> depth, out Vector<float> localNormalAX, out Vector<float> localNormalAY, out Vector<float> localNormalAZ)
        {
            //Tests one axis of B against all three axes of A.
            var x2 = edgeBDirection.X * edgeBDirection.X;
            var y2 = edgeBDirection.Y * edgeBDirection.Y;
            var z2 = edgeBDirection.Z * edgeBDirection.Z;
            {
                //A.X x edgeB
                var length = Vector.SquareRoot(y2 + z2);
                var inverseLength = Vector<float>.One / length;
                localNormalAX = Vector<float>.Zero;
                localNormalAY = edgeBDirection.Z * inverseLength;
                localNormalAZ = -edgeBDirection.Y * inverseLength;
                var extremeA = Vector.Abs(localNormalAY) * halfHeightA + Vector.Abs(localNormalAZ) * halfLengthA;
                var nBX = localNormalAY * rBX.Y + localNormalAZ * rBX.Z;
                var nBY = localNormalAY * rBY.Y + localNormalAZ * rBY.Z;
                var nBZ = localNormalAY * rBZ.Y + localNormalAZ * rBZ.Z;
                var extremeB = Vector.Abs(nBX) * halfWidthB + Vector.Abs(nBY) * halfHeightB + Vector.Abs(nBZ) * halfLengthB;
                depth = extremeA + extremeB - Vector.Abs(offsetBY * localNormalAY + offsetBZ * localNormalAZ);
                depth = Vector.ConditionalSelect(Vector.LessThan(length, new Vector<float>(1e-7f)), new Vector<float>(float.MaxValue), depth);
            }
            {
                //A.Y x edgeB
                var length = Vector.SquareRoot(x2 + z2);
                var inverseLength = Vector<float>.One / length;
                var nX = edgeBDirection.Z * inverseLength;
                var nZ = -edgeBDirection.X * inverseLength;
                var extremeA = Vector.Abs(nX) * halfWidthA + Vector.Abs(nZ) * halfLengthA;
                var nBX = nX * rBX.X + nZ * rBX.Z;
                var nBY = nX * rBY.X + nZ * rBY.Z;
                var nBZ = nX * rBZ.X + nZ * rBZ.Z;
                var extremeB = Vector.Abs(nBX) * halfWidthB + Vector.Abs(nBY) * halfHeightB + Vector.Abs(nBZ) * halfLengthB;
                var d = extremeA + extremeB - Vector.Abs(offsetBX * nX + offsetBZ * nZ);
                d = Vector.ConditionalSelect(Vector.LessThan(length, new Vector<float>(1e-7f)), new Vector<float>(float.MaxValue), d);
                var useY = Vector.LessThan(d, depth);
                depth = Vector.ConditionalSelect(useY, d, depth);
                localNormalAX = Vector.ConditionalSelect(useY, nX, localNormalAX);
                localNormalAY = Vector.ConditionalSelect(useY, Vector<float>.Zero, localNormalAY);
                localNormalAZ = Vector.ConditionalSelect(useY, nZ, localNormalAZ);
            }
            {
                //A.Z x edgeB
                var length = Vector.SquareRoot(x2 + y2);
                var inverseLength = Vector<float>.One / length;
                var nX = edgeBDirection.Y * inverseLength;
                var nY = -edgeBDirection.X * inverseLength;
                var extremeA = Vector.Abs(nX) * halfWidthA + Vector.Abs(nY) * halfHeightA;
                var nBX = nX * rBX.X + nY * rBX.Y;
                var nBY = nX * rBY.X + nY * rBY.Y;
                var nBZ = nX * rBZ.X + nY * rBZ.Y;
                var extremeB = Vector.Abs(nBX) * halfWidthB + Vector.Abs(nBY) * halfHeightB + Vector.Abs(nBZ) * halfLengthB;
                var d = extremeA + extremeB - Vector.Abs(offsetBX * nX + offsetBY * nY);
                d = Vector.ConditionalSelect(Vector.LessThan(length, new Vector<float>(1e-7f)), new Vector<float>(float.MaxValue), d);
                var useZ = Vector.LessThan(d, depth);
                depth = Vector.ConditionalSelect(useZ, d, depth);
                localNormalAX = Vector.ConditionalSelect(useZ, nX, localNormalAX);
                localNormalAY = Vector.ConditionalSelect(useZ, nY, localNormalAY);
                localNormalAZ = Vector.ConditionalSelect(useZ, Vector<float>.Zero, localNormalAZ);
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestFace(ref Vector<float> halfLengthA,
            ref Vector<float> halfWidthB, ref Vector<float> halfHeightB, ref Vector<float> halfLengthB,
            ref Vector<float> offsetBZ,
            ref Vector3Wide rBX, ref Vector3Wide rBY, ref Vector3Wide rBZ,
            out Vector<float> depth)
        {
            depth = halfLengthA + halfWidthB * Vector.Abs(rBX.Z) + halfHeightB * Vector.Abs(rBY.Z) + halfLengthB * Vector.Abs(rBZ.Z) - Vector.Abs(offsetBZ);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Select(
            ref Vector<float> depth, ref Vector3Wide normal,
            ref Vector<float> candidateDepth, ref Vector<float> candidateNX, ref Vector<float> candidateNY, ref Vector<float> candidateNZ)
        {
            var useCandidate = Vector.LessThan(candidateDepth, depth);
            depth = Vector.ConditionalSelect(useCandidate, candidateDepth, depth);
            normal.X = Vector.ConditionalSelect(useCandidate, candidateNX, normal.X);
            normal.Y = Vector.ConditionalSelect(useCandidate, candidateNY, normal.Y);
            normal.Z = Vector.ConditionalSelect(useCandidate, candidateNZ, normal.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void AddBoxAVertex(in Vector3Wide vertex, in Vector<int> featureId, in Vector3Wide faceNormalB, in Vector3Wide contactNormal, in Vector<float> inverseContactNormalDotFaceNormalB,
            in Vector3Wide faceCenterB, in Vector3Wide faceTangentBX, in Vector3Wide faceTangentBY, in Vector<float> halfSpanBX, in Vector<float> halfSpanBY,
            ref ManifoldCandidate candidates, ref Vector<int> candidateCount, int pairCount, in Vector<int> allowContacts)
        {
            //Cast a ray from the box A vertex up to the box B face along the contact normal.
            Vector3Wide.Subtract(vertex, faceCenterB, out var pointOnBToVertex);
            Vector3Wide.Dot(faceNormalB, pointOnBToVertex, out var planeDistance);
            Vector3Wide.Scale(contactNormal, planeDistance * inverseContactNormalDotFaceNormalB, out var offset);
            //Contact normal points from B to A by convention, so we have to subtract.
            Vector3Wide.Subtract(vertex, offset, out var vertexOnBFace);

            Vector3Wide.Subtract(vertexOnBFace, faceCenterB, out var vertexOffsetOnBFace);
            Unsafe.SkipInit(out ManifoldCandidate candidate);
            Vector3Wide.Dot(vertexOffsetOnBFace, faceTangentBX, out candidate.X);
            Vector3Wide.Dot(vertexOffsetOnBFace, faceTangentBY, out candidate.Y);
            candidate.FeatureId = featureId;

            //Note that plane normals are assumed to point inward.
            var contained = Vector.BitwiseAnd(
                Vector.LessThanOrEqual(Vector.Abs(candidate.X), halfSpanBX),
                Vector.LessThanOrEqual(Vector.Abs(candidate.Y), halfSpanBY));

            //While we explicitly used an epsilon during edge contact generation, there is a risk of buffer overrun during the face vertex phase.
            //Rather than assuming our numerical epsilon is guaranteed to always work, explicitly clamp the count. This should essentially never be needed,
            //but it is very cheap and guarantees no memory stomping with a pretty reasonable fallback.
            var belowBufferCapacity = Vector.LessThan(candidateCount, new Vector<int>(8));
            var contactExists = Vector.BitwiseAnd(allowContacts, Vector.BitwiseAnd(contained, belowBufferCapacity));
            ManifoldCandidateHelper.AddCandidate(ref candidates, ref candidateCount, candidate, contactExists, pairCount);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void AddBoxAVertices(in Vector3Wide faceCenterB, in Vector3Wide faceTangentBX, in Vector3Wide faceTangentBY, in Vector<float> halfSpanBX, in Vector<float> halfSpanBY,
            in Vector3Wide faceNormalB, in Vector3Wide contactNormal,
            in Vector3Wide v00, in Vector3Wide v01, in Vector3Wide v10, in Vector3Wide v11,
            in Vector<int> f00, in Vector<int> f01, in Vector<int> f10, in Vector<int> f11,
            ref ManifoldCandidate candidates, ref Vector<int> candidateCount, int pairCount, in Vector<int> allowContacts)
        {
            Vector3Wide.Dot(faceNormalB, contactNormal, out var normalDot);
#if DEBUG
            //Note that we don't handle the case where the normalDot is negative. The contact normal points from B to A and the representative face was chosen based on alignment
            //with the contact normal, so there should never be a case where the contact normal and face normal are opposed.
            for (int i = 0; i < pairCount; ++i)
            {
                Debug.Assert(normalDot[i] >= 0);
            }
#endif
            var inverseContactNormalDotFaceNormalB = Vector.ConditionalSelect(Vector.GreaterThan(Vector.Abs(normalDot), new Vector<float>(1e-10f)), Vector<float>.One / normalDot, new Vector<float>(float.MaxValue));

            AddBoxAVertex(v00, f00, faceNormalB, contactNormal, inverseContactNormalDotFaceNormalB, faceCenterB, faceTangentBX, faceTangentBY, halfSpanBX, halfSpanBY,
                ref candidates, ref candidateCount, pairCount, allowContacts);
            AddBoxAVertex(v01, f01, faceNormalB, contactNormal, inverseContactNormalDotFaceNormalB, faceCenterB, faceTangentBX, faceTangentBY, halfSpanBX, halfSpanBY,
                ref candidates, ref candidateCount, pairCount, allowContacts);
            AddBoxAVertex(v10, f10, faceNormalB, contactNormal, inverseContactNormalDotFaceNormalB, faceCenterB, faceTangentBX, faceTangentBY, halfSpanBX, halfSpanBY,
                ref candidates, ref candidateCount, pairCount, allowContacts);
            AddBoxAVertex(v11, f11, faceNormalB, contactNormal, inverseContactNormalDotFaceNormalB, faceCenterB, faceTangentBX, faceTangentBY, halfSpanBX, halfSpanBY,
                ref candidates, ref candidateCount, pairCount, allowContacts);
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ClipBoxBEdgeAgainstBoxAFace(in Vector3Wide edgeDirection,
            in Vector3Wide edgeStartB0ToEdgeAnchorA00, in Vector3Wide edgeStartB0ToEdgeAnchorA11,
            in Vector3Wide edgeStartB1ToEdgeAnchorA00, in Vector3Wide edgeStartB1ToEdgeAnchorA11,
            in Vector3Wide boxEdgePlaneNormal,
            out Vector<float> min0, out Vector<float> max0,
            out Vector<float> min1, out Vector<float> max1)
        {
            Vector3Wide.Dot(edgeStartB0ToEdgeAnchorA00, boxEdgePlaneNormal, out var distance00);
            Vector3Wide.Dot(edgeStartB0ToEdgeAnchorA11, boxEdgePlaneNormal, out var distance01);
            Vector3Wide.Dot(edgeStartB1ToEdgeAnchorA00, boxEdgePlaneNormal, out var distance10);
            Vector3Wide.Dot(edgeStartB1ToEdgeAnchorA11, boxEdgePlaneNormal, out var distance11);
            Vector3Wide.Dot(boxEdgePlaneNormal, edgeDirection, out var velocity);
            var inverseVelocity = Vector<float>.One / velocity;

            //If the distances to the planes have opposing signs, then the start must be between the two.
            var edgeStartIsInside0 = Vector.LessThanOrEqual(distance00 * distance01, Vector<float>.Zero);
            var edgeStartIsInside1 = Vector.LessThanOrEqual(distance10 * distance11, Vector<float>.Zero);
            var dontUseFallback = Vector.GreaterThan(Vector.Abs(velocity), new Vector<float>(1e-15f));
            var t00 = distance00 * inverseVelocity;
            var t01 = distance01 * inverseVelocity;
            var t10 = distance10 * inverseVelocity;
            var t11 = distance11 * inverseVelocity;
            //If the edge direction and plane surface is parallel, then the interval is defined entirely by whether the edge starts inside or outside.
            //If it's inside, then it's -inf to inf. If it's outside, then there is no overlap and we'll use inf to -inf.
            var largeNegative = new Vector<float>(-float.MaxValue);
            var largePositive = new Vector<float>(float.MaxValue);
            min0 = Vector.ConditionalSelect(dontUseFallback, Vector.Min(t00, t01), Vector.ConditionalSelect(edgeStartIsInside0, largeNegative, largePositive));
            max0 = Vector.ConditionalSelect(dontUseFallback, Vector.Max(t00, t01), Vector.ConditionalSelect(edgeStartIsInside0, largePositive, largeNegative));
            min1 = Vector.ConditionalSelect(dontUseFallback, Vector.Min(t10, t11), Vector.ConditionalSelect(edgeStartIsInside1, largeNegative, largePositive));
            max1 = Vector.ConditionalSelect(dontUseFallback, Vector.Max(t10, t11), Vector.ConditionalSelect(edgeStartIsInside1, largePositive, largeNegative));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ClipBoxBEdgesAgainstBoxAFace(in Vector3Wide edgeStartB0, in Vector3Wide edgeStartB1, in Vector3Wide edgeDirectionB, in Vector<float> halfSpanB,
            in Vector3Wide vertexA00, in Vector3Wide vertexA11, in Vector3Wide edgePlaneNormalAX, in Vector3Wide edgePlaneNormalAY,
            out Vector<float> min0, out Vector<float> max0, out Vector<float> min1, out Vector<float> max1)
        {
            Vector3Wide.Subtract(vertexA00, edgeStartB0, out var edgeStartB0ToVA00);
            Vector3Wide.Subtract(vertexA11, edgeStartB0, out var edgeStartB0ToVA11);
            Vector3Wide.Subtract(vertexA00, edgeStartB1, out var edgeStartB1ToVA00);
            Vector3Wide.Subtract(vertexA11, edgeStartB1, out var edgeStartB1ToVA11);
            ClipBoxBEdgeAgainstBoxAFace(edgeDirectionB, edgeStartB0ToVA00, edgeStartB0ToVA11, edgeStartB1ToVA00, edgeStartB1ToVA11, edgePlaneNormalAX,
                out var minX0, out var maxX0, out var minX1, out var maxX1);
            ClipBoxBEdgeAgainstBoxAFace(edgeDirectionB, edgeStartB0ToVA00, edgeStartB0ToVA11, edgeStartB1ToVA00, edgeStartB1ToVA11, edgePlaneNormalAY,
                out var minY0, out var maxY0, out var minY1, out var maxY1);
            var negativeHalfSpanB = -halfSpanB;
            //Note that we are computing the intersection of the two intervals.
            //If they overlap, then the minimum is the greater of the two minimums, and the maximum is the lesser of the two maximums.
            //After applying those filters, if the min is greater than the max, then the interval has no actual overlap.
            //Note that we explicitly do not clamp both sides of the interval. We want to preserve any interval where the max is below -halfSpanB, or the min is above halfSpanB.
            //Such cases correspond to no contacts.
            //(Note that we do clamp the maximum to the halfSpan. If an interval maximum reaches the end of the interval, it is used to create a contact representing the associated B vertex.
            //The minimums are also clamped, because two of the edges flip their intervals during contact generation to ensure winding. The minimum becomes the maximum in that case.)
            min0 = Vector.Max(negativeHalfSpanB, Vector.Max(minX0, minY0));
            max0 = Vector.Min(halfSpanB, Vector.Min(maxX0, maxY0));
            min1 = Vector.Max(negativeHalfSpanB, Vector.Max(minX1, minY1));
            max1 = Vector.Min(halfSpanB, Vector.Min(maxX1, maxY1));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void AddContactsForEdge(in Vector<float> min, in ManifoldCandidate minCandidate, in Vector<float> max, in ManifoldCandidate maxCandidate, in Vector<float> halfSpanB,
            in Vector<float> epsilon, ref ManifoldCandidate candidates, ref Vector<int> candidateCount, in Vector<int> allowContacts, int pairCount)
        {
            //If -halfSpan<min<halfSpan && (max-min)>epsilon for an edge, use the min intersection as a contact.
            //If -halfSpan<=max<=halfSpan && max>=min, use the max intersection as a contact.
            //Note the comparisons: if the max lies on a face vertex, it is used, but if the min lies on a face vertex, it is not. This avoids redundant entries.
            var minExists = Vector.BitwiseAnd(
                allowContacts,
                Vector.BitwiseAnd(
                    Vector.GreaterThan(max - min, epsilon),
                    Vector.LessThan(Vector.Abs(min), halfSpanB)));
            ManifoldCandidateHelper.AddCandidate(ref candidates, ref candidateCount, minCandidate, minExists, pairCount);

            var maxExists = Vector.BitwiseAnd(
                allowContacts,
                Vector.BitwiseAnd(
                    Vector.GreaterThanOrEqual(max, min),
                    Vector.LessThanOrEqual(Vector.Abs(max), halfSpanB)));
            ManifoldCandidateHelper.AddCandidate(ref candidates, ref candidateCount, maxCandidate, maxExists, pairCount);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void CreateEdgeContacts(
            in Vector3Wide faceCenterB, in Vector3Wide faceTangentBX, in Vector3Wide faceTangentBY, in Vector<float> halfSpanBX, in Vector<float> halfSpanBY,
            in Vector3Wide vertexA00, in Vector3Wide vertexA11, in Vector3Wide faceTangentAX, in Vector3Wide faceTangentAY, in Vector3Wide contactNormal,
            in Vector<int> featureIdX0, in Vector<int> featureIdX1, in Vector<int> featureIdY0, in Vector<int> featureIdY1,
            in Vector<float> epsilonScale, ref ManifoldCandidate candidates, ref Vector<int> candidateCount, int pairCount, in Vector<int> allowContacts)
        {
            //The critical observation here is that we are working in a contact plane defined by the contact normal- not the triangle face normal or the box face normal.
            //So, when performing clipping, we actually want to clip on the contact normal plane.
            //This is consistent with the box vertex test where we cast a ray from the box vertex to triangle along the contact normal.

            //This is almost identical to testing against the unmodified box face, but we need to change the face edge plane normals.
            //Rather than being the simple tangent axes, we must compute the plane normals such that the plane embeds the box edge while being perpendicular to the contact normal.
            Vector3Wide.CrossWithoutOverlap(faceTangentAY, contactNormal, out var edgePlaneNormalAX);
            Vector3Wide.CrossWithoutOverlap(faceTangentAX, contactNormal, out var edgePlaneNormalAY);

            Vector3Wide.Scale(faceTangentBY, halfSpanBY, out var edgeOffsetBX);
            Vector3Wide.Scale(faceTangentBX, halfSpanBX, out var edgeOffsetBY);
            Vector3Wide.Subtract(faceCenterB, edgeOffsetBX, out var edgeStartBX0);
            Vector3Wide.Add(faceCenterB, edgeOffsetBX, out var edgeStartBX1);
            ClipBoxBEdgesAgainstBoxAFace(edgeStartBX0, edgeStartBX1, faceTangentBX, halfSpanBX, vertexA00, vertexA11, edgePlaneNormalAX, edgePlaneNormalAY,
                out var minX0, out var maxX0, out var unflippedMinX1, out var unflippedMaxX1);
            Vector3Wide.Subtract(faceCenterB, edgeOffsetBY, out var edgeStartBY0);
            Vector3Wide.Add(faceCenterB, edgeOffsetBY, out var edgeStartBY1);
            ClipBoxBEdgesAgainstBoxAFace(edgeStartBY0, edgeStartBY1, faceTangentBY, halfSpanBY, vertexA00, vertexA11, edgePlaneNormalAX, edgePlaneNormalAY,
                out var unflippedMinY0, out var unflippedMaxY0, out var minY1, out var maxY1);

            //The intervals were computed with the edge direction pointing in the same direction for both sides of the face. We want to have a consistent winding all the way around so that
            //the end of one edge butts up against the start of the next. Given how we choose to create contacts based on the interval min/max, this ensures a good contact distribution.
            //Flip the X1 and Y0 intervals.
            var minX1 = -unflippedMaxX1;
            var maxX1 = -unflippedMinX1;
            var minY0 = -unflippedMaxY0;
            var maxY0 = -unflippedMinY0;

            //We now have intervals for all four box B edges.
            var edgeFeatureIdOffset = new Vector<int>(64);
            var epsilon = epsilonScale * 1e-5f;
            Unsafe.SkipInit(out ManifoldCandidate min);
            Unsafe.SkipInit(out ManifoldCandidate max);
            //X0
            min.FeatureId = featureIdX0;
            min.X = minX0;
            min.Y = -halfSpanBY;
            max.FeatureId = featureIdX0 + edgeFeatureIdOffset;
            max.X = maxX0;
            max.Y = min.Y;
            AddContactsForEdge(minX0, min, maxX0, max, halfSpanBX, epsilon, ref candidates, ref candidateCount, allowContacts, pairCount);

            //Y1
            min.FeatureId = featureIdY1;
            min.X = halfSpanBX;
            min.Y = minY1;
            max.FeatureId = featureIdY1 + edgeFeatureIdOffset;
            max.X = halfSpanBX;
            max.Y = maxY1;
            AddContactsForEdge(minY1, min, maxY1, max, halfSpanBY, epsilon, ref candidates, ref candidateCount, allowContacts, pairCount);

            //X1
            min.FeatureId = featureIdX1;
            min.X = unflippedMaxX1;
            min.Y = halfSpanBY;
            max.FeatureId = featureIdX1 + edgeFeatureIdOffset;
            max.X = unflippedMinX1;
            max.Y = halfSpanBY;
            AddContactsForEdge(minX1, min, maxX1, max, halfSpanBX, epsilon, ref candidates, ref candidateCount, allowContacts, pairCount);

            //Y0
            min.FeatureId = featureIdY0;
            min.X = -halfSpanBX;
            min.Y = unflippedMaxY0;
            max.FeatureId = featureIdY0 + edgeFeatureIdOffset;
            max.X = min.X;
            max.Y = unflippedMinY0;
            AddContactsForEdge(minY0, min, maxY0, max, halfSpanBY, epsilon, ref candidates, ref candidateCount, allowContacts, pairCount);
        }




        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Test(
            ref BoxWide a, ref BoxWide b, ref Vector<float> speculativeMargin,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount,
            out Convex4ContactManifoldWide manifold)
        {
            Unsafe.SkipInit(out manifold);
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var worldRA);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var worldRB);
            Matrix3x3Wide.MultiplyByTransposeWithoutOverlap(worldRB, worldRA, out var rB);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, worldRA, out var localOffsetB);

            Vector3Wide localNormal;
            //b.X
            TestEdgeEdge(
                ref a.HalfWidth, ref a.HalfHeight, ref a.HalfLength,
                ref b.HalfWidth, ref b.HalfHeight, ref b.HalfLength,
                ref localOffsetB.X, ref localOffsetB.Y, ref localOffsetB.Z,
                ref rB.X, ref rB.Y, ref rB.Z, ref rB.X,
                out var depth, out localNormal.X, out localNormal.Y, out localNormal.Z);
            //b.Y
            TestEdgeEdge(
                ref a.HalfWidth, ref a.HalfHeight, ref a.HalfLength,
                ref b.HalfWidth, ref b.HalfHeight, ref b.HalfLength,
                ref localOffsetB.X, ref localOffsetB.Y, ref localOffsetB.Z,
                ref rB.X, ref rB.Y, ref rB.Z, ref rB.Y,
                out var edgeYDepth, out var edgeYNX, out var edgeYNY, out var edgeYNZ);
            Select(ref depth, ref localNormal,
                ref edgeYDepth, ref edgeYNX, ref edgeYNY, ref edgeYNZ);
            //b.Z
            TestEdgeEdge(
                ref a.HalfWidth, ref a.HalfHeight, ref a.HalfLength,
                ref b.HalfWidth, ref b.HalfHeight, ref b.HalfLength,
                ref localOffsetB.X, ref localOffsetB.Y, ref localOffsetB.Z,
                ref rB.X, ref rB.Y, ref rB.Z, ref rB.Z,
                out var edgeZDepth, out var edgeZNX, out var edgeZNY, out var edgeZNZ);
            Select(ref depth, ref localNormal,
                ref edgeZDepth, ref edgeZNX, ref edgeZNY, ref edgeZNZ);

            //Test face normals of A. Working in local space of A means potential axes are just (1,0,0) etc.
            Vector3Wide.Abs(rB.X, out var absRBX);
            Vector3Wide.Abs(rB.Y, out var absRBY);
            Vector3Wide.Abs(rB.Z, out var absRBZ);
            var faceAXDepth = a.HalfWidth + b.HalfWidth * absRBX.X + b.HalfHeight * absRBY.X + b.HalfLength * absRBZ.X - Vector.Abs(localOffsetB.X);
            var one = Vector<float>.One;
            var zero = Vector<float>.Zero;
            Select(ref depth, ref localNormal, ref faceAXDepth, ref one, ref zero, ref zero);
            var faceAYDepth = a.HalfHeight + b.HalfWidth * absRBX.Y + b.HalfHeight * absRBY.Y + b.HalfLength * absRBZ.Y - Vector.Abs(localOffsetB.Y);
            Select(ref depth, ref localNormal, ref faceAYDepth, ref zero, ref one, ref zero);
            var faceAZDepth = a.HalfLength + b.HalfWidth * absRBX.Z + b.HalfHeight * absRBY.Z + b.HalfLength * absRBZ.Z - Vector.Abs(localOffsetB.Z);
            Select(ref depth, ref localNormal, ref faceAZDepth, ref zero, ref zero, ref one);

            //Test face normals of B. Rows of A->B rotation.
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(localOffsetB, rB, out var bLocalOffsetB);
            var faceBXDepth = b.HalfWidth + a.HalfWidth * absRBX.X + a.HalfHeight * absRBX.Y + a.HalfLength * absRBX.Z - Vector.Abs(bLocalOffsetB.X);
            Select(ref depth, ref localNormal, ref faceBXDepth, ref rB.X.X, ref rB.X.Y, ref rB.X.Z);
            var faceBYDepth = b.HalfHeight + a.HalfWidth * absRBY.X + a.HalfHeight * absRBY.Y + a.HalfLength * absRBY.Z - Vector.Abs(bLocalOffsetB.Y);
            Select(ref depth, ref localNormal, ref faceBYDepth, ref rB.Y.X, ref rB.Y.Y, ref rB.Y.Z);
            var faceBZDepth = b.HalfLength + a.HalfWidth * absRBZ.X + a.HalfHeight * absRBZ.Y + a.HalfLength * absRBZ.Z - Vector.Abs(bLocalOffsetB.Z);
            Select(ref depth, ref localNormal, ref faceBZDepth, ref rB.Z.X, ref rB.Z.Y, ref rB.Z.Z);

            var activeLanes = BundleIndexing.CreateMaskForCountInBundle(pairCount);
            var minimumDepth = -speculativeMargin;
            var allowContacts = Vector.BitwiseAnd(activeLanes, Vector.GreaterThanOrEqual(depth, minimumDepth));
            if (Vector.EqualsAll(allowContacts, Vector<int>.Zero))
            {
                manifold.Contact0Exists = default;
                manifold.Contact1Exists = default;
                manifold.Contact2Exists = default;
                manifold.Contact3Exists = default;
                return;
            }
            //Calibrate the normal to point from B to A, matching convention.
            Vector3Wide.Dot(localNormal, localOffsetB, out var normalDotOffsetB);
            var shouldNegateNormal = Vector.GreaterThan(normalDotOffsetB, Vector<float>.Zero);
            localNormal.X = Vector.ConditionalSelect(shouldNegateNormal, -localNormal.X, localNormal.X);
            localNormal.Y = Vector.ConditionalSelect(shouldNegateNormal, -localNormal.Y, localNormal.Y);
            localNormal.Z = Vector.ConditionalSelect(shouldNegateNormal, -localNormal.Z, localNormal.Z);
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, worldRA, out manifold.Normal);

            //Contact generation always assumes face-face clipping. Other forms of contact generation are just special cases of face-face, and since we pay
            //for all code paths, there's no point in handling them separately.
            //We just have to guarantee that the face chosen on each box is guaranteed to include the deepest feature along the contact normal.
            //To do this, choose the face on each box associated with the maximum axis dot with the collision normal.

            //We represent each face as a center position, its two tangent axes, and the length along those axes.
            //Technically, we could leave A's tangents implicit by swizzling components, but that complicates things a little bit for not much gain.
            //Since we're not taking advantage of the dimension reduction of working in A's local space from here on out, just use the world axes to avoid a final retransform.
            Vector3Wide.Dot(manifold.Normal, worldRA.X, out var axDot);
            Vector3Wide.Dot(manifold.Normal, worldRA.Y, out var ayDot);
            Vector3Wide.Dot(manifold.Normal, worldRA.Z, out var azDot);
            var absAXDot = Vector.Abs(axDot);
            var absAYDot = Vector.Abs(ayDot);
            var absAZDot = Vector.Abs(azDot);
            var maxADot = Vector.Max(absAXDot, Vector.Max(absAYDot, absAZDot));
            var useAX = Vector.Equals(maxADot, absAXDot);
            var useAY = Vector.AndNot(Vector.Equals(maxADot, absAYDot), useAX);
            Vector3Wide.ConditionalSelect(useAX, worldRA.X, worldRA.Z, out var normalA);
            Vector3Wide.ConditionalSelect(useAY, worldRA.Y, normalA, out normalA);
            Vector3Wide.ConditionalSelect(useAX, worldRA.Z, worldRA.Y, out var tangentAX);
            Vector3Wide.ConditionalSelect(useAY, worldRA.X, tangentAX, out tangentAX);
            Vector3Wide.ConditionalSelect(useAX, worldRA.Y, worldRA.X, out var tangentAY);
            Vector3Wide.ConditionalSelect(useAY, worldRA.Z, tangentAY, out tangentAY);
            var halfSpanAX = Vector.ConditionalSelect(useAX, a.HalfLength, Vector.ConditionalSelect(useAY, a.HalfWidth, a.HalfHeight));
            var halfSpanAY = Vector.ConditionalSelect(useAX, a.HalfHeight, Vector.ConditionalSelect(useAY, a.HalfLength, a.HalfWidth));
            var halfSpanAZ = Vector.ConditionalSelect(useAX, a.HalfWidth, Vector.ConditionalSelect(useAY, a.HalfHeight, a.HalfLength));
            //We'll construct vertex feature ids from axis ids. 
            //Vertex ids will be constructed by setting or not setting the relevant bit for each axis.
            var localXId = new Vector<int>(1);
            var localYId = new Vector<int>(4);
            var localZId = new Vector<int>(16);
            var axisIdAX = Vector.ConditionalSelect(useAX, localZId, Vector.ConditionalSelect(useAY, localXId, localYId));
            var axisIdAY = Vector.ConditionalSelect(useAX, localYId, Vector.ConditionalSelect(useAY, localZId, localXId));
            var axisIdAZ = Vector.ConditionalSelect(useAX, localXId, Vector.ConditionalSelect(useAY, localYId, localZId));

            Vector3Wide.Dot(manifold.Normal, worldRB.X, out var bxDot);
            Vector3Wide.Dot(manifold.Normal, worldRB.Y, out var byDot);
            Vector3Wide.Dot(manifold.Normal, worldRB.Z, out var bzDot);
            var absBXDot = Vector.Abs(bxDot);
            var absBYDot = Vector.Abs(byDot);
            var absBZDot = Vector.Abs(bzDot);
            var maxBDot = Vector.Max(absBXDot, Vector.Max(absBYDot, absBZDot));
            var useBX = Vector.Equals(maxBDot, absBXDot);
            var useBY = Vector.AndNot(Vector.Equals(maxBDot, absBYDot), useBX);
            Vector3Wide.ConditionalSelect(useBX, worldRB.X, worldRB.Z, out var normalB);
            Vector3Wide.ConditionalSelect(useBY, worldRB.Y, normalB, out normalB);
            Vector3Wide.ConditionalSelect(useBX, worldRB.Z, worldRB.Y, out var tangentBX);
            Vector3Wide.ConditionalSelect(useBY, worldRB.X, tangentBX, out tangentBX);
            Vector3Wide.ConditionalSelect(useBX, worldRB.Y, worldRB.X, out var tangentBY);
            Vector3Wide.ConditionalSelect(useBY, worldRB.Z, tangentBY, out tangentBY);
            var halfSpanBX = Vector.ConditionalSelect(useBX, b.HalfLength, Vector.ConditionalSelect(useBY, b.HalfWidth, b.HalfHeight));
            var halfSpanBY = Vector.ConditionalSelect(useBX, b.HalfHeight, Vector.ConditionalSelect(useBY, b.HalfLength, b.HalfWidth));
            var halfSpanBZ = Vector.ConditionalSelect(useBX, b.HalfWidth, Vector.ConditionalSelect(useBY, b.HalfHeight, b.HalfLength));
            //We'll construct edge feature ids from axis ids. 
            //Edge ids will be 6 bits total, representing 3 possible states (-1, 0, 1) for each of the 3 axes. Multiply the axis id by 1, 2, or 3 to get the edge id contribution for the axis.
            var axisIdBX = Vector.ConditionalSelect(useBX, localZId, Vector.ConditionalSelect(useBY, localXId, localYId));
            var axisIdBY = Vector.ConditionalSelect(useBX, localYId, Vector.ConditionalSelect(useBY, localZId, localXId));
            var axisIdBZ = Vector.ConditionalSelect(useBX, localXId, Vector.ConditionalSelect(useBY, localYId, localZId));

            //Calibrate normalB to face toward A, and normalA to face toward B.
            Vector3Wide.Dot(normalA, manifold.Normal, out var calibrationDotA);
            var shouldNegateNormalA = Vector.GreaterThan(calibrationDotA, Vector<float>.Zero);
            normalA.X = Vector.ConditionalSelect(shouldNegateNormalA, -normalA.X, normalA.X);
            normalA.Y = Vector.ConditionalSelect(shouldNegateNormalA, -normalA.Y, normalA.Y);
            normalA.Z = Vector.ConditionalSelect(shouldNegateNormalA, -normalA.Z, normalA.Z);
            Vector3Wide.Dot(normalB, manifold.Normal, out var calibrationDotB);
            var shouldNegateNormalB = Vector.LessThan(calibrationDotB, Vector<float>.Zero);
            normalB.X = Vector.ConditionalSelect(shouldNegateNormalB, -normalB.X, normalB.X);
            normalB.Y = Vector.ConditionalSelect(shouldNegateNormalB, -normalB.Y, normalB.Y);
            normalB.Z = Vector.ConditionalSelect(shouldNegateNormalB, -normalB.Z, normalB.Z);

            //Note that we only allocate up to 8 candidates. It is not possible for this process to generate more than 8 (unless there are numerical problems, which we guard against).
            int byteCount = Unsafe.SizeOf<ManifoldCandidate>() * 8;
            var buffer = stackalloc byte[byteCount];
            ref var candidates = ref Unsafe.As<byte, ManifoldCandidate>(ref *buffer);

            //Face B edges against face A bound planes
            Vector3Wide.Scale(normalA, halfSpanAZ, out var faceCenterA);
            Vector3Wide.Scale(normalB, halfSpanBZ, out var faceCenterB);
            Vector3Wide.Add(faceCenterB, offsetB, out faceCenterB);
            Vector3Wide.Subtract(faceCenterA, faceCenterB, out var faceCenterBToFaceCenterA);
            Vector3Wide.Scale(tangentAY, halfSpanAY, out var edgeOffsetAX);
            Vector3Wide.Scale(tangentAX, halfSpanAX, out var edgeOffsetAY);

            Vector3Wide.Subtract(faceCenterA, edgeOffsetAX, out var vertexA0);
            Vector3Wide.Subtract(vertexA0, edgeOffsetAY, out var vertexA00);
            Vector3Wide.Add(faceCenterA, edgeOffsetAX, out var vertexA1);
            Vector3Wide.Add(vertexA1, edgeOffsetAY, out var vertexA11);

            var epsilonScale = Vector.Min(Vector.Max(halfSpanAX, Vector.Max(halfSpanAY, halfSpanAZ)), Vector.Max(halfSpanBX, Vector.Max(halfSpanBY, halfSpanBZ)));
            var twiceAxisIdBX = axisIdBX * new Vector<int>(2);
            var three = new Vector<int>(3);
            var axisZEdgeIdContribution = axisIdBZ * three;
            var edgeIdBX0 = twiceAxisIdBX + axisIdBY + axisZEdgeIdContribution;
            var edgeIdBX1 = twiceAxisIdBX + axisIdBY * three + axisZEdgeIdContribution;
            var twiceAxisIdBY = axisIdBY * new Vector<int>(2);
            var edgeIdBY0 = axisIdBX + twiceAxisIdBY + axisZEdgeIdContribution;
            var edgeIdBY1 = axisIdBX * three + twiceAxisIdBY + axisZEdgeIdContribution;
            var candidateCount = Vector<int>.Zero;
            CreateEdgeContacts(faceCenterB, tangentBX, tangentBY, halfSpanBX, halfSpanBY, vertexA00, vertexA11, tangentAX, tangentAY, manifold.Normal,
                edgeIdBX0, edgeIdBX1, edgeIdBY0, edgeIdBY1, epsilonScale, ref candidates, ref candidateCount, pairCount, allowContacts);

            //Face A vertices
            //Vertex ids only have two states per axis, so scale id by 0 or 1 before adding. Equivalent to conditional or.          
            //Note that the feature id is negated. This disambiguates between edge-edge contacts and vertex contacts.
            var vertexId00 = -axisIdAZ;
            Vector3Wide.Add(vertexA0, edgeOffsetAY, out var vertexA01);
            var vertexId01 = -(axisIdAZ + axisIdAY);
            Vector3Wide.Subtract(vertexA1, edgeOffsetAY, out var vertexA10);
            var vertexId10 = -(axisIdAZ + axisIdAX);
            var vertexId11 = -(axisIdAZ + axisIdAX + axisIdAY);
            AddBoxAVertices(faceCenterB, tangentBX, tangentBY, halfSpanBX, halfSpanBY, normalB, manifold.Normal,
                vertexA00, vertexA01, vertexA10, vertexA11, vertexId00, vertexId01, vertexId10, vertexId11, ref candidates, ref candidateCount, pairCount, allowContacts);

            ManifoldCandidateHelper.Reduce(ref candidates, candidateCount, 8, normalA, new Vector<float>(-1f) / Vector.Abs(calibrationDotA), faceCenterBToFaceCenterA, tangentBX, tangentBY, epsilonScale, minimumDepth, pairCount,
                out var contact0, out var contact1, out var contact2, out var contact3,
                out manifold.Contact0Exists, out manifold.Contact1Exists, out manifold.Contact2Exists, out manifold.Contact3Exists);

            //Transform the contacts into the manifold.
            TransformContactToManifold(ref contact0, ref faceCenterB, ref tangentBX, ref tangentBY, ref manifold.OffsetA0, ref manifold.Depth0, ref manifold.FeatureId0);
            TransformContactToManifold(ref contact1, ref faceCenterB, ref tangentBX, ref tangentBY, ref manifold.OffsetA1, ref manifold.Depth1, ref manifold.FeatureId1);
            TransformContactToManifold(ref contact2, ref faceCenterB, ref tangentBX, ref tangentBY, ref manifold.OffsetA2, ref manifold.Depth2, ref manifold.FeatureId2);
            TransformContactToManifold(ref contact3, ref faceCenterB, ref tangentBX, ref tangentBY, ref manifold.OffsetA3, ref manifold.Depth3, ref manifold.FeatureId3);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void TransformContactToManifold(
            ref ManifoldCandidate rawContact, ref Vector3Wide faceCenterB, ref Vector3Wide tangentBX, ref Vector3Wide tangentBY,
            ref Vector3Wide manifoldOffsetA, ref Vector<float> manifoldDepth, ref Vector<int> manifoldFeatureId)
        {
            Vector3Wide.Scale(tangentBX, rawContact.X, out manifoldOffsetA);
            Vector3Wide.Scale(tangentBY, rawContact.Y, out var y);
            Vector3Wide.Add(manifoldOffsetA, y, out manifoldOffsetA);
            Vector3Wide.Add(manifoldOffsetA, faceCenterB, out manifoldOffsetA);
            manifoldDepth = rawContact.Depth;
            manifoldFeatureId = rawContact.FeatureId;
        }

        public void Test(ref BoxWide a, ref BoxWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref BoxWide a, ref BoxWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }
}
