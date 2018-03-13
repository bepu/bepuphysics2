using BepuPhysics.Collidables;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using static BepuPhysics.GatherScatter;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct BoxPairTester : IPairTester<BoxWide, BoxWide, Convex4ContactManifoldWide>
    {
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
        public unsafe void Test(
            ref BoxWide a, ref BoxWide b,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Convex4ContactManifoldWide manifold)
        {
            Matrix3x3Wide.CreateFromQuaternion(ref orientationA, out var worldRA);
            Matrix3x3Wide.CreateFromQuaternion(ref orientationB, out var worldRB);
            Matrix3x3Wide.MultiplyByTransposeWithoutOverlap(ref worldRB, ref worldRA, out var rB);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ref offsetB, ref worldRA, out var localOffsetB);

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
            Vector3Wide.Abs(ref rB.X, out var absRBX);
            Vector3Wide.Abs(ref rB.Y, out var absRBY);
            Vector3Wide.Abs(ref rB.Z, out var absRBZ);
            var faceAXDepth = a.HalfWidth + b.HalfWidth * absRBX.X + b.HalfHeight * absRBY.X + b.HalfLength * absRBZ.X - Vector.Abs(localOffsetB.X);
            var one = Vector<float>.One;
            var zero = Vector<float>.Zero;
            Select(ref depth, ref localNormal, ref faceAXDepth, ref one, ref zero, ref zero);
            var faceAYDepth = a.HalfHeight + b.HalfWidth * absRBX.Y + b.HalfHeight * absRBY.Y + b.HalfLength * absRBZ.Y - Vector.Abs(localOffsetB.Y);
            Select(ref depth, ref localNormal, ref faceAYDepth, ref zero, ref one, ref zero);
            var faceAZDepth = a.HalfLength + b.HalfWidth * absRBX.Z + b.HalfHeight * absRBY.Z + b.HalfLength * absRBZ.Z - Vector.Abs(localOffsetB.Z);
            Select(ref depth, ref localNormal, ref faceAZDepth, ref zero, ref zero, ref one);

            //Test face normals of B. Rows of A->B rotation.
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ref localOffsetB, ref rB, out var bLocalOffsetB);
            var faceBXDepth = b.HalfWidth + a.HalfWidth * absRBX.X + a.HalfHeight * absRBX.Y + a.HalfLength * absRBX.Z - Vector.Abs(bLocalOffsetB.X);
            Select(ref depth, ref localNormal, ref faceBXDepth, ref rB.X.X, ref rB.X.Y, ref rB.X.Z);
            var faceBYDepth = b.HalfHeight + a.HalfWidth * absRBY.X + a.HalfHeight * absRBY.Y + a.HalfLength * absRBY.Z - Vector.Abs(bLocalOffsetB.Y);
            Select(ref depth, ref localNormal, ref faceBYDepth, ref rB.Y.X, ref rB.Y.Y, ref rB.Y.Z);
            var faceBZDepth = b.HalfLength + a.HalfWidth * absRBZ.X + a.HalfHeight * absRBZ.Y + a.HalfLength * absRBZ.Z - Vector.Abs(bLocalOffsetB.Z);
            Select(ref depth, ref localNormal, ref faceBZDepth, ref rB.Z.X, ref rB.Z.Y, ref rB.Z.Z);

            //Calibrate the normal to point from B to A, matching convention.
            Vector3Wide.Dot(ref localNormal, ref localOffsetB, out var normalDotOffsetB);
            var shouldNegateNormal = Vector.GreaterThan(normalDotOffsetB, Vector<float>.Zero);
            localNormal.X = Vector.ConditionalSelect(shouldNegateNormal, -localNormal.X, localNormal.X);
            localNormal.Y = Vector.ConditionalSelect(shouldNegateNormal, -localNormal.Y, localNormal.Y);
            localNormal.Z = Vector.ConditionalSelect(shouldNegateNormal, -localNormal.Z, localNormal.Z);
            Matrix3x3Wide.TransformWithoutOverlap(ref localNormal, ref worldRA, out manifold.Normal);

            //Contact generation always assumes face-face clipping. Other forms of contact generation are just special cases of face-face, and since we pay
            //for all code paths, there's no point in handling them separately.
            //We just have to guarantee that the face chosen on each box is guaranteed to include the deepest feature along the contact normal.
            //To do this, choose the face on each box associated with the maximum axis dot with the collision normal.

            //We represent each face as a center position, its two tangent axes, and the length along those axes.
            //Technically, we could leave A's tangents implicit by swizzling components, but that complicates things a little bit for not much gain.
            //Since we're not taking advantage of the dimension reduction of working in A's local space from here on out, just use the world axes to avoid a final retransform.
            Vector3Wide.Dot(ref manifold.Normal, ref worldRA.X, out var axDot);
            Vector3Wide.Dot(ref manifold.Normal, ref worldRA.Y, out var ayDot);
            Vector3Wide.Dot(ref manifold.Normal, ref worldRA.Z, out var azDot);
            var absAXDot = Vector.Abs(axDot);
            var absAYDot = Vector.Abs(ayDot);
            var absAZDot = Vector.Abs(azDot);
            var maxADot = Vector.Max(absAXDot, Vector.Max(absAYDot, absAZDot));
            var useAX = Vector.Equals(maxADot, absAXDot);
            var useAY = Vector.AndNot(Vector.Equals(maxADot, absAYDot), useAX);
            Vector3Wide tangentAX, tangentAY, normalA;
            Vector<float> halfSpanAX, halfSpanAY, halfSpanAZ;
            Vector<int> axisIdAX, axisIdAY, axisIdAZ;
            Vector3Wide.ConditionalSelect(ref useAX, ref worldRA.X, ref worldRA.Z, out normalA);
            Vector3Wide.ConditionalSelect(ref useAY, ref worldRA.Y, ref normalA, out normalA);
            Vector3Wide.ConditionalSelect(ref useAX, ref worldRA.Z, ref worldRA.Y, out tangentAX);
            Vector3Wide.ConditionalSelect(ref useAY, ref worldRA.X, ref tangentAX, out tangentAX);
            Vector3Wide.ConditionalSelect(ref useAX, ref worldRA.Y, ref worldRA.X, out tangentAY);
            Vector3Wide.ConditionalSelect(ref useAY, ref worldRA.Z, ref tangentAY, out tangentAY);
            halfSpanAX = Vector.ConditionalSelect(useAX, a.HalfLength, Vector.ConditionalSelect(useAY, a.HalfWidth, a.HalfHeight));
            halfSpanAY = Vector.ConditionalSelect(useAX, a.HalfHeight, Vector.ConditionalSelect(useAY, a.HalfLength, a.HalfWidth));
            halfSpanAZ = Vector.ConditionalSelect(useAX, a.HalfWidth, Vector.ConditionalSelect(useAY, a.HalfHeight, a.HalfLength));
            //We'll construct vertex feature ids from axis ids. 
            //Vertex ids will be constructed by setting or not setting the relevant bit for each axis.
            var localXId = new Vector<int>(1);
            var localYId = new Vector<int>(4);
            var localZId = new Vector<int>(16);
            axisIdAX = Vector.ConditionalSelect(useAX, localZId, Vector.ConditionalSelect(useAY, localXId, localYId));
            axisIdAY = Vector.ConditionalSelect(useAX, localYId, Vector.ConditionalSelect(useAY, localZId, localXId));
            axisIdAZ = Vector.ConditionalSelect(useAX, localXId, Vector.ConditionalSelect(useAY, localYId, localZId));

            Vector3Wide.Dot(ref manifold.Normal, ref worldRB.X, out var bxDot);
            Vector3Wide.Dot(ref manifold.Normal, ref worldRB.Y, out var byDot);
            Vector3Wide.Dot(ref manifold.Normal, ref worldRB.Z, out var bzDot);
            var absBXDot = Vector.Abs(bxDot);
            var absBYDot = Vector.Abs(byDot);
            var absBZDot = Vector.Abs(bzDot);
            var maxBDot = Vector.Max(absBXDot, Vector.Max(absBYDot, absBZDot));
            var useBX = Vector.Equals(maxBDot, absBXDot);
            var useBY = Vector.AndNot(Vector.Equals(maxBDot, absBYDot), useBX);
            Vector3Wide tangentBX, tangentBY, normalB;
            Vector<float> halfSpanBX, halfSpanBY, halfSpanBZ;
            Vector<int> axisIdBX, axisIdBY, axisIdBZ;
            Vector3Wide.ConditionalSelect(ref useBX, ref worldRB.X, ref worldRB.Z, out normalB);
            Vector3Wide.ConditionalSelect(ref useBY, ref worldRB.Y, ref normalB, out normalB);
            Vector3Wide.ConditionalSelect(ref useBX, ref worldRB.Z, ref worldRB.Y, out tangentBX);
            Vector3Wide.ConditionalSelect(ref useBY, ref worldRB.X, ref tangentBX, out tangentBX);
            Vector3Wide.ConditionalSelect(ref useBX, ref worldRB.Y, ref worldRB.X, out tangentBY);
            Vector3Wide.ConditionalSelect(ref useBY, ref worldRB.Z, ref tangentBY, out tangentBY);
            halfSpanBX = Vector.ConditionalSelect(useBX, b.HalfLength, Vector.ConditionalSelect(useBY, b.HalfWidth, b.HalfHeight));
            halfSpanBY = Vector.ConditionalSelect(useBX, b.HalfHeight, Vector.ConditionalSelect(useBY, b.HalfLength, b.HalfWidth));
            halfSpanBZ = Vector.ConditionalSelect(useBX, b.HalfWidth, Vector.ConditionalSelect(useBY, b.HalfHeight, b.HalfLength));
            //We'll construct edge feature ids from axis ids. 
            //Edge ids will be 6 bits total, representing 3 possible states (-1, 0, 1) for each of the 3 axes. Multiply the axis id by 1, 2, or 3 to get the edge id contribution for the axis.
            axisIdBX = Vector.ConditionalSelect(useBX, localZId, Vector.ConditionalSelect(useBY, localXId, localYId));
            axisIdBY = Vector.ConditionalSelect(useBX, localYId, Vector.ConditionalSelect(useBY, localZId, localXId));
            axisIdBZ = Vector.ConditionalSelect(useBX, localXId, Vector.ConditionalSelect(useBY, localYId, localZId));

            //Calibrate normalB to face toward A, and normalA to face toward B.
            Vector3Wide.Dot(ref normalA, ref manifold.Normal, out var calibrationDotA);
            var shouldNegateNormalA = Vector.GreaterThan(calibrationDotA, Vector<float>.Zero);
            normalA.X = Vector.ConditionalSelect(shouldNegateNormalA, -normalA.X, normalA.X);
            normalA.Y = Vector.ConditionalSelect(shouldNegateNormalA, -normalA.Y, normalA.Y);
            normalA.Z = Vector.ConditionalSelect(shouldNegateNormalA, -normalA.Z, normalA.Z);
            Vector3Wide.Dot(ref normalB, ref manifold.Normal, out var calibrationDotB);
            var shouldNegateNormalB = Vector.LessThan(calibrationDotB, Vector<float>.Zero);
            normalB.X = Vector.ConditionalSelect(shouldNegateNormalB, -normalB.X, normalB.X);
            normalB.Y = Vector.ConditionalSelect(shouldNegateNormalB, -normalB.Y, normalB.Y);
            normalB.Z = Vector.ConditionalSelect(shouldNegateNormalB, -normalB.Z, normalB.Z);

            //Clip edges of B against the face bounds of A to collect all edge-bound contacts.       
            Vector3Wide.Scale(ref normalB, ref halfSpanBZ, out var faceCenterB);
            Vector3Wide.Add(ref faceCenterB, ref offsetB, out faceCenterB);
            Vector3Wide.Scale(ref tangentBY, ref halfSpanBY, out var edgeOffsetBX);
            Vector3Wide.Scale(ref tangentBX, ref halfSpanBX, out var edgeOffsetBY);
            Vector3Wide.Dot(ref tangentAX, ref tangentBX, out var axbx);
            Vector3Wide.Dot(ref tangentAY, ref tangentBX, out var aybx);
            Vector3Wide.Dot(ref tangentAX, ref tangentBY, out var axby);
            Vector3Wide.Dot(ref tangentAY, ref tangentBY, out var ayby);
            GetEdgeVersusFaceBoundsIntervals(ref tangentAX, ref tangentAY, ref halfSpanAX, ref halfSpanAY, ref axbx, ref aybx, ref faceCenterB, ref halfSpanBX, ref edgeOffsetBX,
                out var bX0Min, out var bX0Max, out var bX1Min, out var bX1Max);
            GetEdgeVersusFaceBoundsIntervals(ref tangentAX, ref tangentAY, ref halfSpanAX, ref halfSpanAY, ref axby, ref ayby, ref faceCenterB, ref halfSpanBY, ref edgeOffsetBY,
                out var bY0Min, out var bY0Max, out var bY1Min, out var bY1Max);

            //Note that we only allocate up to 8 candidates. It is not possible for this process to generate more than 8 (unless there are numerical problems, which we guard against).
            int byteCount = Unsafe.SizeOf<Candidate>() * 8;
            var buffer = stackalloc byte[byteCount];
            ref var candidates = ref Unsafe.As<byte, Candidate>(ref *buffer);

            //Note that using a raw absolute epsilon would have a varying effect based on the scale of the involved boxes.
            //The minimum across the maxes is intended to avoid cases like a huge box being used as a plane, causing a massive size disparity.
            //Using its sizes as a threshold would tend to kill off perfectly valid contacts.
            var epsilonScale = Vector.Min(Vector.Max(halfSpanAX, Vector.Max(halfSpanAY, halfSpanAZ)), Vector.Max(halfSpanBX, Vector.Max(halfSpanBY, halfSpanBZ)));

            var negativeHalfSpanBY = -halfSpanBY;
            var rawContactCount = Vector<int>.Zero;
            var three = new Vector<int>(3);
            var twiceAxisIdBX = axisIdBX * new Vector<int>(2);
            var axisZEdgeIdContribution = axisIdBZ * three;
            //Edge BX, -y offset
            var edgeIdBX0 = twiceAxisIdBX + axisIdBY + axisZEdgeIdContribution;
            AddEdgeContacts(ref candidates, ref rawContactCount, ref halfSpanBX, ref epsilonScale, ref bX0Min, ref bX0Max,
                ref bX0Min, ref negativeHalfSpanBY, ref bX0Max, ref negativeHalfSpanBY, ref edgeIdBX0);
            //Edge BX, +y offset
            //Note that the interval is flipped. This makes the edge intervals wound clockwise, 
            //so creating contacts when the interval max is at the halfSpan will put contacts at each of the 4 corners rather than 2 in the same spot.
            var flippedBX1Min = -bX1Max;
            var flippedBX1Max = -bX1Min;
            var edgeIdBX1 = twiceAxisIdBX + axisIdBY * three + axisZEdgeIdContribution;
            AddEdgeContacts(ref candidates, ref rawContactCount, ref halfSpanBX, ref epsilonScale, ref flippedBX1Min, ref flippedBX1Max,
                ref bX1Max, ref halfSpanBY, ref bX1Min, ref halfSpanBY, ref edgeIdBX1);

            var negativeHalfSpanBX = -halfSpanBX;
            var twiceAxisIdBY = axisIdBY * new Vector<int>(2);
            //Edge BY, -x offset
            //Note that the interval is flipped for winding.
            var flippedBY0Min = -bY0Max;
            var flippedBY0Max = -bY0Min;
            var edgeIdBY0 = axisIdBX + twiceAxisIdBY + axisZEdgeIdContribution;
            AddEdgeContacts(ref candidates, ref rawContactCount, ref halfSpanBY, ref epsilonScale, ref flippedBY0Min, ref flippedBY0Max,
                ref negativeHalfSpanBX, ref bY0Max, ref negativeHalfSpanBX, ref bY0Min, ref edgeIdBY0);
            //Edge BY, +x offset
            var edgeIdBY1 = axisIdBX * three + twiceAxisIdBY + axisZEdgeIdContribution;
            AddEdgeContacts(ref candidates, ref rawContactCount, ref halfSpanBY, ref epsilonScale, ref bY1Min, ref bY1Max,
                ref halfSpanBX, ref bY1Min, ref halfSpanBX, ref bY1Max, ref edgeIdBY1);

            Vector3Wide.Scale(ref normalA, ref halfSpanAZ, out var faceCenterA);
            Vector3Wide.Subtract(ref faceCenterA, ref faceCenterB, out var faceCenterBToFaceCenterA);
            Vector3Wide.Scale(ref tangentAY, ref halfSpanAY, out var edgeOffsetAX);
            Vector3Wide.Scale(ref tangentAX, ref halfSpanAX, out var edgeOffsetAY);
            //Vertex A -x, -y
            Vector3Wide.Subtract(ref faceCenterBToFaceCenterA, ref edgeOffsetAX, out var vertexA);
            Vector3Wide.Subtract(ref vertexA, ref edgeOffsetAY, out vertexA);
            //Vertex ids only have two states per axis, so scale id by 0 or 1 before adding. Equivalent to conditional or.          
            //Note that the feature id is negated. This disambiguates between edge-edge contacts and vertex contacts.
            var vertexId = -axisIdAZ;

            AddFaceVertexContact(ref vertexA, ref vertexId,
                ref tangentBX, ref tangentBY, ref halfSpanBX, ref halfSpanBY,
                ref candidates, ref rawContactCount);

            //Vertex A -x, +y
            Vector3Wide.Subtract(ref faceCenterBToFaceCenterA, ref edgeOffsetAX, out vertexA);
            Vector3Wide.Add(ref vertexA, ref edgeOffsetAY, out vertexA);
            //Vertex ids only have two states per axis, so scale id by 0 or 1 before adding. Equivalent to conditional or.          
            //Note that the feature id is negated. This disambiguates between edge-edge contacts and vertex contacts.
            vertexId = -(axisIdAZ + axisIdAY);

            AddFaceVertexContact(ref vertexA, ref vertexId,
                ref tangentBX, ref tangentBY, ref halfSpanBX, ref halfSpanBY,
                ref candidates, ref rawContactCount);

            //Vertex A +x, -y
            Vector3Wide.Add(ref faceCenterBToFaceCenterA, ref edgeOffsetAX, out vertexA);
            Vector3Wide.Subtract(ref vertexA, ref edgeOffsetAY, out vertexA);
            //Vertex ids only have two states per axis, so scale id by 0 or 1 before adding. Equivalent to conditional or.          
            //Note that the feature id is negated. This disambiguates between edge-edge contacts and vertex contacts.
            vertexId = -(axisIdAZ + axisIdAX);

            AddFaceVertexContact(ref vertexA, ref vertexId,
                ref tangentBX, ref tangentBY, ref halfSpanBX, ref halfSpanBY,
                ref candidates, ref rawContactCount);

            //Vertex A +x, +y
            Vector3Wide.Add(ref faceCenterBToFaceCenterA, ref edgeOffsetAX, out vertexA);
            Vector3Wide.Add(ref vertexA, ref edgeOffsetAY, out vertexA);
            //Vertex ids only have two states per axis, so scale id by 0 or 1 before adding. Equivalent to conditional or.          
            //Note that the feature id is negated. This disambiguates between edge-edge contacts and vertex contacts.
            vertexId = -(axisIdAZ + axisIdAX + axisIdAY);

            AddFaceVertexContact(ref vertexA, ref vertexId,
                ref tangentBX, ref tangentBY, ref halfSpanBX, ref halfSpanBY,
                ref candidates, ref rawContactCount);


            //There are up to 8 contacts in the raw set. That's too many; four is plenty. We should choose how to get rid of the extra ones.
            //It's important to keep the deepest contact if there's any significant depth disparity, so we need to calculate depths before reduction.
            //Conceptually, we project the points from the surface of face B down onto face A, then measure the separation of those two points along the normal:
            //depth = dot(pointOnFaceB - faceCenterA, faceNormalA) / dot(faceNormalA, normal)
            //dotAxis = faceNormalA / dot(faceNormalA, normal)
            //depth = dot(pointOnFaceB - faceCenterA, dotAxis)
            //depth = dot(faceCenterB + tangentBX * candidate.X + tangentBY * candidate.Y - faceCenterA, dotAxis)
            //depth = dot(faceCenterB - faceCenterA, dotAxis) + dot(tangentBX, dotAxis) * candidate.X + dot(tangentBY, dotAxis) * candidate.Y
            Vector3Wide.Dot(ref normalA, ref manifold.Normal, out var axisScale);
            axisScale = Vector<float>.One / axisScale;
            Vector3Wide.Scale(ref normalA, ref axisScale, out var dotAxis);
            Vector3Wide.Dot(ref faceCenterBToFaceCenterA, ref dotAxis, out var negativeBaseDot);
            Vector3Wide.Dot(ref tangentBX, ref dotAxis, out var xDot);
            Vector3Wide.Dot(ref tangentBY, ref dotAxis, out var yDot);
            //minor todo: don't really need to waste time initializing to an invalid value.
            var minDepth = new Vector<float>(float.MaxValue);
            var maxExtreme = new Vector<float>(-float.MaxValue);
            Candidate deepest, extreme;
            deepest.Depth = new Vector<float>(-float.MaxValue);
            //minor todo: could try with a horizontal min on counts to avoid checking all 8 if no manifolds have all 8.

            for (int i = 0; i < 8; ++i)
            {
                ref var candidate = ref Unsafe.Add(ref candidates, i);
                candidate.Depth = candidate.X * xDot + candidate.Y * yDot - negativeBaseDot;
                //Note X+Y instead of X or Y alone. Boxes are very often stacked in a nonrandom way; choosing +x or +y alone would lead to near-ties in such cases.
                //This is a pretty small detail, but it is cheap enough that there's no reason not to take advantage of it.
                var index = new Vector<int>(i);
                var indexIsValid = Vector.LessThan(new Vector<int>(i), rawContactCount);
                var candidateIsDeepest = Vector.BitwiseAnd(indexIsValid, Vector.GreaterThan(candidate.Depth, deepest.Depth));
                //Note that we gather the candidate into a local variable rather than storing an index. This avoids the need for a gather (or scalar gather emulation).
                //May be worth doing deeper testing on whether that's worth it, but it'll be a minor difference regardless.
                ConditionalSelect(ref candidateIsDeepest, ref candidate, ref deepest, ref deepest);
                var extremeCandidate = Vector.Max(maxExtreme, candidate.X + candidate.Y);
                var candidateIsMostExtreme = Vector.BitwiseAnd(indexIsValid, Vector.GreaterThan(extremeCandidate, maxExtreme));
                ConditionalSelect(ref candidateIsMostExtreme, ref candidate, ref extreme, ref extreme);
                minDepth = Vector.ConditionalSelect(indexIsValid, Vector.Min(candidate.Depth, minDepth), minDepth);
                maxExtreme = Vector.ConditionalSelect(indexIsValid, extremeCandidate, maxExtreme);
            }

            //Choose the starting point for the contact reduction. Two options:
            //If depth disparity is high, use the deepest index.
            //If depth disparity is too small, use the extreme X index to avoid flickering between manifold start locations.
            var useDepthIndex = Vector.GreaterThan(deepest.Depth - minDepth, new Vector<float>(1e-2f) * epsilonScale);
            Candidate contact0;
            ConditionalSelect(ref useDepthIndex, ref deepest, ref extreme, ref contact0);
            var contact0Exists = Vector.GreaterThan(rawContactCount, Vector<int>.Zero);
            manifold.Count = Vector.ConditionalSelect(contact0Exists, Vector<int>.One, Vector<int>.Zero);

            //Find the most distant point from the starting contact.
            var maxDistanceSquared = Vector<float>.Zero;
            Candidate contact1;
            for (int i = 0; i < 8; ++i)
            {
                ref var candidate = ref Unsafe.Add(ref candidates, i);
                var offsetX = candidate.X - contact0.X;
                var offsetY = candidate.Y - contact0.Y;
                var distanceSquared = offsetX * offsetX + offsetY * offsetY;
                var candidateIsMostDistant = Vector.BitwiseAnd(Vector.GreaterThan(distanceSquared, maxDistanceSquared), Vector.LessThan(new Vector<int>(i), rawContactCount));
                maxDistanceSquared = Vector.ConditionalSelect(candidateIsMostDistant, distanceSquared, maxDistanceSquared);
                ConditionalSelect(ref candidateIsMostDistant, ref candidate, ref contact1, ref contact1);
            }
            //There's no point in additional contacts if the distance between the first and second candidates is zero. Note that this captures the case where there is 0 or 1 contact.
            var contact1Exists = Vector.GreaterThan(maxDistanceSquared, epsilonScale * epsilonScale * new Vector<float>(1e-6f));
            manifold.Count = Vector.ConditionalSelect(contact1Exists, manifold.Count + Vector<int>.One, manifold.Count);

            //Now identify two more points. Using the two existing contacts as a starting edge, pick the points which, when considered as a triangle with the edge,
            //have the largest magnitude negative and positive signed areas.
            var edgeOffsetX = contact1.X - contact0.X;
            var edgeOffsetY = contact1.Y - contact0.Y;
            var minSignedArea = Vector<float>.Zero;
            var maxSignedArea = Vector<float>.Zero;
            Candidate contact2, contact3;
            for (int i = 0; i < 8; ++i)
            {
                //The area of a triangle is proportional to the magnitude of the cross product of two of its edge offsets. 
                //To retain sign, we will conceptually dot against the face's normal. Since we're working on the surface of face B, the arithmetic simplifies heavily to a perp-dot product.
                ref var candidate = ref Unsafe.Add(ref candidates, i);
                var candidateOffsetX = candidate.X - contact0.X;
                var candidateOffsetY = candidate.Y - contact0.Y;
                var signedArea = candidateOffsetX * edgeOffsetY - candidateOffsetY * edgeOffsetX;

                var indexIsValid = Vector.LessThan(new Vector<int>(i), rawContactCount);
                var isMinArea = Vector.BitwiseAnd(Vector.LessThan(signedArea, minSignedArea), indexIsValid);
                minSignedArea = Vector.ConditionalSelect(isMinArea, signedArea, minSignedArea);
                ConditionalSelect(ref isMinArea, ref candidate, ref contact2, ref contact2);
                var isMaxArea = Vector.BitwiseAnd(Vector.GreaterThan(signedArea, maxSignedArea), indexIsValid);
                maxSignedArea = Vector.ConditionalSelect(isMaxArea, signedArea, maxSignedArea);
                ConditionalSelect(ref isMaxArea, ref candidate, ref contact3, ref contact3);
            }

            //Area can be appropximated as a box for the purposes of an epsilon test: (edgeLength0 * span)^2 = (edgeLength0 * edgeLength0 * span * span).
            //This comparison is basically checking if 'span' is irrelevantly small, so we can change this to:
            //edgeLength0 * edgeLength0 * (edgeLength0 * tinyNumber) * (edgeLength0 * tinyNumber) = (edgeLength0^2) * (edgelength0^2) * (tinyNumber^2)
            var epsilon = maxDistanceSquared * maxDistanceSquared * new Vector<float>(1e-6f);
            //Note that minSignedArea is guaranteed to be zero or lower by construction, so it's safe to square for magnitude comparison.
            //Note that these epsilons capture the case where there are two or less raw contacts.
            var minExists = Vector.GreaterThan(minSignedArea * minSignedArea, epsilon);
            manifold.Count = Vector.ConditionalSelect(minExists, manifold.Count + Vector<int>.One, manifold.Count);
            var maxExists = Vector.GreaterThan(maxSignedArea * maxSignedArea, epsilon);
            manifold.Count = Vector.ConditionalSelect(maxExists, manifold.Count + Vector<int>.One, manifold.Count);

            //If there is no min, then the fourth contact should just get put into the third slot. Note that we only had to deal with this now (and not on the previous two contacts)
            //because the existence of the second contact depended on the first, and the third upon the second.
            //But we don't know ahead of time whether there exist min or max signed area contacts.
            var maxShouldGoIntoThirdSlot = Vector.AndNot(maxExists, minExists);
            ConditionalSelect(ref maxShouldGoIntoThirdSlot, ref contact3, ref contact2, ref contact2);

            //Transform the contacts into the manifold.
            TransformContactToManifold(ref contact0, ref faceCenterB, ref tangentBX, ref tangentBY, ref manifold.OffsetA0, ref manifold.Depth0, ref manifold.FeatureId0);
            TransformContactToManifold(ref contact1, ref faceCenterB, ref tangentBX, ref tangentBY, ref manifold.OffsetA1, ref manifold.Depth1, ref manifold.FeatureId1);
            TransformContactToManifold(ref contact2, ref faceCenterB, ref tangentBX, ref tangentBY, ref manifold.OffsetA2, ref manifold.Depth2, ref manifold.FeatureId2);
            TransformContactToManifold(ref contact3, ref faceCenterB, ref tangentBX, ref tangentBY, ref manifold.OffsetA3, ref manifold.Depth3, ref manifold.FeatureId3);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void TransformContactToManifold(ref Candidate rawContact, ref Vector3Wide faceCenterB, ref Vector3Wide tangentBX, ref Vector3Wide tangentBY,
            ref Vector3Wide manifoldOffsetA, ref Vector<float> manifoldDepth, ref Vector<int> manifoldFeatureId)
        {
            Vector3Wide.Scale(ref tangentBX, ref rawContact.X, out manifoldOffsetA);
            Vector3Wide.Scale(ref tangentBY, ref rawContact.Y, out var y);
            Vector3Wide.Add(ref manifoldOffsetA, ref y, out manifoldOffsetA);
            Vector3Wide.Add(ref manifoldOffsetA, ref faceCenterB, out manifoldOffsetA);
            manifoldDepth = rawContact.Depth;
            manifoldFeatureId = rawContact.FeatureId;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ConditionalSelect(ref Vector<int> useA, ref Candidate a, ref Candidate b, ref Candidate result)
        {
            result.X = Vector.ConditionalSelect(useA, a.X, b.X);
            result.Y = Vector.ConditionalSelect(useA, a.Y, b.Y);
            result.Depth = Vector.ConditionalSelect(useA, a.Depth, b.Depth);
            result.FeatureId = Vector.ConditionalSelect(useA, a.FeatureId, b.FeatureId);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void AddFaceVertexContact(ref Vector3Wide faceCenterBToVertexA, ref Vector<int> vertexId,
            ref Vector3Wide tangentBX, ref Vector3Wide tangentBY, ref Vector<float> halfSpanBX, ref Vector<float> halfSpanBY,
            ref Candidate candidates, ref Vector<int> rawContactCount)
        {
            //Get the closest point on face B to vertex A.
            //Note that contacts outside of face B are not added; that could generate contacts outside of either representative face, which can cause some poor contact choices.
            Candidate candidate;
            candidate.FeatureId = vertexId;
            Vector3Wide.Dot(ref faceCenterBToVertexA, ref tangentBX, out candidate.X);
            Vector3Wide.Dot(ref faceCenterBToVertexA, ref tangentBY, out candidate.Y);
            var containedInFaceB = Vector.BitwiseAnd(
                Vector.BitwiseAnd(Vector.GreaterThanOrEqual(candidate.X, -halfSpanBX), Vector.LessThanOrEqual(candidate.X, halfSpanBX)),
                Vector.BitwiseAnd(Vector.GreaterThanOrEqual(candidate.Y, -halfSpanBY), Vector.LessThanOrEqual(candidate.Y, halfSpanBY)));
            //candidate.X = Vector.Min(Vector.Max(candidate.X, -halfSpanBX), halfSpanBX);
            //candidate.Y = Vector.Min(Vector.Max(candidate.Y, -halfSpanBY), halfSpanBY);
            //While we explicitly used an epsilon during edge contact generation, there is a risk of buffer overrun during the face vertex phase.
            //Rather than assuming our numerical epsilon is guaranteed to always work, explicitly clamp the count. This should essentially never be needed,
            //but it is very cheap and guarantees no memory stomping with a pretty reasonable fallback.
            var belowBufferCapacity = Vector.LessThan(rawContactCount, new Vector<int>(8));
            var contactExists = Vector.BitwiseAnd(containedInFaceB, belowBufferCapacity);
            AddContactCandidate(ref candidates, ref candidate, ref contactExists, ref rawContactCount);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void AddEdgeContacts(
            ref Candidate candidates, ref Vector<int> rawContactCount,
            ref Vector<float> halfSpanB, ref Vector<float> epsilonScale,
            ref Vector<float> tMin, ref Vector<float> tMax,
            ref Vector<float> candidateMinX, ref Vector<float> candidateMinY,
            ref Vector<float> candidateMaxX, ref Vector<float> candidateMaxY,
            ref Vector<int> edgeIdB)
        {
            //If -halfSpan<min<halfSpan && max>min for an edge, use the min intersection as a contact.
            //If -halfSpan<max<=halfSpan && (max-min)>epsilon, use the max intersection as a contact.
            //Note the comparisons: if the max lies on a face vertex, it is used, but if the min lies on a face vertex, it is not. This avoids redundant entries.

            //Note that the candidates are stored in terms of locations on the face of B along the tangentBX and tangentBY.
            Candidate candidate;
            candidate.X = candidateMinX;
            candidate.Y = candidateMinY;
            //Note that we use only the edge id of B, regardless of which face bounds contributed to the contact. This is a little permissive, since changes to the 
            //contributors from A won't affect accumulated impulse reuse. I suspect it won't cause any serious issues.
            candidate.FeatureId = edgeIdB;
            var minContactExists = Vector.BitwiseAnd(Vector.BitwiseAnd(
                Vector.GreaterThanOrEqual(tMax, tMin),
                Vector.GreaterThan(tMin, -halfSpanB)),
                Vector.LessThan(tMin, halfSpanB));
            AddContactCandidate(ref candidates, ref candidate, ref minContactExists, ref rawContactCount);
            candidate.X = candidateMaxX;
            candidate.Y = candidateMaxY;
            candidate.FeatureId = edgeIdB + new Vector<int>(64);
            var maxContactExists = Vector.BitwiseAnd(Vector.BitwiseAnd(
                Vector.GreaterThan(tMax, -halfSpanB),
                Vector.LessThanOrEqual(tMax, halfSpanB)),
                Vector.GreaterThan(tMax - tMin, new Vector<float>(1e-5f) * epsilonScale));
            AddContactCandidate(ref candidates, ref candidate, ref maxContactExists, ref rawContactCount);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static unsafe void GetEdgeVersusBoundsIntersections(ref Vector3Wide tangentA, ref Vector<float> halfSpanA,
          ref Vector<float> tangentDotBoundsNormal, ref Vector3Wide faceCenterB, ref Vector3Wide edgeOffsetB,
          out Vector<float> t0Min, out Vector<float> t0Max, out Vector<float> t1Min, out Vector<float> t1Max)
        {
            //Intersect both tangentB edges against the planes with normal equal to tangentA.
            //By protecting against division by zero while maintaining sign, the resulting intervals will still be usable.
            Vector3Wide.Subtract(ref faceCenterB, ref edgeOffsetB, out var edgeCenter0);
            Vector3Wide.Add(ref faceCenterB, ref edgeOffsetB, out var edgeCenter1);
            Vector3Wide.Dot(ref edgeCenter0, ref tangentA, out var taEdgeCenter0);
            Vector3Wide.Dot(ref edgeCenter1, ref tangentA, out var taEdgeCenter1);
            var inverseTangentBoundsNormal = Vector.ConditionalSelect(Vector.LessThan(tangentDotBoundsNormal, Vector<float>.Zero), -Vector<float>.One, Vector<float>.One) /
                Vector.Max(new Vector<float>(1e-15f), Vector.Abs(tangentDotBoundsNormal));
            var axbxScaledHalfSpanAX = halfSpanA * inverseTangentBoundsNormal;
            var axbxScaledAXEdgeCenterX0 = -taEdgeCenter0 * inverseTangentBoundsNormal;
            var axbxScaledAXEdgeCenterX1 = -taEdgeCenter1 * inverseTangentBoundsNormal;
            //These are the t values for intersection between tangentB edges and the tangentA bounding planes.
            //01 refers to the negative offset edge on B, and the positive offset bounding plane on A.
            //Note that they are left unclamped against the edge's extents. We defer that until later to avoid duplicate work.
            var t00 = axbxScaledAXEdgeCenterX0 - axbxScaledHalfSpanAX;
            var t01 = axbxScaledAXEdgeCenterX0 + axbxScaledHalfSpanAX;
            var t10 = axbxScaledAXEdgeCenterX1 - axbxScaledHalfSpanAX;
            var t11 = axbxScaledAXEdgeCenterX1 + axbxScaledHalfSpanAX;
            t0Min = Vector.Min(t00, t01);
            t0Max = Vector.Max(t00, t01);
            t1Min = Vector.Min(t10, t11);
            t1Max = Vector.Max(t10, t11);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static unsafe void GetEdgeVersusFaceBoundsIntervals(ref Vector3Wide tangentAX, ref Vector3Wide tangentAY, ref Vector<float> halfSpanAX, ref Vector<float> halfSpanAY,
            ref Vector<float> axb, ref Vector<float> ayb, ref Vector3Wide faceCenterB, ref Vector<float> halfSpanB, ref Vector3Wide edgeOffsetB,
            out Vector<float> b0Min, out Vector<float> b0Max, out Vector<float> b1Min, out Vector<float> b1Max)
        {
            GetEdgeVersusBoundsIntersections(ref tangentAX, ref halfSpanAX, ref axb, ref faceCenterB, ref edgeOffsetB,
                out var tAX0Min, out var tAX0Max, out var tAX1Min, out var tAX1Max);
            GetEdgeVersusBoundsIntersections(ref tangentAY, ref halfSpanAY, ref ayb, ref faceCenterB, ref edgeOffsetB,
                out var tAY0Min, out var tAY0Max, out var tAY1Min, out var tAY1Max);
            var negativeHalfSpanB = -halfSpanB;
            //Note that we are computing the intersection of the two intervals.
            //If they overlap, then the minimum is the greater of the two minimums, and the maximum is the lesser of the two maximums.
            //After applying those filters, if the min is greater than the max, then the interval has no actual overlap.
            //Note that we explicitly do not clamp both sides of the interval. We want to preserve any interval where the max is below -halfSpanB, or the min is above halfSpanB.
            //Such cases correspond to no contacts.
            //(Note that we do clamp the maximum to the halfSpan. If an interval maximum reaches the end of the interval, it is used to create a contact representing the associated B vertex.
            //The minimums are also clamped, because two of the edges flip their intervals during contact generation to ensure winding. The minimum becomes the maximum in that case.)
            b0Min = Vector.Max(negativeHalfSpanB, Vector.Max(tAX0Min, tAY0Min));
            b0Max = Vector.Min(halfSpanB, Vector.Min(tAX0Max, tAY0Max));
            b1Min = Vector.Max(negativeHalfSpanB, Vector.Max(tAX1Min, tAY1Min));
            b1Max = Vector.Min(halfSpanB, Vector.Min(tAX1Max, tAY1Max));
        }

        struct Candidate
        {
            public Vector<float> X;
            public Vector<float> Y;
            public Vector<float> Depth;
            public Vector<int> FeatureId;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void AddContactCandidate(ref Candidate candidates, ref Candidate candidate, ref Vector<int> newContactExists, ref Vector<int> count)
        {
            //Incrementally maintaining a list is unfortunately a very poor fit for wide vectorization.
            //Each pair has its own count, so the target memory location for storing a new contact in the list is different.
            //If we had efficient scatters, this would look something like:
            //stride = Unsafe.SizeOf<Candidate>() / Unsafe.SizeOf<float>();
            //laneIndices = new Vector<int> { 0, 1, 2, 3... }
            //targetIndices = count * stride + laneIndices;
            //Scatter(ref candidate.X, ref Unsafe.As<Vector<float>, float>(ref candidates[0].X), count)
            //Scatter(ref candidate.Y, ref Unsafe.As<Vector<float>, float>(ref candidates[0].Y), count)
            //Scatter(ref candidate.FeatureId, ref Unsafe.As<Vector<int>, float>(ref candidates[0].FeatureId), count)
            //But we don't have scatter at the moment. We have two options:
            //1) Maintain the vectorized pipeline and emulate the scatters as well as we can with scalar operations. Some risk for running into undefined behavior or compiler issues.
            //2) Immediately drop to full scalar mode, and output an array of AOS ContactManifolds from this test.
            //Note that we perform a conceptual scatter after the completion of each bundle for vectorized pairs anyway, so this wouldn't be catastrophic- 
            //we did get SOME benefit out of vectorization for all the math above. 
            //My guess is that #2 would have a slight advantage overall, but it requires more work at the API level- we'd need a version of the ExecuteBatch that could handle AOS output.
            //For now, we're going to proceed with #1. Emulate the scatters as best we can with scalar code, and maintain the vectorized API.

            for (int i = 0; i < Vector<int>.Count; ++i)
            {
                if (newContactExists[i] < 0)
                {
                    var targetIndex = count[i];
                    ref var target = ref GetOffsetInstance(ref Unsafe.Add(ref candidates, targetIndex), i);
                    //TODO: Check codegen. May be worth doing another offset instance for source data if the compiler inserts bounds checks.
                    GetFirst(ref target.X) = candidate.X[i];
                    GetFirst(ref target.Y) = candidate.Y[i];
                    GetFirst(ref target.FeatureId) = candidate.FeatureId[i];
                }
            }
            count = Vector.ConditionalSelect(newContactExists, count + Vector<int>.One, count);
        }

        public void Test(ref BoxWide a, ref BoxWide b, ref Vector3Wide offsetB, ref QuaternionWide orientationB, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref BoxWide a, ref BoxWide b, ref Vector3Wide offsetB, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }

    public class BoxPairCollisionTask : CollisionTask
    {
        public BoxPairCollisionTask()
        {
            BatchSize = 32;
            ShapeTypeIndexA = default(Box).TypeId;
            ShapeTypeIndexB = default(Box).TypeId;
        }


        //Every single collision task type will mirror this general layout.
        public unsafe override void ExecuteBatch<TContinuations, TFilters>(ref UntypedList batch, ref StreamingBatcher<TFilters, TContinuations> batcher)
        {
            CollisionTaskCommon.ExecuteBatch
                <TContinuations, TFilters,
                Box, BoxWide, Box, BoxWide, UnflippableTestPairWide<Box, BoxWide, Box, BoxWide>,
                Convex4ContactManifoldWide, BoxPairTester>(ref batch, ref batcher);
        }
    }
}
