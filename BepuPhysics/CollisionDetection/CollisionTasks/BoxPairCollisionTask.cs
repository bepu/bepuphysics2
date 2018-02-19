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
            out Vector<float> depth, out Vector<float> localNormalAX, out Vector<float> localNormalAY, out Vector<float> localNormalAZ)
        {

            //Tests one axis of B, passed as rBZ, against all three axes of A.
            var x2 = rBZ.X * rBZ.X;
            var y2 = rBZ.Y * rBZ.Y;
            var z2 = rBZ.Z * rBZ.Z;
            {
                //A.X x B.Z
                var length = Vector.SquareRoot(x2 + y2);
                var inverseLength = Vector<float>.One / length;
                localNormalAX = Vector<float>.Zero;
                localNormalAY = rBZ.Z * inverseLength;
                localNormalAZ = -rBZ.Y * inverseLength;
                var extremeA = Vector.Abs(localNormalAY) * halfHeightA + Vector.Abs(localNormalAZ) * halfLengthB;
                var nBX = localNormalAY * rBX.Y + localNormalAZ * rBX.Z;
                var nBY = localNormalAY * rBY.Y + localNormalAZ * rBY.Z;
                var extremeB = Vector.Abs(nBX) * halfWidthB + Vector.Abs(nBY) * halfHeightB;
                depth = extremeA + extremeB - Vector.Abs(offsetBY * localNormalAY) - Vector.Abs(offsetBZ * localNormalAZ);
                depth = Vector.ConditionalSelect(Vector.LessThan(length, new Vector<float>(1e-7f)), new Vector<float>(float.MaxValue), depth);
            }
            {
                //A.Y x B.Z
                var length = Vector.SquareRoot(x2 + z2);
                var inverseLength = Vector<float>.One / length;
                var nX = rBZ.Z * inverseLength;
                var nZ = -rBZ.X * inverseLength;
                var extremeA = Vector.Abs(nX) * halfWidthA + Vector.Abs(nZ) * halfLengthA;
                var nBX = nX * rBX.X + nZ * rBX.Z;
                var nBY = nX * rBY.X + nZ * rBY.Z;
                var extremeB = Vector.Abs(nBX) * halfWidthB + Vector.Abs(nBY) * halfHeightB;
                var d = extremeA + extremeB - Vector.Abs(offsetBX * nX) - Vector.Abs(offsetBZ * nZ);
                d = Vector.ConditionalSelect(Vector.LessThan(length, new Vector<float>(1e-7f)), new Vector<float>(float.MaxValue), d);
                var useY = Vector.LessThan(d, depth);
                depth = Vector.ConditionalSelect(useY, d, depth);
                localNormalAX = Vector.ConditionalSelect(useY, nX, localNormalAX);
                localNormalAY = Vector.ConditionalSelect(useY, Vector<float>.Zero, localNormalAY);
                localNormalAZ = Vector.ConditionalSelect(useY, nZ, localNormalAZ);
            }
            {
                //A.Z x B.Z
                var length = Vector.SquareRoot(y2 + z2);
                var inverseLength = Vector<float>.One / length;
                var nX = rBZ.Y * inverseLength;
                var nY = -rBZ.X * inverseLength;
                var extremeA = Vector.Abs(nX) * halfWidthA + Vector.Abs(nY) * halfHeightA;
                var nBX = nX * rBX.X + nY * rBX.Y;
                var nBY = nX * rBY.X + nY * rBY.Y;
                var extremeB = Vector.Abs(nBX) * halfWidthB + Vector.Abs(nBY) * halfHeightB;
                var d = extremeA + extremeB - Vector.Abs(offsetBX * nX) - Vector.Abs(offsetBY * nY);
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
            QuaternionWide.Conjugate(ref orientationA, out var toLocalA);
            QuaternionWide.TransformWithoutOverlap(ref offsetB, ref toLocalA, out var localOffsetB);
            QuaternionWide.ConcatenateWithoutOverlap(ref orientationB, ref toLocalA, out var localOrientationB);
            Matrix3x3Wide.CreateFromQuaternion(ref localOrientationB, out var rB);

            Vector3Wide localNormal;
            //b.X: XYZ -> YZX
            TestEdgeEdge(
                ref a.HalfWidth, ref a.HalfHeight, ref a.HalfLength,
                ref b.HalfHeight, ref b.HalfLength, ref b.HalfWidth,
                ref localOffsetB.X, ref localOffsetB.Y, ref localOffsetB.Z,
                ref rB.Y, ref rB.Z, ref rB.X,
                out var depth, out localNormal.X, out localNormal.Y, out localNormal.Z);
            //b.Y: XYZ -> XZY
            TestEdgeEdge(
                ref a.HalfWidth, ref a.HalfHeight, ref a.HalfLength,
                ref b.HalfWidth, ref b.HalfLength, ref b.HalfHeight,
                ref localOffsetB.X, ref localOffsetB.Y, ref localOffsetB.Z,
                ref rB.X, ref rB.Z, ref rB.Y,
                out var edgeYDepth, out var edgeYNX, out var edgeYNY, out var edgeYNZ);
            Select(ref depth, ref localNormal,
                ref edgeYDepth, ref edgeYNX, ref edgeYNY, ref edgeYNZ);
            //b.Z: XYZ -> XYZ
            TestEdgeEdge(
                ref a.HalfWidth, ref a.HalfHeight, ref a.HalfLength,
                ref b.HalfWidth, ref b.HalfHeight, ref b.HalfLength,
                ref localOffsetB.X, ref localOffsetB.Y, ref localOffsetB.Z,
                ref rB.X, ref rB.Y, ref rB.Z,
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
            Vector3Wide.Dot(ref localNormal, ref offsetB, out var normalDotOffsetB);
            var shouldNegateNormal = Vector.GreaterThan(normalDotOffsetB, Vector<float>.Zero);
            localNormal.X = Vector.ConditionalSelect(shouldNegateNormal, -localNormal.X, localNormal.X);
            localNormal.Y = Vector.ConditionalSelect(shouldNegateNormal, -localNormal.Y, localNormal.Y);
            localNormal.Z = Vector.ConditionalSelect(shouldNegateNormal, -localNormal.Z, localNormal.Z);

            //Contact generation always assumes face-face clipping. Other forms of contact generation are just special cases of face-face, and since we pay
            //for all code paths, there's no point in handling them separately.
            //We just have to guarantee that the face chosen on each box is guaranteed to include the deepest feature along the contact normal.
            //To do this, choose the face on each box associated with the minimal depth of all of its face axes.
            //Note that there are other options- choosing the face normal with greatest dot with the collision normal would also provide the same guarantee,
            //but the depth choice sometimes chooses more appropriate faces.

            //We represent each face as a center position, its two tangent axes, and the length along those axes.
            //Technically, we could leave A's tangents implicit by swizzling components, but that complicates things a little bit for not much gain.
            var minDepthA = Vector.Min(faceAXDepth, Vector.Min(faceAYDepth, faceAZDepth));
            var useAX = Vector.Equals(minDepthA, faceAXDepth);
            var useAY = Vector.AndNot(Vector.Equals(minDepthA, faceAYDepth), useAX);
            var useAZ = Vector.OnesComplement(Vector.BitwiseOr(useAX, useAY));
            Vector3Wide tangentAX, tangentAY, normalA;
            Vector<float> halfSpanAX, halfSpanAY, halfSpanAZ;
            normalA.X = Vector.ConditionalSelect(useAX, Vector<float>.One, Vector<float>.Zero);
            normalA.Y = Vector.ConditionalSelect(useAY, Vector<float>.One, Vector<float>.Zero);
            normalA.Z = Vector.ConditionalSelect(useAZ, Vector<float>.One, Vector<float>.Zero);
            tangentAX.X = Vector.ConditionalSelect(useAY, Vector<float>.One, Vector<float>.Zero);
            tangentAX.Y = Vector.ConditionalSelect(useAZ, Vector<float>.One, Vector<float>.Zero);
            tangentAX.Z = Vector.ConditionalSelect(useAX, Vector<float>.One, Vector<float>.Zero);
            tangentAY.X = Vector.ConditionalSelect(useAZ, Vector<float>.One, Vector<float>.Zero);
            tangentAY.Y = Vector.ConditionalSelect(useAX, Vector<float>.One, Vector<float>.Zero);
            tangentAY.Z = Vector.ConditionalSelect(useAY, Vector<float>.One, Vector<float>.Zero);
            halfSpanAX = Vector.ConditionalSelect(useAX, a.HalfLength, Vector.ConditionalSelect(useAY, a.HalfWidth, a.HalfHeight));
            halfSpanAY = Vector.ConditionalSelect(useAX, a.HalfHeight, Vector.ConditionalSelect(useAY, a.HalfLength, a.HalfWidth));
            halfSpanAZ = Vector.ConditionalSelect(useAX, a.HalfWidth, Vector.ConditionalSelect(useAY, a.HalfHeight, a.HalfLength));

            var minDepthB = Vector.Min(faceBXDepth, Vector.Min(faceBYDepth, faceBZDepth));
            var useBX = Vector.Equals(minDepthB, faceBXDepth);
            var useBY = Vector.AndNot(Vector.Equals(minDepthB, faceBYDepth), useBX);
            Vector3Wide tangentBX, tangentBY, normalB;
            Vector<float> halfSpanBX, halfSpanBY, halfSpanBZ;
            Vector3Wide.ConditionalSelect(ref useBX, ref rB.X, ref rB.Z, out normalB);
            Vector3Wide.ConditionalSelect(ref useBY, ref rB.Y, ref normalB, out normalB);
            Vector3Wide.ConditionalSelect(ref useBX, ref rB.Z, ref rB.Y, out tangentBX);
            Vector3Wide.ConditionalSelect(ref useBY, ref rB.X, ref tangentBX, out tangentBX);
            Vector3Wide.ConditionalSelect(ref useBX, ref rB.Y, ref rB.X, out tangentBY);
            Vector3Wide.ConditionalSelect(ref useBY, ref rB.Z, ref tangentBY, out tangentBY);
            halfSpanBX = Vector.ConditionalSelect(useBX, b.HalfLength, Vector.ConditionalSelect(useBY, b.HalfWidth, b.HalfHeight));
            halfSpanBY = Vector.ConditionalSelect(useBX, b.HalfHeight, Vector.ConditionalSelect(useBY, b.HalfLength, b.HalfWidth));
            halfSpanBZ = Vector.ConditionalSelect(useBX, b.HalfWidth, Vector.ConditionalSelect(useBY, b.HalfHeight, b.HalfLength));

            //Collect contact points. There are a maximum of 8 possible contacts as a result of this process, but we will consider a total of 12 candidates.
            var buffer = stackalloc byte[Unsafe.SizeOf<Candidate>() * 8];
            ref var candidates = ref Unsafe.As<byte, Candidate>(ref *buffer);

            //Two phases:
            //1) Intersect face edges of B with face planes of A. 
            //If -halfSpan<min<halfSpan for an edge, use the min intersection as a contact.
            //If -halfSpan<max<=halfSpan && (max-min)>epsilon*halfSpan, use the max intersection as a contact.
            //Note the comparisons: if the max lies on a face vertex, it is used, but if the min lies on a face vertex, it is not. This avoids redundant entries.
            //2) Test vertices of face A against the bounds of B projected onto face A's plane.

            //Edge ids use 6 bits per. This has lots of redundancy, but it's easy to construct edge ids from axis ids.
            //The only combinations that are used are those which have 1 value in the intermediate state and 2 values in extreme states (12 edges).
            //x: 1, 2, 3 (2 bits)
            //y: 4, 8, 12 (2 bits)
            //z: 16, 32, 48 (2 bits)

            //Vertex ids just use 3 bits, representing the 8 possible corners.
            //+x: 1
            //+y: 2
            //+z: 4

            //Its feature id is aEdgeId << 6 + bEdgeId. (We don't have shifts, so just multiply by 1<<6.)

            //Its feature id is simply the negative vertex's feature id. The negation disambiguates between edge-edge pairs and vertex contacts.

            //While we explicitly used an epsilon earlier during edge contact generation, there is a risk of buffer overrun during the face vertex phase.
            //Rather than assuming our numerical epsilon is guaranteed to always work, explicitly clamp the count. This should essentially never be needed,
            //but it is very cheap and guarantees no memory stomping with a pretty reasonable fallback.

            //Per-contact depths can be computed using dot(abs(normal), halfExtentsB) - dot(normal, vertexPosition - localOffsetB), because we projected all contacts
            //onto the representative face of A.

            //Reduce the manifold.
            //If deepest - shallowest is large, then pick the deepest candidate as the first contact.
            //If there is not a significant depth disparity between contacts, then attempting to use the deepest contact would result in an unstable manifold- 
            //the represented features would be inconsistent and the solver would often lose history for some contacts if the raw manifold had more than 4 contacts.

            //Transform the normal and reduced manifold positions back into world space.

        }


        struct Candidate
        {
            public Vector<float> X;
            public Vector<float> Y;
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
            //Scatter(ref candidate.Z, ref Unsafe.As<Vector<float>, float>(ref candidates[0].Z), count)
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
        public unsafe override void ExecuteBatch<TContinuations, TFilters>(ref UntypedList batch, ref StreamingBatcher batcher, ref TContinuations continuations, ref TFilters filters)
        {
            CollisionTaskCommon.ExecuteBatch
                <TContinuations, TFilters,
                Box, BoxWide, Box, BoxWide, UnflippableTestPairWide<Box, BoxWide, Box, BoxWide>,
                Convex4ContactManifoldWide, BoxPairTester>(ref batch, ref batcher, ref continuations, ref filters);
        }
    }
}
