using BepuPhysics.Collidables;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct BoxPairTester : IPairTester<BoxWide, BoxWide, Convex4ContactManifoldWide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestEdgeEdge2(
            ref Vector<float> halfWidthA, ref Vector<float> halfHeightA,
            ref Vector<float> halfWidthB, ref Vector<float> halfHeightB, ref Vector<float> halfLengthB,
            ref Vector<float> offsetBX, ref Vector<float> offsetBY,
            ref Vector3Wide rBX, ref Vector3Wide rBY, ref Vector3Wide rBZ,
            out Vector<float> depth, out Vector<float> localNormalAX, out Vector<float> localNormalAY, out Vector<float> localNormalAZ)
        {
            //This function is hardcoded for testing the z axis of each box against each other.
            //In use, the parameters are swizzled to test the other axes.
            var length = Vector.SquareRoot(rBZ.Y * rBZ.Y + rBZ.X * rBZ.X);
            var inverseLength = Vector<float>.One / length;
            localNormalAX = rBZ.Y * inverseLength;
            localNormalAY = -rBZ.X * inverseLength;
            var extremeA = Vector.Abs(localNormalAX) * halfWidthA + Vector.Abs(localNormalAY) * halfWidthA;
            var localNormalBX = localNormalAX * rBX.X + localNormalAY * rBX.Y;
            var localNormalBY = localNormalAX * rBY.X + localNormalAY * rBY.Y;
            var extremeB = Vector.Abs(localNormalBX) * halfWidthB + Vector.Abs(localNormalBY) * halfHeightB;
            depth = extremeA + extremeB - Vector.Abs(offsetBX * localNormalAX) - Vector.Abs(offsetBY * localNormalAY);
            //If the edges are parallel, then the depth we computed here is invalid and should be ignored. 
            //Fortunately, during intersection, one of the face normals will result in a depth no worse than an edge->edge offset based fallback.
            //(In the separated case, the edge->edge offset would be a superior choice, but that's a good bit of extra work for marginal benefit in a corner case.
            //I suppose if you wanted to stack blocks edge on edge, this would be an issue. We can worry about that later.)
            Vector.ConditionalSelect(Vector.LessThan(length, new Vector<float>(1e-7f)), new Vector<float>(float.MaxValue), depth);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestEdgeEdge3(
                    ref Vector<float> halfWidthA, ref Vector<float> halfHeightA,
                    ref Vector<float> halfWidthB, ref Vector<float> halfHeightB,
                    ref Vector<float> offsetBX, ref Vector<float> offsetBY,
                    ref Vector<float> rBXX, ref Vector<float> rBXY,
                    ref Vector<float> rBYX, ref Vector<float> rBYY,
                    ref Vector<float> rBZX, ref Vector<float> rBZY,
                    out Vector<float> depth, out Vector<float> localNormalAX, out Vector<float> localNormalAY)
        {
            //This function is hardcoded for testing the z axis of A against the z axis of B.
            //In use, the parameters are swizzled to test the other axes.
            var length = Vector.SquareRoot(rBZY * rBZY + rBZX * rBZX);
            var inverseLength = Vector<float>.One / length;
            localNormalAX = rBZY * inverseLength;
            localNormalAY = -rBZX * inverseLength;
            var extremeA = Vector.Abs(localNormalAX) * halfWidthA + Vector.Abs(localNormalAY) * halfWidthA;
            var localNormalBX = localNormalAX * rBXX + localNormalAY * rBXY;
            var localNormalBY = localNormalAX * rBYX + localNormalAY * rBYY;
            var extremeB = Vector.Abs(localNormalBX) * halfWidthB + Vector.Abs(localNormalBY) * halfHeightB;
            depth = extremeA + extremeB - Vector.Abs(offsetBX * localNormalAX) - Vector.Abs(offsetBY * localNormalAY);
            //If the edges are parallel, then the depth we computed here is invalid and should be ignored. 
            //Fortunately, during intersection, one of the face normals will result in a depth no worse than an edge->edge offset based fallback.
            //(In the separated case, the edge->edge offset would be a superior choice, but that's a good bit of extra work for marginal benefit in a corner case.
            //I suppose if you wanted to stack blocks edge on edge, this would be an issue. We can worry about that later.)
            Vector.ConditionalSelect(Vector.LessThan(length, new Vector<float>(1e-7f)), new Vector<float>(float.MaxValue), depth);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Select(
            ref Vector<float> depth, ref Vector<float> nX, ref Vector<float> nY, ref Vector<float> nZ,
            ref Vector<float> candidateDepth, ref Vector<float> candidateNX, ref Vector<float> candidateNY, ref Vector<float> candidateNZ)
        {
            var useCandidate = Vector.LessThan(candidateDepth, depth);
            depth = Vector.ConditionalSelect(useCandidate, candidateDepth, depth);
            nX = Vector.ConditionalSelect(useCandidate, candidateNX, nX);
            nY = Vector.ConditionalSelect(useCandidate, candidateNY, nY);
            nZ = Vector.ConditionalSelect(useCandidate, candidateNZ, nZ);
        }

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
        public void Test(
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
            Select(ref depth, ref localNormal.X, ref localNormal.Y, ref localNormal.Z,
                ref edgeYDepth, ref edgeYNX, ref edgeYNY, ref edgeYNZ);
            //b.Z: XYZ -> XYZ
            TestEdgeEdge(
                ref a.HalfWidth, ref a.HalfHeight, ref a.HalfLength,
                ref b.HalfWidth, ref b.HalfHeight, ref b.HalfLength,
                ref localOffsetB.X, ref localOffsetB.Y, ref localOffsetB.Z,
                ref rB.X, ref rB.Y, ref rB.Z,
                out var edgeZDepth, out var edgeZNX, out var edgeZNY, out var edgeZNZ);
            Select(ref depth, ref localNormal.X, ref localNormal.Y, ref localNormal.Z,
                ref edgeZDepth, ref edgeZNX, ref edgeZNY, ref edgeZNZ);

            Vector3Wide.Abs(ref rB.X, out var absRBX);
            Vector3Wide.Abs(ref rB.Y, out var absRBY);
            Vector3Wide.Abs(ref rB.Z, out var absRBZ);
            var faceAXDepth = a.HalfWidth + b.HalfWidth * absRBX.X + b.HalfHeight * absRBY.X + b.HalfLength * absRBZ.X - Vector.Abs(localOffsetB.X);
            var faceAYDepth = a.HalfHeight + b.HalfWidth * absRBX.Y + b.HalfHeight * absRBY.Y + b.HalfLength * absRBZ.Y - Vector.Abs(localOffsetB.Y);
            var faceAZDepth = a.HalfLength + b.HalfWidth * absRBX.Z + b.HalfHeight * absRBY.Z + b.HalfLength * absRBZ.Z - Vector.Abs(localOffsetB.Z);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ref localOffsetB, ref rB, out var bLocalOffsetB);
            var faceBXDepth = b.HalfWidth + a.HalfWidth * absRBX.X + a.HalfHeight * absRBX.Y + a.HalfLength * absRBX.Z - Vector.Abs(bLocalOffsetB.X);
            var faceBYDepth = b.HalfHeight + a.HalfWidth * absRBY.X + a.HalfHeight * absRBY.Y + a.HalfLength * absRBY.Z - Vector.Abs(bLocalOffsetB.Y);
            var faceBZDepth = b.HalfLength + a.HalfWidth * absRBZ.X + a.HalfHeight * absRBZ.Y + a.HalfLength * absRBZ.Z - Vector.Abs(bLocalOffsetB.Z);
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
