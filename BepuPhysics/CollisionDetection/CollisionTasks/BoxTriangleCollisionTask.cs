using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using static BepuUtilities.GatherScatter;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct BoxTriangleTester : IPairTester<BoxWide, TriangleWide, Convex4ContactManifoldWide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestBoxEdgeAgainstTriangleEdge(
            in Vector<float> triangleEdgeOffsetY, in Vector<float> triangleEdgeOffsetZ,
            in Vector<float> triangleCenterY, in Vector<float> triangleCenterZ,
            in Vector<float> edgeOffsetYSquared, in Vector<float> edgeOffsetZSquared,
            in Vector<float> halfHeight, in Vector<float> halfLength,
            in Vector3Wide vA, in Vector3Wide vB, in Vector3Wide vC,
            out Vector<float> depth, out Vector3Wide localNormal)
        {
            //We assume we're working with the box's X edge direction. Caller will swizzle.
            localNormal.X = Vector<float>.Zero;
            localNormal.Y = triangleEdgeOffsetZ;
            localNormal.Z = -triangleEdgeOffsetY;

            //Calibrate the normal to point from the triangle to the box while normalizing.
            var calibrationDot = triangleCenterY * localNormal.Y + triangleCenterZ * localNormal.Z;
            var length = Vector.SquareRoot(edgeOffsetYSquared + edgeOffsetZSquared);
            var inverseLength = Vector.ConditionalSelect(Vector.LessThan(calibrationDot, Vector<float>.Zero), Vector<float>.One, new Vector<float>(-1f)) / length;
            localNormal.Y *= inverseLength;
            localNormal.Z *= inverseLength;

            var extremeA = Vector.Abs(localNormal.Y) * halfHeight + Vector.Abs(localNormal.Z) * halfLength;
            var nVA = vA.Y * localNormal.Y + vA.Z * localNormal.Z;
            var nVB = vB.Y * localNormal.Y + vB.Z * localNormal.Z;
            var nVC = vC.Y * localNormal.Y + vC.Z * localNormal.Z;
            var extremeB = Vector.Min(nVA, Vector.Min(nVB, nVC));
            depth = extremeA - extremeB;
            depth = Vector.ConditionalSelect(Vector.LessThan(length, new Vector<float>(1e-7f)), new Vector<float>(float.MaxValue), depth);

        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestBoxEdgesAgainstTriangleEdge(
            in BoxWide a,
            in Vector3Wide triangleEdgeOffset, in Vector3Wide triangleCenter,
            in Vector3Wide vA, in Vector3Wide vB, in Vector3Wide vC,
            out Vector<float> depth, out Vector3Wide localNormal)
        {

            var x2 = triangleEdgeOffset.X * triangleEdgeOffset.X;
            var y2 = triangleEdgeOffset.Y * triangleEdgeOffset.Y;
            var z2 = triangleEdgeOffset.Z * triangleEdgeOffset.Z;
            //A.X x edgeB
            TestBoxEdgeAgainstTriangleEdge(triangleEdgeOffset.Y, triangleEdgeOffset.Z,
                triangleCenter.Y, triangleCenter.Z,
                y2, z2, a.HalfHeight, a.HalfLength,
                vA, vB, vC, out depth, out localNormal);

            //A.Y x edgeB
            TestBoxEdgeAgainstTriangleEdge(triangleEdgeOffset.X, triangleEdgeOffset.Z,
                triangleCenter.X, triangleCenter.Z,
                x2, z2, a.HalfWidth, a.HalfLength,
                vA, vB, vC, out var depthCandidate, out var localNormalCandidate);
            Vector3Wide.ConditionalSelect(Vector.LessThan(depthCandidate, depth), localNormalCandidate, localNormal, out localNormal);
            depth = Vector.Min(depth, depthCandidate);

            //A.Z x edgeB
            TestBoxEdgeAgainstTriangleEdge(triangleEdgeOffset.X, triangleEdgeOffset.Y,
                triangleCenter.X, triangleCenter.Y,
                x2, y2, a.HalfWidth, a.HalfHeight,
                vA, vB, vC, out depthCandidate, out localNormalCandidate);
            Vector3Wide.ConditionalSelect(Vector.LessThan(depthCandidate, depth), localNormalCandidate, localNormal, out localNormal);
            depth = Vector.Min(depth, depthCandidate);

        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Select(
            ref Vector<float> depth, ref Vector3Wide normal,
            in Vector<float> depthCandidate, in Vector3Wide normalCandidate)
        {
            var useCandidate = Vector.LessThan(depthCandidate, depth);
            Vector3Wide.ConditionalSelect(useCandidate, normalCandidate, normal, out normal);
            depth = Vector.Min(depth, depthCandidate);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Select(
            ref Vector<float> depth, ref Vector3Wide normal,
            in Vector<float> depthCandidate, in Vector<float> nxCandidate, in Vector<float> nyCandidate, in Vector<float> nzCandidate)
        {
            var useCandidate = Vector.LessThan(depthCandidate, depth);
            normal.X = Vector.ConditionalSelect(useCandidate, nxCandidate, normal.X);
            normal.Y = Vector.ConditionalSelect(useCandidate, nyCandidate, normal.Y);
            normal.Z = Vector.ConditionalSelect(useCandidate, nzCandidate, normal.Z);
            depth = Vector.Min(depth, depthCandidate);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void ClipBoxEdgesAgainstTriangle(in Vector3Wide vA, in Vector3Wide vB, in Vector3Wide vC, 
            in Vector3Wide boxEdgeDirection, in Vector3Wide boxEdgeCenterOffsetDirection, in Vector<float> edgeHalfLength, in Vector<float> edgeCenterOffset, 
            in Vector3Wide boxFaceCenter, in Vector<int> baseId, in Vector<int> edgeDirectionId, in Vector<int> edgeCenterOffsetId, 
            in Vector<float> epsilonScale, ref ManifoldCandidate candidates, ref Vector<int> candidateCount)
        {
            //The base id is the id of the vertex in the corner along the negative boxEdgeDirection and boxEdgeCenterOffsetDirection.
            //The edgeDirectionId is the amount to add when you move along the boxEdgeDirection to the other vertex.
            //The edgeCenterOffsetId is the amount to add when you move along the boxEdgeCenterOffsetDirection to the other vertex.

            throw new NotImplementedException();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Test(
            ref BoxWide a, ref TriangleWide b, ref Vector<float> speculativeMargin,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Convex4ContactManifoldWide manifold)
        {
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var worldRA);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var worldRB);
            Matrix3x3Wide.MultiplyByTransposeWithoutOverlap(worldRB, worldRA, out var rB);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, worldRA, out var localOffsetB);
            Matrix3x3Wide.Transform(b.A, rB, out var vA);
            Vector3Wide.Add(vA, localOffsetB, out vA);
            Matrix3x3Wide.Transform(b.B, rB, out var vB);
            Vector3Wide.Add(vB, localOffsetB, out vB);
            Matrix3x3Wide.Transform(b.C, rB, out var vC);
            Vector3Wide.Add(vC, localOffsetB, out vC);

            Vector3Wide.Add(vA, vB, out var localTriangleCenter);
            Vector3Wide.Add(localTriangleCenter, vC, out localTriangleCenter);
            Vector3Wide.Scale(localTriangleCenter, new Vector<float>(1f / 3f), out localTriangleCenter);

            Vector3Wide.Subtract(vB, vA, out var ab);
            Vector3Wide.Subtract(vC, vB, out var bc);
            Vector3Wide.Subtract(vA, vC, out var ca);

            TestBoxEdgesAgainstTriangleEdge(a, ab, localTriangleCenter, vA, vB, vC, out var depth, out var localNormal);
            TestBoxEdgesAgainstTriangleEdge(a, bc, localTriangleCenter, vA, vB, vC, out var depthCandidate, out var localNormalCandidate);
            Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            TestBoxEdgesAgainstTriangleEdge(a, ca, localTriangleCenter, vA, vB, vC, out depthCandidate, out localNormalCandidate);
            Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);

            //Test face normals of A. Working in local space of A means potential axes are just (1,0,0) etc.
            var xNormalSign = Vector.ConditionalSelect(Vector.LessThan(localTriangleCenter.X, Vector<float>.Zero), Vector<float>.One, new Vector<float>(-1f));
            var yNormalSign = Vector.ConditionalSelect(Vector.LessThan(localTriangleCenter.Y, Vector<float>.Zero), Vector<float>.One, new Vector<float>(-1f));
            var zNormalSign = Vector.ConditionalSelect(Vector.LessThan(localTriangleCenter.Z, Vector<float>.Zero), Vector<float>.One, new Vector<float>(-1f));
            var faceAXDepth = a.HalfWidth - Vector.Min(xNormalSign * vA.X, Vector.Min(xNormalSign * vB.X, xNormalSign * vC.X));
            var faceAYDepth = a.HalfHeight - Vector.Min(yNormalSign * vA.Y, Vector.Min(yNormalSign * vB.Y, yNormalSign * vC.Y));
            var faceAZDepth = a.HalfLength - Vector.Min(zNormalSign * vA.Z, Vector.Min(zNormalSign * vB.Z, zNormalSign * vC.Z));
            Select(ref depth, ref localNormal, faceAXDepth, xNormalSign, Vector<float>.Zero, Vector<float>.Zero);
            Select(ref depth, ref localNormal, faceAYDepth, Vector<float>.Zero, yNormalSign, Vector<float>.Zero);
            Select(ref depth, ref localNormal, faceAZDepth, Vector<float>.Zero, Vector<float>.Zero, zNormalSign);

            //Test face normal of B.
            Vector3Wide.CrossWithoutOverlap(ab, ca, out var triangleNormal);
            Vector3Wide.Length(triangleNormal, out var triangleNormalLength);
            Vector3Wide.Scale(triangleNormal, Vector<float>.One / triangleNormalLength, out triangleNormal);
            //Note that we do not calibrate the triangle normal. The backside of the triangle does not collide, so any normal that ends up pointing that direction will be ignored anyway.
            Vector3Wide.Dot(triangleNormal, localTriangleCenter, out var trianglePlaneOffset);
            var triangleFaceDepth =
                Vector.Abs(triangleNormal.X) * a.HalfWidth + Vector.Abs(triangleNormal.Y) * a.HalfHeight + Vector.Abs(triangleNormal.Z) * a.HalfLength + trianglePlaneOffset;
            Select(ref depth, ref localNormal, triangleFaceDepth, triangleNormal);

            //At this point, we have computed the minimum depth and associated local normal.
            //We now need to compute some contact locations, their per-contact depths, and the feature ids.

            //Contact generation always assumes face-face clipping. Other forms of contact generation are just special cases of face-face, and since we pay
            //for all code paths, there's no point in handling them separately.
            //We just have to guarantee that the face chosen on the box is guaranteed to include the deepest feature along the contact normal.
            //To do this, choose the face on box A associated with the minimum axis dot with the collision normal. (The normal is calibrated to point triangle->box, hence minimum.)

            //We represent the chosen box face as its two tangent axes and the extent along those axes.
            var absNX = Vector.Abs(localNormal.X);
            var absNY = Vector.Abs(localNormal.Y);
            var absNZ = Vector.Abs(localNormal.Z);
            var xBiggerThanY = Vector.GreaterThan(absNX, absNY);
            var xBiggerThanZ = Vector.GreaterThan(absNX, absNZ);
            var yBiggerThanZ = Vector.GreaterThan(absNY, absNZ);
            var useAX = Vector.BitwiseAnd(xBiggerThanY, xBiggerThanZ);
            var useAY = Vector.AndNot(yBiggerThanZ, useAX);
            var useAZ = Vector.OnesComplement(Vector.BitwiseOr(useAX, useAY));

            var normalIsNegativeX = Vector.LessThan(localNormal.X, Vector<float>.Zero);
            var normalIsNegativeY = Vector.LessThan(localNormal.Y, Vector<float>.Zero);
            var normalIsNegativeZ = Vector.LessThan(localNormal.Z, Vector<float>.Zero);

            //If we're using face with normal X, the tangent axes are Z and Y
            //If we're using face with normal Y, the tangent axes are X and Z
            //If we're using face with normal Z, the tangent axes are X and Y
            Vector3Wide boxTangentX, boxTangentY, boxFaceNormal;
            boxTangentX.X = Vector.ConditionalSelect(useAZ, Vector<float>.One, Vector<float>.Zero);
            boxTangentX.Y = Vector<float>.Zero;
            boxTangentX.Z = Vector.ConditionalSelect(Vector.BitwiseOr(useAY, useAZ), Vector<float>.One, Vector<float>.Zero);
            boxTangentY.X = Vector<float>.Zero;
            boxTangentY.Y = Vector.ConditionalSelect(Vector.BitwiseOr(useAX, useAZ), Vector<float>.One, Vector<float>.Zero);
            boxTangentY.Z = Vector.ConditionalSelect(useAY, Vector<float>.One, Vector<float>.Zero);
            var negativeOne = new Vector<float>(-1f);
            boxFaceNormal.X = Vector.ConditionalSelect(useAX, Vector.ConditionalSelect(normalIsNegativeX, Vector<float>.One, negativeOne), Vector<float>.Zero);
            boxFaceNormal.Y = Vector.ConditionalSelect(useAY, Vector.ConditionalSelect(normalIsNegativeY, Vector<float>.One, negativeOne), Vector<float>.Zero);
            boxFaceNormal.Z = Vector.ConditionalSelect(useAZ, Vector.ConditionalSelect(normalIsNegativeZ, Vector<float>.One, negativeOne), Vector<float>.Zero);

            var halfExtentX = Vector.ConditionalSelect(useAX, a.HalfLength, Vector.ConditionalSelect(useAY, a.HalfWidth, a.HalfWidth));
            var halfExtentY = Vector.ConditionalSelect(useAX, a.HalfHeight, Vector.ConditionalSelect(useAY, a.HalfLength, a.HalfHeight));
            var halfExtentZ = Vector.ConditionalSelect(useAX, a.HalfWidth, Vector.ConditionalSelect(useAY, a.HalfHeight, a.HalfLength));
            Vector3Wide.Scale(boxFaceNormal, halfExtentZ, out var boxFaceCenter);

            //For feature ids, we will use:
            //Triangle vertex index (0, 1, 2) for box face contacts 
            //Box vertex index (0, 1, ... 7, 8) + 3 for triangle face contacts
            //And (boxEdgeId (0, 1, ... 11, 12) << 4) | (triangleEdgeId (0, 1, 2) << 8)) for edge-edge contacts.
            var localXId = new Vector<int>(1);
            var localYId = new Vector<int>(2);
            var localZId = new Vector<int>(4);
            var axisIdTangentX = Vector.ConditionalSelect(useAX, localZId, Vector.ConditionalSelect(useAY, localXId, localXId));
            var axisIdTangentY = Vector.ConditionalSelect(useAX, localYId, Vector.ConditionalSelect(useAY, localZId, localYId));
            var axisIdNormal =
                Vector.ConditionalSelect(useAX, Vector.ConditionalSelect(
                    normalIsNegativeX, localXId, Vector<int>.Zero),
                Vector.ConditionalSelect(useAY, Vector.ConditionalSelect(
                    normalIsNegativeY, localYId, Vector<int>.Zero),
                Vector.ConditionalSelect(
                    normalIsNegativeZ, localZId, Vector<int>.Zero)));


            //Note that using a raw absolute epsilon would have a varying effect based on the scale of the involved shapes.
            //The minimum across the maxes is intended to avoid cases like a huge box being used as a plane, causing a massive size disparity.
            //Using its sizes as a threshold would tend to kill off perfectly valid contacts.
            Vector3Wide.LengthSquared(ab, out var abLengthSquared);
            Vector3Wide.LengthSquared(bc, out var bcLengthSquared);
            Vector3Wide.LengthSquared(ca, out var caLengthSquared);
            var epsilonScale = Vector.Min(
                Vector.Max(a.HalfWidth, Vector.Max(a.HalfHeight, a.HalfLength)),
                Vector.SquareRoot(Vector.Max(abLengthSquared, Vector.Max(bcLengthSquared, caLengthSquared))));

            //We will be working on the surface of the triangle, but we'd still like a 2d parameterization of the surface for contact reduction.
            //So, we'll create tangent axes from the edge and edge x normal.
            Vector3Wide.Scale(ab, Vector<float>.One / Vector.SquareRoot(abLengthSquared), out var triangleTangentX);
            Vector3Wide.CrossWithoutOverlap(triangleTangentX, triangleNormal, out var triangleTangentY);
            
            //Note that we only allocate up to 6 candidates. Each triangle edge can contribute at most two contacts (any more would require a nonconvex clip region).
            //Numerical issues can cause more to occur, but they're guarded against (both directly, and in the sense of checking count before adding any candidates beyond the sixth).
            int byteCount = Unsafe.SizeOf<ManifoldCandidate>() * 6;
            var buffer = stackalloc byte[byteCount];
            var candidateCount = Vector<int>.Zero;
            ref var candidates = ref Unsafe.As<byte, ManifoldCandidate>(ref *buffer);

            ClipBoxEdgesAgainstTriangle(vA, vB, vC, boxTangentX, boxTangentY, halfExtentX, halfExtentY, boxFaceCenter,  
                axisIdNormal, axisIdTangentX, axisIdTangentY, 
                epsilonScale, ref candidates, ref candidateCount);
            ClipBoxEdgesAgainstTriangle(vA, vB, vC, boxTangentY, boxTangentX, halfExtentY, halfExtentX, boxFaceCenter,
                axisIdNormal, axisIdTangentY, axisIdTangentX,
                epsilonScale, ref candidates, ref candidateCount);

            //While the edge clipping will find any triangleEdge-boxEdge or boxVertex--triangleFace contacts, it will not find triangleVertex-boxFace contacts.
            //Add them independently.
            //TODO: Do that!


            Vector3Wide.Subtract(boxFaceCenter, localTriangleCenter, out var faceCenterBToFaceCenterA);
            ManifoldCandidateHelper.Reduce(ref candidates, candidateCount, 6, boxFaceNormal, localNormal, faceCenterBToFaceCenterA, triangleTangentX, triangleTangentY, epsilonScale,
                out var contact0, out var contact1, out var contact2, out var contact3,
                out manifold.Contact0Exists, out manifold.Contact1Exists, out manifold.Contact2Exists, out manifold.Contact3Exists);

            //Transform the contacts into the manifold.
            var minimumAcceptedDepth = -speculativeMargin;
            //Move the basis into world rotation so that we don't have to transform the individual contacts.
            Matrix3x3Wide.TransformWithoutOverlap(triangleTangentX, worldRA, out var worldTangentBX);
            Matrix3x3Wide.TransformWithoutOverlap(triangleTangentY, worldRA, out var worldTangentBY);
            Matrix3x3Wide.TransformWithoutOverlap(localTriangleCenter, worldRA, out var worldTriangleCenter);
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, worldRA, out manifold.Normal);
            TransformContactToManifold(contact0, worldTriangleCenter, worldTangentBX, worldTangentBY, minimumAcceptedDepth, ref manifold.Contact0Exists, out manifold.OffsetA0, out manifold.Depth0, out manifold.FeatureId0);
            TransformContactToManifold(contact1, worldTriangleCenter, worldTangentBX, worldTangentBY, minimumAcceptedDepth, ref manifold.Contact1Exists, out manifold.OffsetA1, out manifold.Depth1, out manifold.FeatureId1);
            TransformContactToManifold(contact2, worldTriangleCenter, worldTangentBX, worldTangentBY, minimumAcceptedDepth, ref manifold.Contact2Exists, out manifold.OffsetA2, out manifold.Depth2, out manifold.FeatureId2);
            TransformContactToManifold(contact3, worldTriangleCenter, worldTangentBX, worldTangentBY, minimumAcceptedDepth, ref manifold.Contact3Exists, out manifold.OffsetA3, out manifold.Depth3, out manifold.FeatureId3);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void TransformContactToManifold(
            in ManifoldCandidate rawContact, in Vector3Wide faceCenterB, in Vector3Wide tangentBX, in Vector3Wide tangentBY, in Vector<float> minimumAcceptedDepth,
            ref Vector<int> contactExists, out Vector3Wide manifoldOffsetA, out Vector<float> manifoldDepth, out Vector<int> manifoldFeatureId)
        {
            Vector3Wide.Scale(tangentBX, rawContact.X, out manifoldOffsetA);
            Vector3Wide.Scale(tangentBY, rawContact.Y, out var y);
            Vector3Wide.Add(manifoldOffsetA, y, out manifoldOffsetA);
            Vector3Wide.Add(manifoldOffsetA, faceCenterB, out manifoldOffsetA);
            //Note that we delayed the speculative margin depth test until the end. This ensures area maximization has meaningful contacts to work with.
            //If we were more aggressive about the depth testing, the final manifold would tend to have more contacts, but less meaningful contacts.
            contactExists = Vector.BitwiseAnd(contactExists, Vector.GreaterThanOrEqual(rawContact.Depth, minimumAcceptedDepth));
            manifoldDepth = rawContact.Depth;
            manifoldFeatureId = rawContact.FeatureId;
        }

        public void Test(ref BoxWide a, ref TriangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref BoxWide a, ref TriangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }

    public class BoxTriangleCollisionTask : CollisionTask
    {
        public BoxTriangleCollisionTask()
        {
            BatchSize = 32;
            ShapeTypeIndexA = default(Box).TypeId;
            ShapeTypeIndexB = default(Triangle).TypeId;
        }


        //Every single collision task type will mirror this general layout.
        public unsafe override void ExecuteBatch<TCallbacks>(ref UntypedList batch, ref CollisionBatcher<TCallbacks> batcher)
        {
            ConvexCollisionTaskCommon.ExecuteBatch
                <TCallbacks,
                Box, BoxWide, Triangle, TriangleWide, TestPairWide<Box, BoxWide, Triangle, TriangleWide>,
                Convex4ContactManifoldWide, BoxTriangleTester>(ref batch, ref batcher);
        }
    }
}
