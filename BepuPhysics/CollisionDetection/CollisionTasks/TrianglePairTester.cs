using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct TrianglePairTester : IPairTester<TriangleWide, TriangleWide, Convex4ContactManifoldWide>
    {
        public int BatchSize => 32;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void GetIntervalForNormal(in Vector3Wide a, in Vector3Wide b, in Vector3Wide c, in Vector3Wide normal, out Vector<float> min, out Vector<float> max)
        {
            Vector3Wide.Dot(normal, a, out var dA);
            Vector3Wide.Dot(normal, b, out var dB);
            Vector3Wide.Dot(normal, c, out var dC);
            min = Vector.Min(dA, Vector.Min(dB, dC));
            max = Vector.Max(dA, Vector.Max(dB, dC));
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void GetDepthForNormal(in Vector3Wide aA, in Vector3Wide bA, in Vector3Wide cA, in Vector3Wide aB, in Vector3Wide bB, in Vector3Wide cB,
            in Vector3Wide normal, out Vector<float> depth)
        {
            GetIntervalForNormal(aA, bA, cA, normal, out var minA, out var maxA);
            GetIntervalForNormal(aB, bB, cB, normal, out var minB, out var maxB);
            depth = Vector.Min(maxA - minB, maxB - minA);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestEdgeEdge(
            in Vector3Wide edgeDirectionA, in Vector3Wide edgeDirectionB,
            in Vector3Wide aA, in Vector3Wide bA, in Vector3Wide cA, in Vector3Wide aB, in Vector3Wide bB, in Vector3Wide cB,
            out Vector<float> depth, out Vector3Wide normal)
        {
            //Calibrate the normal to point from the triangle to the box while normalizing.
            Vector3Wide.CrossWithoutOverlap(edgeDirectionA, edgeDirectionB, out normal);
            Vector3Wide.Length(normal, out var normalLength);
            //Note that we do not calibrate yet. The depth calculation does not rely on calibration, so we punt it until after all normals have been tested.
            Vector3Wide.Scale(normal, Vector<float>.One / normalLength, out normal);
            GetDepthForNormal(aA, bA, cA, aB, bB, cB, normal, out depth);
            //Protect against bad normals.
            depth = Vector.ConditionalSelect(Vector.LessThan(normalLength, new Vector<float>(1e-10f)), new Vector<float>(float.MaxValue), depth);
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
        private static void TryAddTriangleAVertex(in Vector3Wide vertex, in Vector<int> vertexId,
            in Vector3Wide tangentBX, in Vector3Wide tangentBY, in Vector3Wide triangleCenterB, in Vector3Wide contactNormal,in Vector3Wide faceNormalB,
            in Vector3Wide edgeABPlaneNormalB, in Vector3Wide edgeBCPlaneNormalB, in Vector3Wide edgeCAPlaneNormalB, in Vector3Wide bA, in Vector3Wide bB,
            in Vector<int> contactNormalDotFaceNormalBIsValid, in Vector<float> inverseContactNormalDotFaceNormalB,
            ref ManifoldCandidate candidates, ref Vector<int> candidateCount, int pairCount)
        {
            //Test edge edge plane sign for all three edges of B. We can test the vertex directly rather than the unprojected vertex because the ray cast follows the contact normal,
            //and all of these plane normals are perpendicular to the contact normal.
            Vector3Wide.Subtract(vertex, bA, out var bAToVertex);
            Vector3Wide.Subtract(vertex, bB, out var bBToVertex);
            Vector3Wide.Dot(bAToVertex, edgeABPlaneNormalB, out var abDot);
            Vector3Wide.Dot(bBToVertex, edgeBCPlaneNormalB, out var bcDot);
            Vector3Wide.Dot(bAToVertex, edgeCAPlaneNormalB, out var caDot);
            var abContained = Vector.GreaterThan(abDot, Vector<float>.Zero);
            var bcContained = Vector.GreaterThan(bcDot, Vector<float>.Zero);
            var caContained = Vector.GreaterThan(caDot, Vector<float>.Zero);
            var contained = Vector.BitwiseAnd(abContained, Vector.BitwiseAnd(bcContained, caContained));

            //Cast a ray from triangle A's vertex along the contact normal up to the plane of triangle B and check for containment.
            //We use the contact normal rather than the face normal to reduce contact generation dependency on pair ordering.
            //(There are other ways to implement the combined ray cast->containment test; we're just going with the simple way.)
            //(-contactNormal * t - vertexOnA + triangleCenterB) * faceNormalB = 0
            //(-contactNormal * t + (triangleCenterB - vertexOnA) * faceNormalB = 0
            //(-contactNormal * faceNormalB) * t = (triangleCenterB - vertexOnA) * faceNormalB
            //t = (vertexOnA - triangleCenterB) * faceNormalB / (contactNormal * faceNormalB)
            Vector3Wide.Subtract(triangleCenterB, vertex, out var offset);
            Vector3Wide.Dot(offset, faceNormalB, out var distance);
            var t = distance * inverseContactNormalDotFaceNormalB;
            Vector3Wide.Scale(contactNormal, t, out var unprojectedVertex);
            Vector3Wide.Add(unprojectedVertex, vertex, out unprojectedVertex);

            ManifoldCandidate candidate;
            Vector3Wide.Subtract(unprojectedVertex, triangleCenterB, out var offsetOnB);
            Vector3Wide.Dot(offsetOnB, tangentBX, out candidate.X);
            Vector3Wide.Dot(offsetOnB, tangentBY, out candidate.Y);
            candidate.FeatureId = vertexId;
            ManifoldCandidateHelper.AddCandidate(ref candidates, ref candidateCount, candidate, Vector.BitwiseAnd(contactNormalDotFaceNormalBIsValid, contained), pairCount);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ClipEdge(in Vector3Wide edgeStart, in Vector3Wide edgeDirection, in Vector3Wide pointOnPlane, in Vector3Wide planeNormal, out Vector<float> entry, out Vector<float> exit)
        {
            //The edge plane normal points toward the inside of the bounding triangle.
            //intersection = dot(planeNormal, pointOnPlane - edgeStart) / dot(planeNormal, edgeDirectionB)
            Vector3Wide.Subtract(pointOnPlane, edgeStart, out var edgeToPlane);
            Vector3Wide.Dot(edgeToPlane, planeNormal, out var edgePlaneNormalDot);
            Vector3Wide.Dot(edgeDirection, planeNormal, out var velocity);
            var t = edgePlaneNormalDot / velocity;
            var isEntry = Vector.GreaterThanOrEqual(velocity, Vector<float>.Zero);
            var validVelocity = Vector.GreaterThan(Vector.Abs(velocity), new Vector<float>(1e-10f));
            entry = Vector.ConditionalSelect(Vector.BitwiseAnd(validVelocity, isEntry), t, new Vector<float>(float.MinValue));
            exit = Vector.ConditionalSelect(Vector.AndNot(validVelocity, isEntry), t, new Vector<float>(float.MaxValue));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ClipBEdgeAgainstABounds(
             in Vector3Wide edgeABPlaneNormalA, in Vector3Wide edgeBCPlaneNormalA, in Vector3Wide edgeCAPlaneNormalA,
             in Vector3Wide aA, in Vector3Wide aB,
             in Vector3Wide edgeDirectionB, in Vector3Wide edgeStartB, in Vector<int> entryId, in Vector<int> exitIdOffset,
             in Vector3Wide triangleCenterB, in Vector3Wide tangentBX, in Vector3Wide tangentBY,
             in Vector<float> epsilon, ref ManifoldCandidate candidates, ref Vector<int> candidateCount, int pairCount)
        {
            //The base id is the id of the vertex in the corner along the negative boxEdgeDirection and boxEdgeCenterOffsetDirection.
            //The edgeDirectionId is the amount to add when you move along the boxEdgeDirection to the other vertex.
            //The edgeCenterOffsetId is the amount to add when you move along the boxEdgeCenterOffsetDirection to the other vertex.

            //We have three edge planes created by the edges of triangle A.
            //We want to test the triangle B edge against all three of the edges.
            ClipEdge(edgeStartB, edgeDirectionB, aA, edgeABPlaneNormalA, out var entryAB, out var exitAB);
            ClipEdge(edgeStartB, edgeDirectionB, aB, edgeBCPlaneNormalA, out var entryBC, out var exitBC);
            ClipEdge(edgeStartB, edgeDirectionB, aA, edgeCAPlaneNormalA, out var entryCA, out var exitCA);
            var entry = Vector.Max(Vector.Max(Vector<float>.Zero, entryAB), Vector.Max(entryBC, entryCA));
            var exit = Vector.Min(Vector.Min(Vector<float>.One, exitAB), Vector.Min(exitBC, exitCA));

            //entryX = dot(entry * edgeDirectionA + edgeStartA - triangleCenterB, tangentBX)
            //entryY = dot(entry * edgeDirectionA + edgeStartA - triangleCenterB, tangentBY)
            //exitX = dot(exit * edgeDirectionA + edgeStartA - triangleCenterB, tangentBX)
            //exitY = dot(exit * edgeDirectionA + edgeStartA - triangleCenterB, tangentBY)
            Vector3Wide.Subtract(edgeStartB, triangleCenterB, out var offset);
            Vector3Wide.Dot(offset, tangentBX, out var offsetX);
            Vector3Wide.Dot(offset, tangentBY, out var offsetY);
            Vector3Wide.Dot(tangentBX, edgeDirectionB, out var edgeDirectionX);
            Vector3Wide.Dot(tangentBY, edgeDirectionB, out var edgeDirectionY);

            ManifoldCandidate candidate;
            var six = new Vector<int>(6);
            //Entry
            var exists = Vector.BitwiseAnd(Vector.BitwiseAnd(
                Vector.LessThan(candidateCount, six),
                Vector.GreaterThanOrEqual(exit - entry, epsilon)),
                Vector.BitwiseAnd(
                    Vector.LessThan(entry, Vector<float>.One),
                    Vector.GreaterThan(entry, Vector<float>.Zero)));
            candidate.X = entry * edgeDirectionX + offsetX;
            candidate.Y = entry * edgeDirectionY + offsetY;
            candidate.FeatureId = entryId;
            ManifoldCandidateHelper.AddCandidate(ref candidates, ref candidateCount, candidate, exists, pairCount);
            //Exit
            exists = Vector.BitwiseAnd(Vector.BitwiseAnd(
                Vector.LessThan(candidateCount, six),
                Vector.GreaterThanOrEqual(exit, entry)),
                Vector.BitwiseAnd(
                    Vector.LessThanOrEqual(exit, Vector<float>.One),
                    Vector.GreaterThanOrEqual(exit, Vector<float>.Zero)));
            candidate.X = exit * edgeDirectionX + offsetX;
            candidate.Y = exit * edgeDirectionY + offsetY;
            candidate.FeatureId = entryId + exitIdOffset;
            ManifoldCandidateHelper.AddCandidate(ref candidates, ref candidateCount, candidate, exists, pairCount);
        }



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Test(
            ref TriangleWide a, ref TriangleWide b, ref Vector<float> speculativeMargin,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount,
            out Convex4ContactManifoldWide manifold)
        {
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var worldRA);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var worldRB);
            Matrix3x3Wide.MultiplyByTransposeWithoutOverlap(worldRB, worldRA, out var rB);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, worldRA, out var localOffsetB);
            Matrix3x3Wide.TransformWithoutOverlap(b.A, rB, out var bA);
            Vector3Wide.Add(bA, localOffsetB, out bA);
            Matrix3x3Wide.TransformWithoutOverlap(b.B, rB, out var bB);
            Vector3Wide.Add(bB, localOffsetB, out bB);
            Matrix3x3Wide.TransformWithoutOverlap(b.C, rB, out var bC);
            Vector3Wide.Add(bC, localOffsetB, out bC);

            Vector3Wide.Add(bA, bB, out var localTriangleCenterB);
            Vector3Wide.Add(localTriangleCenterB, bC, out localTriangleCenterB);
            Vector3Wide.Scale(localTriangleCenterB, new Vector<float>(1f / 3f), out localTriangleCenterB);

            Vector3Wide.Subtract(bB, bA, out var abB);
            Vector3Wide.Subtract(bC, bB, out var bcB);
            Vector3Wide.Subtract(bA, bC, out var caB);

            Vector3Wide.Add(a.A, a.B, out var localTriangleCenterA);
            Vector3Wide.Add(localTriangleCenterA, a.C, out localTriangleCenterA);
            Vector3Wide.Scale(localTriangleCenterA, new Vector<float>(1f / 3f), out localTriangleCenterA);

            Vector3Wide.Subtract(a.B, a.A, out var abA);
            Vector3Wide.Subtract(a.C, a.B, out var bcA);
            Vector3Wide.Subtract(a.A, a.C, out var caA);

            //A AB x *
            TestEdgeEdge(abA, abB, a.A, a.B, a.C, bA, bB, bC, out var depth, out var localNormal);
            TestEdgeEdge(abA, bcB, a.A, a.B, a.C, bA, bB, bC, out var depthCandidate, out var localNormalCandidate);
            Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            TestEdgeEdge(abA, caB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);

            //A BC x *
            TestEdgeEdge(bcA, abB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            TestEdgeEdge(bcA, bcB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            TestEdgeEdge(bcA, caB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);

            //A CA x *
            TestEdgeEdge(caA, abB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            TestEdgeEdge(caA, bcB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            TestEdgeEdge(caA, caB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);

            //Face normals
            Vector3Wide.CrossWithoutOverlap(abA, caA, out var faceNormalA);
            Vector3Wide.Length(faceNormalA, out var faceNormalALength);
            Vector3Wide.Scale(faceNormalA, Vector<float>.One / faceNormalALength, out faceNormalA);
            GetDepthForNormal(a.A, a.B, a.C, bA, bB, bC, faceNormalA, out depthCandidate);
            Select(ref depth, ref localNormal, depthCandidate, faceNormalA);
            Vector3Wide.CrossWithoutOverlap(abB, caB, out var faceNormalB);
            Vector3Wide.Length(faceNormalB, out var faceNormalBLength);
            Vector3Wide.Scale(faceNormalB, Vector<float>.One / faceNormalBLength, out faceNormalB);
            GetDepthForNormal(a.A, a.B, a.C, bA, bB, bC, faceNormalB, out var faceDepthB);
            Select(ref depth, ref localNormal, faceDepthB, faceNormalB);

            //Point the normal from B to A by convention.
            Vector3Wide.Subtract(localTriangleCenterB, localTriangleCenterA, out var centerAToCenterB);
            Vector3Wide.Dot(localNormal, centerAToCenterB, out var calibrationDot);
            var shouldFlip = Vector.GreaterThan(calibrationDot, Vector<float>.Zero);
            localNormal.X = Vector.ConditionalSelect(shouldFlip, -localNormal.X, localNormal.X);
            localNormal.Y = Vector.ConditionalSelect(shouldFlip, -localNormal.Y, localNormal.Y);
            localNormal.Z = Vector.ConditionalSelect(shouldFlip, -localNormal.Z, localNormal.Z);

            //At this point, we have computed the minimum depth and associated local normal.
            //We now need to compute some contact locations, their per-contact depths, and the feature ids.

            //Contact generation always assumes face-face clipping. Other forms of contact generation are just special cases of face-face, and since we pay
            //for all code paths, there's no point in handling them separately.            

            //We will be working on the surface of the triangle, but we'd still like a 2d parameterization of the surface for contact reduction.
            //So, we'll create tangent axes from the edge and edge x normal.
            Vector3Wide.LengthSquared(abB, out var abBLengthSquared);
            Vector3Wide.Scale(abB, Vector<float>.One / Vector.SquareRoot(abBLengthSquared), out var tangentBX);
            Vector3Wide.CrossWithoutOverlap(tangentBX, faceNormalB, out var tangentBY);

            //Note that we only allocate up to 6 candidates. Each triangle edge can contribute at most two contacts (any more would require a nonconvex clip region).
            //Numerical issues can cause more to occur, but they're guarded against (both directly, and in the sense of checking count before adding any candidates beyond the sixth).
            int byteCount = Unsafe.SizeOf<ManifoldCandidate>() * 6;
            var buffer = stackalloc byte[byteCount];
            var candidateCount = Vector<int>.Zero;
            ref var candidates = ref Unsafe.As<byte, ManifoldCandidate>(ref *buffer);

            //While the edge clipping will find any edge-edge or aVertex-bFace contacts, it will not find bVertex-aFace contacts.
            //Add them independently.
            //(Adding these first allows us to simply skip capacity tests, since there can only be a total of three bVertex-aFace contacts.)

            Vector3Wide.Dot(faceNormalB, localNormal, out var contactNormalDotFaceNormalB);
            var inverseContactNormalDotFaceNormalB = Vector<float>.One / contactNormalDotFaceNormalB;
            var contactNormalDotFaceNormalBIsValid = Vector.GreaterThan(Vector.Abs(contactNormalDotFaceNormalB), new Vector<float>(1e-10f));
            Vector3Wide.CrossWithoutOverlap(abB, localNormal, out var edgeABPlaneNormalB);
            Vector3Wide.CrossWithoutOverlap(bcB, localNormal, out var edgeBCPlaneNormalB);
            Vector3Wide.CrossWithoutOverlap(caB, localNormal, out var edgeCAPlaneNormalB);
            TryAddTriangleAVertex(a.A, Vector<int>.Zero, tangentBX, tangentBY, localTriangleCenterB, localNormal, faceNormalB, edgeABPlaneNormalB, edgeBCPlaneNormalB, edgeCAPlaneNormalB, bA, bB, contactNormalDotFaceNormalBIsValid, inverseContactNormalDotFaceNormalB, ref candidates, ref candidateCount, pairCount);
            TryAddTriangleAVertex(a.B, Vector<int>.One, tangentBX, tangentBY, localTriangleCenterB, localNormal, faceNormalB, edgeABPlaneNormalB, edgeBCPlaneNormalB, edgeCAPlaneNormalB, bA, bB, contactNormalDotFaceNormalBIsValid, inverseContactNormalDotFaceNormalB, ref candidates, ref candidateCount, pairCount);
            TryAddTriangleAVertex(a.C, new Vector<int>(2), tangentBX, tangentBY, localTriangleCenterB, localNormal, faceNormalB, edgeABPlaneNormalB, edgeBCPlaneNormalB, edgeCAPlaneNormalB, bA, bB, contactNormalDotFaceNormalBIsValid, inverseContactNormalDotFaceNormalB, ref candidates, ref candidateCount, pairCount);

            //Note that edge cases will also add triangle A vertices that are within triangle B's bounds, so no A vertex case is required.
            //Note that each of these calls can generate 4 contacts, so we have to start checking capacities.

            //Create a scale-sensitive epsilon for comparisons based on the size of the involved shapes. This helps avoid varying behavior based on how large involved objects are.
            Vector3Wide.LengthSquared(abA, out var abALengthSquared);
            Vector3Wide.LengthSquared(caA, out var caALengthSquared);
            Vector3Wide.LengthSquared(caB, out var caBLengthSquared);
            var epsilonScale = Vector.SquareRoot(Vector.Min(
                Vector.Max(abALengthSquared, caALengthSquared),
                Vector.Max(abBLengthSquared, caBLengthSquared)));
            var edgeEpsilon = new Vector<float>(1e-5f) * epsilonScale;
            var exitIdOffset = new Vector<int>(3);
            //Note the use of localNormal here, NOT faceNormalA. Why? Just like in the vertexA case, we're not creating contacts in triangle A's face voronoi region.
            //Instead, the test region is skewed along the contact normal. These planes intersect A's edges and have the contact normal as a tangent.
            //This avoids dependency on pair order (consider what happens when A and B swap) and produces nicer contacts in corner cases (consider two near perpendicular triangles).
            Vector3Wide.CrossWithoutOverlap(localNormal, abA, out var edgeABPlaneNormalA);
            Vector3Wide.CrossWithoutOverlap(localNormal, bcA, out var edgeBCPlaneNormalA);
            Vector3Wide.CrossWithoutOverlap(localNormal, caA, out var edgeCAPlaneNormalA);
            ClipBEdgeAgainstABounds(edgeABPlaneNormalA, edgeBCPlaneNormalA, edgeCAPlaneNormalA, a.A, a.B, abB, bA, new Vector<int>(3), exitIdOffset, localTriangleCenterB, tangentBX, tangentBY, edgeEpsilon, ref candidates, ref candidateCount, pairCount);
            ClipBEdgeAgainstABounds(edgeABPlaneNormalA, edgeBCPlaneNormalA, edgeCAPlaneNormalA, a.A, a.B, bcB, bB, new Vector<int>(4), exitIdOffset, localTriangleCenterB, tangentBX, tangentBY, edgeEpsilon, ref candidates, ref candidateCount, pairCount);
            ClipBEdgeAgainstABounds(edgeABPlaneNormalA, edgeBCPlaneNormalA, edgeCAPlaneNormalA, a.A, a.B, caB, bC, new Vector<int>(5), exitIdOffset, localTriangleCenterB, tangentBX, tangentBY, edgeEpsilon, ref candidates, ref candidateCount, pairCount);

            Vector3Wide.Subtract(localTriangleCenterA, localTriangleCenterB, out var faceCenterBToFaceCenterA);
            ManifoldCandidateHelper.Reduce(ref candidates, candidateCount, 6, faceNormalA, localNormal, faceCenterBToFaceCenterA, tangentBX, tangentBY, epsilonScale, -speculativeMargin, pairCount,
                out var contact0, out var contact1, out var contact2, out var contact3,
                out manifold.Contact0Exists, out manifold.Contact1Exists, out manifold.Contact2Exists, out manifold.Contact3Exists);

            //Transform the contacts into the manifold.
            //Move the basis into world rotation so that we don't have to transform the individual contacts.
            Matrix3x3Wide.TransformWithoutOverlap(tangentBX, worldRA, out var worldTangentBX);
            Matrix3x3Wide.TransformWithoutOverlap(tangentBY, worldRA, out var worldTangentBY);
            Matrix3x3Wide.TransformWithoutOverlap(localTriangleCenterB, worldRA, out var worldTriangleCenter);
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, worldRA, out manifold.Normal);
            //If the local normal points against either triangle normal, then it's on the backside and should not collide.
            Vector3Wide.Dot(localNormal, faceNormalA, out var normalDotA);
            Vector3Wide.Dot(localNormal, faceNormalB, out var normalDotB);
            var allowContacts = Vector.BitwiseAnd(Vector.LessThan(normalDotA, Vector<float>.Zero), Vector.GreaterThan(normalDotB, Vector<float>.Zero));
            manifold.Contact0Exists = Vector.BitwiseAnd(manifold.Contact0Exists, allowContacts);
            manifold.Contact1Exists = Vector.BitwiseAnd(manifold.Contact1Exists, allowContacts);
            manifold.Contact2Exists = Vector.BitwiseAnd(manifold.Contact2Exists, allowContacts);
            manifold.Contact3Exists = Vector.BitwiseAnd(manifold.Contact3Exists, allowContacts);
            TransformContactToManifold(contact0, worldTriangleCenter, worldTangentBX, worldTangentBY, out manifold.OffsetA0, out manifold.Depth0, out manifold.FeatureId0);
            TransformContactToManifold(contact1, worldTriangleCenter, worldTangentBX, worldTangentBY, out manifold.OffsetA1, out manifold.Depth1, out manifold.FeatureId1);
            TransformContactToManifold(contact2, worldTriangleCenter, worldTangentBX, worldTangentBY, out manifold.OffsetA2, out manifold.Depth2, out manifold.FeatureId2);
            TransformContactToManifold(contact3, worldTriangleCenter, worldTangentBX, worldTangentBY, out manifold.OffsetA3, out manifold.Depth3, out manifold.FeatureId3);
            //Note that we privilege triangle B. Boundary smoothing is only performed on one of the two meshes.
            var faceFlag = Vector.ConditionalSelect(
                Vector.GreaterThanOrEqual(normalDotB, new Vector<float>(MeshReduction.MinimumDotForFaceCollision)), new Vector<int>(MeshReduction.FaceCollisionFlag), Vector<int>.Zero);
            manifold.FeatureId0 += faceFlag;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void TransformContactToManifold(
            in ManifoldCandidate rawContact, in Vector3Wide faceCenterB, in Vector3Wide tangentBX, in Vector3Wide tangentBY,
            out Vector3Wide manifoldOffsetA, out Vector<float> manifoldDepth, out Vector<int> manifoldFeatureId)
        {
            Vector3Wide.Scale(tangentBX, rawContact.X, out manifoldOffsetA);
            Vector3Wide.Scale(tangentBY, rawContact.Y, out var y);
            Vector3Wide.Add(manifoldOffsetA, y, out manifoldOffsetA);
            Vector3Wide.Add(manifoldOffsetA, faceCenterB, out manifoldOffsetA);
            manifoldDepth = rawContact.Depth;
            manifoldFeatureId = rawContact.FeatureId;
        }

        public void Test(ref TriangleWide a, ref TriangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref TriangleWide a, ref TriangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }
}
