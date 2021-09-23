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
        static void TestEdgeEdge2(
            in Vector3Wide edgeStartA, in Vector3Wide edgeOffsetA, in Vector<float> edgeOffsetALengthSquared, in Vector<float> inverseEdgeOffsetALengthSquared,
            in Vector3Wide edgeStartB, in Vector3Wide edgeOffsetB, in Vector<float> edgeOffsetBLengthSquared, in Vector<float> inverseEdgeOffsetBLengthSquared,
            in Vector3Wide aA, in Vector3Wide bA, in Vector3Wide cA, in Vector3Wide aB, in Vector3Wide bB, in Vector3Wide cB,
            out Vector<float> depth, out Vector3Wide normal)
        {
            Vector3Wide.Subtract(edgeStartA, edgeStartB, out var bStartToAStart);
            Vector3Wide.Dot(edgeOffsetA, bStartToAStart, out var oADotAB);
            Vector3Wide.Dot(edgeOffsetB, bStartToAStart, out var oBDotAB);
            Vector3Wide.Dot(edgeOffsetA, edgeOffsetB, out var oADotOB);
            var denominator = edgeOffsetALengthSquared * edgeOffsetBLengthSquared - oADotOB * oADotOB; //TODO: div 0 guard if edge offsets are parallel.
            //Compute the first guess for tA and clamp it.
            var tA = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, (oADotOB * oBDotAB - oADotAB * edgeOffsetBLengthSquared) / denominator));
            //Compute the closest point on B to the first guess.
            var tB = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, (oADotOB * tA + oBDotAB) * inverseEdgeOffsetBLengthSquared));
            //Finally, compute the true tA.
            tA = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, (tB * oADotOB - oADotAB) * inverseEdgeOffsetALengthSquared));

            Vector3Wide.Scale(edgeOffsetA, tA, out var startToClosestA);
            Vector3Wide.Add(edgeStartA, startToClosestA, out var a);
            Vector3Wide.Scale(edgeOffsetB, tB, out var startToClosestB);
            Vector3Wide.Add(edgeStartB, startToClosestB, out var b);

            //If the segments touch, then fall back to using the edge direction as the normal candidate.
            Vector3Wide.Subtract(a, b, out var ba);
            Vector3Wide.LengthSquared(ba, out var baLengthSquared);
            var useFallback = Vector.LessThan(baLengthSquared, new Vector<float>(1e-12f));
            Vector3Wide.CrossWithoutOverlap(edgeOffsetA, edgeOffsetB, out var fallbackNormal);
            Vector3Wide.ConditionalSelect(useFallback, fallbackNormal, ba, out normal);
            Vector3Wide.Length(normal, out var normalLength);
            Vector3Wide.Scale(normal, Vector<float>.One / normalLength, out normal);
            GetDepthForNormal(aA, bA, cA, aB, bB, cB, normal, out depth);
            //Protect against bad normals.
            depth = Vector.ConditionalSelect(Vector.LessThan(normalLength, new Vector<float>(1e-10f)), new Vector<float>(float.MaxValue), depth);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestVertexNormal(in Vector3Wide vertex,
            in Vector3Wide a, in Vector3Wide b, in Vector3Wide c,
            in Vector3Wide opposingA, in Vector3Wide opposingB, in Vector3Wide opposingC,
            in Vector3Wide opposingAB, in Vector3Wide opposingBC, in Vector3Wide opposingCA,
            in Vector<float> inverseLengthSquaredAB, in Vector<float> inverseLengthSquaredBC, in Vector<float> inverseLengthSquaredCA,
            out Vector<float> depth, out Vector3Wide normal)
        {
            //Project the vertex onto all three edges. Take the closest approach as a normal candidate.
            //Note that this will try normals that cross over the triangle's face, but that's fine- it'll just be a crappy normal candidate and another option will be chosen instead.
            Vector3Wide.Subtract(vertex, opposingA, out var av);
            Vector3Wide.Subtract(vertex, opposingB, out var bv);
            Vector3Wide.Subtract(vertex, opposingC, out var cv);
            Vector3Wide.Dot(av, opposingAB, out var avDotAB);
            Vector3Wide.Dot(bv, opposingBC, out var bvDotBC);
            Vector3Wide.Dot(cv, opposingCA, out var cvDotCA);
            var tAB = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, avDotAB * inverseLengthSquaredAB));
            var tBC = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, bvDotBC * inverseLengthSquaredBC));
            var tCA = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, cvDotCA * inverseLengthSquaredCA));
            Vector3Wide vToAB, vToBC, vToCA;
            vToAB.X = opposingA.X + opposingAB.X * tAB - vertex.X;
            vToAB.Y = opposingA.Y + opposingAB.Y * tAB - vertex.Y;
            vToAB.Z = opposingA.Z + opposingAB.Z * tAB - vertex.Z;
            vToBC.X = opposingB.X + opposingBC.X * tBC - vertex.X;
            vToBC.Y = opposingB.Y + opposingBC.Y * tBC - vertex.Y;
            vToBC.Z = opposingB.Z + opposingBC.Z * tBC - vertex.Z;
            vToCA.X = opposingC.X + opposingCA.X * tCA - vertex.X;
            vToCA.Y = opposingC.Y + opposingCA.Y * tCA - vertex.Y;
            vToCA.Z = opposingC.Z + opposingCA.Z * tCA - vertex.Z;
            Vector3Wide.LengthSquared(vToAB, out var abDistanceSquared);
            Vector3Wide.LengthSquared(vToBC, out var bcDistanceSquared);
            Vector3Wide.LengthSquared(vToCA, out var caDistanceSquared);

            var distanceSquared = Vector.Min(abDistanceSquared, Vector.Min(bcDistanceSquared, caDistanceSquared));
            Vector3Wide.ConditionalSelect(Vector.Equals(distanceSquared, abDistanceSquared), vToAB, vToCA, out normal);
            Vector3Wide.ConditionalSelect(Vector.Equals(distanceSquared, bcDistanceSquared), vToBC, normal, out normal);

            Vector3Wide.Scale(normal, Vector<float>.One / Vector.SquareRoot(distanceSquared), out normal);
            GetDepthForNormal(a, b, c, opposingA, opposingB, opposingC, normal, out depth);
            //Protect against bad normals.
            depth = Vector.ConditionalSelect(Vector.LessThan(distanceSquared, new Vector<float>(1e-12f)), new Vector<float>(float.MaxValue), depth);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestVertexNormal2(in Vector3Wide vertex,
        in Vector3Wide opposingA, in Vector3Wide opposingB, in Vector3Wide opposingC,
        in Vector3Wide edgePlaneNormalAB, in Vector3Wide edgePlaneNormalBC, in Vector3Wide edgePlaneNormalCA,
        in Vector3Wide triangleNormal, in Vector<float> inverseTriangleNormalLength,
        in Vector<float> inverseLengthSquaredAB, in Vector<float> inverseLengthSquaredBC, in Vector<float> inverseLengthSquaredCA)
        {
            Vector3Wide.Subtract(vertex, opposingA, out var av);
            Vector3Wide.Subtract(vertex, opposingB, out var bv);
            Vector3Wide.Subtract(vertex, opposingC, out var cv);
            Vector3Wide.Dot(av, edgePlaneNormalAB, out var avDotPlaneAB);
            Vector3Wide.Dot(bv, edgePlaneNormalBC, out var bvDotPlaneBC);
            Vector3Wide.Dot(cv, edgePlaneNormalCA, out var cvDotPlaneCA);
            //Push the vertex down to the triangle plane.
            Vector3Wide.Dot(av, triangleNormal, out var flattenDot);
            Vector3Wide vertexToTrianglePlane;
            vertexToTrianglePlane.X = flattenDot * triangleNormal.X;
            vertexToTrianglePlane.Y = flattenDot * triangleNormal.Y;
            vertexToTrianglePlane.Z = flattenDot * triangleNormal.Z;
            //Edge plane normals should have the same magnitude as the edge lengths, since they're just rotated 90 degrees. Triangle normal is perfectly perpendicular to the edges by construction.
            var tAB = avDotPlaneAB * inverseLengthSquaredAB;
            var tBC = bvDotPlaneBC * inverseLengthSquaredBC;
            var tCA = cvDotPlaneCA * inverseLengthSquaredCA;
            Vector3Wide toAB, toBC, toCA;
            toAB.X = vertexToTrianglePlane.X - edgePlaneNormalAB.X * tAB;
            toAB.Y = vertexToTrianglePlane.Y - edgePlaneNormalAB.Y * tAB;
            toAB.Z = vertexToTrianglePlane.Z - edgePlaneNormalAB.Z * tAB;
            toBC.X = vertexToTrianglePlane.X - edgePlaneNormalBC.X * tBC;
            toBC.Y = vertexToTrianglePlane.Y - edgePlaneNormalBC.Y * tBC;
            toBC.Z = vertexToTrianglePlane.Z - edgePlaneNormalBC.Z * tBC;
            toCA.X = vertexToTrianglePlane.X - edgePlaneNormalCA.X * tCA;
            toCA.Y = vertexToTrianglePlane.Y - edgePlaneNormalCA.Y * tCA;
            toCA.Z = vertexToTrianglePlane.Z - edgePlaneNormalCA.Z * tCA;

            //If the vertex is outside one edge plane, use the vertex projected onto that edge.
            //If the vertex is outside two edge planes, it is on the shared vertex.
            var outsideAB = Vector.GreaterThanOrEqual(avDotPlaneAB, Vector<float>.Zero);
            var outsideBC = Vector.GreaterThanOrEqual(bvDotPlaneBC, Vector<float>.Zero);
            var outsideCA = Vector.GreaterThanOrEqual(cvDotPlaneCA, Vector<float>.Zero);
            var outsideA = Vector.BitwiseAnd(outsideAB, outsideCA);
            var outsideB = Vector.BitwiseAnd(outsideAB, outsideBC);
            var outsideC = Vector.BitwiseAnd(outsideBC, outsideCA);

            //Vector3Wide.ConditionalSelect

            //var wa = bvDotPlaneBC * inverseTriangleNormalLength;
            //var wb = cvDotPlaneCA * inverseTriangleNormalLength;
            //var wc = Vector<float>.One - wa - wb;
            ////dot(ab x N, av) = dot(av x ab, N)
            ////barycentricCoordinate = dot(ab x N * ||ab x ca||, av) / ||ab x ca||^2 = dot(ab x N, av) / ||ab x ca|| 
            //avDotPlaneAB = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, avDotPlaneAB * inverseLengthSquaredAB));
            //bvDotPlaneBC = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, bvDotPlaneBC * inverseLengthSquaredBC));
            //cvDotPlaneCA = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, cvDotPlaneCA * inverseLengthSquaredCA));


        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestVertexNormal3(in Vector3Wide vertex,
            in Vector3Wide opposingA, in Vector3Wide opposingB, in Vector3Wide opposingC,
            in Vector3Wide opposingAB, in Vector3Wide opposingBC, in Vector3Wide opposingCA,
            in Vector3Wide opposingEdgePlaneAB, in Vector3Wide opposingEdgePlaneBC,
            in Vector<float> inverseLengthSquaredAB, in Vector<float> inverseLengthSquaredBC, in Vector<float> inverseLengthSquaredCA, in Vector<float> triangleNormalLength,
            out Vector<float> distanceSquared, out Vector3Wide offset)
        {
            //Project the vertex onto all three edges. Take the closest approach as a normal candidate.
            //Note that this will try normals that cross over the triangle's face, but that's fine- it'll just be a crappy normal candidate and another option will be chosen instead.
            Vector3Wide.Subtract(vertex, opposingA, out var av);
            Vector3Wide.Subtract(vertex, opposingB, out var bv);
            Vector3Wide.Subtract(vertex, opposingC, out var cv);
            Vector3Wide.Dot(av, opposingEdgePlaneAB, out var avDotEdgePlaneAB);
            Vector3Wide.Dot(bv, opposingEdgePlaneBC, out var bvDotEdgePlaneBC);
            //Because of the equivalence between the edge plane tests and barycentric coordinates, we can compute the last edge plane dot implicitly:
            var cvDotEdgePlaneCA = triangleNormalLength - avDotEdgePlaneAB - bvDotEdgePlaneBC;
            var outsideTriangle = Vector.BitwiseOr(Vector.GreaterThan(avDotEdgePlaneAB, Vector<float>.Zero), Vector.BitwiseOr(Vector.GreaterThan(bvDotEdgePlaneBC, Vector<float>.Zero), Vector.GreaterThan(cvDotEdgePlaneCA, Vector<float>.Zero)));
            Vector3Wide.Dot(av, opposingAB, out var avDotAB);
            Vector3Wide.Dot(bv, opposingBC, out var bvDotBC);
            Vector3Wide.Dot(cv, opposingCA, out var cvDotCA);
            var tAB = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, avDotAB * inverseLengthSquaredAB));
            var tBC = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, bvDotBC * inverseLengthSquaredBC));
            var tCA = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, cvDotCA * inverseLengthSquaredCA));
            Vector3Wide vToAB, vToBC, vToCA;
            vToAB.X = opposingA.X + opposingAB.X * tAB - vertex.X;
            vToAB.Y = opposingA.Y + opposingAB.Y * tAB - vertex.Y;
            vToAB.Z = opposingA.Z + opposingAB.Z * tAB - vertex.Z;
            vToBC.X = opposingB.X + opposingBC.X * tBC - vertex.X;
            vToBC.Y = opposingB.Y + opposingBC.Y * tBC - vertex.Y;
            vToBC.Z = opposingB.Z + opposingBC.Z * tBC - vertex.Z;
            vToCA.X = opposingC.X + opposingCA.X * tCA - vertex.X;
            vToCA.Y = opposingC.Y + opposingCA.Y * tCA - vertex.Y;
            vToCA.Z = opposingC.Z + opposingCA.Z * tCA - vertex.Z;
            Vector3Wide.LengthSquared(vToAB, out var abDistanceSquared);
            Vector3Wide.LengthSquared(vToBC, out var bcDistanceSquared);
            Vector3Wide.LengthSquared(vToCA, out var caDistanceSquared);

            distanceSquared = Vector.Min(abDistanceSquared, Vector.Min(bcDistanceSquared, caDistanceSquared));
            Vector3Wide.ConditionalSelect(Vector.Equals(distanceSquared, abDistanceSquared), vToAB, vToCA, out offset);
            Vector3Wide.ConditionalSelect(Vector.Equals(distanceSquared, bcDistanceSquared), vToBC, offset, out offset);

            //Ignore this vertex if it's contained within the triangle. We'll detect distances that are too close to generate normals outside.
            distanceSquared = Vector.ConditionalSelect(outsideTriangle, distanceSquared, new Vector<float>(float.MaxValue));
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
        private static void TryAddTriangleAVertex(in Vector3Wide vertex, in Vector2Wide flattenedVertex, in Vector<int> vertexId,
            in Vector3Wide tangentBX, in Vector3Wide tangentBY, in Vector3Wide triangleCenterB, in Vector3Wide contactNormal, in Vector3Wide faceNormalB,
            in Vector2Wide edgeAB, in Vector2Wide edgeBC, in Vector2Wide edgeCA, in Vector2Wide bA, in Vector2Wide bB,
            in Vector<int> allowContacts, in Vector<float> inverseContactNormalDotFaceNormalB, in Vector<float> minimumDepth,
            ref ManifoldCandidate candidates, ref Vector<int> candidateCount, int pairCount)
        {
            //Test edge edge plane sign for all three edges of B. We can test the vertex directly rather than the unprojected vertex because the ray cast follows the contact normal,
            //and all of these plane normals are perpendicular to the contact normal.
            Vector2Wide.Subtract(flattenedVertex, bA, out var bAToVertex);
            Vector2Wide.Subtract(flattenedVertex, bB, out var bBToVertex);
            var abEdgePlaneDot = bAToVertex.Y * edgeAB.X - bAToVertex.X * edgeAB.Y;
            var bcEdgePlaneDot = bBToVertex.Y * edgeBC.X - bBToVertex.X * edgeBC.Y;
            var caEdgePlaneDot = bAToVertex.Y * edgeCA.X - bAToVertex.X * edgeCA.Y;
            var abContained = Vector.GreaterThan(abEdgePlaneDot, Vector<float>.Zero);
            var bcContained = Vector.GreaterThan(bcEdgePlaneDot, Vector<float>.Zero);
            var caContained = Vector.GreaterThan(caEdgePlaneDot, Vector<float>.Zero);
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
            Unsafe.SkipInit(out ManifoldCandidate candidate);
            candidate.Depth = distance * inverseContactNormalDotFaceNormalB;
            Vector3Wide.Scale(contactNormal, candidate.Depth, out var unprojectedVertex);
            Vector3Wide.Add(unprojectedVertex, vertex, out unprojectedVertex);

            Vector3Wide.Subtract(unprojectedVertex, triangleCenterB, out var offsetOnB);
            Vector3Wide.Dot(offsetOnB, tangentBX, out candidate.X);
            Vector3Wide.Dot(offsetOnB, tangentBY, out candidate.Y);
            candidate.FeatureId = vertexId;
            ManifoldCandidateHelper.AddCandidateWithDepth(ref candidates, ref candidateCount, candidate, Vector.BitwiseAnd(Vector.GreaterThanOrEqual(candidate.Depth, minimumDepth), Vector.BitwiseAnd(allowContacts, contained)), pairCount);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ClipEdge(
            in Vector2Wide edgeStartB, in Vector2Wide edgeOffsetB,
            in Vector2Wide edgeStartA, in Vector2Wide edgeOffsetA, in Vector<float> inverseEdgeLengthSquaredA, in Vector<float> edgeStartADotNormal, in Vector<float> edgeOffsetADotNormal,
            out Vector<int> intersectionExists, out Vector<float> tB, out Vector<float> depthContributionA)
        {
            //The edge plane normal points toward the inside of the bounding triangle.
            //intersection = dot(planeNormal, pointOnPlane - edgeStart) / dot(planeNormal, edgeDirectionB)
            var edgePlaneNormalDot = (edgeStartA.X - edgeStartB.X) * edgeOffsetA.Y - (edgeStartA.Y - edgeStartB.Y) * edgeOffsetA.X;
            var velocity = edgeOffsetB.X * edgeOffsetA.Y - edgeOffsetB.Y * edgeOffsetA.X;
            var parallelThreshold = new Vector<float>(1e-20f);
            var parallel = Vector.LessThan(Vector.Abs(velocity), parallelThreshold);
            var denominator = Vector.ConditionalSelect(parallel, Vector.ConditionalSelect(Vector.LessThan(velocity, Vector<float>.Zero), -parallelThreshold, parallelThreshold), velocity);
            tB = edgePlaneNormalDot / denominator;
            //To be valid, an intersection must be within both edge bounds.
            var intersectionPointX = tB * edgeOffsetB.X + edgeStartB.X;
            var intersectionPointY = tB * edgeOffsetB.Y + edgeStartB.Y;
            var tA = ((intersectionPointX - edgeStartA.X) * edgeOffsetA.X + (intersectionPointY - edgeStartA.Y) * edgeOffsetA.Y) * inverseEdgeLengthSquaredA;
            intersectionExists = Vector.BitwiseAnd(Vector.GreaterThanOrEqual(tA, Vector<float>.Zero), Vector.LessThanOrEqual(tA, Vector<float>.One));
            depthContributionA = edgeStartADotNormal + edgeOffsetADotNormal * tA;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ClipBEdgeAgainstABounds( //maybe we should have.. MORE parameters
             in Vector2Wide aA, in Vector2Wide aB, in Vector2Wide aC,
             in Vector2Wide edgeOffsetABOnA, in Vector2Wide edgeOffsetBCOnA, in Vector2Wide edgeOffsetCAOnA,
             in Vector<float> inverseEdgeOffsetABOnALengthSquared, in Vector<float> inverseEdgeOffsetBCOnALengthSquared, in Vector<float> inverseEdgeOffsetCAOnALengthSquared,
             in Vector<float> aDotNormalOnA, in Vector<float> bDotNormalOnA, in Vector<float> cDotNormalOnA,
             in Vector<float> abDotNormalOnA, in Vector<float> bcDotNormalOnA, in Vector<float> caDotNormalOnA,
             in Vector2Wide flatEdgeStartB, in Vector2Wide flatEdgeOffsetB, in Vector3Wide edgeStartB, in Vector3Wide edgeOffsetB,
             in Vector<int> entryId, in Vector<int> exitIdOffset,
             in Vector3Wide triangleCenterB, in Vector3Wide tangentBX, in Vector3Wide tangentBY,
             in Vector3Wide localNormal, in Vector<float> minimumDepth, Vector<int> allowContacts, ref ManifoldCandidate candidates, ref Vector<int> candidateCount, int pairCount)
        {
            //The base id is the id of the vertex in the corner along the negative boxEdgeDirection and boxEdgeCenterOffsetDirection.
            //The edgeDirectionId is the amount to add when you move along the boxEdgeDirection to the other vertex.
            //The edgeCenterOffsetId is the amount to add when you move along the boxEdgeCenterOffsetDirection to the other vertex.
            //We have three edge planes created by the edges of triangle A.
            //We want to test the triangle B edge against all three of the edges.
            ClipEdge(flatEdgeStartB, flatEdgeOffsetB, aA, edgeOffsetABOnA, inverseEdgeOffsetABOnALengthSquared, aDotNormalOnA, abDotNormalOnA, out var abIntersected, out var tAB, out var depthContributionABOnA);
            ClipEdge(flatEdgeStartB, flatEdgeOffsetB, aB, edgeOffsetBCOnA, inverseEdgeOffsetBCOnALengthSquared, bDotNormalOnA, bcDotNormalOnA, out var bcIntersected, out var tBC, out var depthContributionBCOnA);
            ClipEdge(flatEdgeStartB, flatEdgeOffsetB, aC, edgeOffsetCAOnA, inverseEdgeOffsetCAOnALengthSquared, cDotNormalOnA, caDotNormalOnA, out var caIntersected, out var tCA, out var depthContributionCAOnA);
            var minValue = new Vector<float>(float.MinValue);
            var maxValue = new Vector<float>(float.MaxValue);
            var entryAB = Vector.ConditionalSelect(abIntersected, tAB, maxValue);
            var entryBC = Vector.ConditionalSelect(bcIntersected, tBC, maxValue);
            var entryCA = Vector.ConditionalSelect(caIntersected, tCA, maxValue);
            var exitAB = Vector.ConditionalSelect(abIntersected, tAB, minValue);
            var exitBC = Vector.ConditionalSelect(bcIntersected, tBC, minValue);
            var exitCA = Vector.ConditionalSelect(caIntersected, tCA, minValue);
            var entry = Vector.Min(entryAB, Vector.Min(entryBC, entryCA));
            var exit = Vector.Max(exitAB, Vector.Max(exitBC, exitCA));
            var useABAsEntry = Vector.Equals(entry, tAB);
            var useBCAsEntry = Vector.Equals(entry, tBC);
            //var useCAAsEntry = Vector.Equals(entry, tCA);
            var useABAsExit = Vector.Equals(exit, tAB);
            var useBCAsExit = Vector.Equals(exit, tBC);
            //var useCAAsExit = Vector.Equals(exit, tCA);
            var depthContributionAAtEntry = Vector.ConditionalSelect(useABAsEntry, depthContributionABOnA, Vector.ConditionalSelect(useBCAsEntry, depthContributionBCOnA, depthContributionCAOnA));
            var depthContributionAAtExit = Vector.ConditionalSelect(useABAsExit, depthContributionABOnA, Vector.ConditionalSelect(useBCAsExit, depthContributionBCOnA, depthContributionCAOnA));
            //If an edge fails to generate any interval, then it's not intersecting the triangle bounds and should not generate contacts.
            allowContacts = Vector.AndNot(allowContacts, Vector.BitwiseOr(Vector.Equals(entry, minValue), Vector.Equals(exit, maxValue)));
            entry = Vector.Max(Vector<float>.Zero, entry);
            exit = Vector.Min(Vector<float>.One, exit);

            Vector3Wide.Dot(edgeStartB, localNormal, out var edgeStartBDotNormal);
            Vector3Wide.Dot(edgeOffsetB, localNormal, out var edgeOffsetBDotNormal);
            var depthContributionBAtEntry = edgeStartBDotNormal + entry * edgeOffsetBDotNormal;
            var depthContributionBAtExit = edgeStartBDotNormal + exit * edgeOffsetBDotNormal;

            //entryX = dot(entry * edgeDirectionA + edgeStartA - triangleCenterB, tangentBX)
            //entryY = dot(entry * edgeDirectionA + edgeStartA - triangleCenterB, tangentBY)
            //exitX = dot(exit * edgeDirectionA + edgeStartA - triangleCenterB, tangentBX)
            //exitY = dot(exit * edgeDirectionA + edgeStartA - triangleCenterB, tangentBY)
            Vector3Wide.Subtract(edgeStartB, triangleCenterB, out var offset);
            Vector3Wide.Dot(offset, tangentBX, out var offsetX);
            Vector3Wide.Dot(offset, tangentBY, out var offsetY);
            Vector3Wide.Dot(tangentBX, edgeOffsetB, out var edgeDirectionX);
            Vector3Wide.Dot(tangentBY, edgeOffsetB, out var edgeDirectionY);

            Unsafe.SkipInit(out ManifoldCandidate candidate);
            var six = new Vector<int>(6);
            //Entry
            candidate.Depth = depthContributionBAtEntry - depthContributionAAtEntry;
            var exists = Vector.BitwiseAnd(Vector.BitwiseAnd(allowContacts, Vector.GreaterThanOrEqual(candidate.Depth, minimumDepth)), Vector.BitwiseAnd(
                Vector.BitwiseAnd(
                    Vector.LessThan(candidateCount, six),
                    Vector.GreaterThanOrEqual(exit - entry, new Vector<float>(1e-5f))), //note fixed threshold; the exit and entry values are in terms of the edge's length already. 
                Vector.BitwiseAnd(
                    Vector.LessThan(entry, Vector<float>.One),
                    Vector.GreaterThan(entry, Vector<float>.Zero))));
            candidate.X = entry * edgeDirectionX + offsetX;
            candidate.Y = entry * edgeDirectionY + offsetY;
            candidate.FeatureId = entryId;
            ManifoldCandidateHelper.AddCandidateWithDepth(ref candidates, ref candidateCount, candidate, exists, pairCount);
            //Exit
            candidate.Depth = depthContributionBAtExit - depthContributionAAtExit;
            exists = Vector.BitwiseAnd(Vector.BitwiseAnd(allowContacts, Vector.GreaterThanOrEqual(candidate.Depth, minimumDepth)), Vector.BitwiseAnd(
                Vector.BitwiseAnd(
                    Vector.LessThan(candidateCount, six),
                    Vector.GreaterThanOrEqual(exit, entry)),
                Vector.BitwiseAnd(
                    Vector.LessThanOrEqual(exit, Vector<float>.One),
                    Vector.GreaterThanOrEqual(exit, Vector<float>.Zero))));
            candidate.X = exit * edgeDirectionX + offsetX;
            candidate.Y = exit * edgeDirectionY + offsetY;
            candidate.FeatureId = entryId + exitIdOffset;
            ManifoldCandidateHelper.AddCandidateWithDepth(ref candidates, ref candidateCount, candidate, exists, pairCount);
        }



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Test(
            ref TriangleWide a, ref TriangleWide b, ref Vector<float> speculativeMargin,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount,
            out Convex4ContactManifoldWide manifold)
        {
            Unsafe.SkipInit(out manifold);
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

            ManifoldCandidateHelper.CreateActiveMask(pairCount, out var allowContacts);

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


            var tryVertexNormals = Vector.BitwiseAnd(Vector.LessThan(depth, Vector<float>.Zero), allowContacts);
            Vector3Wide.LengthSquared(abA, out var abALengthSquared);
            Vector3Wide.LengthSquared(abB, out var abBLengthSquared);
            Vector3Wide.LengthSquared(caA, out var caALengthSquared);
            Vector3Wide.LengthSquared(caB, out var caBLengthSquared);
            if (Vector.LessThanAny(tryVertexNormals, Vector<int>.Zero))
            {
                //Vertex normals are not required to determine penetration versus separation. They are only used to ensure correct separated speculative normals.
                //This isn't strictly required for behavior in the general case, but MeshReduction depends on it.
                Vector3Wide.LengthSquared(bcA, out var bcALengthSquared);
                Vector3Wide.LengthSquared(bcB, out var bcBLengthSquared);
                var inverseABALengthSquared = Vector<float>.One / abALengthSquared;
                var inverseBCALengthSquared = Vector<float>.One / bcALengthSquared;
                var inverseCAALengthSquared = Vector<float>.One / caALengthSquared;
                var inverseABBLengthSquared = Vector<float>.One / abBLengthSquared;
                var inverseBCBLengthSquared = Vector<float>.One / bcBLengthSquared;
                var inverseCABLengthSquared = Vector<float>.One / caBLengthSquared;
                Vector3Wide.Cross(abA, faceNormalA, out var edgeNormalABOnA);
                Vector3Wide.Cross(bcA, faceNormalA, out var edgeNormalBCOnA);
                Vector3Wide.Cross(abB, faceNormalB, out var edgeNormalABOnB);
                Vector3Wide.Cross(bcB, faceNormalB, out var edgeNormalBCOnB);
                TestVertexNormal3(a.A, bA, bB, bC, abB, bcB, caB, edgeNormalABOnB, edgeNormalBCOnB, inverseABBLengthSquared, inverseBCBLengthSquared, inverseCABLengthSquared, faceNormalBLength, out var distanceSquared, out var offset);
                Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
                TestVertexNormal3(a.B, bA, bB, bC, abB, bcB, caB, edgeNormalABOnB, edgeNormalBCOnB, inverseABBLengthSquared, inverseBCBLengthSquared, inverseCABLengthSquared, faceNormalBLength, out var distanceSquaredCandidate, out var offsetCandidate);
                Select(ref distanceSquared, ref offset, distanceSquaredCandidate, offsetCandidate);
                TestVertexNormal3(a.C, bA, bB, bC, abB, bcB, caB, edgeNormalABOnB, edgeNormalBCOnB, inverseABBLengthSquared, inverseBCBLengthSquared, inverseCABLengthSquared, faceNormalBLength, out distanceSquaredCandidate, out offsetCandidate);
                Select(ref distanceSquared, ref offset, distanceSquaredCandidate, offsetCandidate);
                TestVertexNormal3(bA, a.A, a.B, a.C, abA, bcA, caA, edgeNormalABOnA, edgeNormalBCOnA, inverseABALengthSquared, inverseBCALengthSquared, inverseCAALengthSquared, faceNormalALength, out distanceSquaredCandidate, out offsetCandidate);
                Select(ref distanceSquared, ref offset, distanceSquaredCandidate, offsetCandidate);
                TestVertexNormal3(bB, a.A, a.B, a.C, abA, bcA, caA, edgeNormalABOnA, edgeNormalBCOnA, inverseABALengthSquared, inverseBCALengthSquared, inverseCAALengthSquared, faceNormalALength, out distanceSquaredCandidate, out offsetCandidate);
                Select(ref distanceSquared, ref offset, distanceSquaredCandidate, offsetCandidate);
                TestVertexNormal3(bC, a.A, a.B, a.C, abA, bcA, caA, edgeNormalABOnA, edgeNormalBCOnA, inverseABALengthSquared, inverseBCALengthSquared, inverseCAALengthSquared, faceNormalALength, out distanceSquaredCandidate, out offsetCandidate);
                Select(ref distanceSquared, ref offset, distanceSquaredCandidate, offsetCandidate);

                var distance = Vector.SquareRoot(distanceSquared);
                Vector3Wide.Scale(offset, Vector<float>.One / distance, out localNormalCandidate);
                //Don't try to use distances that are so small that the resulting normal will be numerically bad. That's close enough to intersecting that the previous normals will handle it.
                GetDepthForNormal(a.A, a.B, a.C, bA, bB, bC, localNormalCandidate, out depthCandidate);
                depthCandidate = Vector.ConditionalSelect(Vector.GreaterThan(distance, new Vector<float>(1e-7f)), depthCandidate, new Vector<float>(float.MaxValue));
                Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            }

            ////A AB x *
            //TestEdgeEdge(abA, abB, a.A, a.B, a.C, bA, bB, bC, out var depth, out var localNormal);
            //TestEdgeEdge(abA, bcB, a.A, a.B, a.C, bA, bB, bC, out var depthCandidate, out var localNormalCandidate);
            //Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            //TestEdgeEdge(abA, caB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            //Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);

            ////A BC x *
            //TestEdgeEdge(bcA, abB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            //Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            //TestEdgeEdge(bcA, bcB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            //Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            //TestEdgeEdge(bcA, caB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            //Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);

            ////A CA x *
            //TestEdgeEdge(caA, abB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            //Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            //TestEdgeEdge(caA, bcB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            //Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            //TestEdgeEdge(caA, caB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            //Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);

            ////Face normals
            //Vector3Wide.CrossWithoutOverlap(abA, caA, out var faceNormalA);
            //Vector3Wide.Length(faceNormalA, out var faceNormalALength);
            //Vector3Wide.Scale(faceNormalA, Vector<float>.One / faceNormalALength, out faceNormalA);
            //GetDepthForNormal(a.A, a.B, a.C, bA, bB, bC, faceNormalA, out depthCandidate);
            //Select(ref depth, ref localNormal, depthCandidate, faceNormalA);
            //Vector3Wide.CrossWithoutOverlap(abB, caB, out var faceNormalB);
            //Vector3Wide.Length(faceNormalB, out var faceNormalBLength);
            //Vector3Wide.Scale(faceNormalB, Vector<float>.One / faceNormalBLength, out faceNormalB);
            //GetDepthForNormal(a.A, a.B, a.C, bA, bB, bC, faceNormalB, out var faceDepthB);
            //Select(ref depth, ref localNormal, faceDepthB, faceNormalB);


            //var tryVertexNormals = Vector.BitwiseAnd(Vector.LessThan(depth, Vector<float>.Zero), allowContacts);
            //Vector3Wide.LengthSquared(abA, out var abALengthSquared);
            //Vector3Wide.LengthSquared(abB, out var abBLengthSquared);
            //Vector3Wide.LengthSquared(caA, out var caALengthSquared);
            //Vector3Wide.LengthSquared(caB, out var caBLengthSquared);
            //if (Vector.LessThanAny(tryVertexNormals, Vector<int>.Zero))
            //{
            //    //Vertex normals are not required to determine penetration versus separation. They are only used to ensure correct separated speculative normals.
            //    //This isn't strictly required for behavior in the general case, but MeshReduction depends on it.
            //    Vector3Wide.LengthSquared(bcA, out var bcALengthSquared);
            //    Vector3Wide.LengthSquared(bcB, out var bcBLengthSquared);
            //    var inverseABALengthSquared = Vector<float>.One / abALengthSquared;
            //    var inverseBCALengthSquared = Vector<float>.One / bcALengthSquared;
            //    var inverseCAALengthSquared = Vector<float>.One / caALengthSquared;
            //    var inverseABBLengthSquared = Vector<float>.One / abBLengthSquared;
            //    var inverseBCBLengthSquared = Vector<float>.One / bcBLengthSquared;
            //    var inverseCABLengthSquared = Vector<float>.One / caBLengthSquared;
            //    TestVertexNormal(a.A, a.A, a.B, a.C, bA, bB, bC, abB, bcB, caB, inverseABBLengthSquared, inverseBCBLengthSquared, inverseCABLengthSquared, out depthCandidate, out localNormalCandidate);
            //    Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            //    TestVertexNormal(a.B, a.A, a.B, a.C, bA, bB, bC, abB, bcB, caB, inverseABBLengthSquared, inverseBCBLengthSquared, inverseCABLengthSquared, out depthCandidate, out localNormalCandidate);
            //    Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            //    TestVertexNormal(a.C, a.A, a.B, a.C, bA, bB, bC, abB, bcB, caB, inverseABBLengthSquared, inverseBCBLengthSquared, inverseCABLengthSquared, out depthCandidate, out localNormalCandidate);
            //    Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            //    TestVertexNormal(bA, bA, bB, bC, a.A, a.B, a.C, abA, bcA, caA, inverseABALengthSquared, inverseBCALengthSquared, inverseCAALengthSquared, out depthCandidate, out localNormalCandidate);
            //    Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            //    TestVertexNormal(bB, bA, bB, bC, a.A, a.B, a.C, abA, bcA, caA, inverseABALengthSquared, inverseBCALengthSquared, inverseCAALengthSquared, out depthCandidate, out localNormalCandidate);
            //    Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            //    TestVertexNormal(bC, bA, bB, bC, a.A, a.B, a.C, abA, bcA, caA, inverseABALengthSquared, inverseBCALengthSquared, inverseCAALengthSquared, out depthCandidate, out localNormalCandidate);
            //    Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            //}

            //Vector3Wide.LengthSquared(abA, out var abALengthSquared);
            //Vector3Wide.LengthSquared(abB, out var abBLengthSquared);
            //Vector3Wide.LengthSquared(caA, out var caALengthSquared);
            //Vector3Wide.LengthSquared(caB, out var caBLengthSquared);
            //Vector3Wide.LengthSquared(bcA, out var bcALengthSquared);
            //Vector3Wide.LengthSquared(bcB, out var bcBLengthSquared);
            //var inverseABALengthSquared = Vector<float>.One / abALengthSquared;
            //var inverseBCALengthSquared = Vector<float>.One / bcALengthSquared;
            //var inverseCAALengthSquared = Vector<float>.One / caALengthSquared;
            //var inverseABBLengthSquared = Vector<float>.One / abBLengthSquared;
            //var inverseBCBLengthSquared = Vector<float>.One / bcBLengthSquared;
            //var inverseCABLengthSquared = Vector<float>.One / caBLengthSquared;
            ////A AB x *
            //TestEdgeEdge2(a.A, abA, abALengthSquared, inverseABALengthSquared, bA, abB, abBLengthSquared, inverseABBLengthSquared, a.A, a.B, a.C, bA, bB, bC, out var depth, out var localNormal);
            //TestEdgeEdge2(a.A, abA, abALengthSquared, inverseABALengthSquared, bB, bcB, bcBLengthSquared, inverseBCBLengthSquared, a.A, a.B, a.C, bA, bB, bC, out var depthCandidate, out var localNormalCandidate);
            //Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            //TestEdgeEdge2(a.A, abA, abALengthSquared, inverseABALengthSquared, bC, caB, caBLengthSquared, inverseCABLengthSquared, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            //Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);

            ////A BC x *
            //TestEdgeEdge2(a.B, bcA, bcALengthSquared, inverseBCALengthSquared, bA, abB, abBLengthSquared, inverseABBLengthSquared, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            //Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            //TestEdgeEdge2(a.B, bcA, bcALengthSquared, inverseBCALengthSquared, bB, bcB, bcBLengthSquared, inverseBCBLengthSquared, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            //Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            //TestEdgeEdge2(a.B, bcA, bcALengthSquared, inverseBCALengthSquared, bC, caB, caBLengthSquared, inverseCABLengthSquared, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            //Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);

            ////A CA x *
            //TestEdgeEdge2(a.C, caA, caALengthSquared, inverseCAALengthSquared, bA, abB, abBLengthSquared, inverseABBLengthSquared, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            //Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            //TestEdgeEdge2(a.C, caA, caALengthSquared, inverseCAALengthSquared, bB, bcB, bcBLengthSquared, inverseBCBLengthSquared, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            //Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            //TestEdgeEdge2(a.C, caA, caALengthSquared, inverseCAALengthSquared, bC, caB, caBLengthSquared, inverseCABLengthSquared, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            //Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);

            ////Face normals
            //Vector3Wide.CrossWithoutOverlap(abA, caA, out var faceNormalA);
            //Vector3Wide.Length(faceNormalA, out var faceNormalALength);
            //Vector3Wide.Scale(faceNormalA, Vector<float>.One / faceNormalALength, out faceNormalA);
            //GetDepthForNormal(a.A, a.B, a.C, bA, bB, bC, faceNormalA, out depthCandidate);
            //Select(ref depth, ref localNormal, depthCandidate, faceNormalA);
            //Vector3Wide.CrossWithoutOverlap(abB, caB, out var faceNormalB);
            //Vector3Wide.Length(faceNormalB, out var faceNormalBLength);
            //Vector3Wide.Scale(faceNormalB, Vector<float>.One / faceNormalBLength, out faceNormalB);
            //GetDepthForNormal(a.A, a.B, a.C, bA, bB, bC, faceNormalB, out var faceDepthB);
            //Select(ref depth, ref localNormal, faceDepthB, faceNormalB);

            //Point the normal from B to A by convention.
            Vector3Wide.Subtract(localTriangleCenterB, localTriangleCenterA, out var centerAToCenterB);
            Vector3Wide.Dot(localNormal, centerAToCenterB, out var calibrationDot);
            var shouldFlip = Vector.GreaterThan(calibrationDot, Vector<float>.Zero);
            localNormal.X = Vector.ConditionalSelect(shouldFlip, -localNormal.X, localNormal.X);
            localNormal.Y = Vector.ConditionalSelect(shouldFlip, -localNormal.Y, localNormal.Y);
            localNormal.Z = Vector.ConditionalSelect(shouldFlip, -localNormal.Z, localNormal.Z);

            Vector3Wide.Dot(localNormal, faceNormalA, out var localNormalDotFaceNormalA);
            Vector3Wide.Dot(localNormal, faceNormalB, out var localNormalDotFaceNormalB);
            allowContacts = Vector.BitwiseAnd(allowContacts, Vector.BitwiseAnd(
                Vector.LessThan(localNormalDotFaceNormalA, new Vector<float>(-SphereTriangleTester.BackfaceNormalDotRejectionThreshold)),
                Vector.GreaterThan(localNormalDotFaceNormalB, new Vector<float>(SphereTriangleTester.BackfaceNormalDotRejectionThreshold))));
            if (Vector.EqualsAll(allowContacts, Vector<int>.Zero))
            {
                manifold.Contact0Exists = default;
                manifold.Contact1Exists = default;
                manifold.Contact2Exists = default;
                manifold.Contact3Exists = default;
                return;
            }

            //At this point, we have computed the minimum depth and associated local normal.
            //We now need to compute some contact locations, their per-contact depths, and the feature ids.

            //Flatten both triangles onto a plane with normal equal to the detected local normal to perform clipping.
            //Note that the final result does not depend on the orientation of the flattening basis, so we can just use the orthonormal builder.
            Helpers.BuildOrthonormalBasis(localNormal, out var flattenX, out var flattenY);
            Vector2Wide flatVertexAOnA, flatVertexBOnA, flatVertexCOnA;
            Vector2Wide flatVertexAOnB, flatVertexBOnB, flatVertexCOnB;
            Vector3Wide.Dot(a.A, flattenX, out flatVertexAOnA.X);
            Vector3Wide.Dot(a.A, flattenY, out flatVertexAOnA.Y);
            Vector3Wide.Dot(a.B, flattenX, out flatVertexBOnA.X);
            Vector3Wide.Dot(a.B, flattenY, out flatVertexBOnA.Y);
            Vector3Wide.Dot(a.C, flattenX, out flatVertexCOnA.X);
            Vector3Wide.Dot(a.C, flattenY, out flatVertexCOnA.Y);
            Vector3Wide.Dot(bA, flattenX, out flatVertexAOnB.X);
            Vector3Wide.Dot(bA, flattenY, out flatVertexAOnB.Y);
            Vector3Wide.Dot(bB, flattenX, out flatVertexBOnB.X);
            Vector3Wide.Dot(bB, flattenY, out flatVertexBOnB.Y);
            Vector3Wide.Dot(bC, flattenX, out flatVertexCOnB.X);
            Vector3Wide.Dot(bC, flattenY, out flatVertexCOnB.Y);

            Vector2Wide.Subtract(flatVertexBOnA, flatVertexAOnA, out var flatEdgeABOnA);
            Vector2Wide.Subtract(flatVertexCOnA, flatVertexBOnA, out var flatEdgeBCOnA);
            Vector2Wide.Subtract(flatVertexAOnA, flatVertexCOnA, out var flatEdgeCAOnA);
            Vector2Wide.Subtract(flatVertexBOnB, flatVertexAOnB, out var flatEdgeABOnB);
            Vector2Wide.Subtract(flatVertexCOnB, flatVertexBOnB, out var flatEdgeBCOnB);
            Vector2Wide.Subtract(flatVertexAOnB, flatVertexCOnB, out var flatEdgeCAOnB);

            var edgeThreshold = new Vector<float>(0.2f);
            var useEdgeCaseForA = Vector.LessThan(Vector.Abs(localNormalDotFaceNormalA), edgeThreshold);
            var useEdgeCaseForB = Vector.LessThan(Vector.Abs(localNormalDotFaceNormalB), edgeThreshold);
            var useFaceCaseForA = Vector.OnesComplement(useEdgeCaseForA);
            var useFaceCaseForB = Vector.OnesComplement(useEdgeCaseForB);
            useEdgeCaseForA = Vector.BitwiseAnd(allowContacts, useEdgeCaseForA);
            useEdgeCaseForB = Vector.BitwiseAnd(allowContacts, useEdgeCaseForB);
            useFaceCaseForA = Vector.BitwiseAnd(allowContacts, useFaceCaseForA);
            useFaceCaseForB = Vector.BitwiseAnd(allowContacts, useFaceCaseForB);

            //We will be working on the surface of triangleB, but we'd still like a 2d parameterization of the surface for contact reduction.
            //So, we'll create tangent axes from the edge and edge x normal.
            //Vector3Wide.LengthSquared(abB, out var abBLengthSquared);
            Vector3Wide.Scale(abB, Vector<float>.One / Vector.SquareRoot(abBLengthSquared), out var tangentBX);
            Vector3Wide.CrossWithoutOverlap(tangentBX, faceNormalB, out var tangentBY);

            //Note that we only allocate up to 6 candidates. Each triangle edge can contribute at most two contacts (any more would require a nonconvex clip region).
            //Numerical issues can cause more to occur, but they're guarded against (both directly, and in the sense of checking count before adding any candidates beyond the sixth).
            var buffer = stackalloc ManifoldCandidate[6];
            var candidateCount = Vector<int>.Zero;
            ref var candidates = ref *buffer;

            var minimumDepth = -speculativeMargin;
            if (Vector.LessThanAny(useFaceCaseForB, Vector<int>.Zero))
            {
                //While the edge clipping will find any edge-edge or bVertex-aFace contacts, it will not find aVertex-bFace contacts.
                //Add them independently.
                //(Adding these first allows us to simply skip capacity tests, since there can only be a total of three bVertex-aFace contacts.)
                //Note that division by zero is protected by useFaceCaseForB. These contacts aren't relevant for the edge case anyway.
                var inverseContactNormalDotFaceNormalB = Vector<float>.One / localNormalDotFaceNormalB;
                TryAddTriangleAVertex(a.A, flatVertexAOnA, Vector<int>.Zero, tangentBX, tangentBY, localTriangleCenterB, localNormal, faceNormalB, flatEdgeABOnB, flatEdgeBCOnB, flatEdgeCAOnB, flatVertexAOnB, flatVertexBOnB, useFaceCaseForB, inverseContactNormalDotFaceNormalB, minimumDepth, ref candidates, ref candidateCount, pairCount);
                TryAddTriangleAVertex(a.B, flatVertexBOnA, Vector<int>.One, tangentBX, tangentBY, localTriangleCenterB, localNormal, faceNormalB, flatEdgeABOnB, flatEdgeBCOnB, flatEdgeCAOnB, flatVertexAOnB, flatVertexBOnB, useFaceCaseForB, inverseContactNormalDotFaceNormalB, minimumDepth, ref candidates, ref candidateCount, pairCount);
                TryAddTriangleAVertex(a.C, flatVertexCOnA, new Vector<int>(2), tangentBX, tangentBY, localTriangleCenterB, localNormal, faceNormalB, flatEdgeABOnB, flatEdgeBCOnB, flatEdgeCAOnB, flatVertexAOnB, flatVertexBOnB, useFaceCaseForB, inverseContactNormalDotFaceNormalB, minimumDepth, ref candidates, ref candidateCount, pairCount);
            }
            //Note that edge cases will also add triangle B vertices that are within triangle A's bounds, so no B vertex case is required.
            var three = new Vector<int>(3);
            //Note the use of localNormal here, NOT faceNormalA. Why? Just like in the vertexA case, we're not creating contacts in triangle A's face voronoi region.
            //Instead, the test region is skewed along the contact normal. These planes intersect A's edges and have the contact normal as a tangent.
            //This avoids dependency on pair order (consider what happens when A and B swap).
            var stillCouldUseClippingContacts = Vector.BitwiseAnd(allowContacts, Vector.LessThan(candidateCount, three));
            if (Vector.LessThanAny(stillCouldUseClippingContacts, Vector<int>.Zero))
            {
                //At least one lane may need edge clipped contacts.
                Vector2Wide.LengthSquared(flatEdgeABOnA, out var flatEdgeOffsetABOnALengthSquared);
                Vector2Wide.LengthSquared(flatEdgeBCOnA, out var flatEdgeOffsetBCOnALengthSquared);
                Vector2Wide.LengthSquared(flatEdgeCAOnA, out var flatEdgeOffsetCAOnALengthSquared);
                var inverseFlatEdgeOffsetABOnALengthSquared = Vector<float>.One / flatEdgeOffsetABOnALengthSquared;
                var inverseFlatEdgeOffsetBCOnALengthSquared = Vector<float>.One / flatEdgeOffsetBCOnALengthSquared;
                var inverseFlatEdgeOffsetCAOnALengthSquared = Vector<float>.One / flatEdgeOffsetCAOnALengthSquared;
                //These clipping routines compute depth directly, rather than relying on reduction to compute it for us (thanks, triangles).
                //We can precompute the depth contribution for A's edges.
                Vector3Wide.Dot(localNormal, a.A, out var aDotNormalOnA);
                Vector3Wide.Dot(localNormal, a.B, out var bDotNormalOnA);
                Vector3Wide.Dot(localNormal, a.C, out var cDotNormalOnA);
                Vector3Wide.Dot(localNormal, abA, out var abDotNormalOnA);
                Vector3Wide.Dot(localNormal, bcA, out var bcDotNormalOnA);
                Vector3Wide.Dot(localNormal, caA, out var caDotNormalOnA);
                ClipBEdgeAgainstABounds(flatVertexAOnA, flatVertexBOnA, flatVertexCOnA, flatEdgeABOnA, flatEdgeBCOnA, flatEdgeCAOnA, inverseFlatEdgeOffsetABOnALengthSquared, inverseFlatEdgeOffsetBCOnALengthSquared, inverseFlatEdgeOffsetCAOnALengthSquared, aDotNormalOnA, bDotNormalOnA, cDotNormalOnA, abDotNormalOnA, bcDotNormalOnA, caDotNormalOnA, flatVertexAOnB, flatEdgeABOnB, bA, abB, new Vector<int>(3), three, localTriangleCenterB, tangentBX, tangentBY, localNormal, minimumDepth, stillCouldUseClippingContacts, ref candidates, ref candidateCount, pairCount);
                ClipBEdgeAgainstABounds(flatVertexAOnA, flatVertexBOnA, flatVertexCOnA, flatEdgeABOnA, flatEdgeBCOnA, flatEdgeCAOnA, inverseFlatEdgeOffsetABOnALengthSquared, inverseFlatEdgeOffsetBCOnALengthSquared, inverseFlatEdgeOffsetCAOnALengthSquared, aDotNormalOnA, bDotNormalOnA, cDotNormalOnA, abDotNormalOnA, bcDotNormalOnA, caDotNormalOnA, flatVertexBOnB, flatEdgeBCOnB, bB, bcB, new Vector<int>(4), three, localTriangleCenterB, tangentBX, tangentBY, localNormal, minimumDepth, stillCouldUseClippingContacts, ref candidates, ref candidateCount, pairCount);
                ClipBEdgeAgainstABounds(flatVertexAOnA, flatVertexBOnA, flatVertexCOnA, flatEdgeABOnA, flatEdgeBCOnA, flatEdgeCAOnA, inverseFlatEdgeOffsetABOnALengthSquared, inverseFlatEdgeOffsetBCOnALengthSquared, inverseFlatEdgeOffsetCAOnALengthSquared, aDotNormalOnA, bDotNormalOnA, cDotNormalOnA, abDotNormalOnA, bcDotNormalOnA, caDotNormalOnA, flatVertexCOnB, flatEdgeCAOnB, bC, caB, new Vector<int>(5), three, localTriangleCenterB, tangentBX, tangentBY, localNormal, minimumDepth, stillCouldUseClippingContacts, ref candidates, ref candidateCount, pairCount);
            }

            //Create a scale-sensitive epsilon for comparisons based on the size of the involved shapes. This helps avoid varying behavior based on how large involved objects are.
            //Vector3Wide.LengthSquared(abA, out var abALengthSquared);
            //Vector3Wide.LengthSquared(caA, out var caALengthSquared);
            //Vector3Wide.LengthSquared(caB, out var caBLengthSquared);
            var epsilonScale = Vector.SquareRoot(Vector.Min(
                Vector.Max(abALengthSquared, caALengthSquared),
                Vector.Max(abBLengthSquared, caBLengthSquared)));
            var edgeEpsilon = new Vector<float>(1e-5f) * epsilonScale;
            ManifoldCandidateHelper.ReduceWithoutComputingDepths(ref candidates, candidateCount, 6, epsilonScale, minimumDepth, pairCount,
                out var contact0, out var contact1, out var contact2, out var contact3,
                out manifold.Contact0Exists, out manifold.Contact1Exists, out manifold.Contact2Exists, out manifold.Contact3Exists);

            //Transform the contacts into the manifold.
            //Move the basis into world rotation so that we don't have to transform the individual contacts.
            Matrix3x3Wide.TransformWithoutOverlap(tangentBX, worldRA, out var worldTangentBX);
            Matrix3x3Wide.TransformWithoutOverlap(tangentBY, worldRA, out var worldTangentBY);
            Matrix3x3Wide.TransformWithoutOverlap(localTriangleCenterB, worldRA, out var worldTriangleCenter);
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, worldRA, out manifold.Normal);
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
                Vector.GreaterThanOrEqual(localNormalDotFaceNormalB, new Vector<float>(MeshReduction.MinimumDotForFaceCollision)), new Vector<int>(MeshReduction.FaceCollisionFlag), Vector<int>.Zero);
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
