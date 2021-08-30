using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.SweepTasks;
using BepuUtilities;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    using DepthRefiner = DepthRefiner<Cylinder, CylinderWide, CylinderSupportFinder, Triangle, TriangleWide, PretransformedTriangleSupportFinder>;

    public struct PretransformedTriangleSupportFinder : ISupportFinder<Triangle, TriangleWide>
    {

        public void ComputeLocalSupport(in TriangleWide shape, in Vector3Wide direction, in Vector<int> terminatedLanes, out Vector3Wide support)
        {
            throw new NotImplementedException();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeSupport(in TriangleWide shape, in Matrix3x3Wide orientation, in Vector3Wide direction, in Vector<int> terminatedLanes, out Vector3Wide support)
        {
            Vector3Wide.Dot(shape.A, direction, out var a);
            Vector3Wide.Dot(shape.B, direction, out var b);
            Vector3Wide.Dot(shape.C, direction, out var c);
            var max = Vector.Max(a, Vector.Max(b, c));
            Vector3Wide.ConditionalSelect(Vector.Equals(max, a), shape.A, shape.B, out support);
            Vector3Wide.ConditionalSelect(Vector.Equals(max, c), shape.C, support, out support);
        }

        public bool HasMargin => false;
        public void GetMargin(in TriangleWide shape, out Vector<float> margin)
        {
            throw new NotImplementedException();
        }
    }


    public struct TriangleCylinderTester : IPairTester<TriangleWide, CylinderWide, Convex4ContactManifoldWide>
    {
        public int BatchSize => 16;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TryAddInteriorPoint(in Vector2Wide point, in Vector<int> featureId,
            in Vector2Wide projectedA, Vector2Wide projectedAB, in Vector2Wide projectedB, Vector2Wide projectedBC, in Vector2Wide projectedC, Vector2Wide projectedCA,
            in Vector<int> allowContact, ref ManifoldCandidate candidates, ref Vector<int> candidateCount, int pairCount)
        {
            Vector2Wide.Subtract(point, projectedA, out var ap);
            Vector2Wide.Subtract(point, projectedB, out var bp);
            Vector2Wide.Subtract(point, projectedC, out var cp);
            //Signs are dependent on the cylinder orientation. Rather than dealing with that, we just look for whether all three signs are the same.
            //If the point is outside the triangle, it can't be outside all three edges at the same time.
            var abDot = ap.X * projectedAB.Y - ap.Y * projectedAB.X;
            var bcDot = bp.X * projectedBC.Y - bp.Y * projectedBC.X;
            var caDot = cp.X * projectedCA.Y - cp.Y * projectedCA.X;
            var sum = Vector.GreaterThan(abDot, Vector<float>.Zero) + Vector.GreaterThan(bcDot, Vector<float>.Zero) + Vector.GreaterThan(caDot, Vector<float>.Zero);

            var contained = Vector.BitwiseAnd(allowContact, Vector.BitwiseOr(Vector.Equals(sum, Vector<int>.Zero), Vector.Equals(sum, new Vector<int>(-3))));
            Unsafe.SkipInit(out ManifoldCandidate candidate);
            candidate.X = point.X;
            candidate.Y = point.Y;
            candidate.FeatureId = featureId;
            ManifoldCandidateHelper.AddCandidate(ref candidates, ref candidateCount, candidate, contained, pairCount);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Test(
            ref TriangleWide a, ref CylinderWide b, ref Vector<float> speculativeMargin,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount,
            out Convex4ContactManifoldWide manifold)
        {
            Unsafe.SkipInit(out manifold);
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var worldRA);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var worldRB);
            //Work in b's local space.
            Matrix3x3Wide.MultiplyByTransposeWithoutOverlap(worldRA, worldRB, out var rA);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, worldRB, out var localOffsetB);

            //Pretransform and recenter the triangle so that the DepthRefiner can work with it (and to avoid unnecessary transforms).
            TriangleWide triangle;
            Matrix3x3Wide.TransformWithoutOverlap(a.A, rA, out triangle.A);
            Matrix3x3Wide.TransformWithoutOverlap(a.B, rA, out triangle.B);
            Matrix3x3Wide.TransformWithoutOverlap(a.C, rA, out triangle.C);
            Vector3Wide.Add(triangle.A, triangle.B, out var centroid);
            Vector3Wide.Add(triangle.C, centroid, out centroid);
            Vector3Wide.Scale(centroid, new Vector<float>(1f / 3f), out centroid);
            Vector3Wide.Subtract(triangle.A, centroid, out triangle.A);
            Vector3Wide.Subtract(triangle.B, centroid, out triangle.B);
            Vector3Wide.Subtract(triangle.C, centroid, out triangle.C);
            Vector3Wide.Subtract(centroid, localOffsetB, out var localTriangleCenter);

            Vector3Wide.Length(localTriangleCenter, out var length);
            Vector3Wide.Scale(localTriangleCenter, Vector<float>.One / length, out var initialNormal);
            var useInitialSampleFallback = Vector.LessThan(length, new Vector<float>(1e-10f));
            initialNormal.X = Vector.ConditionalSelect(useInitialSampleFallback, Vector<float>.Zero, initialNormal.X);
            initialNormal.Y = Vector.ConditionalSelect(useInitialSampleFallback, Vector<float>.One, initialNormal.Y);
            initialNormal.Z = Vector.ConditionalSelect(useInitialSampleFallback, Vector<float>.Zero, initialNormal.Z);

            Vector3Wide.Subtract(triangle.B, triangle.A, out var triangleAB);
            Vector3Wide.Subtract(triangle.C, triangle.B, out var triangleBC);
            Vector3Wide.Subtract(triangle.A, triangle.C, out var triangleCA);
            //We'll be using B-local triangle vertices quite a bit, so cache them.
            Vector3Wide.Add(triangle.A, localTriangleCenter, out var triangleA);
            Vector3Wide.Add(triangle.B, localTriangleCenter, out var triangleB);
            Vector3Wide.Add(triangle.C, localTriangleCenter, out var triangleC);
            Vector3Wide.CrossWithoutOverlap(triangleAB, triangleCA, out var triangleNormal);
            Vector3Wide.Length(triangleNormal, out var triangleNormalLength);
            //Note that degenerate triangles (triangleNormalLength near zero) are ignored completely by the inactiveLanes mask.
            Vector3Wide.Scale(triangleNormal, Vector<float>.One / triangleNormalLength, out triangleNormal);

            //Check if the cylinder's position is within the triangle and below the triangle plane. If so, we can ignore it.
            Vector3Wide.Dot(triangleNormal, localTriangleCenter, out var cylinderToTriangleDot);
            var cylinderBelowPlane = Vector.GreaterThanOrEqual(cylinderToTriangleDot, Vector<float>.Zero);
            Vector3Wide.CrossWithoutOverlap(triangleAB, triangleNormal, out var edgePlaneAB);
            Vector3Wide.CrossWithoutOverlap(triangleBC, triangleNormal, out var edgePlaneBC);
            Vector3Wide.CrossWithoutOverlap(triangleCA, triangleNormal, out var edgePlaneCA);
            //Is the cylinder position within the triangle bounds?
            Vector3Wide.Dot(edgePlaneAB, triangleA, out var abPlaneTest);
            Vector3Wide.Dot(edgePlaneBC, triangleB, out var bcPlaneTest);
            Vector3Wide.Dot(edgePlaneCA, triangleC, out var caPlaneTest);
            var cylinderInsideTriangleEdgePlanes = Vector.BitwiseAnd(
                Vector.LessThanOrEqual(abPlaneTest, Vector<float>.Zero),
                Vector.BitwiseAnd(
                    Vector.LessThanOrEqual(bcPlaneTest, Vector<float>.Zero),
                    Vector.LessThanOrEqual(caPlaneTest, Vector<float>.Zero)));
            var cylinderInsideAndBelowTriangle = Vector.BitwiseAnd(cylinderInsideTriangleEdgePlanes, cylinderBelowPlane);

            ManifoldCandidateHelper.CreateInactiveMask(pairCount, out var inactiveLanes);
            var degenerate = Vector.LessThan(triangleNormalLength, new Vector<float>(1e-10f));
            inactiveLanes = Vector.BitwiseOr(degenerate, inactiveLanes);
            inactiveLanes = Vector.BitwiseOr(cylinderInsideAndBelowTriangle, inactiveLanes);
            if (Vector.LessThanAll(inactiveLanes, Vector<int>.Zero))
            {
                //All lanes are either inactive or were found to have a depth lower than the speculative margin, so we can just quit early.
                manifold = default;
                return;
            }

            PretransformedTriangleSupportFinder triangleSupportFinder = default;
            CylinderSupportFinder cylinderSupportFinder = default;
            //Create a simplex entry for the triangle face normal.
            Vector3Wide.Negate(triangleNormal, out var negatedTriangleNormal);
            cylinderSupportFinder.ComputeLocalSupport(b, negatedTriangleNormal, inactiveLanes, out var cylinderSupportAlongNegatedTriangleNormal);
            Vector3Wide.Subtract(cylinderSupportAlongNegatedTriangleNormal, localTriangleCenter, out var negatedTriangleNormalSupport);
            Vector3Wide.Dot(negatedTriangleNormalSupport, negatedTriangleNormal, out var triangleFaceDepth);

            //Check if the extreme point on the cylinder is contained within the bounds of the triangle face. If it is, there is no need for a full depth refinement.
            Vector3Wide.Subtract(triangleA, cylinderSupportAlongNegatedTriangleNormal, out var closestToA);
            Vector3Wide.Subtract(triangleB, cylinderSupportAlongNegatedTriangleNormal, out var closestToB);
            Vector3Wide.Subtract(triangleC, cylinderSupportAlongNegatedTriangleNormal, out var closestToC);
            Vector3Wide.Dot(edgePlaneAB, closestToA, out var extremeABPlaneTest);
            Vector3Wide.Dot(edgePlaneBC, closestToB, out var extremeBCPlaneTest);
            Vector3Wide.Dot(edgePlaneCA, closestToC, out var extremeCAPlaneTest);
            var triangleNormalIsMinimal = Vector.BitwiseAnd(
                Vector.BitwiseAnd(
                    Vector.AndNot(cylinderInsideTriangleEdgePlanes, cylinderBelowPlane),
                    Vector.LessThanOrEqual(extremeABPlaneTest, Vector<float>.Zero)),
                Vector.BitwiseAnd(
                    Vector.LessThanOrEqual(extremeBCPlaneTest, Vector<float>.Zero),
                    Vector.LessThanOrEqual(extremeCAPlaneTest, Vector<float>.Zero)));

            var depthThreshold = -speculativeMargin;
            var skipDepthRefine = Vector.BitwiseOr(triangleNormalIsMinimal, inactiveLanes);
            Vector3Wide localNormal, closestOnB;
            Vector<float> depth;
            var epsilonScale = Vector.Max(b.HalfLength, b.Radius);
            if (Vector.EqualsAny(skipDepthRefine, Vector<int>.Zero))
            {
                DepthRefiner.FindMinimumDepth(
                    b, triangle, localTriangleCenter, rA, ref cylinderSupportFinder, ref triangleSupportFinder, initialNormal, skipDepthRefine, 1e-5f * epsilonScale, depthThreshold,
                    out var refinedDepth, out var refinedNormal, out var refinedClosestOnHull);
                Vector3Wide.ConditionalSelect(skipDepthRefine, cylinderSupportAlongNegatedTriangleNormal, refinedClosestOnHull, out closestOnB);
                Vector3Wide.ConditionalSelect(skipDepthRefine, negatedTriangleNormal, refinedNormal, out localNormal);
                depth = Vector.ConditionalSelect(skipDepthRefine, triangleFaceDepth, refinedDepth);
            }
            else
            {
                //No depth refine ran; the extreme point prepass did everything we needed. Just use the initial normal.
                localNormal = negatedTriangleNormal;
                closestOnB = cylinderSupportAlongNegatedTriangleNormal;
                depth = triangleFaceDepth;
            }

            //If the cylinder is too far away or if it's on the backside of the triangle, don't generate any contacts.
            Vector3Wide.Dot(triangleNormal, localNormal, out var faceNormalADotNormal);
            inactiveLanes = Vector.BitwiseOr(inactiveLanes, Vector.BitwiseOr(Vector.GreaterThan(faceNormalADotNormal, new Vector<float>(-SphereTriangleTester.BackfaceNormalDotRejectionThreshold)), Vector.LessThan(depth, depthThreshold)));
            if (Vector.LessThanAll(inactiveLanes, Vector<int>.Zero))
            {
                //All lanes are either inactive or were found to have a depth lower than the speculative margin, so we can just quit early.
                manifold = default;
                return;
            }

            //We generate contacts according to the dominant features along the collision normal.
            //The possible pairs are:
            //Face A-Cap A
            //Face A-Side A

            var capCenterBY = Vector.ConditionalSelect(Vector.LessThan(localNormal.Y, Vector<float>.Zero), -b.HalfLength, b.HalfLength);

            var useCap = Vector.AndNot(Vector.GreaterThan(Vector.Abs(localNormal.Y), new Vector<float>(0.70710678118f)), inactiveLanes);

            //When the contact normal is nearly perpendicular to the triangle normal, computing per contact depths can become numerically impossible.
            //To help with this case, we use an 'effective' triangle face normal for anything related to computing contact depths.
            Vector3Wide effectiveFaceNormal;
            var absFaceNormalADotNormal = Vector.Abs(faceNormalADotNormal);
            const float faceNormalFallbackThreshold = 1e-4f;
            var needsFallbackFaceNormal = Vector.AndNot(Vector.LessThan(absFaceNormalADotNormal, new Vector<float>(faceNormalFallbackThreshold)), inactiveLanes);
            Vector<float> effectiveFaceNormalDotNormal;
            if (Vector.EqualsAny(needsFallbackFaceNormal, new Vector<int>(-1)))
            {
                //Near-zero faceNormalADotNormal values only occur during edge or vertex collisions.
                //During edge collisions, the local normal is perpendicular to the edge.
                //We can safely push the triangle normal along the local normal to get an 'effective' normal that doesn't cause numerical catastrophe.
                var pushScale = Vector.Max(Vector<float>.Zero, (absFaceNormalADotNormal - new Vector<float>(faceNormalFallbackThreshold)) / new Vector<float>(-faceNormalFallbackThreshold));
                Vector3Wide.Scale(localNormal, pushScale * new Vector<float>(0.1f), out var effectiveFaceNormalPush);
                Vector3Wide.Add(triangleNormal, effectiveFaceNormalPush, out effectiveFaceNormal);
                //Length is guaranteed to be nonzero since this is only used when normal is near perpendicular.
                Vector3Wide.Normalize(effectiveFaceNormal, out effectiveFaceNormal);
                //Normalization could change things numerically, so we can't normalize any lanes that aren't supposed to be taking this code path.
                Vector3Wide.ConditionalSelect(needsFallbackFaceNormal, effectiveFaceNormal, triangleNormal, out effectiveFaceNormal);
            }
            else
            {
                //No near-perpendicular contacts in this bundle.
                effectiveFaceNormal = triangleNormal;
                effectiveFaceNormalDotNormal = faceNormalADotNormal;

            }
            //Given the push above, this is guaranteed not to divide by zero.
            Vector3Wide.Dot(effectiveFaceNormal, localNormal, out effectiveFaceNormalDotNormal);
            var inverseEffectiveFaceNormalDotNormal = Vector<float>.One / effectiveFaceNormalDotNormal;

            if (Vector.LessThanAny(useCap, Vector<int>.Zero))
            {
                //At least one lane needs a cap-face manifold.
                int byteCount = Unsafe.SizeOf<ManifoldCandidate>() * 10;
                var buffer = stackalloc byte[byteCount];
                ref var candidates = ref Unsafe.As<byte, ManifoldCandidate>(ref *buffer);
                var candidateCount = Vector<int>.Zero;

                //Project the edges down onto the cap's plane.
                var inverseLocalNormalY = Vector<float>.One / localNormal.Y;
                CylinderPairTester.ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, triangleA, out var pA);
                CylinderPairTester.ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, triangleB, out var pB);
                CylinderPairTester.ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, triangleC, out var pC);
                Vector2Wide.Subtract(pB, pA, out var projectedAB);
                Vector2Wide.Subtract(pC, pB, out var projectedBC);
                Vector2Wide.Subtract(pA, pC, out var projectedCA);
                BoxCylinderTester.IntersectLineCircle(pA, projectedAB, b.Radius, out var tMinAB, out var tMaxAB, out var intersectedAB);
                BoxCylinderTester.IntersectLineCircle(pB, projectedBC, b.Radius, out var tMinBC, out var tMaxBC, out var intersectedBC);
                BoxCylinderTester.IntersectLineCircle(pC, projectedCA, b.Radius, out var tMinCA, out var tMaxCA, out var intersectedCA);

                tMinAB = Vector.Min(Vector.Max(tMinAB, Vector<float>.Zero), Vector<float>.One);
                tMaxAB = Vector.Min(Vector.Max(tMaxAB, Vector<float>.Zero), Vector<float>.One);
                tMinBC = Vector.Min(Vector.Max(tMinBC, Vector<float>.Zero), Vector<float>.One);
                tMaxBC = Vector.Min(Vector.Max(tMaxBC, Vector<float>.Zero), Vector<float>.One);
                tMinCA = Vector.Min(Vector.Max(tMinCA, Vector<float>.Zero), Vector<float>.One);
                tMaxCA = Vector.Min(Vector.Max(tMaxCA, Vector<float>.Zero), Vector<float>.One);

                BoxCylinderTester.AddCandidateForEdge(pA, projectedAB, tMinAB, tMaxAB, intersectedAB, Vector<int>.Zero, useCap, pairCount, ref candidates, ref candidateCount);
                BoxCylinderTester.AddCandidateForEdge(pB, projectedBC, tMinBC, tMaxBC, intersectedBC, Vector<int>.One, useCap, pairCount, ref candidates, ref candidateCount);
                BoxCylinderTester.AddCandidateForEdge(pC, projectedCA, tMinCA, tMaxCA, intersectedCA, new Vector<int>(2), useCap, pairCount, ref candidates, ref candidateCount);

                BoxCylinderTester.GenerateInteriorPoints(b, localNormal, closestOnB, out var interior0, out var interior1, out var interior2, out var interior3);

                //Test the four points against the edge plane. Note that signs depend on the orientation of the cylinder.
                TryAddInteriorPoint(interior0, new Vector<int>(8), pA, projectedAB, pB, projectedBC, pC, projectedCA, useCap, ref candidates, ref candidateCount, pairCount);
                TryAddInteriorPoint(interior1, new Vector<int>(9), pA, projectedAB, pB, projectedBC, pC, projectedCA, useCap, ref candidates, ref candidateCount, pairCount);
                TryAddInteriorPoint(interior2, new Vector<int>(10), pA, projectedAB, pB, projectedBC, pC, projectedCA, useCap, ref candidates, ref candidateCount, pairCount);
                TryAddInteriorPoint(interior3, new Vector<int>(11), pA, projectedAB, pB, projectedBC, pC, projectedCA, useCap, ref candidates, ref candidateCount, pairCount);

                //Use the closest point on the triangle as the face origin.
                //The effective normal is not necessarily the same as the triangle normal at near parallel angles as a numerical safety,
                //so the projection plane anchor must be on the feature that is responsible for the normal.
                Vector3Wide.Scale(localNormal, depth, out var closestOnAToClosestOnB);
                Vector3Wide.Subtract(closestOnB, closestOnAToClosestOnB, out var capCenterToTriangle);
                capCenterToTriangle.Y -= capCenterBY;
                Vector3Wide tangentBX, tangentBY;
                tangentBX.X = Vector<float>.One;
                tangentBX.Y = Vector<float>.Zero;
                tangentBX.Z = Vector<float>.Zero;
                tangentBY.X = Vector<float>.Zero;
                tangentBY.Y = Vector<float>.Zero;
                tangentBY.Z = Vector<float>.One;
                ManifoldCandidateHelper.Reduce(ref candidates, candidateCount, 10, effectiveFaceNormal, inverseEffectiveFaceNormalDotNormal, capCenterToTriangle, tangentBX, tangentBY, epsilonScale, depthThreshold, pairCount,
                    out var candidate0, out var candidate1, out var candidate2, out var candidate3,
                    out manifold.Contact0Exists, out manifold.Contact1Exists, out manifold.Contact2Exists, out manifold.Contact3Exists);

                Vector3Wide localContact;
                localContact.X = candidate0.X;
                localContact.Y = capCenterBY;
                localContact.Z = candidate0.Y;
                Vector3Wide.Add(localContact, localOffsetB, out var aToLocalContact);
                Matrix3x3Wide.TransformWithoutOverlap(aToLocalContact, worldRB, out manifold.OffsetA0);
                localContact.X = candidate1.X;
                localContact.Z = candidate1.Y;
                Vector3Wide.Add(localContact, localOffsetB, out aToLocalContact);
                Matrix3x3Wide.TransformWithoutOverlap(aToLocalContact, worldRB, out manifold.OffsetA1);
                localContact.X = candidate2.X;
                localContact.Z = candidate2.Y;
                Vector3Wide.Add(localContact, localOffsetB, out aToLocalContact);
                Matrix3x3Wide.TransformWithoutOverlap(aToLocalContact, worldRB, out manifold.OffsetA2);
                localContact.X = candidate3.X;
                localContact.Z = candidate3.Y;
                Vector3Wide.Add(localContact, localOffsetB, out aToLocalContact);
                Matrix3x3Wide.TransformWithoutOverlap(aToLocalContact, worldRB, out manifold.OffsetA3);
                manifold.FeatureId0 = candidate0.FeatureId;
                manifold.FeatureId1 = candidate1.FeatureId;
                manifold.FeatureId2 = candidate2.FeatureId;
                manifold.FeatureId3 = candidate3.FeatureId;
                manifold.Depth0 = candidate0.Depth;
                manifold.Depth1 = candidate1.Depth;
                manifold.Depth2 = candidate2.Depth;
                manifold.Depth3 = candidate3.Depth;
            }
            else
            {
                manifold.Contact0Exists = default;
                manifold.Contact1Exists = default;
                manifold.Contact2Exists = default;
                manifold.Contact3Exists = default;
            }

            var useSide = Vector.AndNot(Vector.OnesComplement(useCap), inactiveLanes);
            if (Vector.LessThanAny(useSide, Vector<int>.Zero))
            {
                //At least one lane needs a side-face manifold.
                //Intersect the single edge of A with the edge planes of face A.
                //Note that the edge planes are skewed to follow the local normal. Equivalent to projecting the side edge onto face A.
                //Edge normals point inward.
                Vector3Wide.CrossWithoutOverlap(localNormal, triangleAB, out var edgeNormalAB);
                Vector3Wide.CrossWithoutOverlap(localNormal, triangleBC, out var edgeNormalBC);
                Vector3Wide.CrossWithoutOverlap(localNormal, triangleCA, out var edgeNormalCA);

                //Beware, the edge normals could be very close to parallel with the local normal, causing numerical poopiness.
                //angle between localNormal and triangleAB = asin(length(edgeNormalAB) / length(triangleAB))
                //sin(angle)^2 = lengthSquared(edgeNormalAB) / lengthSquared(triangleAB)
                //So we can check against a scale invariant threshold without any square roots or divisions.
                const float minimumSinAngle = 1e-5f;
                var edgeNormalThreshold = new Vector<float>(minimumSinAngle * minimumSinAngle);
                Vector3Wide.LengthSquared(edgeNormalAB, out var edgeNormalABLengthSquared);
                Vector3Wide.LengthSquared(triangleAB, out var triangleABLengthSquared);
                var useFallbackAB = Vector.LessThan(edgeNormalABLengthSquared, edgeNormalThreshold * triangleABLengthSquared);
                Vector3Wide.LengthSquared(edgeNormalBC, out var edgeNormalBCLengthSquared);
                Vector3Wide.LengthSquared(triangleBC, out var triangleBCLengthSquared);
                var useFallbackBC = Vector.LessThan(edgeNormalBCLengthSquared, edgeNormalThreshold * triangleBCLengthSquared);
                Vector3Wide.LengthSquared(edgeNormalCA, out var edgeNormalCALengthSquared);
                Vector3Wide.LengthSquared(triangleCA, out var triangleCALengthSquared);
                var useFallbackCA = Vector.LessThan(edgeNormalCALengthSquared, edgeNormalThreshold * triangleCALengthSquared);

                //Center of the side line is just (closestOnB.X, 0, closestOnB.Z), sideLineDirection is just (0, 1, 0).
                //t = dot(sideLineStart - pointOnFaceEdge, edgeNormal) / dot(sideLineDirection, edgeNormal)
                var negativeOne = new Vector<float>(-1f);
                //Guard against divisions by zero to avoid NaN infection.
                var minValue = new Vector<float>(float.MinValue);
                var maxValue = new Vector<float>(float.MaxValue);
                var abDenominator = negativeOne / edgeNormalAB.Y;
                var bcDenominator = negativeOne / edgeNormalBC.Y;
                var caDenominator = negativeOne / edgeNormalCA.Y;
                Vector3Wide aToSideLine, bToSideLine, cToSideLine;
                aToSideLine.X = closestOnB.X - triangleA.X;
                aToSideLine.Y = -triangleA.Y;
                aToSideLine.Z = closestOnB.Z - triangleA.Z;
                bToSideLine.X = closestOnB.X - triangleB.X;
                bToSideLine.Y = -triangleB.Y;
                bToSideLine.Z = closestOnB.Z - triangleB.Z;
                cToSideLine.X = closestOnB.X - triangleC.X;
                cToSideLine.Y = -triangleC.Y;
                cToSideLine.Z = closestOnB.Z - triangleC.Z;

                Vector3Wide.Dot(edgeNormalAB, aToSideLine, out var abNumerator);
                Vector3Wide.Dot(edgeNormalBC, bToSideLine, out var bcNumerator);
                Vector3Wide.Dot(edgeNormalCA, cToSideLine, out var caNumerator);
                var tAB = abNumerator * abDenominator;
                var tBC = bcNumerator * bcDenominator;
                var tCA = caNumerator * caDenominator;
                //As the side aligns with one of the edge directions, unrestrict that axis to avoid numerical noise.
                //Do something similar to capsule tests: 
                //sin(angleFromEdgePlane) = dot(sideLineDirection, edgeNormal / ||edgeNormal||) / ||sideLineDirection||
                //sin(angleFromEdgePlane) = dot(sideLineDirection, edgeNormal / ||edgeNormal||)
                //Only relevant in very small angles, so sin(x) ~= x:
                //angleFromEdgePlane = dot(sideLineDirection, edgeNormal / ||edgeNormal||)
                //Interpolation behavior is pretty arbitrary, so squaring is fine:
                //angleFromEdgePlane^2 = dot(sideLineDirection, edgeNormal)^2 / ||edgeNormal||^2
                const float lowerThresholdAngle = 0.01f;
                const float upperThresholdAngle = 0.02f;
                const float lowerThreshold = lowerThresholdAngle * lowerThresholdAngle;
                const float upperThreshold = upperThresholdAngle * upperThresholdAngle;
                var interpolationMin = new Vector<float>(upperThreshold);
                var inverseInterpolationSpan = new Vector<float>(1f / (upperThreshold - lowerThreshold));
                var inverseEdgeNormalABLengthSquared = Vector.ConditionalSelect(Vector.Equals(edgeNormalABLengthSquared, Vector<float>.Zero), maxValue, Vector<float>.One / edgeNormalABLengthSquared);
                var inverseEdgeNormalBCLengthSquared = Vector.ConditionalSelect(Vector.Equals(edgeNormalBCLengthSquared, Vector<float>.Zero), maxValue, Vector<float>.One / edgeNormalBCLengthSquared);
                var inverseEdgeNormalCALengthSquared = Vector.ConditionalSelect(Vector.Equals(edgeNormalCALengthSquared, Vector<float>.Zero), maxValue, Vector<float>.One / edgeNormalCALengthSquared);
                var unrestrictWeightAB = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, (interpolationMin - edgeNormalAB.Y * edgeNormalAB.Y * inverseEdgeNormalABLengthSquared) * inverseInterpolationSpan));
                var unrestrictWeightBC = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, (interpolationMin - edgeNormalBC.Y * edgeNormalBC.Y * inverseEdgeNormalBCLengthSquared) * inverseInterpolationSpan));
                var unrestrictWeightCA = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, (interpolationMin - edgeNormalCA.Y * edgeNormalCA.Y * inverseEdgeNormalCALengthSquared) * inverseInterpolationSpan));
                var regularWeightAB = Vector<float>.One - unrestrictWeightAB;
                var regularWeightBC = Vector<float>.One - unrestrictWeightBC;
                var regularWeightCA = Vector<float>.One - unrestrictWeightCA;
                var negativeHalfLength = -b.HalfLength;

                var enteringAB = Vector.GreaterThan(edgeNormalAB.Y, Vector<float>.Zero);
                var enteringBC = Vector.GreaterThan(edgeNormalBC.Y, Vector<float>.Zero);
                var enteringCA = Vector.GreaterThan(edgeNormalCA.Y, Vector<float>.Zero);
                var weightedAB = regularWeightAB * tAB;
                var weightedBC = regularWeightBC * tBC;
                var weightedCA = regularWeightCA * tCA;
                var tABEntry = Vector.ConditionalSelect(enteringAB, unrestrictWeightAB * negativeHalfLength + weightedAB, minValue);
                var tABExit = Vector.ConditionalSelect(enteringAB, maxValue, unrestrictWeightAB * b.HalfLength + weightedAB);
                var tBCEntry = Vector.ConditionalSelect(enteringBC, unrestrictWeightBC * negativeHalfLength + weightedBC, minValue);
                var tBCExit = Vector.ConditionalSelect(enteringBC, maxValue, unrestrictWeightBC * b.HalfLength + weightedBC);
                var tCAEntry = Vector.ConditionalSelect(enteringCA, unrestrictWeightCA * negativeHalfLength + weightedCA, minValue);
                var tCAExit = Vector.ConditionalSelect(enteringCA, maxValue, unrestrictWeightCA * b.HalfLength + weightedCA);
                tABEntry = Vector.ConditionalSelect(useFallbackAB, minValue, tABEntry);
                tABExit = Vector.ConditionalSelect(useFallbackAB, maxValue, tABExit);
                tBCEntry = Vector.ConditionalSelect(useFallbackBC, minValue, tBCEntry);
                tBCExit = Vector.ConditionalSelect(useFallbackBC, maxValue, tBCExit);
                tCAEntry = Vector.ConditionalSelect(useFallbackCA, minValue, tCAEntry);
                tCAExit = Vector.ConditionalSelect(useFallbackCA, maxValue, tCAExit);
                var tMax = Vector.Min(b.HalfLength, Vector.Max(negativeHalfLength, Vector.Min(tABExit, Vector.Min(tBCExit, tCAExit))));
                var tMin = Vector.Min(b.HalfLength, Vector.Max(negativeHalfLength, Vector.Max(tABEntry, Vector.Max(tBCEntry, tCAEntry))));

                Vector3Wide localContact0, localContact1;
                localContact0.X = localContact1.X = closestOnB.X;
                //Mathematically, closestOnB.Y should be in the tMin to tMax interval. Numerically, this is not guaranteed. So force it.
                localContact0.Y = Vector.Min(closestOnB.Y, tMin);
                localContact1.Y = Vector.Max(closestOnB.Y, tMax);
                localContact0.Z = localContact1.Z = closestOnB.Z;

                Matrix3x3Wide.TransformWithoutOverlap(localContact0, worldRB, out var contact0);
                Matrix3x3Wide.TransformWithoutOverlap(localContact1, worldRB, out var contact1);
                Vector3Wide.Add(contact0, offsetB, out contact0);
                Vector3Wide.Add(contact1, offsetB, out contact1);
                Vector3Wide.ConditionalSelect(useSide, contact0, manifold.OffsetA0, out manifold.OffsetA0);
                Vector3Wide.ConditionalSelect(useSide, contact1, manifold.OffsetA1, out manifold.OffsetA1);
                manifold.FeatureId0 = Vector.ConditionalSelect(useSide, Vector<int>.Zero, manifold.FeatureId0);
                manifold.FeatureId1 = Vector.ConditionalSelect(useSide, Vector<int>.One, manifold.FeatureId1);

                //We know the depth to the minimum contact at t = closestOnB.Y.
                //Map movement along the cylinder side edge to a change in depth:
                //contactDepth = depth + (t - closestOnB.Y) * dot(localCapsuleAxis, triangleNormal) / dot(localNormal, triangleNormal)
                //             = depth + (t - closestOnB.Y) * triangleNormal.Y / dot(localNormal, triangleNormal)
                //Note the use of the 'effective' normal. It's either the face normal, or a fallback created from edge normals and the local normal above.
                //We have to handle the case where the normal was perpendicular to the face normal, making a direct projection numerically impossible.
                //The effective face normal is guaranteed to not be perpendicular to the local normal.
                var depthScale = effectiveFaceNormal.Y * inverseEffectiveFaceNormalDotNormal;
                var o0 = localContact0.Y - closestOnB.Y;
                var o1 = localContact1.Y - closestOnB.Y;
                var depth0 = depth + Vector.Min(o0 * localNormal.Y, o0 * depthScale);
                var depth1 = depth + Vector.Min(o1 * localNormal.Y, o1 * depthScale);

                manifold.Depth0 = Vector.ConditionalSelect(useSide, depth0, manifold.Depth0);
                manifold.Depth1 = Vector.ConditionalSelect(useSide, depth1, manifold.Depth1);
                manifold.Contact0Exists = Vector.ConditionalSelect(useSide, Vector.GreaterThan(depth0, depthThreshold), manifold.Contact0Exists);
                manifold.Contact1Exists = Vector.ConditionalSelect(useSide, Vector.BitwiseAnd(Vector.GreaterThan(depth1, depthThreshold), Vector.GreaterThan(localContact1.Y, localContact0.Y)), manifold.Contact1Exists);
                manifold.Contact2Exists = Vector.ConditionalSelect(useSide, Vector<int>.Zero, manifold.Contact2Exists);
                manifold.Contact3Exists = Vector.ConditionalSelect(useSide, Vector<int>.Zero, manifold.Contact3Exists);
            }

            Matrix3x3Wide.TransformWithoutOverlap(localNormal, worldRB, out manifold.Normal);

            //Mesh reductions rely on the contact positions being on the surface of the triangle. Push the offsets accordingly.
            Vector3Wide.Scale(manifold.Normal, manifold.Depth0, out var offset0);
            Vector3Wide.Scale(manifold.Normal, manifold.Depth1, out var offset1);
            Vector3Wide.Scale(manifold.Normal, manifold.Depth2, out var offset2);
            Vector3Wide.Scale(manifold.Normal, manifold.Depth3, out var offset3);
            Vector3Wide.Subtract(manifold.OffsetA0, offset0, out manifold.OffsetA0);
            Vector3Wide.Subtract(manifold.OffsetA1, offset1, out manifold.OffsetA1);
            Vector3Wide.Subtract(manifold.OffsetA2, offset2, out manifold.OffsetA2);
            Vector3Wide.Subtract(manifold.OffsetA3, offset3, out manifold.OffsetA3);
            //Mesh reductions also make use of a face contact flag in the feature id.
            var faceCollisionFlag = Vector.ConditionalSelect(
                Vector.LessThan(faceNormalADotNormal, new Vector<float>(-MeshReduction.MinimumDotForFaceCollision)), new Vector<int>(MeshReduction.FaceCollisionFlag), Vector<int>.Zero);
            manifold.FeatureId0 += faceCollisionFlag;
        }


        public void Test(ref TriangleWide a, ref CylinderWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref TriangleWide a, ref CylinderWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }
}