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
            ManifoldCandidate candidate;
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
            Vector<int> cylinderInsideAndBelowTriangle;
            Vector3Wide.CrossWithoutOverlap(triangleAB, triangleNormal, out var edgePlaneAB);
            Vector3Wide.CrossWithoutOverlap(triangleBC, triangleNormal, out var edgePlaneBC);
            Vector3Wide.CrossWithoutOverlap(triangleCA, triangleNormal, out var edgePlaneCA);
            if (Vector.LessThanAny(cylinderBelowPlane, Vector<int>.Zero))
            {
                //Is the cylinder position within the triangle bounds?
                Vector3Wide.Dot(edgePlaneAB, triangleA, out var abPlaneTest);
                Vector3Wide.Dot(edgePlaneBC, triangleB, out var bcPlaneTest);
                Vector3Wide.Dot(edgePlaneCA, triangleC, out var caPlaneTest);
                cylinderInsideAndBelowTriangle = Vector.BitwiseAnd(
                    Vector.BitwiseAnd(cylinderBelowPlane, Vector.LessThanOrEqual(abPlaneTest, Vector<float>.Zero)),
                    Vector.BitwiseAnd(Vector.LessThanOrEqual(bcPlaneTest, Vector<float>.Zero), Vector.LessThanOrEqual(caPlaneTest, Vector<float>.Zero)));
            }
            else
            {
                cylinderInsideAndBelowTriangle = Vector<int>.Zero;
            }

            //We now have a decent estimate for the local normal and an initial simplex to work from. Refine it to a local minimum.

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
            //Check if the extreme point of the cylinder toward the triangle along its face normal lies inside the triangle.
            //If it is, then there's no need for depth refinement.
            Vector<int> triangleNormalIsMinimal;
            DepthRefiner.SimplexWithWitness simplex;
            //Create a simplex entry for the direction from the hull center to triangle center.
            DepthRefiner.FindSupport(b, triangle, localTriangleCenter, rA, ref cylinderSupportFinder, ref triangleSupportFinder, initialNormal, inactiveLanes, out simplex.A.Support, out simplex.A.SupportOnA);
            Vector3Wide.Dot(simplex.A.Support, initialNormal, out var depth);
            simplex.A.Exists = Vector.OnesComplement(inactiveLanes);
            //Create a simplex entry for the triangle face normal.
            Vector3Wide.Negate(triangleNormal, out var negatedTriangleNormal);
            cylinderSupportFinder.ComputeLocalSupport(b, negatedTriangleNormal, inactiveLanes, out var cylinderSupportAlongTriangleNormal);
            simplex.B.SupportOnA = cylinderSupportAlongTriangleNormal;
            Vector3Wide.Subtract(simplex.B.SupportOnA, localTriangleCenter, out simplex.B.Support);
            Vector3Wide.Dot(simplex.B.Support, negatedTriangleNormal, out var triangleFaceDepth);
            var useTriangleFace = Vector.LessThan(triangleFaceDepth, depth);
            Vector3Wide.ConditionalSelect(useTriangleFace, negatedTriangleNormal, initialNormal, out initialNormal);
            depth = Vector.ConditionalSelect(useTriangleFace, triangleFaceDepth, depth);
            simplex.B.Exists = simplex.A.Exists;
            simplex.C.Exists = default;

            //Check if the extreme point on the hull is contained within the bounds of the triangle face. If it is, there is no need for a full depth refinement.
            Vector3Wide.Subtract(triangleA, cylinderSupportAlongTriangleNormal, out var closestToA);
            Vector3Wide.Subtract(triangleB, cylinderSupportAlongTriangleNormal, out var closestToB);
            Vector3Wide.Subtract(triangleC, cylinderSupportAlongTriangleNormal, out var closestToC);
            Vector3Wide.Dot(edgePlaneAB, closestToA, out var extremeABPlaneTest);
            Vector3Wide.Dot(edgePlaneBC, closestToB, out var extremeBCPlaneTest);
            Vector3Wide.Dot(edgePlaneCA, closestToC, out var extremeCAPlaneTest);
            triangleNormalIsMinimal = Vector.BitwiseAnd(Vector.LessThanOrEqual(extremeABPlaneTest, Vector<float>.Zero), Vector.BitwiseAnd(Vector.LessThanOrEqual(extremeBCPlaneTest, Vector<float>.Zero), Vector.LessThanOrEqual(extremeCAPlaneTest, Vector<float>.Zero)));

            var depthThreshold = -speculativeMargin;
            var skipDepthRefine = Vector.BitwiseOr(triangleNormalIsMinimal, inactiveLanes);
            Vector3Wide localNormal, closestOnB;
            var epsilonScale = Vector.Max(b.HalfLength, b.Radius);
            if (Vector.EqualsAny(skipDepthRefine, Vector<int>.Zero))
            {
                DepthRefiner.FindMinimumDepth(
                    b, triangle, localTriangleCenter, rA, ref cylinderSupportFinder, ref triangleSupportFinder, ref simplex, initialNormal, depth, skipDepthRefine, 1e-5f * epsilonScale, depthThreshold,
                    out var refinedDepth, out var refinedNormal, out var refinedClosestOnHull);
                Vector3Wide.ConditionalSelect(skipDepthRefine, cylinderSupportAlongTriangleNormal, refinedClosestOnHull, out closestOnB);
                Vector3Wide.ConditionalSelect(skipDepthRefine, initialNormal, refinedNormal, out localNormal);
                depth = Vector.ConditionalSelect(skipDepthRefine, depth, refinedDepth);
            }
            else
            {
                //No depth refine ran; the extreme point prepass did everything we needed. Just use the initial normal.
                localNormal = initialNormal;
                closestOnB = cylinderSupportAlongTriangleNormal;
            }

            //If the cylinder is too far away or if it's on the backside of the triangle, don't generate any contacts.
            Vector3Wide.Dot(triangleNormal, localNormal, out var faceNormalADotNormal);
            inactiveLanes = Vector.BitwiseOr(inactiveLanes, Vector.BitwiseOr(Vector.GreaterThanOrEqual(faceNormalADotNormal, Vector<float>.Zero), Vector.LessThan(depth, depthThreshold)));
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

                Vector3Wide capCenterToTriangle;
                capCenterToTriangle.X = triangleA.X;
                capCenterToTriangle.Y = triangleA.Y - capCenterBY;
                capCenterToTriangle.Z = triangleA.Z;
                Vector3Wide tangentBX, tangentBY;
                tangentBX.X = Vector<float>.One;
                tangentBX.Y = Vector<float>.Zero;
                tangentBX.Z = Vector<float>.Zero;
                tangentBY.X = Vector<float>.Zero;
                tangentBY.Y = Vector<float>.Zero;
                tangentBY.Z = Vector<float>.One;
                ManifoldCandidateHelper.Reduce(ref candidates, candidateCount, 10, triangleNormal, localNormal, capCenterToTriangle, tangentBX, tangentBY, epsilonScale, depthThreshold, pairCount,
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

                //Center of the side line is just (closestOnB.X, 0, closestOnB.Z), sideLineDirection is just (0, 1, 0).
                //t = dot(sideLineStart - pointOnFaceEdge, edgeNormal) / dot(sideLineDirection, edgeNormal)
                var negativeOne = new Vector<float>(-1f);
                //Guard against divisions by zero to avoid NaN infection.
                var minValue = new Vector<float>(float.MinValue);
                var maxValue = new Vector<float>(float.MaxValue);
                var abDenominator = Vector.ConditionalSelect(Vector.Equals(edgeNormalAB.Y, Vector<float>.Zero), minValue, negativeOne / edgeNormalAB.Y);
                var bcDenominator = Vector.ConditionalSelect(Vector.Equals(edgeNormalBC.Y, Vector<float>.Zero), minValue, negativeOne / edgeNormalBC.Y);
                var caDenominator = Vector.ConditionalSelect(Vector.Equals(edgeNormalCA.Y, Vector<float>.Zero), minValue, negativeOne / edgeNormalCA.Y);
                Vector3Wide.LengthSquared(edgeNormalAB, out var edgeNormalABLengthSquared);
                Vector3Wide.LengthSquared(edgeNormalBC, out var edgeNormalBCLengthSquared);
                Vector3Wide.LengthSquared(edgeNormalCA, out var edgeNormalCALengthSquared);
                var inverseEdgeNormalABLengthSquared = Vector.ConditionalSelect(Vector.Equals(edgeNormalABLengthSquared, Vector<float>.Zero), maxValue, Vector<float>.One / edgeNormalABLengthSquared);
                var inverseEdgeNormalBCLengthSquared = Vector.ConditionalSelect(Vector.Equals(edgeNormalBCLengthSquared, Vector<float>.Zero), maxValue, Vector<float>.One / edgeNormalBCLengthSquared);
                var inverseEdgeNormalCALengthSquared = Vector.ConditionalSelect(Vector.Equals(edgeNormalCALengthSquared, Vector<float>.Zero), maxValue, Vector<float>.One / edgeNormalCALengthSquared);
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
                var tMax = Vector.Min(b.HalfLength, Vector.Max(-b.HalfLength, Vector.Min(tABExit, Vector.Min(tBCExit, tCAExit))));
                var tMin = Vector.Min(b.HalfLength, Vector.Max(-b.HalfLength, Vector.Max(tABEntry, Vector.Max(tBCEntry, tCAEntry))));

                var inverseFaceNormalDotLocalNormal = Vector<float>.One / faceNormalADotNormal;
                Vector3Wide localContact0, localContact1;
                localContact0.X = localContact1.X = closestOnB.X;
                localContact0.Y = tMin;
                localContact1.Y = tMax;
                localContact0.Z = localContact1.Z = closestOnB.Z;
                Matrix3x3Wide.TransformWithoutOverlap(localContact0, worldRB, out var contact0);
                Matrix3x3Wide.TransformWithoutOverlap(localContact1, worldRB, out var contact1);
                Vector3Wide.Add(contact0, offsetB, out contact0);
                Vector3Wide.Add(contact1, offsetB, out contact1);
                Vector3Wide.ConditionalSelect(useSide, contact0, manifold.OffsetA0, out manifold.OffsetA0);
                Vector3Wide.ConditionalSelect(useSide, contact1, manifold.OffsetA1, out manifold.OffsetA1);
                manifold.FeatureId0 = Vector.ConditionalSelect(useSide, Vector<int>.Zero, manifold.FeatureId0);
                manifold.FeatureId1 = Vector.ConditionalSelect(useSide, Vector<int>.One, manifold.FeatureId1);
                //depth = dot(pointOnFaceB - faceCenterA, faceNormalA) / dot(faceNormalA, normal)
                Vector3Wide.Subtract(localContact0, triangleA, out var faceToContact0);
                Vector3Wide.Subtract(localContact1, triangleA, out var faceToContact1);
                Vector3Wide.Dot(faceToContact0, triangleNormal, out var contact0Dot);
                Vector3Wide.Dot(faceToContact1, triangleNormal, out var contact1Dot);
                var depth0 = contact0Dot * inverseFaceNormalDotLocalNormal;
                var depth1 = contact1Dot * inverseFaceNormalDotLocalNormal;
                manifold.Depth0 = Vector.ConditionalSelect(useSide, depth0, manifold.Depth0);
                manifold.Depth1 = Vector.ConditionalSelect(useSide, depth1, manifold.Depth1);
                manifold.Contact0Exists = Vector.ConditionalSelect(useSide, Vector.GreaterThan(depth0, depthThreshold), manifold.Contact0Exists);
                manifold.Contact1Exists = Vector.ConditionalSelect(useSide, Vector.BitwiseAnd(Vector.GreaterThan(depth1, depthThreshold), Vector.GreaterThan(tMax, tMin)), manifold.Contact1Exists);
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