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
        public static void CreateEffectiveTriangleFaceNormal(in Vector3Wide triangleNormal, in Vector3Wide normal, Vector<float> faceNormalADotNormal, Vector<int> inactiveLanes,
            out Vector3Wide effectiveFaceNormal, out Vector<float> inverseEffectiveFaceNormalDotNormal)
        {
            //NOTE: Triangle normal is expected by convention to be pointing in the direction of the backface.
            //When the contact normal is nearly perpendicular to the triangle normal, computing per contact depths can become numerically impossible.
            //To help with this case, we use an 'effective' triangle face normal for anything related to computing contact depths.
            var absFaceNormalADotNormal = Vector.Abs(faceNormalADotNormal);
            const float faceNormalFallbackThreshold = 1e-4f;
            var needsFallbackFaceNormal = Vector.AndNot(Vector.LessThan(absFaceNormalADotNormal, new Vector<float>(faceNormalFallbackThreshold)), inactiveLanes);
            if (Vector.EqualsAny(needsFallbackFaceNormal, new Vector<int>(-1)))
            {
                //Near-zero faceNormalADotNormal values only occur during edge or vertex collisions.
                //During edge collisions, the local normal is perpendicular to the edge.
                //We can safely push the triangle normal along the local normal to get an 'effective' normal that doesn't cause numerical catastrophe.
                var pushScale = Vector.Max(Vector<float>.Zero, (absFaceNormalADotNormal - new Vector<float>(faceNormalFallbackThreshold)) / new Vector<float>(-faceNormalFallbackThreshold));
                Vector3Wide.Scale(normal, pushScale * new Vector<float>(0.1f), out var effectiveFaceNormalPush);
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

            }
            //Given the push above, this is guaranteed not to divide by zero.
            Vector3Wide.Dot(effectiveFaceNormal, normal, out var effectiveFaceNormalDotNormal);
            inverseEffectiveFaceNormalDotNormal = Vector<float>.One / effectiveFaceNormalDotNormal;
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

            var inactiveLanes = BundleIndexing.CreateTrailingMaskForCountInBundle(pairCount);
            TriangleWide.ComputeNondegenerateTriangleMask(triangleAB, triangleCA, triangleNormalLength, out var triangleEpsilonScale, out var nondegenerateMask);
            inactiveLanes = Vector.BitwiseOr(Vector.OnesComplement(nondegenerateMask), inactiveLanes);
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
            inactiveLanes = Vector.BitwiseOr(inactiveLanes, Vector.BitwiseOr(Vector.GreaterThan(faceNormalADotNormal, new Vector<float>(-TriangleWide.BackfaceNormalDotRejectionThreshold)), Vector.LessThan(depth, depthThreshold)));
            if (Vector.LessThanAll(inactiveLanes, Vector<int>.Zero))
            {
                //All lanes are either inactive or were found to have a depth lower than the speculative margin, so we can just quit early.
                manifold = default;
                return;
            }

            //Swap over to an edge case if the normal is not face aligned. For other shapes we tend to take the closest feature regardless, but here we're favoring the face a bit more by choosing a lower threshold.
            var useTriangleEdgeCase = Vector.AndNot(Vector.LessThan(Vector.Abs(faceNormalADotNormal), new Vector<float>(0.2f)), inactiveLanes);

            //We generate contacts according to the dominant features along the collision normal.
            var capCenterBY = Vector.ConditionalSelect(Vector.LessThan(localNormal.Y, Vector<float>.Zero), -b.HalfLength, b.HalfLength);

            var useCap = Vector.AndNot(Vector.GreaterThan(Vector.Abs(localNormal.Y), new Vector<float>(0.70710678118f)), inactiveLanes);

            Unsafe.SkipInit(out Vector3Wide localOffsetB0);
            Unsafe.SkipInit(out Vector3Wide localOffsetB1);
            Unsafe.SkipInit(out Vector3Wide localOffsetB2);
            Unsafe.SkipInit(out Vector3Wide localOffsetB3);
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

                //While we projected onto the cylinder cap to do the triangle edge intersections, we use the triangle to create the contact manifold.
                //There is no guaranteed projection from the cap to the triangle face since the normal can become perpendicular, but we can always measure from triangle to cylinder cap.
                //Arbitrarily pick the triangle center as the origin of the tangent space, and AB as the x axis.
                Vector3Wide.Length(triangleAB, out var triangleABLength);
                Vector3Wide.Scale(triangleAB, Vector<float>.One / triangleABLength, out var triangleTangentX);
                Vector3Wide.CrossWithoutOverlap(triangleTangentX, triangleNormal, out var triangleTangentY);

                Vector2Wide tangentA, tangentB, tangentC;
                Vector3Wide.Dot(triangle.A, triangleTangentX, out tangentA.X);
                Vector3Wide.Dot(triangle.A, triangleTangentY, out tangentA.Y);
                Vector3Wide.Dot(triangle.B, triangleTangentX, out tangentB.X);
                Vector3Wide.Dot(triangle.B, triangleTangentY, out tangentB.Y);
                Vector3Wide.Dot(triangle.C, triangleTangentX, out tangentC.X);
                Vector3Wide.Dot(triangle.C, triangleTangentY, out tangentC.Y);

                Vector2Wide.Subtract(tangentB, tangentA, out var tangentAB);
                Vector2Wide.Subtract(tangentC, tangentB, out var tangentBC);
                Vector2Wide.Subtract(tangentA, tangentC, out var tangentCA);

                BoxCylinderTester.AddCandidateForEdge(tangentA, tangentAB, tMinAB, tMaxAB, intersectedAB, Vector<int>.Zero, useCap, pairCount, ref candidates, ref candidateCount);
                BoxCylinderTester.AddCandidateForEdge(tangentB, tangentBC, tMinBC, tMaxBC, intersectedBC, Vector<int>.One, useCap, pairCount, ref candidates, ref candidateCount);
                BoxCylinderTester.AddCandidateForEdge(tangentC, tangentCA, tMinCA, tMaxCA, intersectedCA, new Vector<int>(2), useCap, pairCount, ref candidates, ref candidateCount);

                var useCapTriangleFace = Vector.AndNot(useCap, useTriangleEdgeCase);
                int maximumContactCountInBundle = 6;
                if (Vector.LessThanAny(useCapTriangleFace, Vector<int>.Zero))
                {
                    //Project the points on the cylinder down to the triangle. Note that this is only valid if the normal is not perpendicular to the face normal.
                    maximumContactCountInBundle = 10;
                    BoxCylinderTester.GenerateInteriorPoints(b, localNormal, closestOnB, out var interiorOnCylinder0, out var interiorOnCylinder1, out var interiorOnCylinder2, out var interiorOnCylinder3);
                    //pointOnTrianglePlane = pointOnCylinder + localNormal * t
                    //y = sign(localNormal.Y) * b.HalfLength
                    //pointOnCylinder = (interiorOnCylinderN.X, y, interiorOnCylinderN.Y)
                    //t = dot(localTriangleCenter - pointOnCylinder, triangleNormal) / dot(triangleNormal, localNormal)
                    var inverseDenominator = new Vector<float>(-1f) / faceNormalADotNormal;
                    var yOffset = localTriangleCenter.Y - capCenterBY;
                    var xOffset0 = localTriangleCenter.X - interiorOnCylinder0.X;
                    var zOffset0 = localTriangleCenter.Z - interiorOnCylinder0.Y;
                    var xOffset1 = localTriangleCenter.X - interiorOnCylinder1.X;
                    var zOffset1 = localTriangleCenter.Z - interiorOnCylinder1.Y;
                    var xOffset2 = localTriangleCenter.X - interiorOnCylinder2.X;
                    var zOffset2 = localTriangleCenter.Z - interiorOnCylinder2.Y;
                    var xOffset3 = localTriangleCenter.X - interiorOnCylinder3.X;
                    var zOffset3 = localTriangleCenter.Z - interiorOnCylinder3.Y;
                    var t0 = (xOffset0 * localNormal.X + yOffset * localNormal.Y + zOffset0 * localNormal.Z) * inverseDenominator;
                    var t1 = (xOffset1 * localNormal.X + yOffset * localNormal.Y + zOffset1 * localNormal.Z) * inverseDenominator;
                    var t2 = (xOffset2 * localNormal.X + yOffset * localNormal.Y + zOffset2 * localNormal.Z) * inverseDenominator;
                    var t3 = (xOffset3 * localNormal.X + yOffset * localNormal.Y + zOffset3 * localNormal.Z) * inverseDenominator;
                    //Projecting into the triangle's *tangent space* directly.
                    //pointInTriangleTangentSpace = (dot(pointOnCylinder + localNormal * t, tangentX), dot(pointOnCylinder + localNormal * t, tangentY))
                    Vector2Wide tangentLocalNormal;
                    Vector3Wide.Dot(localNormal, triangleTangentX, out tangentLocalNormal.X);
                    Vector3Wide.Dot(localNormal, triangleTangentY, out tangentLocalNormal.Y);
                    Vector2Wide interior0, interior1, interior2, interior3;
                    var yOnTangentX = yOffset * triangleTangentX.Y;
                    var yOnTangentY = yOffset * triangleTangentY.Y;
                    interior0.X = tangentLocalNormal.X * t0 - xOffset0 * triangleTangentX.X - yOnTangentX - zOffset0 * triangleTangentX.Z;
                    interior0.Y = tangentLocalNormal.Y * t0 - xOffset0 * triangleTangentY.X - yOnTangentY - zOffset0 * triangleTangentY.Z;
                    interior1.X = tangentLocalNormal.X * t1 - xOffset1 * triangleTangentX.X - yOnTangentX - zOffset1 * triangleTangentX.Z;
                    interior1.Y = tangentLocalNormal.Y * t1 - xOffset1 * triangleTangentY.X - yOnTangentY - zOffset1 * triangleTangentY.Z;
                    interior2.X = tangentLocalNormal.X * t2 - xOffset2 * triangleTangentX.X - yOnTangentX - zOffset2 * triangleTangentX.Z;
                    interior2.Y = tangentLocalNormal.Y * t2 - xOffset2 * triangleTangentY.X - yOnTangentY - zOffset2 * triangleTangentY.Z;
                    interior3.X = tangentLocalNormal.X * t3 - xOffset3 * triangleTangentX.X - yOnTangentX - zOffset3 * triangleTangentX.Z;
                    interior3.Y = tangentLocalNormal.Y * t3 - xOffset3 * triangleTangentY.X - yOnTangentY - zOffset3 * triangleTangentY.Z;

                    //Test the four points against the edge plane. Note that signs depend on the orientation of the cylinder.
                    TryAddInteriorPoint(interior0, new Vector<int>(8), tangentA, tangentAB, tangentB, tangentBC, tangentC, tangentCA, useCapTriangleFace, ref candidates, ref candidateCount, pairCount);
                    TryAddInteriorPoint(interior1, new Vector<int>(9), tangentA, tangentAB, tangentB, tangentBC, tangentC, tangentCA, useCapTriangleFace, ref candidates, ref candidateCount, pairCount);
                    TryAddInteriorPoint(interior2, new Vector<int>(10), tangentA, tangentAB, tangentB, tangentBC, tangentC, tangentCA, useCapTriangleFace, ref candidates, ref candidateCount, pairCount);
                    TryAddInteriorPoint(interior3, new Vector<int>(11), tangentA, tangentAB, tangentB, tangentBC, tangentC, tangentCA, useCapTriangleFace, ref candidates, ref candidateCount, pairCount);
                }

                Vector3Wide capNormal;
                capNormal.X = Vector<float>.Zero;
                capNormal.Y = Vector.ConditionalSelect(Vector.LessThan(localNormal.Y, Vector<float>.Zero), Vector<float>.One, new Vector<float>(-1));
                capNormal.Z = Vector<float>.Zero;
                Vector3Wide triangleCenterToCapCenter;
                triangleCenterToCapCenter.X = -localTriangleCenter.X;
                triangleCenterToCapCenter.Y = capCenterBY - localTriangleCenter.Y;
                triangleCenterToCapCenter.Z = -localTriangleCenter.Z;
                ManifoldCandidateHelper.Reduce(ref candidates, candidateCount, maximumContactCountInBundle, capNormal, -capNormal.Y / localNormal.Y, triangleCenterToCapCenter, triangleTangentX, triangleTangentY, epsilonScale, depthThreshold, pairCount,
                    out var candidate0, out var candidate1, out var candidate2, out var candidate3,
                    out manifold.Contact0Exists, out manifold.Contact1Exists, out manifold.Contact2Exists, out manifold.Contact3Exists);

                localOffsetB0.X = triangleTangentX.X * candidate0.X + triangleTangentY.X * candidate0.Y + localTriangleCenter.X;
                localOffsetB0.Y = triangleTangentX.Y * candidate0.X + triangleTangentY.Y * candidate0.Y + localTriangleCenter.Y;
                localOffsetB0.Z = triangleTangentX.Z * candidate0.X + triangleTangentY.Z * candidate0.Y + localTriangleCenter.Z;
                localOffsetB1.X = triangleTangentX.X * candidate1.X + triangleTangentY.X * candidate1.Y + localTriangleCenter.X;
                localOffsetB1.Y = triangleTangentX.Y * candidate1.X + triangleTangentY.Y * candidate1.Y + localTriangleCenter.Y;
                localOffsetB1.Z = triangleTangentX.Z * candidate1.X + triangleTangentY.Z * candidate1.Y + localTriangleCenter.Z;
                localOffsetB2.X = triangleTangentX.X * candidate2.X + triangleTangentY.X * candidate2.Y + localTriangleCenter.X;
                localOffsetB2.Y = triangleTangentX.Y * candidate2.X + triangleTangentY.Y * candidate2.Y + localTriangleCenter.Y;
                localOffsetB2.Z = triangleTangentX.Z * candidate2.X + triangleTangentY.Z * candidate2.Y + localTriangleCenter.Z;
                localOffsetB3.X = triangleTangentX.X * candidate3.X + triangleTangentY.X * candidate3.Y + localTriangleCenter.X;
                localOffsetB3.Y = triangleTangentX.Y * candidate3.X + triangleTangentY.Y * candidate3.Y + localTriangleCenter.Y;
                localOffsetB3.Z = triangleTangentX.Z * candidate3.X + triangleTangentY.Z * candidate3.Y + localTriangleCenter.Z;

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
                var useSideEdgeCase = Vector.BitwiseAnd(useSide, useTriangleEdgeCase);
                Unsafe.SkipInit(out Vector<float> depthTMin);
                Unsafe.SkipInit(out Vector<float> depthTMax);
                Unsafe.SkipInit(out Vector<float> cylinderTMin);
                Unsafe.SkipInit(out Vector<float> cylinderTMax);

                //At least one lane is going to use the side edge case.
                //Identify the dominant edge based on alignment with the local normal.
                Vector3Wide.Dot(edgePlaneAB, localNormal, out var abEdgeAlignment);
                Vector3Wide.Dot(edgePlaneBC, localNormal, out var bcEdgeAlignment);
                Vector3Wide.Dot(edgePlaneCA, localNormal, out var caEdgeAlignment);

                var max = Vector.Max(abEdgeAlignment, Vector.Max(bcEdgeAlignment, caEdgeAlignment));
                var abIsDominant = Vector.Equals(max, abEdgeAlignment);
                var bcIsDominant = Vector.Equals(max, bcEdgeAlignment);

                Vector3Wide.ConditionalSelect(abIsDominant, triangleA, triangleC, out var dominantEdgeStart);
                Vector3Wide.ConditionalSelect(bcIsDominant, triangleB, dominantEdgeStart, out dominantEdgeStart);
                Vector3Wide.ConditionalSelect(abIsDominant, triangleAB, triangleCA, out var dominantEdgeOffset);
                Vector3Wide.ConditionalSelect(bcIsDominant, triangleBC, dominantEdgeOffset, out dominantEdgeOffset);

                //If the contacts are near an edge and the cylinder side is aligned with that edge, it can lead to numerical blippyblips in contact positions.
                //In this case, expand the contact interval for the parallel edge (so either the cylinder side endpoints or the other triangle edges will bound the interval).
                //sin(angle between triangle edge direction and cylinder edge direction) = dot(dominantEdgeOffset, (-localNormal.Z, 0, localNormal.X)) / (||dominantEdgeOffset|| * ||(-localNormal.Z, 0, localNormal.X)||)
                //sinAngle * (||dominantEdgeOffset|| * ||(-localNormal.Z, 0, localNormal.X)||) = dot(dominantEdgeOffset, (-localNormal.Z, 0, localNormal.X))
                //Not concerned about sign; either way works, and we don't really care about perfectly linear interpolation... so square it.
                //sinAngle^2 * ||dominantEdgeOffset||^2 * ||(-localNormal.Z, 0, localNormal.X)||^2 = dot(dominantEdgeOffset, (-localNormal.Z, 0, localNormal.X))^2         
                const float lowerSinAngleThreshold = 0.01f;
                const float upperSinAngleThreshold = 0.02f;
                var dominantEdgeDotHorizontalNormal = dominantEdgeOffset.Z * localNormal.X - dominantEdgeOffset.X * localNormal.Z;
                var dominantEdgeDotHorizontalNormalSquared = dominantEdgeDotHorizontalNormal * dominantEdgeDotHorizontalNormal;
                Vector3Wide.LengthSquared(dominantEdgeOffset, out var dominantEdgeLengthSquared);
                var horizontalNormalLengthSquared = localNormal.X * localNormal.X + localNormal.Z * localNormal.Z;
                var interpolationScale = dominantEdgeLengthSquared * horizontalNormalLengthSquared;
                var interpolationMin = new Vector<float>(lowerSinAngleThreshold * lowerSinAngleThreshold);
                var inverseInterpolationSpan = new Vector<float>(1f / (upperSinAngleThreshold * upperSinAngleThreshold - lowerSinAngleThreshold * lowerSinAngleThreshold));
                var restrictWeight = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, (dominantEdgeDotHorizontalNormalSquared / interpolationScale - interpolationMin) * inverseInterpolationSpan));

                //Either triangle edge-cylinder side, or triangle face-cylinder side.
                if (Vector.LessThanAny(useSideEdgeCase, Vector<int>.Zero))
                {
                    //At least one lane is edge-side. This is relatively simple; we need only test the dominant edge of the triangle with the side of the cylinder.
                    //If the triangle edge and cylinder edge are close to parallel, expand the intersection interval so we can generate more than one contact.
                    //For the single contact case, all we need is to test the triangle edge versus the plane formed by the cylinder side edge and the local normal.
                    //(0,1,0) x localNormal = (-localNormal.Z, 0, localNormal.X)
                    var cylinderEdgeToDominantEdgeStartX = dominantEdgeStart.X - closestOnB.X;
                    var cylinderEdgeToDominantEdgeStartZ = dominantEdgeStart.Z - closestOnB.Z;
                    var numerator = cylinderEdgeToDominantEdgeStartX * localNormal.Z - cylinderEdgeToDominantEdgeStartZ * localNormal.X;
                    var edgeT = numerator / dominantEdgeDotHorizontalNormal;

                    //As the unrestrict weight increases, expand the interval on the triangle edge until it covers the entire cylinder edge.
                    //Past the upper bound, we no longer consider the single contact edgeT, since it will be approaching a singularity.
                    //tCenter = dot(cylinderSideEdgeCenter - dominantEdgeStart, dominantEdgeOffset / ||dominantEdgeOffset||^2)
                    //tMax = tCenter + cylinder.HalfLength * dominantEdgeOffset.Y / ||dominantEdgeOffset||^2
                    var inverseEdgeOffsetLengthSquared = Vector<float>.One / dominantEdgeLengthSquared;
                    var tCenter = -(cylinderEdgeToDominantEdgeStartX * dominantEdgeOffset.X + dominantEdgeStart.Y * dominantEdgeOffset.Y + cylinderEdgeToDominantEdgeStartZ * dominantEdgeOffset.Z) * inverseEdgeOffsetLengthSquared;
                    var projectedExtentOffset = b.HalfLength * Vector.Abs(dominantEdgeOffset.Y) * inverseEdgeOffsetLengthSquared;
                    cylinderTMin = tCenter - projectedExtentOffset;
                    cylinderTMax = tCenter + projectedExtentOffset;
                    //Note that the edgeT value is ignored once the denominator is small enough. Avoids division by zero propagation.
                    var regularContribution = restrictWeight * Vector.ConditionalSelect(Vector.LessThan(dominantEdgeDotHorizontalNormalSquared, interpolationMin), tCenter, edgeT);
                    var unrestrictWeight = Vector<float>.One - restrictWeight;
                    cylinderTMin = regularContribution + unrestrictWeight * cylinderTMin;
                    cylinderTMax = regularContribution + unrestrictWeight * cylinderTMax;

                    cylinderTMin = Vector.Min(Vector.Max(Vector<float>.Zero, cylinderTMin), Vector<float>.One);
                    cylinderTMax = Vector.Min(Vector.Max(Vector<float>.Zero, cylinderTMax), Vector<float>.One);

                    //Compute depth by projecting back to the cylinder plane. 
                    //Its normal is the horizontal local normal.
                    //depth = dot(dominantEdgeStart + dominantEdgeOffset * t - cylinderEdgeCenter, horizontalLocalNormal) / dot(horizontalLocalNormal, localNormal)
                    //dot(horizontalLocalNormal, localNormal) == localNormal.X^2 + localNormal.Z^2
                    //depth = (dot(dominantEdgeStart - cylinderEdgeCenter, horizontalLocalNormal) + t * (dominantEdgeOffset, horizontalLocalNormal)) / (localNormal.X^2 + localNormal.Z^2)
                    var inverseDepthDenominator = new Vector<float>(-1f) / horizontalNormalLengthSquared;
                    var depthBase = (cylinderEdgeToDominantEdgeStartX * localNormal.X + cylinderEdgeToDominantEdgeStartZ * localNormal.Z) * inverseDepthDenominator;
                    var tDepthScale = (dominantEdgeOffset.X * localNormal.X + dominantEdgeOffset.Z * localNormal.Z) * inverseDepthDenominator;
                    depthTMin = depthBase + tDepthScale * cylinderTMin;
                    depthTMax = depthBase + tDepthScale * cylinderTMax;

                    Vector3Wide.Scale(dominantEdgeOffset, cylinderTMin, out var minOffset);
                    Vector3Wide.Scale(dominantEdgeOffset, cylinderTMax, out var maxOffset);
                    localOffsetB0.X = Vector.ConditionalSelect(useSideEdgeCase, dominantEdgeStart.X + minOffset.X, localOffsetB0.X);
                    localOffsetB0.Y = Vector.ConditionalSelect(useSideEdgeCase, dominantEdgeStart.Y + minOffset.Y, localOffsetB0.Y);
                    localOffsetB0.Z = Vector.ConditionalSelect(useSideEdgeCase, dominantEdgeStart.Z + minOffset.Z, localOffsetB0.Z);
                    localOffsetB1.X = Vector.ConditionalSelect(useSideEdgeCase, dominantEdgeStart.X + maxOffset.X, localOffsetB1.X);
                    localOffsetB1.Y = Vector.ConditionalSelect(useSideEdgeCase, dominantEdgeStart.Y + maxOffset.Y, localOffsetB1.Y);
                    localOffsetB1.Z = Vector.ConditionalSelect(useSideEdgeCase, dominantEdgeStart.Z + maxOffset.Z, localOffsetB1.Z);

                }
                var useSideTriangleFace = Vector.AndNot(useSide, useTriangleEdgeCase);
                if (Vector.LessThanAny(useSideTriangleFace, Vector<int>.Zero))
                {
                    //Project the cylinder edge down onto the triangle plane. We know it's numerically valid because useTriangleEdgeCase is false for this lane.
                    //tCenterToTriangleSurface = dot(localTriangleCenter - (closestOnB.X, 0, closestOnB.Z), triangleNormal) / dot(localNormal, triangleNormal)
                    //Note that we *could* use the computed depth for the closest point here, but that would allow some numerical error to compound and it only saves us a single dot product.
                    var inverseDenominator = Vector<float>.One / faceNormalADotNormal;
                    var xzContribution = (localTriangleCenter.X - closestOnB.X) * triangleNormal.X + (localTriangleCenter.Z - closestOnB.Z) * triangleNormal.Z;
                    var tMinToTriangle = (xzContribution + (localTriangleCenter.Y + b.HalfLength) * triangleNormal.Y) * inverseDenominator;
                    var tMaxToTriangle = (xzContribution + (localTriangleCenter.Y - b.HalfLength) * triangleNormal.Y) * inverseDenominator;
                    Vector3Wide minOnTriangle, maxOnTriangle;
                    minOnTriangle.X = tMinToTriangle * localNormal.X + closestOnB.X;
                    minOnTriangle.Y = tMinToTriangle * localNormal.Y - b.HalfLength;
                    minOnTriangle.Z = tMinToTriangle * localNormal.Z + closestOnB.Z;
                    maxOnTriangle.X = tMaxToTriangle * localNormal.X + closestOnB.X;
                    maxOnTriangle.Y = tMaxToTriangle * localNormal.Y + b.HalfLength;
                    maxOnTriangle.Z = tMaxToTriangle * localNormal.Z + closestOnB.Z;
                    Vector3Wide.Subtract(maxOnTriangle, minOnTriangle, out var minToMax);
                    //We now have points on the surface of the triangle. Use them as a ray to intersect the triangle's edge planes.
                    var numeratorAB = (triangleA.X - minOnTriangle.X) * edgePlaneAB.X + (triangleA.Y - minOnTriangle.Y) * edgePlaneAB.Y + (triangleA.Z - minOnTriangle.Z) * edgePlaneAB.Z;
                    var numeratorBC = (triangleB.X - minOnTriangle.X) * edgePlaneBC.X + (triangleB.Y - minOnTriangle.Y) * edgePlaneBC.Y + (triangleB.Z - minOnTriangle.Z) * edgePlaneBC.Z;
                    var numeratorCA = (triangleC.X - minOnTriangle.X) * edgePlaneCA.X + (triangleC.Y - minOnTriangle.Y) * edgePlaneCA.Y + (triangleC.Z - minOnTriangle.Z) * edgePlaneCA.Z;
                    var denominatorAB = minToMax.X * edgePlaneAB.X + minToMax.Y * edgePlaneAB.Y + minToMax.Z * edgePlaneAB.Z;
                    var denominatorBC = minToMax.X * edgePlaneBC.X + minToMax.Y * edgePlaneBC.Y + minToMax.Z * edgePlaneBC.Z;
                    var denominatorCA = minToMax.X * edgePlaneCA.X + minToMax.Y * edgePlaneCA.Y + minToMax.Z * edgePlaneCA.Z;
                    //Protect against division by zero. This preserves sign and allows values to go to relatively enormous values.
                    var threshold = new Vector<float>(1e-30f);
                    var negativeThreshold = -threshold;
                    //A ray is 'exiting' if the ray's direction is aligned with the edge normal, exiting otherwise.
                    var exitingAB = Vector.LessThanOrEqual(denominatorAB, Vector<float>.Zero);
                    var exitingBC = Vector.LessThanOrEqual(denominatorBC, Vector<float>.Zero);
                    var exitingCA = Vector.LessThanOrEqual(denominatorCA, Vector<float>.Zero);
                    denominatorAB = Vector.ConditionalSelect(Vector.LessThan(Vector.Abs(denominatorAB), threshold), Vector.ConditionalSelect(exitingAB, negativeThreshold, threshold), denominatorAB);
                    denominatorBC = Vector.ConditionalSelect(Vector.LessThan(Vector.Abs(denominatorBC), threshold), Vector.ConditionalSelect(exitingBC, negativeThreshold, threshold), denominatorBC);
                    denominatorCA = Vector.ConditionalSelect(Vector.LessThan(Vector.Abs(denominatorCA), threshold), Vector.ConditionalSelect(exitingCA, negativeThreshold, threshold), denominatorCA);
                    var edgeTAB = numeratorAB / denominatorAB;
                    var edgeTBC = numeratorBC / denominatorBC;
                    var edgeTCA = numeratorCA / denominatorCA;

                    //Take the first exit and last entry to create the interval of intersection.
                    var minValue = new Vector<float>(float.MinValue);
                    var maxValue = new Vector<float>(float.MaxValue);
                    var entryAB = Vector.ConditionalSelect(exitingAB, minValue, edgeTAB);
                    var entryBC = Vector.ConditionalSelect(exitingBC, minValue, edgeTBC);
                    var entryCA = Vector.ConditionalSelect(exitingCA, minValue, edgeTCA);
                    var exitAB = Vector.ConditionalSelect(exitingAB, edgeTAB, maxValue);
                    var exitBC = Vector.ConditionalSelect(exitingBC, edgeTBC, maxValue);
                    var exitCA = Vector.ConditionalSelect(exitingCA, edgeTCA, maxValue);

                    //If the dominant edge is parallel with the cylinder side edge, then unrestrict the interval.
                    //Entry T values are unrestricted to 0, exit T values are unrestricted to 1.
                    var caIsDominant = Vector.AndNot(Vector.OnesComplement(abIsDominant), bcIsDominant);
                    entryAB = Vector.ConditionalSelect(abIsDominant, entryAB * restrictWeight, entryAB);
                    entryBC = Vector.ConditionalSelect(bcIsDominant, entryBC * restrictWeight, entryBC);
                    entryCA = Vector.ConditionalSelect(caIsDominant, entryCA * restrictWeight, entryCA);
                    var unrestrictWeight = Vector<float>.One - restrictWeight;
                    exitAB = Vector.ConditionalSelect(abIsDominant, exitAB * restrictWeight + unrestrictWeight, exitAB);
                    exitBC = Vector.ConditionalSelect(bcIsDominant, exitBC * restrictWeight + unrestrictWeight, exitBC);
                    exitCA = Vector.ConditionalSelect(caIsDominant, exitCA * restrictWeight + unrestrictWeight, exitCA);

                    var sideTriangleCylinderTMin = Vector.Max(entryAB, Vector.Max(entryBC, entryCA));
                    var sideTriangleCylinderTMax = Vector.Min(exitAB, Vector.Min(exitBC, exitCA));

                    //Note that the local normal should have been created such that projecting the cylinder edge down along it *should * result in an intersection with the triangle.
                    //That would mean exitT >= entryT.However, due to numerical error, that is not strictly guaranteed.
                    //This will typically happen in a vertex case.
                    //We can choose the vertex in these cases by examining which edges contributed the intersections forming the bounds of the interval.
                    var useVertexFallback = Vector.BitwiseAnd(Vector.LessThan(sideTriangleCylinderTMax, sideTriangleCylinderTMin), useSideTriangleFace);
                    var abContributedBound = Vector.BitwiseOr(Vector.Equals(edgeTAB, sideTriangleCylinderTMin), Vector.Equals(edgeTAB, sideTriangleCylinderTMax));
                    var bcContributedBound = Vector.BitwiseOr(Vector.Equals(edgeTBC, sideTriangleCylinderTMin), Vector.Equals(edgeTBC, sideTriangleCylinderTMax));
                    var caContributedBound = Vector.BitwiseOr(Vector.Equals(edgeTCA, sideTriangleCylinderTMin), Vector.Equals(edgeTCA, sideTriangleCylinderTMax));
                    var useA = Vector.BitwiseAnd(caContributedBound, abContributedBound);
                    var useB = Vector.BitwiseAnd(abContributedBound, bcContributedBound);
                    //var useC = Vector.BitwiseAnd(bcContributedBound, caContributedBound);
                    Vector3Wide.ConditionalSelect(useA, triangleA, triangleC, out var vertexFallback);
                    Vector3Wide.ConditionalSelect(useB, triangleB, vertexFallback, out vertexFallback);

                    //Bound the interval to the cylinder's extent.
                    cylinderTMin = Vector.ConditionalSelect(useSideTriangleFace, Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, sideTriangleCylinderTMin)), cylinderTMin);
                    cylinderTMax = Vector.ConditionalSelect(useSideTriangleFace, Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, sideTriangleCylinderTMax)), cylinderTMax);
                    localOffsetB0.X = Vector.ConditionalSelect(useSideTriangleFace, minOnTriangle.X + minToMax.X * cylinderTMin, localOffsetB0.X);
                    localOffsetB0.Y = Vector.ConditionalSelect(useSideTriangleFace, minOnTriangle.Y + minToMax.Y * cylinderTMin, localOffsetB0.Y);
                    localOffsetB0.Z = Vector.ConditionalSelect(useSideTriangleFace, minOnTriangle.Z + minToMax.Z * cylinderTMin, localOffsetB0.Z);
                    localOffsetB1.X = Vector.ConditionalSelect(useSideTriangleFace, minOnTriangle.X + minToMax.X * cylinderTMax, localOffsetB1.X);
                    localOffsetB1.Y = Vector.ConditionalSelect(useSideTriangleFace, minOnTriangle.Y + minToMax.Y * cylinderTMax, localOffsetB1.Y);
                    localOffsetB1.Z = Vector.ConditionalSelect(useSideTriangleFace, minOnTriangle.Z + minToMax.Z * cylinderTMax, localOffsetB1.Z);

                    Vector3Wide.ConditionalSelect(useVertexFallback, vertexFallback, localOffsetB0, out localOffsetB0);

                    //Ray cast back to the cylinder's side to compute the depth for the contact.
                    //t = dot(localNormal.X0Z, cylinderSideEdgeCenter - {entry, exit}) / dot(localNormal.X0Z, localNormal)
                    var inverseDepthDenominator = Vector<float>.One / (localNormal.X * localNormal.X + localNormal.Z * localNormal.Z);
                    depthTMin = Vector.ConditionalSelect(useSideTriangleFace, (localNormal.X * (closestOnB.X - localOffsetB0.X) + localNormal.Z * (closestOnB.Z - localOffsetB0.Z)) * inverseDepthDenominator, depthTMin);
                    depthTMax = Vector.ConditionalSelect(useSideTriangleFace, (localNormal.X * (closestOnB.X - localOffsetB1.X) + localNormal.Z * (closestOnB.Z - localOffsetB1.Z)) * inverseDepthDenominator, depthTMax);
                }
                manifold.FeatureId0 = Vector.ConditionalSelect(useSide, Vector<int>.Zero, manifold.FeatureId0);
                manifold.FeatureId1 = Vector.ConditionalSelect(useSide, Vector<int>.One, manifold.FeatureId1);
                manifold.Depth0 = Vector.ConditionalSelect(useSide, depthTMin, manifold.Depth0);
                manifold.Depth1 = Vector.ConditionalSelect(useSide, depthTMax, manifold.Depth1);
                manifold.Contact0Exists = Vector.ConditionalSelect(useSide, Vector.GreaterThan(depthTMin, depthThreshold), manifold.Contact0Exists);
                manifold.Contact1Exists = Vector.ConditionalSelect(useSide, Vector.BitwiseAnd(Vector.GreaterThan(depthTMax, depthThreshold), Vector.GreaterThan(cylinderTMax, cylinderTMin)), manifold.Contact1Exists);
                manifold.Contact2Exists = Vector.ConditionalSelect(useSide, Vector<int>.Zero, manifold.Contact2Exists);
                manifold.Contact3Exists = Vector.ConditionalSelect(useSide, Vector<int>.Zero, manifold.Contact3Exists);
            }

            Matrix3x3Wide.TransformWithoutOverlap(localNormal, worldRB, out manifold.Normal);

            Vector3Wide.Add(localOffsetB0, localOffsetB, out var localOffsetA0);
            Vector3Wide.Add(localOffsetB1, localOffsetB, out var localOffsetA1);
            Vector3Wide.Add(localOffsetB2, localOffsetB, out var localOffsetA2);
            Vector3Wide.Add(localOffsetB3, localOffsetB, out var localOffsetA3);
            Matrix3x3Wide.TransformWithoutOverlap(localOffsetA0, worldRB, out manifold.OffsetA0);
            Matrix3x3Wide.TransformWithoutOverlap(localOffsetA1, worldRB, out manifold.OffsetA1);
            Matrix3x3Wide.TransformWithoutOverlap(localOffsetA2, worldRB, out manifold.OffsetA2);
            Matrix3x3Wide.TransformWithoutOverlap(localOffsetA3, worldRB, out manifold.OffsetA3);

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