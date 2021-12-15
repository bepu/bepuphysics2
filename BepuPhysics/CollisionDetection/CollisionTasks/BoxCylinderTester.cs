using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.SweepTasks;
using BepuUtilities;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    using DepthRefiner = DepthRefiner<Cylinder, CylinderWide, CylinderSupportFinder, Box, BoxWide, BoxSupportFinder>;
    public struct BoxCylinderTester : IPairTester<BoxWide, CylinderWide, Convex4ContactManifoldWide>
    {
        public int BatchSize => 16;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void IntersectLineCircle(in Vector2Wide linePosition, in Vector2Wide lineDirection, in Vector<float> radius, out Vector<float> tMin, out Vector<float> tMax, out Vector<int> intersected)
        {
            //||linePosition + lineDirection * t|| = radius
            //dot(linePosition + lineDirection * t, linePosition + lineDirection * t) = radius * radius
            //dot(linePosition, linePosition) - radius * radius + t * 2 * dot(linePosition, lineDirection) + t^2 * dot(lineDirection, lineDirection) = 0
            Vector2Wide.Dot(lineDirection, lineDirection, out var a);
            a = Vector.Max(a, new Vector<float>(2e-38f)); //Guard against division by zero.
            var inverseA = Vector<float>.One / a;
            Vector2Wide.Dot(linePosition, lineDirection, out var b);
            Vector2Wide.Dot(linePosition, linePosition, out var c);
            var radiusSquared = radius * radius;
            c -= radiusSquared;
            var d = b * b - a * c;
            intersected = Vector.GreaterThanOrEqual(d, Vector<float>.Zero);
            var tOffset = Vector.SquareRoot(Vector.Max(Vector<float>.Zero, d)) * inverseA;
            var tBase = -b * inverseA;
            tMin = tBase - tOffset;
            tMax = tBase + tOffset;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void AddCandidateForEdge(in Vector2Wide edgeStart, in Vector2Wide edgeOffset, in Vector<float> tMin, in Vector<float> tMax, in Vector<int> intersected, in Vector<int> edgeId, in Vector<int> allowFeatureContacts, int pairCount,
                 ref ManifoldCandidate candidates, ref Vector<int> candidateCount)
        {
            //We're going to be a little lazy with feature ids. If the box face changes, these will get partially invalidated even if an edge survived.
            //There's an edge id (0, 1, 2, 3), and then a flag for min or max (0 or 4).
            ManifoldCandidate candidate;
            candidate.Depth = default;
            candidate.FeatureId = edgeId;
            candidate.X = edgeStart.X + edgeOffset.X * tMin;
            candidate.Y = edgeStart.Y + edgeOffset.Y * tMin;
            var allowContacts = Vector.BitwiseAnd(intersected, allowFeatureContacts);
            //If tMin is overlapping the previous edge's tMax (or this edge's tMax!), don't bother including it.
            ManifoldCandidateHelper.AddCandidate(ref candidates, ref candidateCount, candidate,
                Vector.BitwiseAnd(allowContacts, Vector.BitwiseAnd(Vector.LessThan(tMin, tMax), Vector.GreaterThan(tMin, Vector<float>.Zero))), pairCount);
            candidate.FeatureId = edgeId + new Vector<int>(4);
            candidate.X = edgeStart.X + edgeOffset.X * tMax;
            candidate.Y = edgeStart.Y + edgeOffset.Y * tMax;
            ManifoldCandidateHelper.AddCandidate(ref candidates, ref candidateCount, candidate,
                Vector.BitwiseAnd(allowContacts, Vector.GreaterThan(tMax, Vector<float>.Zero)), pairCount);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static unsafe void GenerateInteriorPoints(in CylinderWide cylinder, in Vector3Wide cylinderLocalNormal, in Vector3Wide localClosestOnCylinder, out Vector2Wide interior0, out Vector2Wide interior1, out Vector2Wide interior2, out Vector2Wide interior3)
        {
            //Assume we can just use the 4 local extreme points of the cylinder at first.
            //Then, if there is sufficient tilt, replace the closest extreme point to the deepest point with the deepest point.
            var interpolationMin = new Vector<float>(0.9999f);
            var inverseInterpolationSpan = new Vector<float>(1f / 0.00005f);
            var parallelWeight = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, (Vector.Abs(cylinderLocalNormal.Y) - interpolationMin) * inverseInterpolationSpan));
            var deepestWeight = Vector<float>.One - parallelWeight;
            var replaceX = Vector.GreaterThan(Vector.Abs(localClosestOnCylinder.X), Vector.Abs(localClosestOnCylinder.Z));
            var replace0 = Vector.BitwiseAnd(Vector.GreaterThan(localClosestOnCylinder.X, Vector<float>.Zero), replaceX);
            var replace1 = Vector.BitwiseAnd(Vector.LessThanOrEqual(localClosestOnCylinder.X, Vector<float>.Zero), replaceX);
            var replace2 = Vector.AndNot(Vector.GreaterThan(localClosestOnCylinder.Z, Vector<float>.Zero), replaceX);
            var replace3 = Vector.AndNot(Vector.LessThanOrEqual(localClosestOnCylinder.Z, Vector<float>.Zero), replaceX);
            var scaledRadius = parallelWeight * cylinder.Radius;
            interior0.X = Vector.ConditionalSelect(replace0, deepestWeight * localClosestOnCylinder.X + scaledRadius, cylinder.Radius);
            interior0.Y = Vector.ConditionalSelect(replace0, deepestWeight * localClosestOnCylinder.Z, Vector<float>.Zero);
            interior1.X = Vector.ConditionalSelect(replace1, deepestWeight * localClosestOnCylinder.X - scaledRadius, -cylinder.Radius);
            interior1.Y = Vector.ConditionalSelect(replace1, deepestWeight * localClosestOnCylinder.Z, Vector<float>.Zero);
            interior2.X = Vector.ConditionalSelect(replace2, deepestWeight * localClosestOnCylinder.X, Vector<float>.Zero);
            interior2.Y = Vector.ConditionalSelect(replace2, deepestWeight * localClosestOnCylinder.Z + scaledRadius, cylinder.Radius);
            interior3.X = Vector.ConditionalSelect(replace3, deepestWeight * localClosestOnCylinder.X, Vector<float>.Zero);
            interior3.Y = Vector.ConditionalSelect(replace3, deepestWeight * localClosestOnCylinder.Z - scaledRadius, -cylinder.Radius);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TryAddInteriorPoint(in Vector2Wide point, in Vector<int> featureId,
            in Vector2Wide edge0010, in Vector<float> edge0010PlaneMin, in Vector<float> edge0010PlaneMax,
            in Vector2Wide edge1011, in Vector<float> edge1011PlaneMin, in Vector<float> edge1011PlaneMax,
            in Vector<int> allowContact, ref ManifoldCandidate candidates, ref Vector<int> candidateCount, int pairCount)
        {
            var edge0010Dot = point.X * edge0010.Y - point.Y * edge0010.X;
            var edge1011Dot = point.X * edge1011.Y - point.Y * edge1011.X;
            var contained = Vector.BitwiseAnd(allowContact, Vector.BitwiseAnd(
                Vector.BitwiseAnd(Vector.GreaterThanOrEqual(edge0010Dot, edge0010PlaneMin), Vector.LessThanOrEqual(edge0010Dot, edge0010PlaneMax)),
                Vector.BitwiseAnd(Vector.GreaterThanOrEqual(edge1011Dot, edge1011PlaneMin), Vector.LessThanOrEqual(edge1011Dot, edge1011PlaneMax))));
            Unsafe.SkipInit(out ManifoldCandidate candidate);
            candidate.X = point.X;
            candidate.Y = point.Y;
            candidate.FeatureId = featureId;
            ManifoldCandidateHelper.AddCandidate(ref candidates, ref candidateCount, candidate, contained, pairCount);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Test(
            ref BoxWide a, ref CylinderWide b, ref Vector<float> speculativeMargin,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount,
            out Convex4ContactManifoldWide manifold)
        {
            Unsafe.SkipInit(out manifold);
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var worldRA);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var worldRB);
            //Work in b's local space.
            Matrix3x3Wide.MultiplyByTransposeWithoutOverlap(worldRA, worldRB, out var rA);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, worldRB, out var localOffsetB);
            Vector3Wide.Negate(localOffsetB, out var localOffsetA);

            Vector3Wide.Length(localOffsetA, out var length);
            Vector3Wide.Scale(localOffsetA, Vector<float>.One / length, out var localNormal);
            var useInitialSampleFallback = Vector.LessThan(length, new Vector<float>(1e-10f));
            localNormal.X = Vector.ConditionalSelect(useInitialSampleFallback, Vector<float>.Zero, localNormal.X);
            localNormal.Y = Vector.ConditionalSelect(useInitialSampleFallback, Vector<float>.One, localNormal.Y);
            localNormal.Z = Vector.ConditionalSelect(useInitialSampleFallback, Vector<float>.Zero, localNormal.Z);
            BoxSupportFinder boxSupportFinder = default;
            CylinderSupportFinder cylinderSupportFinder = default;

            //We now have a decent estimate for the local normal and an initial simplex to work from. Refine it to a local minimum.
            var inactiveLanes = BundleIndexing.CreateTrailingMaskForCountInBundle(pairCount);

            var depthThreshold = -speculativeMargin;
            var epsilonScale = Vector.Min(Vector.Max(a.HalfWidth, Vector.Max(a.HalfHeight, a.HalfLength)), Vector.Max(b.HalfLength, b.Radius));
            DepthRefiner.FindMinimumDepth(
                b, a, localOffsetA, rA, ref cylinderSupportFinder, ref boxSupportFinder, localNormal, inactiveLanes, epsilonScale * new Vector<float>(1e-6f), depthThreshold,
                out var depth, out localNormal, out var closestOnB, maximumIterations: 25);

            inactiveLanes = Vector.BitwiseOr(inactiveLanes, Vector.LessThan(depth, depthThreshold));
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

            //Identify the box face.
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(localNormal, rA, out var localNormalInA);
            Vector3Wide.Abs(localNormalInA, out var absLocalNormalInA);
            var useX = Vector.BitwiseAnd(Vector.GreaterThan(absLocalNormalInA.X, absLocalNormalInA.Y), Vector.GreaterThan(absLocalNormalInA.X, absLocalNormalInA.Z));
            var useY = Vector.AndNot(Vector.GreaterThan(absLocalNormalInA.Y, absLocalNormalInA.Z), useX);
            Vector3Wide.ConditionalSelect(useX, rA.X, rA.Z, out var boxFaceNormal);
            Vector3Wide.ConditionalSelect(useY, rA.Y, boxFaceNormal, out boxFaceNormal);
            Vector3Wide.ConditionalSelect(useX, rA.Y, rA.X, out var boxFaceX);
            Vector3Wide.ConditionalSelect(useY, rA.Z, boxFaceX, out boxFaceX);
            Vector3Wide.ConditionalSelect(useX, rA.Z, rA.Y, out var boxFaceY);
            Vector3Wide.ConditionalSelect(useY, rA.X, boxFaceY, out boxFaceY);
            var negateFace = 
                Vector.ConditionalSelect(useX, Vector.GreaterThan(localNormalInA.X, Vector<float>.Zero), 
                Vector.ConditionalSelect(useY, Vector.GreaterThan(localNormalInA.Y, Vector<float>.Zero), Vector.GreaterThan(localNormalInA.Z, Vector<float>.Zero)));
            Vector3Wide.ConditionallyNegate(negateFace, ref boxFaceNormal);
            Vector3Wide.ConditionallyNegate(negateFace, ref boxFaceX);
            Vector3Wide.ConditionallyNegate(negateFace, ref boxFaceY);
            var boxFaceHalfWidth = Vector.ConditionalSelect(useX, a.HalfHeight, Vector.ConditionalSelect(useY, a.HalfLength, a.HalfWidth));
            var boxFaceHalfHeight = Vector.ConditionalSelect(useX, a.HalfLength, Vector.ConditionalSelect(useY, a.HalfWidth, a.HalfHeight));
            var boxFaceNormalOffset = Vector.ConditionalSelect(useX, a.HalfWidth, Vector.ConditionalSelect(useY, a.HalfHeight, a.HalfLength));
            Vector3Wide.Scale(boxFaceNormal, boxFaceNormalOffset, out var boxFaceCenterOffset);
            Vector3Wide.Add(boxFaceCenterOffset, localOffsetA, out var boxFaceCenter);
            Vector3Wide.Scale(boxFaceX, boxFaceHalfWidth, out var boxFaceXOffset);
            Vector3Wide.Scale(boxFaceY, boxFaceHalfHeight, out var boxFaceYOffset);
            Vector3Wide.Subtract(boxFaceCenter, boxFaceXOffset, out var v00);
            Vector3Wide.Subtract(v00, boxFaceYOffset, out v00);
            Vector3Wide.Add(boxFaceCenter, boxFaceXOffset, out var v11);
            Vector3Wide.Add(v11, boxFaceYOffset, out v11);

            var capCenterBY = Vector.ConditionalSelect(Vector.LessThan(localNormal.Y, Vector<float>.Zero), -b.HalfLength, b.HalfLength);

            var useCap = Vector.AndNot(Vector.GreaterThan(Vector.Abs(localNormal.Y), new Vector<float>(0.70710678118f)), inactiveLanes);

            Vector3Wide.Dot(boxFaceNormal, localNormal, out var faceNormalDotLocalNormal);
            var inverseFaceNormalDotLocalNormal = Vector<float>.One / faceNormalDotLocalNormal;

            if (Vector.LessThanAny(useCap, Vector<int>.Zero))
            {
                //At least one lane needs a cap-face manifold.
                int byteCount = Unsafe.SizeOf<ManifoldCandidate>() * 12;
                var buffer = stackalloc byte[byteCount];
                ref var candidates = ref Unsafe.As<byte, ManifoldCandidate>(ref *buffer);
                var candidateCount = Vector<int>.Zero;

                //Project the edges down onto the cap's plane.
                var inverseLocalNormalY = Vector<float>.One / localNormal.Y;
                Vector3Wide.Subtract(boxFaceCenter, boxFaceXOffset, out var v01);
                Vector3Wide.Add(v01, boxFaceYOffset, out v01);
                Vector3Wide.Add(boxFaceCenter, boxFaceXOffset, out var v10);
                Vector3Wide.Subtract(v10, boxFaceYOffset, out v10);
                CylinderPairTester.ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, v00, out var p00);
                CylinderPairTester.ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, v01, out var p01);
                CylinderPairTester.ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, v10, out var p10);
                CylinderPairTester.ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, v11, out var p11);
                //Note that winding is important; we'll be choosing contacts based on the intervals. If two edges are unbounded, we only allow one contact to be generated.
                //(Note that there is some room for microoptimization here. The projection was linear; edge0010 and edge1101 are the same but negated, likewise for 1011 and 0100. Opting for simplicity for now.)
                Vector2Wide.Subtract(p10, p00, out var edge0010);
                Vector2Wide.Subtract(p11, p10, out var edge1011);
                Vector2Wide.Subtract(p01, p11, out var edge1101);
                Vector2Wide.Subtract(p00, p01, out var edge0100);
                IntersectLineCircle(p00, edge0010, b.Radius, out var tMin0010, out var tMax0010, out var intersected0010);
                IntersectLineCircle(p01, edge0100, b.Radius, out var tMin0100, out var tMax0100, out var intersected0100);
                IntersectLineCircle(p10, edge1011, b.Radius, out var tMin1011, out var tMax1011, out var intersected1011);
                IntersectLineCircle(p11, edge1101, b.Radius, out var tMin1101, out var tMax1101, out var intersected1101);

                tMin0010 = Vector.Min(Vector.Max(tMin0010, Vector<float>.Zero), Vector<float>.One);
                tMax0010 = Vector.Min(Vector.Max(tMax0010, Vector<float>.Zero), Vector<float>.One);
                tMin1101 = Vector.Min(Vector.Max(tMin1101, Vector<float>.Zero), Vector<float>.One);
                tMax1101 = Vector.Min(Vector.Max(tMax1101, Vector<float>.Zero), Vector<float>.One);
                tMin0100 = Vector.Min(Vector.Max(tMin0100, Vector<float>.Zero), Vector<float>.One);
                tMax0100 = Vector.Min(Vector.Max(tMax0100, Vector<float>.Zero), Vector<float>.One);
                tMin1011 = Vector.Min(Vector.Max(tMin1011, Vector<float>.Zero), Vector<float>.One);
                tMax1011 = Vector.Min(Vector.Max(tMax1011, Vector<float>.Zero), Vector<float>.One);

                AddCandidateForEdge(p00, edge0010, tMin0010, tMax0010, intersected0010, Vector<int>.Zero, useCap, pairCount, ref candidates, ref candidateCount);
                AddCandidateForEdge(p01, edge0100, tMin0100, tMax0100, intersected0100, Vector<int>.One, useCap, pairCount, ref candidates, ref candidateCount);
                AddCandidateForEdge(p10, edge1011, tMin1011, tMax1011, intersected1011, new Vector<int>(2), useCap, pairCount, ref candidates, ref candidateCount);
                AddCandidateForEdge(p11, edge1101, tMin1101, tMax1101, intersected1101, new Vector<int>(3), useCap, pairCount, ref candidates, ref candidateCount);

                GenerateInteriorPoints(b, localNormal, closestOnB, out var interior0, out var interior1, out var interior2, out var interior3);

                //Test the four points against the edge plane. Note that signs depend on the orientation of the cylinder.
                var edge0010Plane0 = p00.X * edge0010.Y - p00.Y * edge0010.X;
                var edge0010Plane1 = p01.X * edge0010.Y - p01.Y * edge0010.X;
                var edge1011Plane0 = p10.X * edge1011.Y - p10.Y * edge1011.X;
                var edge1011Plane1 = p00.X * edge1011.Y - p00.Y * edge1011.X;
                var edge0010PlaneMin = Vector.Min(edge0010Plane0, edge0010Plane1);
                var edge0010PlaneMax = Vector.Max(edge0010Plane0, edge0010Plane1);
                var edge1011PlaneMin = Vector.Min(edge1011Plane0, edge1011Plane1);
                var edge1011PlaneMax = Vector.Max(edge1011Plane0, edge1011Plane1);
                TryAddInteriorPoint(interior0, new Vector<int>(8), edge0010, edge0010PlaneMin, edge0010PlaneMax, edge1011, edge1011PlaneMin, edge1011PlaneMax, useCap, ref candidates, ref candidateCount, pairCount);
                TryAddInteriorPoint(interior1, new Vector<int>(9), edge0010, edge0010PlaneMin, edge0010PlaneMax, edge1011, edge1011PlaneMin, edge1011PlaneMax, useCap, ref candidates, ref candidateCount, pairCount);
                TryAddInteriorPoint(interior2, new Vector<int>(10), edge0010, edge0010PlaneMin, edge0010PlaneMax, edge1011, edge1011PlaneMin, edge1011PlaneMax, useCap, ref candidates, ref candidateCount, pairCount);
                TryAddInteriorPoint(interior3, new Vector<int>(11), edge0010, edge0010PlaneMin, edge0010PlaneMax, edge1011, edge1011PlaneMin, edge1011PlaneMax, useCap, ref candidates, ref candidateCount, pairCount);

                Vector3Wide capCenterToBoxFaceCenter;
                capCenterToBoxFaceCenter.X = boxFaceCenter.X;
                capCenterToBoxFaceCenter.Y = boxFaceCenter.Y - capCenterBY;
                capCenterToBoxFaceCenter.Z = boxFaceCenter.Z;
                Vector3Wide tangentBX, tangentBY;
                tangentBX.X = Vector<float>.One;
                tangentBX.Y = Vector<float>.Zero;
                tangentBX.Z = Vector<float>.Zero;
                tangentBY.X = Vector<float>.Zero;
                tangentBY.Y = Vector<float>.Zero;
                tangentBY.Z = Vector<float>.One;
                ManifoldCandidateHelper.Reduce(ref candidates, candidateCount, 12, boxFaceNormal, inverseFaceNormalDotLocalNormal, capCenterToBoxFaceCenter, tangentBX, tangentBY, epsilonScale, depthThreshold, pairCount,
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
                Vector3Wide.CrossWithoutOverlap(boxFaceX, localNormal, out var edgeNormalX); //Points up
                Vector3Wide.CrossWithoutOverlap(boxFaceY, localNormal, out var edgeNormalY); //Points right
                //Center of the side line is just (closestOnB.X, 0, closestOnB.Z), sideLineDirection is just (0, 1, 0).
                //t = dot(sideLineStart - pointOnFaceEdge, edgeNormal) / dot(sideLineDirection, edgeNormal)
                var negativeOne = new Vector<float>(-1f);
                var xDenominator = negativeOne / edgeNormalX.Y;
                var yDenominator = negativeOne / edgeNormalY.Y;
                Vector3Wide.LengthSquared(edgeNormalX, out var edgeNormalXLengthSquared);
                Vector3Wide.LengthSquared(edgeNormalY, out var edgeNormalYLengthSquared);
                var inverseEdgeNormalXLengthSquared = Vector<float>.One / edgeNormalXLengthSquared;
                var inverseEdgeNormalYLengthSquared = Vector<float>.One / edgeNormalYLengthSquared;
                Vector3Wide v00ToSideLine, v11ToSideLine;
                v00ToSideLine.X = closestOnB.X - v00.X;
                v00ToSideLine.Y = -v00.Y;
                v00ToSideLine.Z = closestOnB.Z - v00.Z;
                v11ToSideLine.X = closestOnB.X - v11.X;
                v11ToSideLine.Y = -v11.Y;
                v11ToSideLine.Z = closestOnB.Z - v11.Z;

                Vector3Wide.Dot(edgeNormalX, v00ToSideLine, out var bottomNumerator);
                Vector3Wide.Dot(edgeNormalY, v00ToSideLine, out var leftNumerator);
                Vector3Wide.Dot(edgeNormalX, v11ToSideLine, out var topNumerator);
                Vector3Wide.Dot(edgeNormalY, v11ToSideLine, out var rightNumerator);
                var xInvalid = Vector.Equals(edgeNormalX.Y, Vector<float>.Zero);
                var yInvalid = Vector.Equals(edgeNormalY.Y, Vector<float>.Zero);
                var minValue = new Vector<float>(float.MinValue);
                var maxValue = new Vector<float>(float.MaxValue);
                var tX0 = bottomNumerator * xDenominator;
                var tX1 = topNumerator * xDenominator;
                var tY0 = leftNumerator * yDenominator;
                var tY1 = rightNumerator * yDenominator;
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
                var unrestrictWeightX = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, (interpolationMin - edgeNormalX.Y * edgeNormalX.Y * inverseEdgeNormalXLengthSquared) * inverseInterpolationSpan));
                var unrestrictWeightY = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, (interpolationMin - edgeNormalY.Y * edgeNormalY.Y * inverseEdgeNormalYLengthSquared) * inverseInterpolationSpan));
                var regularWeightX = Vector<float>.One - unrestrictWeightX;
                var regularWeightY = Vector<float>.One - unrestrictWeightY;
                var negativeHalfLength = -b.HalfLength;
                var tXMin = Vector.ConditionalSelect(xInvalid, minValue, unrestrictWeightX * negativeHalfLength + regularWeightX * Vector.Min(tX0, tX1));
                var tXMax = Vector.ConditionalSelect(xInvalid, maxValue, unrestrictWeightX * b.HalfLength + regularWeightX * Vector.Max(tX0, tX1));
                var tYMin = Vector.ConditionalSelect(yInvalid, minValue, unrestrictWeightY * negativeHalfLength + regularWeightY * Vector.Min(tY0, tY1));
                var tYMax = Vector.ConditionalSelect(yInvalid, maxValue, unrestrictWeightY * b.HalfLength + regularWeightY * Vector.Max(tY0, tY1));
                //Shouldn't need to make contact generation conditional here. The closest points are guaranteed to be on these chosen features;
                //they might just be in the same spot. We do clamp for numerical reasons.
                var tMax = Vector.Min(Vector.Max(negativeHalfLength, Vector.Min(tXMax, tYMax)), b.HalfLength);
                var tMin = Vector.Min(Vector.Max(negativeHalfLength, Vector.Max(tXMin, tYMin)), b.HalfLength);

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
                Vector3Wide.Subtract(localContact0, boxFaceCenter, out var boxFaceToContact0);
                Vector3Wide.Subtract(localContact1, boxFaceCenter, out var boxFaceToContact1);
                Vector3Wide.Dot(boxFaceToContact0, boxFaceNormal, out var contact0Dot);
                Vector3Wide.Dot(boxFaceToContact1, boxFaceNormal, out var contact1Dot);
                var depth0 = contact0Dot * inverseFaceNormalDotLocalNormal;
                var depth1 = contact1Dot * inverseFaceNormalDotLocalNormal;
                manifold.Depth0 = Vector.ConditionalSelect(useSide, depth0, manifold.Depth0);
                manifold.Depth1 = Vector.ConditionalSelect(useSide, depth1, manifold.Depth1);
                manifold.Contact0Exists = Vector.ConditionalSelect(useSide, Vector.GreaterThanOrEqual(depth0, depthThreshold), manifold.Contact0Exists);
                manifold.Contact1Exists = Vector.ConditionalSelect(useSide, Vector.BitwiseAnd(Vector.GreaterThanOrEqual(depth1, depthThreshold), Vector.GreaterThan(tMax, tMin)), manifold.Contact1Exists);
                manifold.Contact2Exists = Vector.ConditionalSelect(useSide, Vector<int>.Zero, manifold.Contact2Exists);
                manifold.Contact3Exists = Vector.ConditionalSelect(useSide, Vector<int>.Zero, manifold.Contact3Exists);
            }

            Matrix3x3Wide.TransformWithoutOverlap(localNormal, worldRB, out manifold.Normal);
        }


        public void Test(ref BoxWide a, ref CylinderWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref BoxWide a, ref CylinderWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }
}