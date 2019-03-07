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
        public int BatchSize => 32;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void IntersectLineCircle(in Vector2Wide linePosition, in Vector2Wide lineDirection, in Vector<float> radius, out Vector<float> tMin, out Vector<float> tMax, out Vector<int> intersected)
        {
            //||linePosition + lineDirection * t|| = radius
            //dot(linePosition + lineDirection * t, linePosition + lineDirection * t) = radius * radius
            //dot(linePosition, linePosition) - radius * radius + t * 2 * dot(linePosition, lineDirection) + t^2 * dot(lineDirection, lineDirection) = 0
            Vector2Wide.Dot(lineDirection, lineDirection, out var a);
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
            //If the projected line direction is zero, just compress the interval to tBase.
            var useFallback = Vector.LessThan(Vector.Abs(a), new Vector<float>(1e-12f));
            tMin = Vector.ConditionalSelect(useFallback, tBase, tMin);
            tMax = Vector.ConditionalSelect(useFallback, tBase, tMax);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void AddCandidateForEdge(in Vector2Wide edgeStart, in Vector2Wide edgeOffset, in Vector<float> tMin, in Vector<float> tMax, in Vector<int> intersected, in Vector<int> edgeId, in Vector<int> allowFeatureContacts, int pairCount,
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
        static void CandidateToOffsetA(in ManifoldCandidate candidate, in Vector3Wide featureCenterB, in Vector3Wide tangentBX, in Vector3Wide tangentBY, in Vector3Wide localOffsetB, in Matrix3x3Wide orientationB, out Vector3Wide offsetA)
        {
            Vector3Wide.Scale(tangentBX, candidate.X, out var x);
            Vector3Wide.Scale(tangentBY, candidate.Y, out var y);
            Vector3Wide.Add(x, y, out var offset);
            Vector3Wide.Add(featureCenterB, offset, out var localContact);
            Vector3Wide.Add(localContact, localOffsetB, out localContact);
            Matrix3x3Wide.TransformWithoutOverlap(localContact, orientationB, out offsetA);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Test(
            ref BoxWide a, ref CylinderWide b, ref Vector<float> speculativeMargin,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount,
            out Convex4ContactManifoldWide manifold)
        {
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var worldRA);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var worldRB);
            //Work in b's local space.
            Matrix3x3Wide.MultiplyByTransposeWithoutOverlap(worldRA, worldRB, out var rA);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, worldRB, out var localOffsetB);
            Vector3Wide.Negate(localOffsetB, out var localOffsetA);

            //First, we'll try a few easy known normal candidates.
            //1) Offset from B to A
            Vector3Wide.Length(localOffsetA, out var length);
            Vector3Wide.Scale(localOffsetA, Vector<float>.One / length, out var localNormal);
            var useInitialSampleFallback = Vector.LessThan(length, new Vector<float>(1e-10f));
            localNormal.X = Vector.ConditionalSelect(useInitialSampleFallback, Vector<float>.Zero, localNormal.X);
            localNormal.Y = Vector.ConditionalSelect(useInitialSampleFallback, Vector<float>.One, localNormal.Y);
            localNormal.Z = Vector.ConditionalSelect(useInitialSampleFallback, Vector<float>.Zero, localNormal.Z);
            BoxSupportFinder boxSupportFinder = default;
            CylinderSupportFinder cylinderSupportFinder = default;
            DepthRefiner.SimplexWithWitness simplex;
            DepthRefiner.FindSupport(b, a, localOffsetA, rA, ref cylinderSupportFinder, ref boxSupportFinder, localNormal, out simplex.A.Support, out simplex.A.SupportOnA);
            simplex.A.Exists = new Vector<int>(-1);
            Vector3Wide.Dot(simplex.A.Support, localNormal, out var depth);

            //2) Cap normal B
            {
                //Use the local Y axis sign that points from B to A.
                Vector3Wide negatedNormalCandidate;
                negatedNormalCandidate.X = Vector<float>.Zero;
                negatedNormalCandidate.Y = Vector.ConditionalSelect(Vector.GreaterThan(localOffsetA.Y, Vector<float>.Zero), new Vector<float>(-1), Vector<float>.One);
                negatedNormalCandidate.Z = Vector<float>.Zero;
                boxSupportFinder.ComputeSupport(a, rA, negatedNormalCandidate, out var supportA);
                Vector3Wide.Add(supportA, localOffsetA, out supportA);
                //A little confusing- DepthRefiner's A is our B and vice versa.       
                simplex.B.SupportOnA.X = Vector<float>.Zero;
                simplex.B.SupportOnA.Y = negatedNormalCandidate.Y * -b.HalfLength;
                simplex.B.SupportOnA.Z = Vector<float>.Zero;
                Vector3Wide.Subtract(simplex.B.SupportOnA, supportA, out simplex.B.Support);
                Vector3Wide.Dot(simplex.B.Support, negatedNormalCandidate, out var negatedDepthCandidate);
                simplex.B.Exists = new Vector<int>(-1);
                var depthCandidate = -negatedDepthCandidate;
                var useCandidate = Vector.LessThan(depthCandidate, depth);
                depth = Vector.ConditionalSelect(useCandidate, depthCandidate, depth);
                localNormal.X = Vector.ConditionalSelect(useCandidate, Vector<float>.Zero, localNormal.X);
                localNormal.Y = Vector.ConditionalSelect(useCandidate, -negatedNormalCandidate.Y, localNormal.Y);
                localNormal.Z = Vector.ConditionalSelect(useCandidate, Vector<float>.Zero, localNormal.Z);
            }
            //We'll leave the last entry unfilled- sampling a box face normal would have limited value; the depth refiner tends to do a very good job with polytope features.
            //(Even sampling the cylinder cap is a little questionable.)
            simplex.C = default;

            //There are some other axes we could try, like axisA x axisB, but they tend to be far less relevant for stacking.
            //Also, keep in mind that feature pairs which form faces, edges, or vertices in minkowski space tend to converge extremely quickly.
            //Blindly trying a bunch of low probability feature pairs- especially those which would fall into the above- isn't very wise.

            //We now have a decent estimate for the local normal and an initial simplex to work from. Refine it to a local minimum.
            ManifoldCandidateHelper.CreateInactiveMask(pairCount, out var inactiveLanes);

            var depthThreshold = -speculativeMargin;
            var epsilonScale = Vector.Min(Vector.Max(a.HalfWidth, Vector.Max(a.HalfHeight, a.HalfLength)), Vector.Max(b.HalfLength, b.Radius));
            DepthRefiner.FindMinimumDepth(
                b, a, localOffsetA, rA, ref cylinderSupportFinder, ref boxSupportFinder, ref simplex, localNormal, depth, inactiveLanes, epsilonScale * new Vector<float>(1e-6f), depthThreshold,
                out depth, out localNormal, out var closestOnB, maximumIterations: 25);

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
            var useX = Vector.BitwiseAnd(Vector.LessThan(localNormalInA.X, localNormalInA.Y), Vector.LessThan(localNormalInA.X, localNormalInA.Z));
            var useY = Vector.AndNot(Vector.LessThan(localNormalInA.Y, localNormalInA.Z), useX);
            Vector3Wide.ConditionalSelect(useX, rA.X, rA.Z, out var boxFaceNormal);
            Vector3Wide.ConditionalSelect(useY, rA.Y, boxFaceNormal, out boxFaceNormal);
            Vector3Wide.ConditionalSelect(useX, rA.Y, rA.X, out var boxFaceX);
            Vector3Wide.ConditionalSelect(useY, rA.Z, boxFaceX, out boxFaceX);
            Vector3Wide.ConditionalSelect(useX, rA.Z, rA.Y, out var boxFaceY);
            Vector3Wide.ConditionalSelect(useY, rA.X, boxFaceY, out boxFaceY);
            var boxFaceHalfWidth = Vector.ConditionalSelect(useX, a.HalfHeight, Vector.ConditionalSelect(useY, a.HalfLength, a.HalfWidth));
            var boxFaceHalfHeight = Vector.ConditionalSelect(useX, a.HalfLength, Vector.ConditionalSelect(useY, a.HalfWidth, a.HalfHeight));
            var boxFaceNormalOffset = Vector.ConditionalSelect(useX, a.HalfWidth, Vector.ConditionalSelect(useY, a.HalfHeight, a.HalfLength));
            Vector3Wide.Scale(boxFaceNormal, boxFaceNormalOffset, out var boxFaceCenterOffset);
            Vector3Wide.Add(boxFaceCenterOffset, localOffsetA, out var boxFaceCenter);

            var capCenterBY = Vector.ConditionalSelect(Vector.LessThan(localNormal.Y, Vector<float>.Zero), -b.HalfLength, b.HalfLength);

            var useCap = Vector.AndNot(Vector.GreaterThan(Vector.Abs(localNormal.Y), new Vector<float>(0.70710678118f)), inactiveLanes);
            
            if (Vector.LessThanAny(useCap, Vector<int>.Zero))
            {
                //At least one lane needs a cap-face manifold.
                //Note that we only allocate up to 8 candidates. It is not possible for this process to generate more than 8.
                int byteCount = Unsafe.SizeOf<ManifoldCandidate>() * 8;
                var buffer = stackalloc byte[byteCount];
                ref var candidates = ref Unsafe.As<byte, ManifoldCandidate>(ref *buffer);
                var candidateCount = Vector<int>.Zero;

                //Project the edges down onto the cap's plane.
                var inverseLocalNormalY = Vector<float>.One / localNormal.Y;
                Vector3Wide.Scale(boxFaceX, boxFaceHalfWidth, out var boxFaceXOffset);
                Vector3Wide.Scale(boxFaceY, boxFaceHalfHeight, out var boxFaceYOffset);
                Vector3Wide.Subtract(boxFaceCenter, boxFaceXOffset, out var v00);
                Vector3Wide.Subtract(v00, boxFaceYOffset, out v00);
                Vector3Wide.Subtract(boxFaceCenter, boxFaceXOffset, out var v01);
                Vector3Wide.Add(v01, boxFaceYOffset, out v01);
                Vector3Wide.Add(boxFaceCenter, boxFaceXOffset, out var v10);
                Vector3Wide.Subtract(v10, boxFaceYOffset, out v10);
                Vector3Wide.Add(boxFaceCenter, boxFaceXOffset, out var v11);
                Vector3Wide.Add(v11, boxFaceYOffset, out v11);
                CylinderPairTester.ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, v00, out var p00);
                CylinderPairTester.ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, v01, out var p01);
                CylinderPairTester.ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, v10, out var p10);
                CylinderPairTester.ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, v11, out var p11);
                //Note that winding is important; we'll be choosing contacts based on the intervals. If two edges are unbounded, we only allow one contact to be generated.
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
                ManifoldCandidateHelper.Reduce(ref candidates, candidateCount, 8, boxFaceNormal, localNormal, capCenterToBoxFaceCenter, tangentBX, tangentBY, epsilonScale, depthThreshold, pairCount,
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

            var useSide = Vector.AndNot(Vector.OnesComplement(useCap), inactiveLanes);
            if (Vector.LessThanAny(useSide, Vector<int>.Zero))
            {
                //At least one lane needs a side-face manifold.
                //Intersect the single edge of A with the edge planes of face A.
                //Note that the edge planes are skewed to follow the local normal. Equivalent to projecting the side edge onto face A.
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