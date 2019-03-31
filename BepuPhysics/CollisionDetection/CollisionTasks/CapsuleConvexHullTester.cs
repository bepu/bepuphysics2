using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct CapsuleConvexHullTester : IPairTester<CapsuleWide, ConvexHullWide, Convex2ContactManifoldWide>
    {
        public int BatchSize => 32;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Test(ref CapsuleWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount, out Convex2ContactManifoldWide manifold)
        {
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var capsuleOrientation);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var hullOrientation);
            Matrix3x3Wide.MultiplyByTransposeWithoutOverlap(capsuleOrientation, hullOrientation, out var hullLocalCapsuleOrientation);
            ref var localCapsuleAxis = ref hullLocalCapsuleOrientation.Y;

            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, hullOrientation, out var localOffsetB);
            Vector3Wide.Negate(localOffsetB, out var localOffsetA);
            Matrix3x3Wide.CreateIdentity(out var identity);
            Vector3Wide.Length(localOffsetA, out var centerDistance);
            Vector3Wide.Scale(localOffsetA, Vector<float>.One / centerDistance, out var initialNormal);
            var useInitialFallback = Vector.LessThan(centerDistance, new Vector<float>(1e-8f));
            initialNormal.X = Vector.ConditionalSelect(useInitialFallback, Vector<float>.Zero, initialNormal.X);
            initialNormal.Y = Vector.ConditionalSelect(useInitialFallback, Vector<float>.One, initialNormal.Y);
            initialNormal.Z = Vector.ConditionalSelect(useInitialFallback, Vector<float>.Zero, initialNormal.Z);
            var hullSupportFinder = default(ConvexHullSupportFinder);
            var capsuleSupportFinder = default(CapsuleSupportFinder);
            ManifoldCandidateHelper.CreateInactiveMask(pairCount, out var inactiveLanes);
            b.EstimateEpsilonScale(inactiveLanes, out var hullEpsilonScale);
            var epsilonScale = Vector.Min(a.Radius, hullEpsilonScale);
            var depthThreshold = -speculativeMargin;
            DepthRefiner<ConvexHull, ConvexHullWide, ConvexHullSupportFinder, Capsule, CapsuleWide, CapsuleSupportFinder>.FindMinimumDepth(
                b, a, localOffsetA, identity, ref hullSupportFinder, ref capsuleSupportFinder, initialNormal, inactiveLanes, 1e-6f * epsilonScale, depthThreshold,
                out var depth, out var localNormal);

            inactiveLanes = Vector.BitwiseOr(inactiveLanes, Vector.LessThan(depth, -speculativeMargin));
            if (Vector.LessThanAll(inactiveLanes, Vector<int>.Zero))
            {
                //No contacts generated.
                manifold = default;
                return;
            }

            //To find the contact manifold, we'll clip the capsule axis against the face as usual, but we're dealing with potentially
            //distinct convex hulls. Rather than vectorizing over the different hulls, we vectorize within each hull.
            Helpers.FillVectorWithLaneIndices(out var slotOffsetIndices);
            for (int slotIndex = 0; slotIndex < pairCount; ++slotIndex)
            {
                if (inactiveLanes[slotIndex] < 0)
                    continue;
                ref var hull = ref b.Hulls[slotIndex];
                //Pick the representative face.
                Vector3Wide.Rebroadcast(localNormal, slotIndex, out var slotLocalNormalBundle);
                Vector3Wide.Dot(hull.BoundingPlanes[0].Normal, slotLocalNormalBundle, out var bestFaceDotBundle);
                var bestIndices = slotOffsetIndices;
                for (int i = 0; i < hull.BoundingPlanes.Length; ++i)
                {
                    var slotIndices = new Vector<int>(i << BundleIndexing.VectorShift) + slotOffsetIndices;
                    //Face normals point outward.
                    //(Bundle slots beyond actual face count contain dummy data chosen to avoid being picked.)
                    Vector3Wide.Dot(hull.BoundingPlanes[i].Normal, slotLocalNormalBundle, out var dot);
                    var useCandidate = Vector.GreaterThan(dot, bestFaceDotBundle);
                    bestFaceDotBundle = Vector.ConditionalSelect(useCandidate, dot, bestFaceDotBundle);
                    bestIndices = Vector.ConditionalSelect(useCandidate, slotIndices, bestIndices);
                }
                var bestFaceDot = bestFaceDotBundle[0];
                var bestIndex = slotOffsetIndices[0];
                for (int i = 1; i < Vector<float>.Count; ++i)
                {
                    var dot = bestFaceDotBundle[i];
                    if (dot > bestFaceDot)
                    {
                        bestFaceDot = dot;
                        bestIndex = slotOffsetIndices[i];
                    }
                }
                BundleIndexing.GetBundleIndices(bestIndex, out var faceBundleIndex, out var faceInnerIndex);
                Vector3Wide.ReadSlot(ref hull.BoundingPlanes[faceBundleIndex].Normal, faceInnerIndex, out var faceNormal);

                //Test each face edge plane against the capsule edge.
                //Note that we do not use the faceNormal x edgeOffset edge plane, but rather edgeOffset x localNormal.
                //(In other words, testing the *projected* capsule axis on the surface of the convex hull face.)
                //The faces are wound counterclockwise.
                hull.GetFaceVertexIndices(bestIndex, out var faceVertexIndices);
                var previousIndex = faceVertexIndices[faceVertexIndices.Length - 1];
                Vector3Wide.ReadSlot(ref hull.Points[previousIndex.BundleIndex], previousIndex.InnerIndex, out var previousVertex);
                Vector3Wide.ReadFirst(slotLocalNormalBundle, out var slotLocalNormal);
                var latestEntryNumerator = new Vector<float>(float.MaxValue);
                var latestEntryDenominator = new Vector<float>(-1);
                var earliestExitNumerator = new Vector<float>(float.MaxValue);
                var earliestExitDenominator = new Vector<float>(1);
                for (int i = 0; i < faceVertexIndices.Length; ++i)
                {
                    var index = faceVertexIndices[i];
                    Vector3Wide.ReadSlot(ref hull.Points[index.BundleIndex], index.InnerIndex, out var vertex);

                    var edgeOffset = vertex - previousVertex;
                    Vector3x.Cross(edgeOffset, slotLocalNormal, out var edgePlaneNormal);
                    Vector3Wide.Broadcast(previousVertex, out var edgeStartBundle);
                    Vector3Wide.Broadcast(edgePlaneNormal, out var edgePlaneNormalBundle);
                    previousVertex = vertex;

                    //t = dot(pointOnPlane - capsuleCenter, planeNormal) / dot(planeNormal, rayDirection)
                    //Note that we can defer the division; we don't need to compute the exact t value of *all* planes.
                    Vector3Wide.Subtract(edgeStartBundle, localOffsetA, out var capsuleToEdge);
                    Vector3Wide.Dot(capsuleToEdge, edgePlaneNormalBundle, out var numerator);
                    Vector3Wide.Dot(edgePlaneNormalBundle, localCapsuleAxis, out var denominator);

                    //A plane is being 'entered' if the ray direction opposes the face normal.
                    //Entry denominators are always negative, exit denominators are always positive. Don't have to worry about comparison sign flips.
                    //If the denominator is zero, just ignore the lane.
                    var useLatestEntryCandidate = Vector.BitwiseAnd(Vector.LessThan(denominator, Vector<float>.Zero), Vector.GreaterThan(numerator * latestEntryDenominator, latestEntryNumerator * denominator));
                    var useEarliestExitCandidate = Vector.BitwiseAnd(Vector.GreaterThan(denominator, Vector<float>.Zero), Vector.LessThan(numerator * earliestExitDenominator, earliestExitNumerator * denominator));
                    latestEntryNumerator = Vector.ConditionalSelect(useLatestEntryCandidate, numerator, latestEntryNumerator);
                    latestEntryDenominator = Vector.ConditionalSelect(useLatestEntryCandidate, denominator, latestEntryDenominator);
                    earliestExitNumerator = Vector.ConditionalSelect(useEarliestExitCandidate, numerator, earliestExitNumerator);
                    earliestExitDenominator = Vector.ConditionalSelect(useEarliestExitCandidate, denominator, earliestExitDenominator);
                }
                var latestEntry = latestEntryNumerator / latestEntryDenominator;
                var earliestExit = Vector.Max(latestEntry, earliestExitNumerator / earliestExitDenominator);
                

            }

            manifold.FeatureId0 = Vector<int>.Zero;
            manifold.FeatureId1 = Vector<int>.Zero;
            manifold.Depth0 = depth;
            manifold.Depth1 = depth;
            manifold.Contact0Exists = Vector.GreaterThanOrEqual(manifold.Depth0, depthThreshold);
            manifold.Contact1Exists = Vector.GreaterThanOrEqual(manifold.Depth1, depthThreshold);
        }

        public void Test(ref CapsuleWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex2ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref CapsuleWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex2ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }
}
