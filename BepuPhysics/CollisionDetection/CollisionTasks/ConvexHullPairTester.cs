using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct ConvexHullPairTester : IPairTester<ConvexHullWide, ConvexHullWide, Convex4ContactManifoldWide>
    {
        struct CachedEdge
        {
            public Vector3 Vertex;
            public Vector3 EdgePlaneNormal;
            public float MaximumContainmentDot;
        }
        public int BatchSize => 16;

        public unsafe void Test(ref ConvexHullWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            Unsafe.SkipInit(out manifold);
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var rA);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var rB);
            Matrix3x3Wide.MultiplyByTransposeWithoutOverlap(rA, rB, out var bLocalOrientationA);

            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, rB, out var localOffsetB);
            Vector3Wide.Negate(localOffsetB, out var localOffsetA);
            Vector3Wide.Length(localOffsetA, out var centerDistance);
            Vector3Wide.Scale(localOffsetA, Vector<float>.One / centerDistance, out var initialNormal);
            var useInitialFallback = Vector.LessThan(centerDistance, new Vector<float>(1e-8f));
            initialNormal.X = Vector.ConditionalSelect(useInitialFallback, Vector<float>.Zero, initialNormal.X);
            initialNormal.Y = Vector.ConditionalSelect(useInitialFallback, Vector<float>.One, initialNormal.Y);
            initialNormal.Z = Vector.ConditionalSelect(useInitialFallback, Vector<float>.Zero, initialNormal.Z);
            var hullSupportFinder = default(ConvexHullSupportFinder);
            var inactiveLanes = BundleIndexing.CreateTrailingMaskForCountInBundle(pairCount);
            a.EstimateEpsilonScale(inactiveLanes, out var aEpsilonScale);
            b.EstimateEpsilonScale(inactiveLanes, out var bEpsilonScale);
            var epsilonScale = Vector.Min(aEpsilonScale, bEpsilonScale);
            var depthThreshold = -speculativeMargin;
            DepthRefiner<ConvexHull, ConvexHullWide, ConvexHullSupportFinder, ConvexHull, ConvexHullWide, ConvexHullSupportFinder>.FindMinimumDepth(
                b, a, localOffsetA, bLocalOrientationA, ref hullSupportFinder, ref hullSupportFinder, initialNormal, inactiveLanes, 1e-5f * epsilonScale, depthThreshold,
                out var depth, out var localNormal, out var closestOnB);

            inactiveLanes = Vector.BitwiseOr(inactiveLanes, Vector.LessThan(depth, depthThreshold));
            //Not every lane will generate contacts. Rather than requiring every lane to carefully clear all contactExists states, just clear them up front.
            manifold.Contact0Exists = default;
            manifold.Contact1Exists = default;
            manifold.Contact2Exists = default;
            manifold.Contact3Exists = default;

            if (Vector.LessThanAll(inactiveLanes, Vector<int>.Zero))
            {
                //No contacts generated.
                return;
            }

            Matrix3x3Wide.TransformByTransposedWithoutOverlap(localNormal, bLocalOrientationA, out var localNormalInA);
            Vector3Wide.Negate(localNormalInA, out var negatedLocalNormalInA);
            Vector3Wide.Scale(localNormal, depth, out var negatedOffsetToClosestOnA);
            Vector3Wide.Subtract(closestOnB, negatedOffsetToClosestOnA, out var closestOnA);
            Vector3Wide.Subtract(closestOnA, localOffsetA, out var aToClosestOnA);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(aToClosestOnA, bLocalOrientationA, out var closestOnAInA);

            //To find the contact manifold, we'll clip the capsule axis against the face as usual, but we're dealing with potentially
            //distinct convex hulls. Rather than vectorizing over the different hulls, we vectorize within each hull.
            Helpers.FillVectorWithLaneIndices(out var slotOffsetIndices);
            var boundingPlaneEpsilon = 1e-3f * epsilonScale;

            for (int slotIndex = 0; slotIndex < pairCount; ++slotIndex)
            {
                if (inactiveLanes[slotIndex] < 0)
                    continue;
                ref var aSlot = ref a.Hulls[slotIndex];
                ref var bSlot = ref b.Hulls[slotIndex];
                ConvexHullTestHelper.PickRepresentativeFace(ref aSlot, slotIndex, ref negatedLocalNormalInA, closestOnAInA, slotOffsetIndices, ref boundingPlaneEpsilon, out var slotFaceNormalAInA, out _, out var bestFaceIndexA);
                Matrix3x3Wide.ReadSlot(ref bLocalOrientationA, slotIndex, out var slotBLocalOrientationA);
                Matrix3x3.Transform(slotFaceNormalAInA, slotBLocalOrientationA, out var slotFaceNormalA);
                Vector3Wide.ReadSlot(ref localOffsetA, slotIndex, out var slotLocalOffsetA);
                ConvexHullTestHelper.PickRepresentativeFace(ref bSlot, slotIndex, ref localNormal, closestOnB, slotOffsetIndices, ref boundingPlaneEpsilon, out var slotFaceNormalB, out var slotLocalNormal, out var bestFaceIndexB);
                Helpers.BuildOrthonormalBasis(slotFaceNormalB, out var bFaceX, out var bFaceY);

                //Test each face edge plane against the capsule edge.
                //Note that we do not use the faceNormal x edgeOffset edge plane, but rather edgeOffset x localNormal.
                //(In other words, testing the *projected* capsule axis on the surface of the convex hull face.)
                //The faces are wound counterclockwise in right handed coordinates.
                aSlot.GetVertexIndicesForFace(bestFaceIndexA, out var faceVertexIndicesA);
                bSlot.GetVertexIndicesForFace(bestFaceIndexB, out var faceVertexIndicesB);

                //Create cached edge data for A.
                var cachedEdges = stackalloc CachedEdge[faceVertexIndicesA.Length];
                var previousIndexA = faceVertexIndicesA[faceVertexIndicesA.Length - 1];
                Vector3Wide.ReadSlot(ref aSlot.Points[previousIndexA.BundleIndex], previousIndexA.InnerIndex, out var previousVertexA);
                Matrix3x3.Transform(previousVertexA, slotBLocalOrientationA, out previousVertexA);
                previousVertexA += slotLocalOffsetA;
                for (int i = 0; i < faceVertexIndicesA.Length; ++i)
                {
                    ref var edge = ref cachedEdges[i];
                    edge.MaximumContainmentDot = float.MinValue;
                    var indexA = faceVertexIndicesA[i];
                    Vector3Wide.ReadSlot(ref aSlot.Points[indexA.BundleIndex], indexA.InnerIndex, out edge.Vertex);
                    Matrix3x3.Transform(edge.Vertex, slotBLocalOrientationA, out edge.Vertex);
                    edge.Vertex += slotLocalOffsetA;
                    //Note flipped cross order; local normal points from B to A.
                    edge.EdgePlaneNormal = Vector3.Cross(slotLocalNormal, edge.Vertex - previousVertexA);
                    previousVertexA = edge.Vertex;
                }
                var maximumCandidateCount = faceVertexIndicesB.Length * 2; //Two contacts per edge.
                var candidates = stackalloc ManifoldCandidateScalar[maximumCandidateCount];
                var candidateCount = 0;
                var previousIndexB = faceVertexIndicesB[faceVertexIndicesB.Length - 1];
                //Clip face B's edges against A's face, and test A's vertices against B's face.
                //We use B's face as the contact surface to be consistent with the other pairs in case we end up implementing a HullCollectionReduction similar to MeshReduction.
                Vector3Wide.ReadSlot(ref bSlot.Points[previousIndexB.BundleIndex], previousIndexB.InnerIndex, out var bFaceOrigin);
                var previousVertexB = bFaceOrigin;
                for (int faceVertexIndexB = 0; faceVertexIndexB < faceVertexIndicesB.Length; ++faceVertexIndexB)
                {
                    var indexB = faceVertexIndicesB[faceVertexIndexB];
                    Vector3Wide.ReadSlot(ref bSlot.Points[indexB.BundleIndex], indexB.InnerIndex, out var vertexB);

                    var edgeOffsetB = vertexB - previousVertexB;
                    var edgePlaneNormalB = Vector3.Cross(edgeOffsetB, slotLocalNormal);

                    var latestEntry = float.MinValue;
                    var earliestExit = float.MaxValue;
                    for (int faceVertexIndexA = 0; faceVertexIndexA < faceVertexIndicesA.Length; ++faceVertexIndexA)
                    {
                        ref var edgeA = ref cachedEdges[faceVertexIndexA];

                        //Check containment in this B edge.
                        var edgeBToEdgeA = edgeA.Vertex - previousVertexB;
                        var containmentDot = Vector3.Dot(edgeBToEdgeA, edgePlaneNormalB);
                        if (edgeA.MaximumContainmentDot < containmentDot)
                            edgeA.MaximumContainmentDot = containmentDot;

                        //t = dot(pointOnEdgeA - pointOnEdgeB, edgePlaneNormalA) / dot(edgePlaneNormalA, edgeOffsetB)
                        //Note that we can defer the division; we don't need to compute the exact t value of *all* planes.

                        var numerator = Vector3.Dot(edgeBToEdgeA, edgeA.EdgePlaneNormal);
                        var denominator = Vector3.Dot(edgeA.EdgePlaneNormal, edgeOffsetB);

                        //A plane is being 'entered' if the ray direction opposes the face normal.
                        //Entry denominators are always negative, exit denominators are always positive. Don't have to worry about comparison sign flips.
                        if (denominator < 0)
                        {
                            //Note compare flip for denominator sign.
                            if (numerator < latestEntry * denominator)
                                latestEntry = numerator / denominator;
                        }
                        else if (denominator > 0)
                        {
                            if (numerator < earliestExit * denominator)
                                earliestExit = numerator / denominator;
                        }
                        else if (numerator < 0)
                        {
                            //The B edge is parallel and outside the edge A, so there can be no intersection.
                            earliestExit = float.MinValue;
                            latestEntry = float.MaxValue;
                        }
                    }
                    //We now have bounds on B's edge.
                    //Denominator signs are opposed; comparison flipped.
                    if (latestEntry <= earliestExit)
                    {
                        //This edge of B was actually contained in A's face. Add contacts for it.
                        latestEntry = latestEntry < 0 ? 0 : latestEntry;
                        earliestExit = earliestExit > 1 ? 1 : earliestExit;
                        //Create max contact if max >= min.
                        //Create min if min < max and min > 0.
                        var startId = (previousIndexB.BundleIndex << BundleIndexing.VectorShift) + previousIndexB.InnerIndex;
                        var endId = (indexB.BundleIndex << BundleIndexing.VectorShift) + indexB.InnerIndex;
                        var baseFeatureId = (startId ^ endId) << 8;
                        if (earliestExit >= latestEntry && candidateCount < maximumCandidateCount)
                        {
                            //Create max contact.
                            var point = edgeOffsetB * earliestExit + previousVertexB - bFaceOrigin;
                            var newContactIndex = candidateCount++;
                            ref var candidate = ref candidates[newContactIndex];
                            candidate.X = Vector3.Dot(point, bFaceX);
                            candidate.Y = Vector3.Dot(point, bFaceY);
                            candidate.FeatureId = baseFeatureId + endId;
                        }
                        if (latestEntry < earliestExit && latestEntry > 0 && candidateCount < maximumCandidateCount)
                        {
                            //Create min contact.
                            var point = edgeOffsetB * latestEntry + previousVertexB - bFaceOrigin;
                            var newContactIndex = candidateCount++;
                            ref var candidate = ref candidates[newContactIndex];
                            candidate.X = Vector3.Dot(point, bFaceX);
                            candidate.Y = Vector3.Dot(point, bFaceY);
                            candidate.FeatureId = baseFeatureId + startId;
                        }
                    }
                    previousIndexB = indexB;
                    previousVertexB = vertexB;
                }
                //We've now analyzed every edge of B. Check for vertices from A to add.
                var inverseLocalNormalADotFaceNormalB = 1f / Vector3.Dot(slotLocalNormal, slotFaceNormalB);
                for (int i = 0; i < faceVertexIndicesA.Length && candidateCount < maximumCandidateCount; ++i)
                {
                    ref var edge = ref cachedEdges[i];
                    if (edge.MaximumContainmentDot <= 0)
                    {
                        //This vertex was contained by all b edge plane normals. Include it.
                        //Project it onto B's surface:
                        //vertexA - localNormal * dot(vertexA - faceOriginB, faceNormalB) / dot(localNormal, faceNormalB); 
                        var bFaceToVertexA = edge.Vertex - bFaceOrigin;
                        var distance = Vector3.Dot(bFaceToVertexA, slotFaceNormalB) * inverseLocalNormalADotFaceNormalB;
                        var bFaceToProjectedVertexA = bFaceToVertexA - slotLocalNormal * distance;

                        var newContactIndex = candidateCount++;
                        ref var candidate = ref candidates[newContactIndex];
                        candidate.X = Vector3.Dot(bFaceX, bFaceToProjectedVertexA);
                        candidate.Y = Vector3.Dot(bFaceY, bFaceToProjectedVertexA);
                        candidate.FeatureId = i;
                    }
                }
                Matrix3x3Wide.ReadSlot(ref rB, slotIndex, out var slotOrientationB);
                Vector3Wide.ReadSlot(ref offsetB, slotIndex, out var slotOffsetB);
                ManifoldCandidateHelper.Reduce(candidates, candidateCount, slotFaceNormalA, 1f / Vector3.Dot(slotFaceNormalA, slotLocalNormal), cachedEdges[0].Vertex, bFaceOrigin, bFaceX, bFaceY, epsilonScale[slotIndex], depthThreshold[slotIndex], slotOrientationB, slotOffsetB, slotIndex, ref manifold);
            }
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, rB, out manifold.Normal);
        }

        public void Test(ref ConvexHullWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref ConvexHullWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }
}
