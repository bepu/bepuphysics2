using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace AosBaselines;

/// <summary>
/// Track 1 relaxed candidate: identical algorithm to the frozen scalar hull-hull tester (same setup, same ScalarDepthRefiner,
/// same face pick / clip / Reduce), but the hull support function's full point scan is replaced by a warm-started adjacency
/// hillclimb over the precomputed CSR topology. Vertex reads in the contact generation also come from the topology's scalar
/// vertex array instead of bundle ReadSlots (same values, cheaper loads).
/// </summary>
public sealed class HillclimbRelaxedTester : IRelaxedHullPairTester
{
    public string Name => "hillclimb";
    public void Test(HullTopology topologyA, HullTopology topologyB, ref ConvexHull a, ref ConvexHull b,
        float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex4ManifoldScalar manifold)
    {
        HillclimbHullPairTester.Test<HullSupportScalarHillclimb>(topologyA, topologyB, ref a, ref b,
            speculativeMargin, offsetB, orientationA, orientationB, out manifold);
    }
}

/// <summary>
/// Same tester with the cold hillclimb finder (fixed start at vertex 0, no warm start) to isolate the warm start's contribution.
/// </summary>
public sealed class HillclimbColdRelaxedTester : IRelaxedHullPairTester
{
    public string Name => "hillclimbcold";
    public void Test(HullTopology topologyA, HullTopology topologyB, ref ConvexHull a, ref ConvexHull b,
        float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex4ManifoldScalar manifold)
    {
        HillclimbHullPairTester.Test<HullSupportScalarHillclimbCold>(topologyA, topologyB, ref a, ref b,
            speculativeMargin, offsetB, orientationA, orientationB, out manifold);
    }
}

static class HillclimbHullPairTester
{
    struct CachedEdge
    {
        public Vector3 Vertex;
        public Vector3 EdgePlaneNormal;
        public float MaximumContainmentDot;
    }

    public static unsafe void Test<TSupport>(HullTopology topologyA, HullTopology topologyB, ref ConvexHull a, ref ConvexHull b,
        float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex4ManifoldScalar manifold)
        where TSupport : IScalarSupportFinder<HillclimbHull>
    {
        manifold = default;
        Matrix3x3.CreateFromQuaternion(orientationA, out Matrix3x3 rA);
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 rB);
        ScalarMath.MultiplyByTranspose(rA, rB, out var bLocalOrientationA);

        var localOffsetB = ScalarMath.TransformByTransposed(offsetB, rB);
        var localOffsetA = -localOffsetB;
        var centerDistance = ScalarMath.Length(localOffsetA);
        var initialNormal = localOffsetA * (1f / centerDistance);
        if (centerDistance < 1e-8f)
        {
            initialNormal = new Vector3(0f, 1f, 0f);
        }
        //EstimateEpsilonScale equivalent: hull point (0,0) is linear vertex 0 in the topology.
        var firstPointA = topologyA.Vertices[0];
        var firstPointB = topologyB.Vertices[0];
        var aEpsilonScale = (MathF.Abs(firstPointA.X) + MathF.Abs(firstPointA.Y) + MathF.Abs(firstPointA.Z)) * (1f / 3f);
        var bEpsilonScale = (MathF.Abs(firstPointB.X) + MathF.Abs(firstPointB.Y) + MathF.Abs(firstPointB.Z)) * (1f / 3f);
        var epsilonScale = float.MinNative(aEpsilonScale, bEpsilonScale);
        var depthThreshold = -speculativeMargin;

        //Per-pair warm start caches, one per hull. The refiner's shape A is hull B (identity orientation) and its shape B is
        //hull A (see the frozen tester / SCOUT notes on the A/B flip). Initial guess: vertex 0.
        int warmStartA = 0, warmStartB = 0;
        var refinerShapeA = new HillclimbHull { Topology = topologyB, WarmStartSlot = &warmStartB };
        var refinerShapeB = new HillclimbHull { Topology = topologyA, WarmStartSlot = &warmStartA };
        ScalarDepthRefiner<HillclimbHull, TSupport, HillclimbHull, TSupport>.FindMinimumDepth(
            refinerShapeA, refinerShapeB, localOffsetA, bLocalOrientationA, initialNormal, 1e-5f * epsilonScale, depthThreshold,
            out var depth, out var localNormal, out var closestOnB);

        if (depth < depthThreshold)
        {
            //No contacts generated.
            return;
        }

        var localNormalInA = ScalarMath.TransformByTransposed(localNormal, bLocalOrientationA);
        var negatedLocalNormalInA = -localNormalInA;
        var negatedOffsetToClosestOnA = localNormal * depth;
        var closestOnA = closestOnB - negatedOffsetToClosestOnA;
        var aToClosestOnA = closestOnA - localOffsetA;
        var closestOnAInA = ScalarMath.TransformByTransposed(aToClosestOnA, bLocalOrientationA);

        var boundingPlaneEpsilon = 1e-3f * epsilonScale;

        ConvexHullPairScalarTester.PickRepresentativeFace(ref a, negatedLocalNormalInA, closestOnAInA, boundingPlaneEpsilon, out var slotFaceNormalAInA, out var bestFaceIndexA);
        Matrix3x3.Transform(slotFaceNormalAInA, bLocalOrientationA, out var slotFaceNormalA);
        ConvexHullPairScalarTester.PickRepresentativeFace(ref b, localNormal, closestOnB, boundingPlaneEpsilon, out var slotFaceNormalB, out var bestFaceIndexB);
        Helpers.BuildOrthonormalBasis(slotFaceNormalB, out var bFaceX, out var bFaceY);

        //Contact generation mirrors the frozen tester's structure; vertex reads use the topology's scalar arrays
        //(identical values to the bundle ReadSlots, contiguous loads) and linear indices double as feature ids.
        var slotLocalNormal = localNormal;
        var slotLocalOffsetA = localOffsetA;

        ref var faceVerticesA = ref MemoryMarshal.GetArrayDataReference(topologyA.FaceVertices);
        ref var verticesA = ref MemoryMarshal.GetArrayDataReference(topologyA.Vertices);
        ref var faceVerticesB = ref MemoryMarshal.GetArrayDataReference(topologyB.FaceVertices);
        ref var verticesB = ref MemoryMarshal.GetArrayDataReference(topologyB.Vertices);
        var faceStartA = topologyA.FaceStarts[bestFaceIndexA];
        var faceCountA = topologyA.FaceStarts[bestFaceIndexA + 1] - faceStartA;
        var faceStartB = topologyB.FaceStarts[bestFaceIndexB];
        var faceCountB = topologyB.FaceStarts[bestFaceIndexB + 1] - faceStartB;

        //Create cached edge data for A.
        var cachedEdges = stackalloc CachedEdge[faceCountA];
        var previousVertexA = Unsafe.Add(ref verticesA, Unsafe.Add(ref faceVerticesA, faceStartA + faceCountA - 1));
        Matrix3x3.Transform(previousVertexA, bLocalOrientationA, out previousVertexA);
        previousVertexA += slotLocalOffsetA;
        for (int i = 0; i < faceCountA; ++i)
        {
            ref var edge = ref cachedEdges[i];
            edge.MaximumContainmentDot = float.MinValue;
            edge.Vertex = Unsafe.Add(ref verticesA, Unsafe.Add(ref faceVerticesA, faceStartA + i));
            Matrix3x3.Transform(edge.Vertex, bLocalOrientationA, out edge.Vertex);
            edge.Vertex += slotLocalOffsetA;
            //Note flipped cross order; local normal points from B to A.
            edge.EdgePlaneNormal = Vector3.Cross(slotLocalNormal, edge.Vertex - previousVertexA);
            previousVertexA = edge.Vertex;
        }
        var maximumCandidateCount = Math.Max(Math.Max(faceCountA, faceCountB), Math.Min(faceCountA * 2, faceCountB * 2));
        var candidates = stackalloc ManifoldCandidateScalar[maximumCandidateCount];
        var candidateCount = 0;
        var previousIndexB = Unsafe.Add(ref faceVerticesB, faceStartB + faceCountB - 1);
        //Clip face B's edges against A's face, and test A's vertices against B's face.
        var bFaceOrigin = Unsafe.Add(ref verticesB, previousIndexB);
        var previousVertexB = bFaceOrigin;
        for (int faceVertexIndexB = 0; faceVertexIndexB < faceCountB; ++faceVertexIndexB)
        {
            var indexB = Unsafe.Add(ref faceVerticesB, faceStartB + faceVertexIndexB);
            var vertexB = Unsafe.Add(ref verticesB, indexB);

            var edgeOffsetB = vertexB - previousVertexB;
            var edgePlaneNormalB = Vector3.Cross(edgeOffsetB, slotLocalNormal);

            var latestEntry = float.MinValue;
            var earliestExit = float.MaxValue;
            for (int faceVertexIndexA = 0; faceVertexIndexA < faceCountA; ++faceVertexIndexA)
            {
                ref var edgeA = ref cachedEdges[faceVertexIndexA];

                //Check containment in this B edge.
                var edgeBToEdgeA = edgeA.Vertex - previousVertexB;
                var containmentDot = Vector3.Dot(edgeBToEdgeA, edgePlaneNormalB);
                if (edgeA.MaximumContainmentDot < containmentDot)
                    edgeA.MaximumContainmentDot = containmentDot;

                var numerator = Vector3.Dot(edgeBToEdgeA, edgeA.EdgePlaneNormal);
                var denominator = Vector3.Dot(edgeA.EdgePlaneNormal, edgeOffsetB);

                //A plane is being 'entered' if the ray direction opposes the face normal.
                if (denominator < 0)
                {
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
            if (latestEntry <= earliestExit)
            {
                latestEntry = latestEntry < 0 ? 0 : latestEntry;
                earliestExit = earliestExit > 1 ? 1 : earliestExit;
                var startId = previousIndexB;
                var endId = indexB;
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
        for (int i = 0; i < faceCountA && candidateCount < maximumCandidateCount; ++i)
        {
            ref var edge = ref cachedEdges[i];
            if (edge.MaximumContainmentDot <= 0)
            {
                //This vertex was contained by all b edge plane normals. Include it, projected onto B's surface.
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
        ConvexHullPairScalarTester.Reduce(candidates, candidateCount, slotFaceNormalA, 1f / Vector3.Dot(slotFaceNormalA, slotLocalNormal), cachedEdges[0].Vertex, bFaceOrigin, bFaceX, bFaceY,
            epsilonScale, depthThreshold, rB, offsetB, ref manifold);
        Matrix3x3.Transform(localNormal, rB, out manifold.Normal);
    }
}
