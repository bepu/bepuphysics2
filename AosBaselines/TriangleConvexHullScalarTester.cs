using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using System.Numerics;

namespace AosBaselines;

using TriangleHullDepthRefiner = ScalarDepthRefiner<ConvexHull, HullSupportScalar, Triangle, PretransformedTriangleSupportScalar>;

/// <summary>
/// Scalar AoS port of TriangleConvexHullTester, bitwise identical per lane.
/// The triangle-versus-hull-face clipping is already per-slot scalar in the wide tester and is copied verbatim (Vector3.Dot
/// intrinsics and MathF clamps included); the ported pieces are the wide setup, the triangle-face prepass that can skip the
/// DepthRefiner, and the refiner itself. Note the refiner convergence epsilon is 1e-4f * epsilonScale here, not 1e-5f.
/// </summary>
public struct TriangleConvexHullScalarTester : IShapeHullScalarTester<Triangle>
{
    public static unsafe void Test(in Triangle a, ref ConvexHull b, float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex4ManifoldScalar manifold)
    {
        manifold = default;
        Matrix3x3.CreateFromQuaternion(orientationA, out Matrix3x3 triangleOrientation);
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 hullOrientation);
        ScalarMath.MultiplyByTranspose(triangleOrientation, hullOrientation, out var hullLocalTriangleOrientation);

        var localOffsetB = ScalarMath.TransformByTransposed(offsetB, hullOrientation);
        var localOffsetA = -localOffsetB;

        Triangle triangle;
        Matrix3x3.Transform(a.A, hullLocalTriangleOrientation, out triangle.A);
        Matrix3x3.Transform(a.B, hullLocalTriangleOrientation, out triangle.B);
        Matrix3x3.Transform(a.C, hullLocalTriangleOrientation, out triangle.C);
        var centroid = triangle.A + triangle.B;
        centroid = triangle.C + centroid;
        centroid = centroid * (1f / 3f);
        triangle.A = triangle.A - centroid;
        triangle.B = triangle.B - centroid;
        triangle.C = triangle.C - centroid;
        var localTriangleCenter = centroid - localOffsetB;
        var triangleAB = triangle.B - triangle.A;
        var triangleBC = triangle.C - triangle.B;
        var triangleCA = triangle.A - triangle.C;
        //We'll be using B-local triangle vertices quite a bit, so cache them.
        var triangleA = triangle.A + localTriangleCenter;
        var triangleB = triangle.B + localTriangleCenter;
        var triangleC = triangle.C + localTriangleCenter;
        var triangleNormal = Vector3.Cross(triangleAB, triangleCA);
        var triangleNormalLength = ScalarMath.Length(triangleNormal);
        triangleNormal = triangleNormal * (1f / triangleNormalLength);

        //Check if the hull's position is within the triangle and below the triangle plane. If so, we can ignore it.
        var hullToTriangleCenterDot = ScalarMath.Dot(triangleNormal, localTriangleCenter);
        var hullBelowPlane = hullToTriangleCenterDot >= 0f;
        var edgePlaneAB = Vector3.Cross(triangleAB, triangleNormal);
        var edgePlaneBC = Vector3.Cross(triangleBC, triangleNormal);
        var edgePlaneCA = Vector3.Cross(triangleCA, triangleNormal);
        var abPlaneTest = ScalarMath.Dot(edgePlaneAB, triangleA);
        var bcPlaneTest = ScalarMath.Dot(edgePlaneBC, triangleB);
        var caPlaneTest = ScalarMath.Dot(edgePlaneCA, triangleC);
        var hullInsideTriangleEdgePlanes = abPlaneTest <= 0f & bcPlaneTest <= 0f & caPlaneTest <= 0f;
        var hullInsideAndBelowTriangle = hullBelowPlane & hullInsideTriangleEdgePlanes;

        //ComputeNondegenerateTriangleMask mirror: LengthSquared sums of squares are association-safe.
        var abLengthSquared = triangleAB.LengthSquared();
        var caLengthSquared = triangleCA.LengthSquared();
        var triangleEpsilonScale = MathF.Sqrt(float.MaxNative(abLengthSquared, caLengthSquared));
        var nondegenerate = triangleNormalLength > TriangleWide.DegenerateTriangleEpsilon * triangleEpsilonScale;
        var hullEpsilonScale = HullScalarHelpers.EstimateEpsilonScale(ref b);
        var epsilonScale = MathF.Min(triangleEpsilonScale, hullEpsilonScale);
        //Note that degenerate triangles will not contribute contacts. They don't have a well defined normal.
        if (!nondegenerate | hullInsideAndBelowTriangle)
        {
            //No contacts generated.
            return;
        }

        //Note the use of the triangle center as the initial normal rather than the localOffsetA.
        var centerDistance = ScalarMath.Length(localTriangleCenter);
        var initialNormal = localTriangleCenter * (1f / centerDistance);
        if (centerDistance < 1e-10f)
        {
            initialNormal.X = 0f;
            initialNormal.Y = 1f;
            initialNormal.Z = 0f;
        }

        //Sample the hull's extreme point along the triangle face normal- if it's contained within the triangle edge planes, we can avoid doing more expensive refinement.
        var negatedTriangleNormal = -triangleNormal;
        var hullSupportAlongNegatedTriangleNormal = HullSupportScalar.ComputeLocalSupport(b, negatedTriangleNormal);
        var supportAlongNegatedTriangleNormal = hullSupportAlongNegatedTriangleNormal - localTriangleCenter;
        var triangleFaceDepth = ScalarMath.Dot(supportAlongNegatedTriangleNormal, negatedTriangleNormal);
        var closestToA = triangleA - hullSupportAlongNegatedTriangleNormal;
        var closestToB = triangleB - hullSupportAlongNegatedTriangleNormal;
        var closestToC = triangleC - hullSupportAlongNegatedTriangleNormal;
        var extremeABPlaneTest = ScalarMath.Dot(edgePlaneAB, closestToA);
        var extremeBCPlaneTest = ScalarMath.Dot(edgePlaneBC, closestToB);
        var extremeCAPlaneTest = ScalarMath.Dot(edgePlaneCA, closestToC);
        //Note that the triangle face extreme point can only be trusted if the hull's center is above the triangle's surface *AND* contained within the edge normals.
        var triangleNormalIsMinimal =
            (hullInsideTriangleEdgePlanes & !hullBelowPlane) &
            extremeABPlaneTest <= 0f &
            extremeBCPlaneTest <= 0f &
            extremeCAPlaneTest <= 0f;

        var depthThreshold = -speculativeMargin;
        Vector3 localNormal, closestOnHull;
        float depth;
        if (!triangleNormalIsMinimal)
        {
            TriangleHullDepthRefiner.FindMinimumDepth(b, triangle, localTriangleCenter, hullLocalTriangleOrientation, initialNormal, 1e-4f * epsilonScale, depthThreshold,
                out depth, out localNormal, out closestOnHull);
        }
        else
        {
            //The extreme point prepass did everything we needed. Just use the triangle face.
            localNormal = negatedTriangleNormal;
            closestOnHull = hullSupportAlongNegatedTriangleNormal;
            depth = triangleFaceDepth;
        }

        var triangleNormalDotLocalNormal = ScalarMath.Dot(triangleNormal, localNormal);
        if (triangleNormalDotLocalNormal > -TriangleWide.BackfaceNormalDotRejectionThreshold | depth < depthThreshold)
        {
            //No contacts generated.
            return;
        }

        var boundingPlaneEpsilon = 1e-3f * epsilonScale;
        ConvexHullPairScalarTester.PickRepresentativeFace(ref b, localNormal, closestOnHull, boundingPlaneEpsilon, out var slotFaceNormal, out var bestFaceIndex);
        b.GetVertexIndicesForFace(bestFaceIndex, out var faceVertexIndices);

        //Wide bundle math; ordered mirrors.
        var hullToA = triangleA - closestOnHull;
        var hullToB = triangleB - closestOnHull;
        var hullToC = triangleC - closestOnHull;
        var numeratorAToHullFace = ScalarMath.Dot(hullToA, slotFaceNormal);
        var numeratorBToHullFace = ScalarMath.Dot(hullToB, slotFaceNormal);
        var numeratorCToHullFace = ScalarMath.Dot(hullToC, slotFaceNormal);
        var denominatorToHullFace = ScalarMath.Dot(localNormal, slotFaceNormal);
        var inverseDenominatorToHullFace = 1f / denominatorToHullFace;
        var tAToHullFace = numeratorAToHullFace * inverseDenominatorToHullFace;
        var tBToHullFace = numeratorBToHullFace * inverseDenominatorToHullFace;
        var tCToHullFace = numeratorCToHullFace * inverseDenominatorToHullFace;
        Vector3 aOnHull, bOnHull, cOnHull;
        aOnHull.X = triangleA.X - localNormal.X * tAToHullFace;
        aOnHull.Y = triangleA.Y - localNormal.Y * tAToHullFace;
        aOnHull.Z = triangleA.Z - localNormal.Z * tAToHullFace;
        bOnHull.X = triangleB.X - localNormal.X * tBToHullFace;
        bOnHull.Y = triangleB.Y - localNormal.Y * tBToHullFace;
        bOnHull.Z = triangleB.Z - localNormal.Z * tBToHullFace;
        cOnHull.X = triangleC.X - localNormal.X * tCToHullFace;
        cOnHull.Y = triangleC.Y - localNormal.Y * tCToHullFace;
        cOnHull.Z = triangleC.Z - localNormal.Z * tCToHullFace;

        var abOnHull = bOnHull - aOnHull;
        var bcOnHull = cOnHull - bOnHull;
        var caOnHull = aOnHull - cOnHull;

        //We do not generate contacts for degenerate triangles, so we're safe to use a triangle edge as a surface basis.
        var triangleABLength = ScalarMath.Length(triangleAB);
        var triangleTangentX = triangleAB * (1f / triangleABLength);
        var triangleTangentY = Vector3.Cross(triangleTangentX, triangleNormal);

        var abEdgePlaneOnHull = Vector3.Cross(abOnHull, slotFaceNormal);
        var bcEdgePlaneOnHull = Vector3.Cross(bcOnHull, slotFaceNormal);
        var caEdgePlaneOnHull = Vector3.Cross(caOnHull, slotFaceNormal);

        var inverseTriangleNormalDotLocalNormal = 1f / triangleNormalDotLocalNormal;

        //Maximum number of edge-related contacts is 6. Maximum number of triangle vertex contacts is 3. Maximum number of hull vertex contacts is whatever the largest face is.
        int maximumContactCount = Math.Max(6, faceVertexIndices.Length);
        var candidates = stackalloc ManifoldCandidateScalar[maximumContactCount];

        //Per-slot scalar region in the wide tester; copied verbatim.
        var slotLocalNormal = localNormal;
        var slotTriangleA = triangleA;
        var slotTriangleB = triangleB;
        var slotTriangleC = triangleC;
        var slotTriangleNormal = triangleNormal;
        var slotInverseTriangleNormalDotLocalNormal = inverseTriangleNormalDotLocalNormal;
        var slotAOnHull = aOnHull;
        var slotBOnHull = bOnHull;
        var slotCOnHull = cOnHull;
        var slotTriangleTangentX = triangleTangentX;
        var slotTriangleTangentY = triangleTangentY;
        var slotABEdgePlaneOnHull = abEdgePlaneOnHull;
        var slotBCEdgePlaneOnHull = bcEdgePlaneOnHull;
        var slotCAEdgePlaneOnHull = caEdgePlaneOnHull;

        var previousIndex = faceVertexIndices[faceVertexIndices.Length - 1];
        Vector3Wide.ReadSlot(ref b.Points[previousIndex.BundleIndex], previousIndex.InnerIndex, out var hullFaceOrigin);
        var previousVertex = hullFaceOrigin;
        var candidateCount = 0;

        float latestEntryAB = float.MinValue, earliestExitAB = float.MaxValue;
        float latestEntryBC = float.MinValue, earliestExitBC = float.MaxValue;
        float latestEntryCA = float.MinValue, earliestExitCA = float.MaxValue;

        var slotABOnHull = slotBOnHull - slotAOnHull;
        var slotBCOnHull = slotCOnHull - slotBOnHull;
        var slotCAOnHull = slotAOnHull - slotCOnHull;
        var slotTriangleAB = slotTriangleB - slotTriangleA;
        var slotTriangleBC = slotTriangleC - slotTriangleB;
        var slotTriangleCA = slotTriangleA - slotTriangleC;

        for (int i = 0; i < faceVertexIndices.Length; ++i)
        {
            var index = faceVertexIndices[i];
            Vector3Wide.ReadSlot(ref b.Points[index.BundleIndex], index.InnerIndex, out var vertex);

            var hullEdgeOffset = vertex - previousVertex;
            previousIndex = index;
            previousVertex = vertex;
            var ap = vertex - slotAOnHull;
            var bp = vertex - slotBOnHull;
            //Note that the edge planes could be zero if the projected edge has zero length. In that case, containment is impossible. So, use strict inequality.
            var vertexContained = Vector3.Dot(ap, slotABEdgePlaneOnHull) < 0 && Vector3.Dot(bp, slotBCEdgePlaneOnHull) < 0 && Vector3.Dot(ap, slotCAEdgePlaneOnHull) < 0;
            if (vertexContained && candidateCount < maximumContactCount)
            {
                //Project the hull vertex down to the triangle's surface.
                var projectionT = Vector3.Dot(vertex - slotTriangleA, slotTriangleNormal) * slotInverseTriangleNormalDotLocalNormal;
                var projectedVertex = vertex - slotLocalNormal * projectionT;
                var newContactIndex = candidateCount++;
                ref var candidate = ref candidates[newContactIndex];
                //Use triangle.A as the surface basis origin.
                var toVertex = projectedVertex - slotTriangleA;
                candidate.X = Vector3.Dot(toVertex, slotTriangleTangentX);
                candidate.Y = Vector3.Dot(toVertex, slotTriangleTangentY);
                //Vertex contacts occupy the feature indices after the edge slots.
                candidate.FeatureId = 6 + i;
            }

            //Intersect the three triangle edges against the hull edge.
            //Use the sign of the denominator to determine if a triangle edge is entering or exiting a given hull edge.
            var hullEdgePlaneNormal = Vector3.Cross(hullEdgeOffset, slotLocalNormal);
            var abNumerator = Vector3.Dot(ap, hullEdgePlaneNormal);
            var abDenominator = Vector3.Dot(hullEdgePlaneNormal, slotABOnHull);
            if (abDenominator < 0)
            {
                if (latestEntryAB * abDenominator > abNumerator) //Note sign flip for comparison.
                    latestEntryAB = abNumerator / abDenominator;
            }
            else if (abDenominator > 0)
            {
                if (earliestExitAB * abDenominator > abNumerator)
                    earliestExitAB = abNumerator / abDenominator;
            }
            else if (abDenominator == 0)
            {
                if (abNumerator < 0)
                {
                    //Parallel and outside the hull face; hull face intersection interval does not exist.
                    earliestExitAB = float.MinValue;
                    latestEntryAB = float.MaxValue;
                }
            }
            var bcNumerator = Vector3.Dot(bp, hullEdgePlaneNormal);
            var bcDenominator = Vector3.Dot(hullEdgePlaneNormal, slotBCOnHull);
            if (bcDenominator < 0)
            {
                if (latestEntryBC * bcDenominator > bcNumerator) //Note sign flip for comparison.
                    latestEntryBC = bcNumerator / bcDenominator;
            }
            else if (bcDenominator > 0)
            {
                if (earliestExitBC * bcDenominator > bcNumerator)
                    earliestExitBC = bcNumerator / bcDenominator;
            }
            else if (bcDenominator == 0)
            {
                if (bcNumerator < 0)
                {
                    earliestExitBC = float.MinValue;
                    latestEntryBC = float.MaxValue;
                }
            }
            var caNumerator = Vector3.Dot(vertex - slotCOnHull, hullEdgePlaneNormal);
            var caDenominator = Vector3.Dot(hullEdgePlaneNormal, slotCAOnHull);
            if (caDenominator < 0)
            {
                if (latestEntryCA * caDenominator > caNumerator) //Note sign flip for comparison.
                    latestEntryCA = caNumerator / caDenominator;
            }
            else if (caDenominator > 0)
            {
                if (earliestExitCA * caDenominator > caNumerator)
                    earliestExitCA = caNumerator / caDenominator;
            }
            else if (caDenominator == 0)
            {
                if (caNumerator < 0)
                {
                    earliestExitCA = float.MinValue;
                    latestEntryCA = float.MaxValue;
                }
            }
        }

        //We now have triangle edge intervals. Add contacts for them.
        latestEntryAB = MathF.Max(latestEntryAB, 0);
        latestEntryBC = MathF.Max(latestEntryBC, 0);
        latestEntryCA = MathF.Max(latestEntryCA, 0);
        earliestExitAB = MathF.Min(earliestExitAB, 1);
        earliestExitBC = MathF.Min(earliestExitBC, 1);
        earliestExitCA = MathF.Min(earliestExitCA, 1);
        //Create max contact if max >= min.
        //Create min if min < max and min > 0.
        if (earliestExitAB >= latestEntryAB && candidateCount < maximumContactCount)
        {
            //Create max contact.
            var point = slotTriangleAB * earliestExitAB; //Note triangle A is origin for surface basis.
            var newContactIndex = candidateCount++;
            ref var candidate = ref candidates[newContactIndex];
            candidate.X = Vector3.Dot(point, slotTriangleTangentX);
            candidate.Y = Vector3.Dot(point, slotTriangleTangentY);
            candidate.FeatureId = 0;

        }
        if (latestEntryAB < earliestExitAB && latestEntryAB > 0 && candidateCount < maximumContactCount)
        {
            //Create min contact.
            var point = slotTriangleAB * latestEntryAB; //Note triangle A is origin for surface basis.
            var newContactIndex = candidateCount++;
            ref var candidate = ref candidates[newContactIndex];
            candidate.X = Vector3.Dot(point, slotTriangleTangentX);
            candidate.Y = Vector3.Dot(point, slotTriangleTangentY);
            candidate.FeatureId = 1;
        }
        if (earliestExitBC >= latestEntryBC && candidateCount < maximumContactCount)
        {
            //Create max contact.
            var point = slotTriangleBC * earliestExitBC + slotTriangleAB;
            var newContactIndex = candidateCount++;
            ref var candidate = ref candidates[newContactIndex];
            candidate.X = Vector3.Dot(point, slotTriangleTangentX);
            candidate.Y = Vector3.Dot(point, slotTriangleTangentY);
            candidate.FeatureId = 2;

        }
        if (latestEntryBC < earliestExitBC && latestEntryBC > 0 && candidateCount < maximumContactCount)
        {
            //Create min contact.
            var point = slotTriangleBC * latestEntryBC + slotTriangleAB;
            var newContactIndex = candidateCount++;
            ref var candidate = ref candidates[newContactIndex];
            candidate.X = Vector3.Dot(point, slotTriangleTangentX);
            candidate.Y = Vector3.Dot(point, slotTriangleTangentY);
            candidate.FeatureId = 3;
        }
        if (earliestExitCA >= latestEntryCA && candidateCount < maximumContactCount)
        {
            //Create max contact.
            var point = slotTriangleCA * earliestExitCA - slotTriangleCA;
            var newContactIndex = candidateCount++;
            ref var candidate = ref candidates[newContactIndex];
            candidate.X = Vector3.Dot(point, slotTriangleTangentX);
            candidate.Y = Vector3.Dot(point, slotTriangleTangentY);
            candidate.FeatureId = 4;

        }
        if (latestEntryCA < earliestExitCA && latestEntryCA > 0 && candidateCount < maximumContactCount)
        {
            //Create min contact.
            var point = slotTriangleCA * latestEntryCA - slotTriangleCA;
            var newContactIndex = candidateCount++;
            ref var candidate = ref candidates[newContactIndex];
            candidate.X = Vector3.Dot(point, slotTriangleTangentX);
            candidate.Y = Vector3.Dot(point, slotTriangleTangentY);
            candidate.FeatureId = 5;
        }

        //We have found all contacts for this hull slot. There may be more contacts than we want (4), so perform a reduction.
        ConvexHullPairScalarTester.Reduce(candidates, candidateCount, slotFaceNormal, -1f / Vector3.Dot(slotFaceNormal, slotLocalNormal), previousVertex, slotTriangleA, slotTriangleTangentX, slotTriangleTangentY,
            epsilonScale, depthThreshold, hullOrientation, offsetB, ref manifold);

        //The reduction does not assign the normal. Fill it in.
        Matrix3x3.Transform(localNormal, hullOrientation, out manifold.Normal);
        //Mesh reductions also make use of a face contact flag in the feature id.
        if (triangleNormalDotLocalNormal < -MeshReduction.MinimumDotForFaceCollision)
            manifold.FeatureId0 += MeshReduction.FaceCollisionFlag;
    }
}
