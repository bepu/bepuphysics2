using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using System.Numerics;

namespace AosBaselines;

using BoxHullDepthRefiner = ScalarDepthRefiner<ConvexHull, HullSupportScalar, Box, BoxSupportScalar>;

/// <summary>
/// Scalar AoS port of BoxConvexHullTester, bitwise identical per lane.
/// The clipping of the box face against the hull face is already per-slot scalar (with within-slot Vector4 SIMD) in the wide
/// tester and is copied verbatim; the ported pieces are the wide setup, box face identification, and the DepthRefiner.
/// </summary>
public struct BoxConvexHullScalarTester : IShapeHullScalarTester<Box>
{
    public static unsafe void Test(in Box a, ref ConvexHull b, float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex4ManifoldScalar manifold)
    {
        manifold = default;
        Matrix3x3.CreateFromQuaternion(orientationA, out Matrix3x3 boxOrientation);
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 hullOrientation);
        ScalarMath.MultiplyByTranspose(boxOrientation, hullOrientation, out var hullLocalBoxOrientation);

        var localOffsetB = ScalarMath.TransformByTransposed(offsetB, hullOrientation);
        var localOffsetA = -localOffsetB;
        var centerDistance = ScalarMath.Length(localOffsetA);
        var initialNormal = localOffsetA * (1f / centerDistance);
        if (centerDistance < 1e-8f)
        {
            initialNormal.X = 0f;
            initialNormal.Y = 1f;
            initialNormal.Z = 0f;
        }
        var hullEpsilonScale = HullScalarHelpers.EstimateEpsilonScale(ref b);
        //MathF.Min/Max mirror Vector.Min/Max's IEEE semantics on .NET 10 (see ScalarDepthRefiner notes).
        var epsilonScale = MathF.Min(MathF.Max(a.HalfWidth, MathF.Max(a.HalfHeight, a.HalfLength)), hullEpsilonScale);
        var depthThreshold = -speculativeMargin;
        BoxHullDepthRefiner.FindMinimumDepth(b, a, localOffsetA, hullLocalBoxOrientation, initialNormal, 1e-5f * epsilonScale, depthThreshold,
            out var depth, out var localNormal, out var closestOnHull);

        if (depth < depthThreshold)
        {
            //No contacts generated.
            return;
        }

        //Identify the box face.
        var localNormalInA = ScalarMath.TransformByTransposed(localNormal, hullLocalBoxOrientation);
        var absLocalNormalInA = Vector3.Abs(localNormalInA);
        var useX = absLocalNormalInA.X > absLocalNormalInA.Y & absLocalNormalInA.X > absLocalNormalInA.Z;
        var useY = absLocalNormalInA.Y > absLocalNormalInA.Z & !useX;
        var boxFaceNormal = useY ? hullLocalBoxOrientation.Y : useX ? hullLocalBoxOrientation.X : hullLocalBoxOrientation.Z;
        var boxFaceX = useY ? hullLocalBoxOrientation.Z : useX ? hullLocalBoxOrientation.Y : hullLocalBoxOrientation.X;
        var boxFaceY = useY ? hullLocalBoxOrientation.X : useX ? hullLocalBoxOrientation.Z : hullLocalBoxOrientation.Y;
        var negateFace = useX ? localNormalInA.X > 0f : useY ? localNormalInA.Y > 0f : localNormalInA.Z > 0f;
        if (negateFace)
            boxFaceNormal = -boxFaceNormal;
        //Winding is important; flip the face bases if necessary.
        if (!negateFace)
            boxFaceX = -boxFaceX;
        var boxFaceHalfWidth = useX ? a.HalfHeight : useY ? a.HalfLength : a.HalfWidth;
        var boxFaceHalfHeight = useX ? a.HalfLength : useY ? a.HalfWidth : a.HalfHeight;
        var boxFaceNormalOffset = useX ? a.HalfWidth : useY ? a.HalfHeight : a.HalfLength;
        var boxFaceCenterOffset = boxFaceNormal * boxFaceNormalOffset;
        var boxFaceCenter = boxFaceCenterOffset + localOffsetA;
        var boxFaceXOffset = boxFaceX * boxFaceHalfWidth;
        var boxFaceYOffset = boxFaceY * boxFaceHalfHeight;
        var v0 = boxFaceCenter - boxFaceXOffset;
        var v1 = boxFaceCenter + boxFaceXOffset;
        var v00 = v0 - boxFaceYOffset;
        var v01 = v0 + boxFaceYOffset;
        var v10 = v1 - boxFaceYOffset;
        var v11 = v1 + boxFaceYOffset;

        var boundingPlaneEpsilon = 1e-3f * epsilonScale;
        ConvexHullPairScalarTester.PickRepresentativeFace(ref b, localNormal, closestOnHull, boundingPlaneEpsilon, out var slotFaceNormal, out var bestFaceIndex);
        b.GetVertexIndicesForFace(bestFaceIndex, out var faceVertexIndices);

        //There can be no more than 8 contacts from edge intersections, but more can be generated from hull faces with many vertices.
        int maximumContactCount = Math.Max(8, faceVertexIndices.Length);
        var candidates = stackalloc ManifoldCandidateScalar[maximumContactCount];

        //From here down this mirrors the wide tester's per-slot scalar contact generation (Vector4/Vector3.Dot intrinsics verbatim).
        var slotLocalNormal = localNormal;
        //Test each face edge plane against the box face.
        //X is 00->10; Y is 10->11; Z is 11->01; W is 01->00.
        var boxEdgeStartX = new Vector4(v00.X, v10.X, v11.X, v01.X);
        var boxEdgeStartY = new Vector4(v00.Y, v10.Y, v11.Y, v01.Y);
        var boxEdgeStartZ = new Vector4(v00.Z, v10.Z, v11.Z, v01.Z);
        var edgeDirectionX = new Vector4(boxFaceX.X, boxFaceY.X, -boxFaceX.X, -boxFaceY.X);
        var edgeDirectionY = new Vector4(boxFaceX.Y, boxFaceY.Y, -boxFaceX.Y, -boxFaceY.Y);
        var edgeDirectionZ = new Vector4(boxFaceX.Z, boxFaceY.Z, -boxFaceX.Z, -boxFaceY.Z);

        var slotLocalNormalX = new Vector4(slotLocalNormal.X);
        var slotLocalNormalY = new Vector4(slotLocalNormal.Y);
        var slotLocalNormalZ = new Vector4(slotLocalNormal.Z);

        //edgePlaneNormal = edgeDirection x localNormal
        var edgePlaneNormalX = edgeDirectionY * slotLocalNormalZ - edgeDirectionZ * slotLocalNormalY;
        var edgePlaneNormalY = edgeDirectionZ * slotLocalNormalX - edgeDirectionX * slotLocalNormalZ;
        var edgePlaneNormalZ = edgeDirectionX * slotLocalNormalY - edgeDirectionY * slotLocalNormalX;

        var previousIndex = faceVertexIndices[faceVertexIndices.Length - 1];
        Vector3Wide.ReadSlot(ref b.Points[previousIndex.BundleIndex], previousIndex.InnerIndex, out var hullFaceOrigin);
        var previousVertex = hullFaceOrigin;
        var candidateCount = 0;
        Helpers.BuildOrthonormalBasis(slotFaceNormal, out var hullFaceX, out var hullFaceY);
        Vector4 maximumVertexContainmentDots = Vector4.Zero;
        for (int i = 0; i < faceVertexIndices.Length; ++i)
        {
            var index = faceVertexIndices[i];
            Vector3Wide.ReadSlot(ref b.Points[index.BundleIndex], index.InnerIndex, out var vertex);

            var hullEdgeOffset = vertex - previousVertex;

            var hullEdgeStartX = new Vector4(previousVertex.X);
            var hullEdgeStartY = new Vector4(previousVertex.Y);
            var hullEdgeStartZ = new Vector4(previousVertex.Z);
            var hullEdgeOffsetX = new Vector4(hullEdgeOffset.X);
            var hullEdgeOffsetY = new Vector4(hullEdgeOffset.Y);
            var hullEdgeOffsetZ = new Vector4(hullEdgeOffset.Z);
            //Containment of a box vertex is tested by checking the sign of the box vertex against the hull's edge plane normal.
            var hullEdgePlaneNormal = Vector3.Cross(hullEdgeOffset, slotLocalNormal);
            var hullEdgePlaneNormalX = new Vector4(hullEdgePlaneNormal.X);
            var hullEdgePlaneNormalY = new Vector4(hullEdgePlaneNormal.Y);
            var hullEdgePlaneNormalZ = new Vector4(hullEdgePlaneNormal.Z);
            var hullEdgeStartToBoxEdgeX = boxEdgeStartX - hullEdgeStartX;
            var hullEdgeStartToBoxEdgeY = boxEdgeStartY - hullEdgeStartY;
            var hullEdgeStartToBoxEdgeZ = boxEdgeStartZ - hullEdgeStartZ;
            var boxVertexContainmentDots = hullEdgePlaneNormalX * hullEdgeStartToBoxEdgeX + hullEdgePlaneNormalY * hullEdgeStartToBoxEdgeY + hullEdgePlaneNormalZ * hullEdgeStartToBoxEdgeZ;
            maximumVertexContainmentDots = Vector4.Max(maximumVertexContainmentDots, boxVertexContainmentDots);
            //t = dot(pointOnBoxEdge - hullEdgeStart, edgePlaneNormal) / dot(edgePlaneNormal, hullEdgeOffset)
            var numerator = hullEdgeStartToBoxEdgeX * edgePlaneNormalX + hullEdgeStartToBoxEdgeY * edgePlaneNormalY + hullEdgeStartToBoxEdgeZ * edgePlaneNormalZ;
            //Since we're sensitive to the sign of the denominator, the winding of the box edges matters.
            var denominator = edgePlaneNormalX * hullEdgeOffsetX + edgePlaneNormalY * hullEdgeOffsetY + edgePlaneNormalZ * hullEdgeOffsetZ;
            var edgeIntersections = numerator / denominator;

            //A plane is being 'entered' if the ray direction opposes the face normal.
            float latestEntry, earliestExit;
            if (denominator.X < 0)
            {
                latestEntry = edgeIntersections.X;
                earliestExit = float.MaxValue;
            }
            else if (denominator.X > 0)
            {
                latestEntry = float.MinValue;
                earliestExit = edgeIntersections.X;
            }
            else if (numerator.X < 0)
            {
                //The B edge is parallel and outside the edge A, so there can be no intersection.
                earliestExit = float.MinValue;
                latestEntry = float.MaxValue;
            }
            else
            {
                //Parallel, but inside.
                latestEntry = float.MinValue;
                earliestExit = float.MaxValue;
            }
            if (denominator.Y < 0)
            {
                if (edgeIntersections.Y > latestEntry)
                    latestEntry = edgeIntersections.Y;
            }
            else if (denominator.Y > 0)
            {
                if (edgeIntersections.Y < earliestExit)
                    earliestExit = edgeIntersections.Y;
            }
            else if (numerator.Y < 0)
            {
                earliestExit = float.MinValue;
                latestEntry = float.MaxValue;
            }
            if (denominator.Z < 0)
            {
                if (edgeIntersections.Z > latestEntry)
                    latestEntry = edgeIntersections.Z;
            }
            else if (denominator.Z > 0)
            {
                if (edgeIntersections.Z < earliestExit)
                    earliestExit = edgeIntersections.Z;
            }
            else if (numerator.Z < 0)
            {
                earliestExit = float.MinValue;
                latestEntry = float.MaxValue;
            }
            if (denominator.W < 0)
            {
                if (edgeIntersections.W > latestEntry)
                    latestEntry = edgeIntersections.W;
            }
            else if (denominator.W > 0)
            {
                if (edgeIntersections.W < earliestExit)
                    earliestExit = edgeIntersections.W;
            }
            else if (numerator.W < 0)
            {
                earliestExit = float.MinValue;
                latestEntry = float.MaxValue;
            }

            //We now have a convex hull edge interval. Add contacts for it.
            latestEntry = latestEntry < 0 ? 0 : latestEntry;
            earliestExit = earliestExit > 1 ? 1 : earliestExit;
            //Create max contact if max >= min.
            //Create min if min < max and min > 0.
            var startId = (previousIndex.BundleIndex << BundleIndexing.VectorShift) + previousIndex.InnerIndex;
            var endId = (index.BundleIndex << BundleIndexing.VectorShift) + index.InnerIndex;
            var baseFeatureId = (startId ^ endId) << 8;
            if (earliestExit >= latestEntry && candidateCount < maximumContactCount)
            {
                //Create max contact.
                var point = hullEdgeOffset * earliestExit + previousVertex - hullFaceOrigin;
                var newContactIndex = candidateCount++;
                ref var candidate = ref candidates[newContactIndex];
                candidate.X = Vector3.Dot(point, hullFaceX);
                candidate.Y = Vector3.Dot(point, hullFaceY);
                candidate.FeatureId = baseFeatureId + endId;

            }
            if (latestEntry < earliestExit && latestEntry > 0 && candidateCount < maximumContactCount)
            {
                //Create min contact.
                var point = hullEdgeOffset * latestEntry + previousVertex - hullFaceOrigin;
                var newContactIndex = candidateCount++;
                ref var candidate = ref candidates[newContactIndex];
                candidate.X = Vector3.Dot(point, hullFaceX);
                candidate.Y = Vector3.Dot(point, hullFaceY);
                candidate.FeatureId = baseFeatureId + startId;

            }

            previousIndex = index;
            previousVertex = vertex;
        }
        if (candidateCount < maximumContactCount)
        {
            //Try adding the box vertex contacts. Project each vertex onto the hull face.
            //t = dot(boxVertex - hullFaceVertex, hullFacePlaneNormal) / dot(hullFacePlaneNormal, localNormal)
            var hullFaceOriginX = new Vector4(hullFaceOrigin.X);
            var hullFaceOriginY = new Vector4(hullFaceOrigin.Y);
            var hullFaceOriginZ = new Vector4(hullFaceOrigin.Z);
            var hullFaceNormalX = new Vector4(slotFaceNormal.X);
            var hullFaceNormalY = new Vector4(slotFaceNormal.Y);
            var hullFaceNormalZ = new Vector4(slotFaceNormal.Z);
            var closestOnHullToBoxEdgeStartX = boxEdgeStartX - hullFaceOriginX;
            var closestOnHullToBoxEdgeStartY = boxEdgeStartY - hullFaceOriginY;
            var closestOnHullToBoxEdgeStartZ = boxEdgeStartZ - hullFaceOriginZ;
            var vertexProjectionNumerator = (closestOnHullToBoxEdgeStartX) * hullFaceNormalX + (closestOnHullToBoxEdgeStartY) * hullFaceNormalY + (closestOnHullToBoxEdgeStartZ) * hullFaceNormalZ;
            var vertexProjectionDenominator = new Vector4(Vector3.Dot(slotFaceNormal, slotLocalNormal));
            var vertexProjectionT = vertexProjectionNumerator / vertexProjectionDenominator;
            //Normal points from B to A.
            var projectedVertexX = closestOnHullToBoxEdgeStartX - vertexProjectionT * slotLocalNormalX;
            var projectedVertexY = closestOnHullToBoxEdgeStartY - vertexProjectionT * slotLocalNormalY;
            var projectedVertexZ = closestOnHullToBoxEdgeStartZ - vertexProjectionT * slotLocalNormalZ;
            var hullFaceXX = new Vector4(hullFaceX.X);
            var hullFaceXY = new Vector4(hullFaceX.Y);
            var hullFaceXZ = new Vector4(hullFaceX.Z);
            var hullFaceYX = new Vector4(hullFaceY.X);
            var hullFaceYY = new Vector4(hullFaceY.Y);
            var hullFaceYZ = new Vector4(hullFaceY.Z);
            var projectedTangentX = projectedVertexX * hullFaceXX + projectedVertexY * hullFaceXY + projectedVertexZ * hullFaceXZ;
            var projectedTangentY = projectedVertexX * hullFaceYX + projectedVertexY * hullFaceYY + projectedVertexZ * hullFaceYZ;
            //We took the maximum of all boxvertex-hulledgeplane tests; if a vertex is outside any edge plane, the maximum dot will be positive.
            if (maximumVertexContainmentDots.X <= 0)
            {
                ref var candidate = ref candidates[candidateCount++];
                candidate.X = projectedTangentX.X;
                candidate.Y = projectedTangentY.X;
                candidate.FeatureId = 0;
            }
            if (candidateCount == maximumContactCount)
                goto SkipVertexCandidates;
            if (maximumVertexContainmentDots.Y <= 0)
            {
                ref var candidate = ref candidates[candidateCount++];
                candidate.X = projectedTangentX.Y;
                candidate.Y = projectedTangentY.Y;
                candidate.FeatureId = 1;
            }
            if (candidateCount == maximumContactCount)
                goto SkipVertexCandidates;
            if (maximumVertexContainmentDots.Z <= 0)
            {
                ref var candidate = ref candidates[candidateCount++];
                candidate.X = projectedTangentX.Z;
                candidate.Y = projectedTangentY.Z;
                candidate.FeatureId = 2;
            }
            if (candidateCount < maximumContactCount && maximumVertexContainmentDots.W <= 0)
            {
                ref var candidate = ref candidates[candidateCount++];
                candidate.X = projectedTangentX.W;
                candidate.Y = projectedTangentY.W;
                candidate.FeatureId = 3;
            }
        SkipVertexCandidates:;
        }
        //We have found all contacts for this hull slot. There may be more contacts than we want (4), so perform a reduction.
        ConvexHullPairScalarTester.Reduce(candidates, candidateCount, boxFaceNormal, 1f / Vector3.Dot(boxFaceNormal, slotLocalNormal), boxFaceCenter, hullFaceOrigin, hullFaceX, hullFaceY,
            epsilonScale, depthThreshold, hullOrientation, offsetB, ref manifold);
        //The reduction does not assign the normal. Fill it in.
        Matrix3x3.Transform(localNormal, hullOrientation, out manifold.Normal);
    }
}
