using BepuPhysics.Collidables;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace AosBaselines;

using CapsuleHullDepthRefiner = ScalarDepthRefiner<ConvexHull, HullSupportScalar, Capsule, CapsuleSupportScalar>;

/// <summary>
/// Shared scalar mirrors for the (convex shape)-hull pair testers.
/// </summary>
internal static class HullScalarHelpers
{
    /// <summary>
    /// Mirrors ConvexHullWide.EstimateEpsilonScale per lane: mean absolute coordinate of the hull's first point.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static float EstimateEpsilonScale(ref ConvexHull hull)
    {
        Vector3Wide.ReadSlot(ref hull.Points[0], 0, out var firstPoint);
        return (MathF.Abs(firstPoint.X) + MathF.Abs(firstPoint.Y) + MathF.Abs(firstPoint.Z)) * (1f / 3f);
    }
}

/// <summary>
/// Scalar AoS port of CapsuleConvexHullTester, bitwise identical per lane.
/// The clipping of the capsule axis against the hull face is already per-slot scalar in the wide tester and is copied verbatim
/// (Vector3.Dot/Cross intrinsics included); the ported pieces are the wide setup/postprocess math and the DepthRefiner.
/// </summary>
public static class CapsuleConvexHullScalarTester
{
    public static void Test(in Capsule a, ref ConvexHull b, float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex2ManifoldScalar manifold)
    {
        manifold = default;
        Matrix3x3.CreateFromQuaternion(orientationA, out Matrix3x3 capsuleOrientation);
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 hullOrientation);
        ScalarMath.MultiplyByTranspose(capsuleOrientation, hullOrientation, out var hullLocalCapsuleOrientation);

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
        //MathF.Min/Max mirror Vector.Min/Max's IEEE semantics on .NET 10 (see ScalarDepthRefiner notes); used for every
        //wide Min/Max mirror in the hull pair testers.
        var epsilonScale = MathF.Min(a.Radius, hullEpsilonScale);
        var depthThreshold = -speculativeMargin;
        CapsuleHullDepthRefiner.FindMinimumDepth(b, a, localOffsetA, hullLocalCapsuleOrientation, initialNormal, 1e-5f * epsilonScale, depthThreshold,
            out var depth, out var localNormal, out var closestOnHull);

        if (depth < depthThreshold)
        {
            //No contacts generated.
            return;
        }

        var boundingPlaneEpsilon = 1e-3f * epsilonScale;
        ConvexHullPairScalarTester.PickRepresentativeFace(ref b, localNormal, closestOnHull, boundingPlaneEpsilon, out var slotFaceNormal, out var bestFaceIndex);

        //Test each face edge plane against the capsule edge. Per-slot scalar region in the wide tester; copied verbatim.
        b.GetVertexIndicesForFace(bestFaceIndex, out var faceVertexIndices);
        var previousIndex = faceVertexIndices[faceVertexIndices.Length - 1];
        Vector3Wide.ReadSlot(ref b.Points[previousIndex.BundleIndex], previousIndex.InnerIndex, out var previousVertex);
        var slotCapsuleAxis = hullLocalCapsuleOrientation.Y;
        var slotLocalOffsetA = localOffsetA;
        var slotLocalNormal = localNormal;
        var latestEntryNumerator = float.MaxValue;
        var latestEntryDenominator = -1f;
        var earliestExitNumerator = float.MaxValue;
        var earliestExitDenominator = 1f;
        for (int i = 0; i < faceVertexIndices.Length; ++i)
        {
            var index = faceVertexIndices[i];
            Vector3Wide.ReadSlot(ref b.Points[index.BundleIndex], index.InnerIndex, out var vertex);

            var edgeOffset = vertex - previousVertex;
            var edgePlaneNormal = Vector3.Cross(edgeOffset, slotLocalNormal);

            //t = dot(pointOnPlane - capsuleCenter, planeNormal) / dot(planeNormal, rayDirection)
            //Note that we can defer the division; we don't need to compute the exact t value of *all* planes.
            var capsuleToEdge = previousVertex - slotLocalOffsetA;
            var numerator = Vector3.Dot(capsuleToEdge, edgePlaneNormal);
            var denominator = Vector3.Dot(edgePlaneNormal, slotCapsuleAxis);
            previousVertex = vertex;

            //A plane is being 'entered' if the ray direction opposes the face normal.
            //Entry denominators are always negative, exit denominators are always positive. Don't have to worry about comparison sign flips.
            var edgePlaneNormalLengthSquared = edgePlaneNormal.LengthSquared();
            var denominatorSquared = denominator * denominator;

            const float min = 1e-5f;
            const float max = 3e-4f;
            const float inverseSpan = 1f / (max - min);
            if (denominatorSquared > min * edgePlaneNormalLengthSquared)
            {
                if (denominatorSquared < max * edgePlaneNormalLengthSquared)
                {
                    //As the angle between the axis and edge plane approaches zero, the axis should unrestrict.
                    var restrictWeight = (denominatorSquared / edgePlaneNormalLengthSquared - min) * inverseSpan;
                    if (restrictWeight < 0)
                        restrictWeight = 0;
                    else if (restrictWeight > 1)
                        restrictWeight = 1;
                    var unrestrictedNumerator = a.HalfLength * denominator;
                    if (denominator < 0)
                        unrestrictedNumerator = -unrestrictedNumerator;
                    numerator = restrictWeight * numerator + (1 - restrictWeight) * unrestrictedNumerator;
                }
                if (denominator < 0)
                {
                    if (numerator * latestEntryDenominator > latestEntryNumerator * denominator)
                    {
                        latestEntryNumerator = numerator;
                        latestEntryDenominator = denominator;
                    }
                }
                else // if (denominator > 0)
                {
                    if (numerator * earliestExitDenominator < earliestExitNumerator * denominator)
                    {
                        earliestExitNumerator = numerator;
                        earliestExitDenominator = denominator;
                    }
                }
            }
        }

        //Wide postprocess; ordered mirrors for the bundle math.
        var tEntry = latestEntryNumerator / latestEntryDenominator;
        var tExit = earliestExitNumerator / earliestExitDenominator;
        var negatedHalfLength = -a.HalfLength;
        tEntry = MathF.Max(negatedHalfLength, MathF.Min(a.HalfLength, tEntry));
        tExit = MathF.Max(negatedHalfLength, MathF.Min(a.HalfLength, tExit));

        var localOffset0 = hullLocalCapsuleOrientation.Y * tEntry;
        var localOffset1 = hullLocalCapsuleOrientation.Y * tExit;

        //Compute the depth of each contact based on the projection along the contact normal to the face.
        //depth = dot(contactRelativeToA - pointOnFaceB, faceNormalB) / dot(faceNormalB, normal)
        var aToPointOnHullFace = localOffsetB + closestOnHull;

        var depthDenominator = ScalarMath.Dot(slotFaceNormal, localNormal);
        var inverseDepthDenominator = 1f / depthDenominator;
        var contact0ToHullFace = aToPointOnHullFace - localOffset0;
        var contact1ToHullFace = aToPointOnHullFace - localOffset1;
        var depthNumerator0 = ScalarMath.Dot(contact0ToHullFace, slotFaceNormal);
        var depthNumerator1 = ScalarMath.Dot(contact1ToHullFace, slotFaceNormal);
        var unexpandedDepth0 = depthNumerator0 * inverseDepthDenominator;
        var unexpandedDepth1 = depthNumerator1 * inverseDepthDenominator;
        manifold.Depth0 = a.Radius + unexpandedDepth0;
        manifold.Depth1 = a.Radius + unexpandedDepth1;
        manifold.FeatureId0 = 0;
        manifold.FeatureId1 = 1;
        manifold.Contact0Exists = manifold.Depth0 >= depthThreshold;
        manifold.Contact1Exists = tExit - tEntry > a.HalfLength * 1e-3f & manifold.Depth1 >= depthThreshold;

        Matrix3x3.Transform(localOffset0, hullOrientation, out manifold.OffsetA0);
        Matrix3x3.Transform(localOffset1, hullOrientation, out manifold.OffsetA1);
        Matrix3x3.Transform(localNormal, hullOrientation, out manifold.Normal);
        //Push the contacts out to be on the surface of the convex hull.
        var contactOffset0 = manifold.Normal * unexpandedDepth0;
        var contactOffset1 = manifold.Normal * unexpandedDepth1;
        manifold.OffsetA0 = manifold.OffsetA0 + contactOffset0;
        manifold.OffsetA1 = manifold.OffsetA1 + contactOffset1;
    }
}
