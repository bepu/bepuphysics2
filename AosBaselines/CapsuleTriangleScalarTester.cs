using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using System.Numerics;

namespace AosBaselines;

/// <summary>
/// Scalar AoS port of CapsuleTriangleTester, bitwise identical per lane. Closed-form: three segment-edge closest point tests
/// plus the face normal, min-depth select, then either an edge interval contact pair or capsule-axis-clipped face contacts.
/// No DepthRefiner. Structure and comments deliberately track CapsuleTriangleTester line by line.
/// Divisions can produce inf/NaN (parallel axes, degenerate triangles); values are computed unconditionally and then
/// selected, matching the wide ConditionalSelects. Bundle-level LessThanAny branches become per-lane ifs on this lane's mask.
/// </summary>
public static class CapsuleTriangleScalarTester
{
    //Mirrors CapsuleTriangleTester.TestEdge. triangle vertices are the recentered ones; capsuleCenter is localOffsetA.
    static void TestEdge(Vector3 triangleA, Vector3 triangleB, Vector3 triangleC, Vector3 triangleNormal,
        Vector3 edgeStart, Vector3 edgeOffset,
        Vector3 capsuleCenter, Vector3 capsuleAxis, float capsuleHalfLength,
        out Vector3 edgeDirection, out float ta, out float tb, out float bMin, out float bMax, out float depth, out Vector3 normal)
    {
        //Compute the closest points between the two line segments. No clamping to begin with.
        var edgeLength = ScalarMath.Length(edgeOffset);
        edgeDirection = edgeOffset * (1f / edgeLength);
        var offsetB = edgeStart - capsuleCenter;
        var daOffsetB = ScalarMath.Dot(capsuleAxis, offsetB);
        var dbOffsetB = ScalarMath.Dot(edgeDirection, offsetB);
        var dadb = ScalarMath.Dot(capsuleAxis, edgeDirection);
        //Note potential division by zero when the axes are parallel. Arbitrarily clamp; near zero values will instead produce extreme values which get clamped to reasonable results.
        ta = (daOffsetB - dbOffsetB * dadb) / MathF.Max(1e-15f, 1f - dadb * dadb);
        //tb = ta * (da * db) - db * (b - a)
        tb = ta * dadb - dbOffsetB;

        //Project each line segment onto the other line segment, clamping against the target's interval.
        var ta0 = MathF.Max(-capsuleHalfLength, MathF.Min(capsuleHalfLength, daOffsetB));
        var ta1 = MathF.Min(capsuleHalfLength, MathF.Max(-capsuleHalfLength, daOffsetB + edgeLength * dadb));
        var aMin = MathF.Min(ta0, ta1);
        var aMax = MathF.Max(ta0, ta1);
        var aOntoBOffset = capsuleHalfLength * MathF.Abs(dadb);
        bMin = MathF.Max(0f, MathF.Min(edgeLength, -aOntoBOffset - dbOffsetB));
        bMax = MathF.Min(edgeLength, MathF.Max(0f, aOntoBOffset - dbOffsetB));
        ta = MathF.Min(MathF.Max(ta, aMin), aMax);
        tb = MathF.Min(MathF.Max(tb, bMin), bMax);

        var closestPointOnCapsule = capsuleAxis * ta + capsuleCenter;
        var closestPointOnEdge = edgeDirection * tb + edgeStart;

        normal = closestPointOnCapsule - closestPointOnEdge;
        var normalLengthSquared = ScalarMath.Dot(normal, normal);
        //In the event that the normal has zero length due to the capsule internal line segment touching the edge, use the calibrated cross product of the edge and axis.
        var fallbackNormal = Vector3.Cross(capsuleAxis, edgeOffset);
        var calibrationDot = ScalarMath.Dot(fallbackNormal, capsuleCenter);
        fallbackNormal = ScalarMath.Select(ScalarMath.LessMask(calibrationDot, 0f), -fallbackNormal, fallbackNormal);
        var fallbackNormalLengthSquared = ScalarMath.Dot(fallbackNormal, fallbackNormal);
        var useFallbackNormal = ScalarMath.LessMask(normalLengthSquared, 1e-13f);
        normal = ScalarMath.Select(useFallbackNormal, fallbackNormal, normal);
        normalLengthSquared = ScalarMath.Select(useFallbackNormal, fallbackNormalLengthSquared, normalLengthSquared);
        //If the edge and axis are parallel, the cross product will ALSO be zero, so use the edge plane normal.
        var secondFallbackNormal = Vector3.Cross(triangleNormal, edgeOffset);
        //Note: the wide code computes this length from fallbackNormal, NOT secondFallbackNormal. Mirrored verbatim; do not fix.
        var secondFallbackNormalLengthSquared = ScalarMath.Dot(fallbackNormal, fallbackNormal);
        var useSecondFallbackNormal = ScalarMath.LessMask(normalLengthSquared, 1e-13f);
        normal = ScalarMath.Select(useSecondFallbackNormal, secondFallbackNormal, normal);
        normalLengthSquared = ScalarMath.Select(useSecondFallbackNormal, secondFallbackNormalLengthSquared, normalLengthSquared);
        //Note that no additional normal calibration happens here.
        normal = normal * (1f / MathF.Sqrt(normalLengthSquared));

        //The normal between the closest points is not necessarily perpendicular to both the edge and capsule axis due to clamping,
        //so depth includes the extent of both.
        var nAxis = ScalarMath.Dot(capsuleAxis, normal);
        var nCapsuleCenter = ScalarMath.Dot(normal, capsuleCenter);
        //Normal calibrated to point from triangle to capsule, so the extreme point pushes down.
        var extremeOnCapsule = nCapsuleCenter - MathF.Abs(nAxis) * capsuleHalfLength;
        var na = ScalarMath.Dot(triangleA, normal);
        var nb = ScalarMath.Dot(triangleB, normal);
        var nc = ScalarMath.Dot(triangleC, normal);
        //Normal calibration implies largest triangle value.
        var extremeOnTriangle = MathF.Max(na, MathF.Max(nb, nc));
        depth = extremeOnTriangle - extremeOnCapsule;
    }

    //Mirrors CapsuleTriangleTester.ClipAgainstEdgePlane.
    static void ClipAgainstEdgePlane(Vector3 edgeStart, Vector3 edgeOffset, Vector3 faceNormal, Vector3 capsuleCenter, Vector3 capsuleAxis,
        out float entry, out float exit)
    {
        //t = -edgeToCapsule * (edgePlaneNormal / ||edgePlaneNormal||) / (capsuleAxis * (edgePlaneNormal / ||edgePlaneNormal||))
        var edgePlaneNormal = Vector3.Cross(faceNormal, edgeOffset);
        var edgeToCapsule = capsuleCenter - edgeStart;
        var distance = ScalarMath.Dot(edgeToCapsule, edgePlaneNormal);
        var velocity = ScalarMath.Dot(capsuleAxis, edgePlaneNormal);
        //Near-zero denominators (parallel axes) result in a properly signed large finite value.
        var velocityIsPositive = velocity > 0f;
        var t = (velocityIsPositive ? -distance : distance) / MathF.Max(1e-15f, MathF.Abs(velocity));
        //An intersection is considered an 'entry' if the ray direction opposes the plane normal.
        entry = velocityIsPositive ? -float.MaxValue : t;
        exit = velocityIsPositive ? t : float.MaxValue;
    }

    public static void Test(in Capsule a, in Triangle b, float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex2ManifoldScalar manifold)
    {
        //Work in the triangle's local space, recentered on the triangle centroid.
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 rB);
        var localTriangleCenter = b.A + b.B;
        localTriangleCenter = b.C + localTriangleCenter;
        localTriangleCenter = localTriangleCenter * (1f / 3f);
        var localOffsetB = ScalarMath.TransformByTransposed(offsetB, rB);
        localOffsetB = localOffsetB + localTriangleCenter;
        var localOffsetA = -localOffsetB;
        var triangleA = b.A - localTriangleCenter;
        var triangleB = b.B - localTriangleCenter;
        var triangleC = b.C - localTriangleCenter;

        //QuaternionWide.TransformUnitY's arithmetic is identical to TransformUnitXY's y output.
        ScalarMath.TransformUnitXY(orientationA, out _, out var worldCapsuleAxis);
        var localCapsuleAxis = ScalarMath.TransformByTransposed(worldCapsuleAxis, rB);

        var ac = b.C - b.A;
        var ab = b.B - b.A;

        var acxab = Vector3.Cross(ac, ab);
        var faceNormalLength = ScalarMath.Length(acxab);
        var faceNormal = acxab * (1f / faceNormalLength);

        //The depth along the face normal is unaffected by the triangle's extent, but the capsule's is.
        var nDotAxis = ScalarMath.Dot(faceNormal, localCapsuleAxis);
        var capsuleOffsetAlongNormal = ScalarMath.Dot(faceNormal, localOffsetA);
        //capsuleOffsetAlongNormal may be negative when the capsule's center is on the non colliding side of the triangle.
        var faceDepth = a.HalfLength * MathF.Abs(nDotAxis) - capsuleOffsetAlongNormal;

        TestEdge(triangleA, triangleB, triangleC, faceNormal, triangleA, ab, localOffsetA, localCapsuleAxis, a.HalfLength,
            out var edgeDirection, out var ta, out var tb, out var bMin, out var bMax, out var edgeDepth, out var edgeNormal);
        TestEdge(triangleA, triangleB, triangleC, faceNormal, triangleA, ac, localOffsetA, localCapsuleAxis, a.HalfLength,
            out var edgeDirectionCandidate, out var taCandidate, out var tbCandidate, out var bMinCandidate, out var bMaxCandidate, out var edgeDepthCandidate, out var edgeNormalCandidate);
        var useAC = ScalarMath.LessMask(edgeDepthCandidate, edgeDepth);
        edgeDirection = ScalarMath.Select(useAC, edgeDirectionCandidate, edgeDirection);
        edgeNormal = ScalarMath.Select(useAC, edgeNormalCandidate, edgeNormal);
        ta = ScalarMath.Select(useAC, taCandidate, ta);
        tb = ScalarMath.Select(useAC, tbCandidate, tb);
        bMin = ScalarMath.Select(useAC, bMinCandidate, bMin);
        bMax = ScalarMath.Select(useAC, bMaxCandidate, bMax);
        edgeDepth = MathF.Min(edgeDepthCandidate, edgeDepth);

        var bc = b.C - b.B;
        TestEdge(triangleA, triangleB, triangleC, faceNormal, triangleB, bc, localOffsetA, localCapsuleAxis, a.HalfLength,
            out edgeDirectionCandidate, out taCandidate, out tbCandidate, out bMinCandidate, out bMaxCandidate, out edgeDepthCandidate, out edgeNormalCandidate);
        var useBC = ScalarMath.LessMask(edgeDepthCandidate, edgeDepth);
        var edgeStart = ScalarMath.Select(useBC, triangleB, triangleA);
        edgeDirection = ScalarMath.Select(useBC, edgeDirectionCandidate, edgeDirection);
        edgeNormal = ScalarMath.Select(useBC, edgeNormalCandidate, edgeNormal);
        ta = ScalarMath.Select(useBC, taCandidate, ta);
        tb = ScalarMath.Select(useBC, tbCandidate, tb);
        bMin = ScalarMath.Select(useBC, bMinCandidate, bMin);
        bMax = ScalarMath.Select(useBC, bMaxCandidate, bMax);
        edgeDepth = MathF.Min(edgeDepthCandidate, edgeDepth);

        var depth = MathF.Min(edgeDepth, faceDepth);
        var useEdge = edgeDepth < faceDepth;
        var localNormal = useEdge ? edgeNormal : faceNormal;
        var localNormalDotFaceNormal = ScalarMath.Dot(localNormal, faceNormal);
        var collidingWithSolidSide = localNormalDotFaceNormal >= TriangleWide.BackfaceNormalDotRejectionThreshold;
        //activeLanes is true for this lane by construction.
        //ComputeNondegenerateTriangleMask mirror: LengthSquared self-dots are association-safe; MaxNative is safe on sums of squares.
        var abLengthSquared = ScalarMath.Dot(ab, ab);
        var acLengthSquared = ScalarMath.Dot(ac, ac);
        var triangleEpsilonScale = MathF.Sqrt(float.MaxNative(abLengthSquared, acLengthSquared));
        var nondegenerate = faceNormalLength > TriangleWide.DegenerateTriangleEpsilon * triangleEpsilonScale;
        var negativeMargin = -speculativeMargin;
        var allowContacts = depth + a.Radius >= negativeMargin & collidingWithSolidSide & nondegenerate;
        if (!allowContacts)
        {
            //The wide early-out fires when every lane fails; for lanes that fail while the bundle continues, the trailing
            //exists masks are ANDed with allowContacts, so both flags end false either way. Exists=false makes every other
            //field unobservable, so returning a default manifold is bitwise-equivalent.
            manifold = default;
            return;
        }
        Vector3 b0 = default;
        Vector3 b1 = default;
        int contactCount;
        //useEdge = useEdge & allowContacts; allowContacts is true here.
        if (useEdge)
        {
            //Coplanarity fade, borrowed from capsule-capsule: rate the angle between the capsule axis and the plane defined
            //by the triangle edge and contact normal; accept the whole interval when coplanar, narrowing to a point as the
            //axes leave coplanarity.
            var planeNormal = Vector3.Cross(edgeDirection, edgeNormal);
            var planeNormalLengthSquared = ScalarMath.Dot(planeNormal, planeNormal);
            var numeratorUnsquared = ScalarMath.Dot(localCapsuleAxis, planeNormal);
            //The quotient can be NaN (0/0); compute then select, mirroring the wide ConditionalSelect.
            var squaredAngle = ScalarMath.Select(ScalarMath.LessMask(planeNormalLengthSquared, 1e-10f), 0f, numeratorUnsquared * numeratorUnsquared / planeNormalLengthSquared);

            //Convert the squared angle to a lerp parameter: full interval below lowerThreshold, fading to zero at upperThreshold.
            const float lowerThresholdAngle = 0.01f;
            const float upperThresholdAngle = 0.05f;
            const float lowerThreshold = lowerThresholdAngle * lowerThresholdAngle;
            const float upperThreshold = upperThresholdAngle * upperThresholdAngle;
            var intervalWeight = MathF.Max(0f, MathF.Min(1f, (upperThreshold - squaredAngle) * (1f / (upperThreshold - lowerThreshold))));
            //Note that this works with tb, the triangle edge parameter: triangle-related contacts must be on the triangle
            //because of boundary smoothing.
            var weightedTb = tb - tb * intervalWeight;
            bMin = intervalWeight * bMin + weightedTb;
            bMax = intervalWeight * bMax + weightedTb;

            b0 = edgeDirection * bMin + edgeStart;
            b1 = edgeDirection * bMax + edgeStart;
            contactCount = bMax > bMin ? 2 : 1;
        }
        else
        {
            //No edges are used; all contacts must be face contacts.
            contactCount = 0;
        }

        //1) If face contact won (no edge contacts), generate two contacts by clipping the capsule axis against the triangle edge planes.
        //2) If an edge contact won with only one contact, try to generate one additional face contact.
        //3) If an edge contact generated two contacts, no additional contacts are required.
        if (contactCount <= 1)
        {
            ClipAgainstEdgePlane(triangleA, ab, faceNormal, localOffsetA, localCapsuleAxis, out var abEntry, out var abExit);
            ClipAgainstEdgePlane(triangleB, bc, faceNormal, localOffsetA, localCapsuleAxis, out var bcEntry, out var bcExit);
            //Winding matters. ab, bc, ca.
            var ca = -ac;
            ClipAgainstEdgePlane(triangleA, ca, faceNormal, localOffsetA, localCapsuleAxis, out var caEntry, out var caExit);
            var triangleIntervalMin = MathF.Max(abEntry, MathF.Max(bcEntry, caEntry));
            var triangleIntervalMax = MathF.Min(abExit, MathF.Min(bcExit, caExit));

            var negativeHalfLength = -a.HalfLength;
            var overlapIntervalMin = MathF.Max(triangleIntervalMin, negativeHalfLength);
            var overlapIntervalMax = MathF.Min(triangleIntervalMax, a.HalfLength);
            //The clamps below are bilateral for good face contacts, so the one contact case tests interval validity up front.
            var intervalIsValidForSecondContact = overlapIntervalMax >= overlapIntervalMin;
            overlapIntervalMin = MathF.Min(overlapIntervalMin, a.HalfLength);
            overlapIntervalMax = MathF.Max(overlapIntervalMax, negativeHalfLength);
            var clippedOnA0 = localCapsuleAxis * overlapIntervalMin + localOffsetA;
            var distanceAlongNormalA0 = ScalarMath.Dot(clippedOnA0, faceNormal);
            var toRemoveA0 = faceNormal * distanceAlongNormalA0;
            var faceCandidate0 = clippedOnA0 - toRemoveA0;

            var clippedOnA1 = localCapsuleAxis * overlapIntervalMax + localOffsetA;
            var distanceAlongNormalA1 = ScalarMath.Dot(clippedOnA1, faceNormal);
            var toRemoveA1 = faceNormal * distanceAlongNormalA1;
            var faceCandidate1 = clippedOnA1 - toRemoveA1;

            //Automatically accept the two candidates if there are no edge contacts, unless the capsule's center is on the
            //backside of the triangle (no 'pulling' contacts).
            var noEdgeContacts = contactCount == 0;
            var allowFaceContacts = capsuleOffsetAlongNormal >= 0f;
            var useFaceContacts = noEdgeContacts & allowFaceContacts;
            b0 = useFaceContacts ? faceCandidate0 : b0;
            b1 = useFaceContacts ? faceCandidate1 : b1;
            contactCount = useFaceContacts ? 2 : contactCount;

            //If there's one edge contact, pick the clipped interval endpoint further from the edge contact on the capsule's axis.
            var useFaceContact1ForSecondContact = MathF.Abs(overlapIntervalMax - ta) > MathF.Abs(overlapIntervalMin - ta);
            var secondContactCandidate = useFaceContact1ForSecondContact ? faceCandidate1 : faceCandidate0;
            var secondContactDistanceAlongNormal = useFaceContact1ForSecondContact ? distanceAlongNormalA1 : distanceAlongNormalA0;
            //To use this as the second contact: 1) the interval is valid, 2) contact count == 1, 3) the candidate is above the triangle.
            var useCandidateForSecondContact = intervalIsValidForSecondContact & contactCount == 1 & secondContactDistanceAlongNormal > 0f;
            b1 = useCandidateForSecondContact ? secondContactCandidate : b1;
            contactCount = useCandidateForSecondContact ? 2 : contactCount;
        }

        //Each contact has its own depth: project the contact on B along the contact normal to the 'face' of A,
        //treating the capsule as having faceNormalA = (localNormal x capsuleAxis) x capsuleAxis.
        var capsuleTangent = Vector3.Cross(localNormal, localCapsuleAxis);
        var faceNormalA = Vector3.Cross(capsuleTangent, localCapsuleAxis);
        var faceNormalADotLocalNormal = ScalarMath.Dot(faceNormalA, localNormal);
        //The reciprocal can be inf; the collapse select below covers the degenerate case. Compute then select.
        var inverseFaceNormalADotLocalNormal = 1f / faceNormalADotLocalNormal;
        var offset0 = localOffsetB + b0;
        var offset1 = localOffsetB + b1;
        var t0 = ScalarMath.Dot(offset0, faceNormalA);
        var t1 = ScalarMath.Dot(offset1, faceNormalA);
        t0 *= inverseFaceNormalADotLocalNormal;
        t1 *= inverseFaceNormalADotLocalNormal;
        manifold.Depth0 = a.Radius + t0;
        manifold.Depth1 = a.Radius + t1;

        //If the local normal and capsule axis are nearly aligned, depths computed this way are numerically poor;
        //collapse the manifold to one contact and use the previously computed depth.
        var collapse = MathF.Abs(faceNormalADotLocalNormal) < 1e-7f;
        manifold.Depth0 = ScalarMath.Select(ScalarMath.Mask(collapse), a.Radius + depth, manifold.Depth0);
        //collidingWithSolidSide is true here (the allowContacts early out included it), so the wide contactCount zeroing select is a no-op.
        contactCount = collidingWithSolidSide ? contactCount : 0;
        manifold.Contact0Exists = contactCount > 0 & manifold.Depth0 > negativeMargin;
        manifold.Contact1Exists = (contactCount == 2 & !collapse) & manifold.Depth1 > negativeMargin;

        //Feature ids from the projected location along the capsule axis: consistent across face/edge contact sources.
        var localOffsetA0 = b0 - localOffsetA;
        var localOffsetA1 = b1 - localOffsetA;
        var ta0 = ScalarMath.Dot(localOffsetA0, localCapsuleAxis);
        var ta1 = ScalarMath.Dot(localOffsetA1, localCapsuleAxis);
        var flipFeatureIds = ta1 < ta0;
        manifold.FeatureId0 = flipFeatureIds ? 1 : 0;
        manifold.FeatureId1 = flipFeatureIds ? 0 : 1;

        var faceFlag = localNormalDotFaceNormal >= MeshReduction.MinimumDotForFaceCollision ? MeshReduction.FaceCollisionFlag : 0;
        manifold.FeatureId0 += faceFlag;

        //Transform contact positions into world space rotation, measured as offsets from the capsule (object A).
        Matrix3x3.Transform(localOffsetA0, rB, out manifold.OffsetA0);
        Matrix3x3.Transform(localOffsetA1, rB, out manifold.OffsetA1);
        Matrix3x3.Transform(localNormal, rB, out manifold.Normal);
    }
}
