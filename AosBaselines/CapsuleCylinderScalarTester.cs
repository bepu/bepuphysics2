using BepuPhysics.Collidables;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace AosBaselines;

/// <summary>
/// Scalar AoS port of CapsuleCylinderTester, bitwise identical per lane. Works in the cylinder's local space like the wide implementation.
/// No DepthRefiner; the wide tester's own iterative segment-cylinder solver is mirrored per lane (wide lanes freeze their state on
/// deactivation, so a per-lane break reproduces the bundle behavior exactly).
/// </summary>
public static class CapsuleCylinderScalarTester
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Dot2(Vector2 a, Vector2 b) => a.X * b.X + a.Y * b.Y;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void Bounce(in Vector3 lineOrigin, in Vector3 lineDirection, float t, in Cylinder b, float radiusSquared, out Vector3 p, out Vector3 clamped)
    {
        //Clamp the point on the capsule line to the bounds of the cylinder, and then project the clamped result back onto the line.
        var pX = lineDirection.X * t + lineOrigin.X;
        var pY = lineDirection.Y * t + lineOrigin.Y;
        var pZ = lineDirection.Z * t + lineOrigin.Z;
        p = new Vector3(pX, pY, pZ);
        var horizontalDistanceSquared = pX * pX + pZ * pZ;
        var needHorizontalClamp = ScalarMath.GreaterMask(horizontalDistanceSquared, radiusSquared);
        //clampScale can be inf/NaN when the horizontal distance is zero; compute then select, matching the wide code.
        var clampScale = b.Radius / MathF.Sqrt(horizontalDistanceSquared);
        clamped = new Vector3(
            ScalarMath.Select(needHorizontalClamp, clampScale * pX, pX),
            MathF.Max(-b.HalfLength, MathF.Min(b.HalfLength, pY)),
            ScalarMath.Select(needHorizontalClamp, clampScale * pZ, pZ));
    }

    /// <summary>
    /// Mirror of CapsuleCylinderTester.GetClosestPointBetweenLineSegmentAndCylinder. The wide loop only updates t while the lane is
    /// active (deactivation latches and min/max churn on frozen lanes never feeds back into t), so breaking at this lane's own
    /// deactivation and then doing the final Bounce is bitwise identical.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void GetClosestPointBetweenLineSegmentAndCylinder(in Vector3 lineOrigin, in Vector3 lineDirection, float halfLength, in Cylinder b,
        out float t, out Vector3 offsetFromCylinderToLineSegment)
    {
        var min = -halfLength;
        var max = halfLength;
        t = 0f;
        var radiusSquared = b.Radius * b.Radius;
        var originDot = ScalarMath.Dot(lineDirection, lineOrigin);
        var epsilon = halfLength * 1e-7f;
        for (int i = 0; i < 12; ++i)
        {
            Bounce(lineOrigin, lineDirection, t, b, radiusSquared, out _, out var clamped);
            var conservativeNewT = ScalarMath.Dot(clamped, lineDirection);
            conservativeNewT = MathF.Max(min, MathF.Min(max, conservativeNewT - originDot));
            var change = conservativeNewT - t;
            if (MathF.Abs(change) < epsilon)
            {
                //This lane deactivates: t is frozen from here on in the wide code.
                break;
            }
            //The bounced projection can be thought of as conservative advancement; the sign of the change updates the bounds.
            if (change > 0f)
                min = conservativeNewT;
            else
                max = conservativeNewT;
            //Bisect the remaining interval.
            t = 0.5f * (min + max);
        }
        Bounce(lineOrigin, lineDirection, t, b, radiusSquared, out var pointOnLine, out var clampedToCylinder);
        offsetFromCylinderToLineSegment = pointOnLine - clampedToCylinder;
    }

    /// <summary>
    /// Mirror of CapsuleCylinderTester.GetClosestPointsBetweenSegments (copied from CylinderPairScalarTester, which fuzz-validated it
    /// against the same wide function).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void GetClosestPointsBetweenSegments(in Vector3 da, in Vector3 localOffsetB, float aHalfLength, float bHalfLength,
        out float ta, out float taMin, out float taMax, out float tb, out float tbMin, out float tbMax)
    {
        var daOffsetB = ScalarMath.Dot(da, localOffsetB);
        var dbOffsetB = localOffsetB.Y;
        var dadb = da.Y;
        //Note potential division by zero when the axes are parallel. Arbitrarily clamp; near zero values will instead produce extreme values which get clamped to reasonable results.
        ta = (daOffsetB - dbOffsetB * dadb) / float.MaxNative(1e-15f, 1f - dadb * dadb);
        tb = ta * dadb - dbOffsetB;

        var absdadb = MathF.Abs(dadb);
        var bOntoAOffset = bHalfLength * absdadb;
        var aOntoBOffset = aHalfLength * absdadb;
        taMin = float.MaxNative(-aHalfLength, float.MinNative(aHalfLength, daOffsetB - bOntoAOffset));
        taMax = float.MinNative(aHalfLength, float.MaxNative(-aHalfLength, daOffsetB + bOntoAOffset));
        tbMin = float.MaxNative(-bHalfLength, float.MinNative(bHalfLength, -aOntoBOffset - dbOffsetB));
        tbMax = float.MinNative(bHalfLength, float.MaxNative(-bHalfLength, aOntoBOffset - dbOffsetB));
        ta = float.MinNative(float.MaxNative(ta, taMin), taMax);
        tb = float.MinNative(float.MaxNative(tb, tbMin), tbMax);
    }

    /// <summary>
    /// Mirror of CapsuleCylinderTester.GetContactIntervalBetweenSegments (copied from CylinderPairScalarTester).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void GetContactIntervalBetweenSegments(float aHalfLength, float bHalfLength, in Vector3 axisA, in Vector3 localNormal,
        float inverseHorizontalNormalLengthSquaredB, in Vector3 offsetB, out float contactTMin, out float contactTMax)
    {
        GetClosestPointsBetweenSegments(axisA, offsetB, aHalfLength, bHalfLength, out var ta, out var taMin, out var taMax, out var tb, out var tbMin, out var tbMax);

        //In the event that the two axes are coplanar, we accept the whole interval as a source of contact; see the wide implementation's commentary.
        var dot = axisA.X * localNormal.Z - axisA.Z * localNormal.X;
        var squaredAngle = dot * dot * inverseHorizontalNormalLengthSquaredB;

        const float lowerThresholdAngle = 0.02f;
        const float upperThresholdAngle = 0.15f;
        const float lowerThreshold = lowerThresholdAngle * lowerThresholdAngle;
        const float upperThreshold = upperThresholdAngle * upperThresholdAngle;
        var intervalWeight = float.MaxNative(0f, float.MinNative(1f, (upperThreshold - squaredAngle) * (1f / (upperThreshold - lowerThreshold))));
        var weightedTb = tb - tb * intervalWeight;
        contactTMin = intervalWeight * tbMin + weightedTb;
        contactTMax = intervalWeight * tbMax + weightedTb;
    }

    public static void Test(
        in Capsule a, in Cylinder b, float speculativeMargin,
        in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex2ManifoldScalar manifold)
    {
        Matrix3x3.CreateFromQuaternion(orientationA, out Matrix3x3 worldRA);
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 worldRB);
        //Work in the cylinder's local space.
        ScalarMath.MultiplyByTranspose(worldRA, worldRB, out var rA);
        var capsuleAxis = rA.Y;
        var localOffsetB = ScalarMath.TransformByTransposed(offsetB, worldRB);
        var localOffsetA = -localOffsetB;

        GetClosestPointBetweenLineSegmentAndCylinder(localOffsetA, capsuleAxis, a.HalfLength, b, out var t, out var localNormal);
        var distanceFromCylinderToLineSegmentSquared = ScalarMath.Dot(localNormal, localNormal);
        var internalLineSegmentIntersected = distanceFromCylinderToLineSegmentSquared < 1e-12f;
        var distanceFromCylinderToLineSegment = MathF.Sqrt(distanceFromCylinderToLineSegmentSquared);
        //Division by zero is protected by the depth selection; if the distance is zero, the depth is set to MaxValue and this normal won't be selected.
        localNormal = localNormal * (1f / distanceFromCylinderToLineSegment);
        var depth = internalLineSegmentIntersected ? float.MaxValue : -distanceFromCylinderToLineSegment;
        var negativeMargin = -speculativeMargin;
        var laneInactive = depth + a.Radius < negativeMargin;
        //The wide deep-intersection block is entered when any lane is intersected and active, but every select inside is gated on
        //internalLineSegmentIntersected (and intersected lanes cannot be depth-inactive since their depth is MaxValue), so a
        //per-lane branch on this lane's own intersection flag is exact.
        if (internalLineSegmentIntersected)
        {
            //At least one lane is intersecting deeply, so we need to examine the other possible normals.
            var endpointVsCapDepth = b.HalfLength + MathF.Abs(capsuleAxis.Y * a.HalfLength) - MathF.Abs(localOffsetA.Y);
            if (endpointVsCapDepth < depth)
            {
                depth = endpointVsCapDepth;
                //Normal calibrated to point from B to A.
                localNormal = new Vector3(0f, localOffsetA.Y > 0f ? 1f : -1f, 0f);
            }

            GetClosestPointsBetweenSegments(capsuleAxis, localOffsetB, a.HalfLength, b.HalfLength, out var ta, out _, out _, out var tb, out _, out _);

            //offset = da * ta - (db * tb + offsetB)
            var closestA = capsuleAxis * ta;
            var offset = closestA - localOffsetB;
            offset.Y -= tb;

            var distance = ScalarMath.Length(offset);
            var inverseDistance = 1f / distance;
            var internalEdgeNormal = offset * inverseDistance;
            if (distance < 1e-7f)
                internalEdgeNormal = new Vector3(1f, 0f, 0f);

            //Compute the depth along the internal edge normal.
            var centerSeparationAlongNormal = ScalarMath.Dot(localOffsetA, internalEdgeNormal);
            //1 - y*y can be -0; MathF.Max mirrors Vector.Max's IEEE zero handling so the sqrt sees +0 like the wide code.
            var cylinderContribution = MathF.Abs(b.HalfLength * internalEdgeNormal.Y) + b.Radius * MathF.Sqrt(MathF.Max(0f, 1f - internalEdgeNormal.Y * internalEdgeNormal.Y));
            var capsuleAxisDotNormal = ScalarMath.Dot(capsuleAxis, internalEdgeNormal);
            var capsuleContribution = MathF.Abs(capsuleAxisDotNormal) * a.HalfLength;
            var internalEdgeDepth = cylinderContribution + capsuleContribution - centerSeparationAlongNormal;

            if (internalEdgeDepth < depth)
            {
                depth = internalEdgeDepth;
                localNormal = internalEdgeNormal;
            }
        }
        //All of the above excluded any consideration of the capsule's radius. Include it now.
        depth += a.Radius;
        laneInactive |= depth < negativeMargin;
        if (laneInactive)
        {
            //The depth cannot create any contacts due to the speculative margin. The wide code would keep executing, but this lane's
            //exists flags are forced false at the end, so nothing else is observable.
            manifold = default;
            return;
        }

        //The wide code computes side contacts for all lanes and overwrites contact0/contact1/contactCount entirely on cap lanes,
        //so a per-lane branch is exact.
        var useCapContacts = MathF.Abs(localNormal.Y) > 0.70710678118f;
        Vector3 contact0, contact1;
        int contactCount;
        if (useCapContacts)
        {
            //Project the capsule endpoints along the normal to the cap plane and clip against the cap circle.
            var capHeight = localNormal.Y > 0f ? b.HalfLength : -b.HalfLength;
            var inverseNormalY = 1f / localNormal.Y;
            var endpointOffset = capsuleAxis * a.HalfLength;
            var positiveX = localOffsetA.X + endpointOffset.X;
            var positiveY = localOffsetA.Y + endpointOffset.Y - capHeight;
            var positiveZ = localOffsetA.Z + endpointOffset.Z;
            var negativeX = localOffsetA.X - endpointOffset.X;
            var negativeY = localOffsetA.Y - endpointOffset.Y - capHeight;
            var negativeZ = localOffsetA.Z - endpointOffset.Z;
            var tNegative = negativeY * inverseNormalY;
            var tPositive = positiveY * inverseNormalY;
            var projectedNegative = new Vector2(negativeX - localNormal.X * tNegative, negativeZ - localNormal.Z * tNegative);
            var projectedPositive = new Vector2(positiveX - localNormal.X * tPositive, positiveZ - localNormal.Z * tPositive);

            //Intersect the line segment (projectedNegative, projectedPositive) with the circle with radius b.Radius positioned at (0,0).
            var projectedOffset = projectedPositive - projectedNegative;
            var coefficientC = Dot2(projectedNegative, projectedNegative);
            coefficientC -= b.Radius * b.Radius;
            var coefficientB = Dot2(projectedNegative, projectedOffset);
            var coefficientA = Dot2(projectedOffset, projectedOffset);
            var inverseA = 1f / coefficientA;
            var tOffset = MathF.Sqrt(MathF.Max(0f, coefficientB * coefficientB - coefficientA * coefficientC)) * inverseA;
            var tBase = -coefficientB * inverseA;
            var tMin = MathF.Max(0f, MathF.Min(1f, tBase - tOffset));
            var tMax = MathF.Max(0f, MathF.Min(1f, tBase + tOffset));
            //If the projected length is zero, just treat both points as being in the same location (at tNegative).
            var useFallback = ScalarMath.LessMask(MathF.Abs(coefficientA), 1e-12f);
            tMin = ScalarMath.Select(useFallback, 0f, tMin);
            tMax = ScalarMath.Select(useFallback, 0f, tMax);
            contact0 = new Vector3(tMin * projectedOffset.X + projectedNegative.X, capHeight, tMin * projectedOffset.Y + projectedNegative.Y);
            contact1 = new Vector3(tMax * projectedOffset.X + projectedNegative.X, capHeight, tMax * projectedOffset.Y + projectedNegative.Y);
            //Fixed epsilon; the t value scales an offset that is generally proportional to object sizes.
            contactCount = tMax - tMin > 1e-5f ? 2 : 1;
        }
        else
        {
            //Phrase the problem as a segment-segment test against the cylinder's side line nearest the normal.
            //The projected normal is known to be nonzero length because the cap condition failed.
            var inverseHorizontalNormalLengthSquared = 1f / (localNormal.X * localNormal.X + localNormal.Z * localNormal.Z);
            var scale = b.Radius * MathF.Sqrt(inverseHorizontalNormalLengthSquared);
            var cylinderSegmentOffsetX = localNormal.X * scale;
            var cylinderSegmentOffsetZ = localNormal.Z * scale;
            var aToSideSegmentCenter = new Vector3(localOffsetB.X + cylinderSegmentOffsetX, localOffsetB.Y, localOffsetB.Z + cylinderSegmentOffsetZ);
            GetContactIntervalBetweenSegments(a.HalfLength, b.HalfLength, capsuleAxis, localNormal, inverseHorizontalNormalLengthSquared, aToSideSegmentCenter, out var contactTMin, out var contactTMax);

            contact0 = new Vector3(cylinderSegmentOffsetX, contactTMin, cylinderSegmentOffsetZ);
            contact1 = new Vector3(cylinderSegmentOffsetX, contactTMax, cylinderSegmentOffsetZ);
            contactCount = MathF.Abs(contactTMax - contactTMin) < b.HalfLength * 1e-5f ? 1 : 2;
        }

        //While we have computed a global depth, each contact has its own depth: project the contact on B along the contact normal
        //to the 'face' of A, faceNormalA = (localNormal x capsuleAxis) x capsuleAxis.
        var capsuleTangent = Vector3.Cross(localNormal, capsuleAxis);
        var faceNormalA = Vector3.Cross(capsuleTangent, capsuleAxis);
        var faceNormalADotLocalNormal = ScalarMath.Dot(faceNormalA, localNormal);
        //Don't have to perform any calibration on the faceNormalA; it appears in both the numerator and denominator so the sign and magnitudes cancel.
        var inverseFaceNormalADotLocalNormal = 1f / faceNormalADotLocalNormal;
        var offset0 = localOffsetB + contact0;
        var offset1 = localOffsetB + contact1;
        var t0 = ScalarMath.Dot(offset0, faceNormalA);
        var t1 = ScalarMath.Dot(offset1, faceNormalA);
        t0 *= inverseFaceNormalADotLocalNormal;
        t1 *= inverseFaceNormalADotLocalNormal;
        manifold.Depth0 = a.Radius + t0;
        manifold.Depth1 = a.Radius + t1;

        //If the capsule axis is parallel with the normal, the contacts collapse to one point and we use the initially computed depth.
        var collapse = MathF.Abs(faceNormalADotLocalNormal) < 1e-7f;
        if (collapse)
            manifold.Depth0 = depth;
        manifold.Contact0Exists = manifold.Depth0 >= negativeMargin;
        manifold.Contact1Exists = contactCount == 2 & !collapse & manifold.Depth1 >= negativeMargin;

        //Push the contacts into world space.
        Matrix3x3.Transform(localNormal, worldRB, out manifold.Normal);
        Matrix3x3.Transform(contact0, worldRB, out manifold.OffsetA0);
        Matrix3x3.Transform(contact1, worldRB, out manifold.OffsetA1);
        manifold.OffsetA0 = manifold.OffsetA0 + offsetB;
        manifold.OffsetA1 = manifold.OffsetA1 + offsetB;

        manifold.FeatureId0 = 0;
        manifold.FeatureId1 = 1;
    }
}
