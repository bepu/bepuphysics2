using BepuPhysics.Collidables;
using System.Numerics;

namespace AosBaselines;

/// <summary>
/// Scalar AoS port of CapsulePairTester, bitwise identical per lane. Closed-form world-space segment-segment closest point
/// with a coplanarity-driven contact interval; no DepthRefiner and no per-lane iteration, so the port is a straight-line mirror.
/// Divisions can produce inf/NaN (parallel axes, touching segments); values are computed unconditionally and then selected,
/// matching the wide ConditionalSelects.
/// </summary>
public static class CapsulePairScalarTester
{
    public static void Test(in Capsule a, in Capsule b, float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex2ManifoldScalar manifold)
    {
        //Compute the closest points between the two line segments. No clamping to begin with.
        //ta = (da * (b - a) + (db * (a - b)) * (da * db)) / (1 - ((da * db) * (da * db))
        ScalarMath.TransformUnitXY(orientationA, out var xa, out var da);
        //QuaternionWide.TransformUnitY's arithmetic is identical to TransformUnitXY's y output.
        ScalarMath.TransformUnitXY(orientationB, out _, out var db);
        var daOffsetB = ScalarMath.Dot(da, offsetB);
        var dbOffsetB = ScalarMath.Dot(db, offsetB);
        var dadb = ScalarMath.Dot(da, db);
        //Note potential division by zero when the axes are parallel; the wide code clamps the denominator to 1e-15.
        var ta = (daOffsetB - dbOffsetB * dadb) / MathF.Max(1e-15f, 1f - dadb * dadb);
        //tb = ta * (da * db) - db * (b - a)
        var tb = ta * dadb - dbOffsetB;

        //Project each line segment onto the other line segment, clamping against the target's interval.
        var absdadb = MathF.Abs(dadb);
        var bOntoAOffset = b.HalfLength * absdadb;
        var aOntoBOffset = a.HalfLength * absdadb;
        var aMin = MathF.Max(-a.HalfLength, MathF.Min(a.HalfLength, daOffsetB - bOntoAOffset));
        var aMax = MathF.Min(a.HalfLength, MathF.Max(-a.HalfLength, daOffsetB + bOntoAOffset));
        var bMin = MathF.Max(-b.HalfLength, MathF.Min(b.HalfLength, -aOntoBOffset - dbOffsetB));
        var bMax = MathF.Min(b.HalfLength, MathF.Max(-b.HalfLength, aOntoBOffset - dbOffsetB));
        ta = MathF.Min(MathF.Max(ta, aMin), aMax);
        tb = MathF.Min(MathF.Max(tb, bMin), bMax);

        var closestPointOnA = da * ta;
        var closestPointOnB = db * tb + offsetB;
        //Note that normals are calibrated to point from B to A by convention.
        var normal = closestPointOnA - closestPointOnB;
        var distance = ScalarMath.Length(normal);
        //inverseDistance can be inf and the scaled normal NaN/inf when the segments touch; compute then select.
        var inverseDistance = 1f / distance;
        normal = normal * inverseDistance;
        var normalIsValid = ScalarMath.GreaterMask(distance, 1e-7f);
        normal = ScalarMath.Select(normalIsValid, normal, xa);

        //Coplanarity fade: squared angle between capsule axis A and the plane defined by segment B and the contact normal.
        var planeNormal = Vector3.Cross(db, normal);
        var planeNormalLengthSquared = ScalarMath.Dot(planeNormal, planeNormal);
        var numeratorUnsquared = ScalarMath.Dot(da, planeNormal);
        //The quotient can be NaN (0/0); compute then select, mirroring the wide ConditionalSelect.
        var squaredAngle = ScalarMath.Select(ScalarMath.LessMask(planeNormalLengthSquared, 1e-10f), 0f, numeratorUnsquared * numeratorUnsquared / planeNormalLengthSquared);

        //Convert the squared angle to a lerp parameter: full interval below lowerThreshold, fading to zero at upperThreshold.
        const float lowerThresholdAngle = 0.01f;
        const float upperThresholdAngle = 0.05f;
        const float lowerThreshold = lowerThresholdAngle * lowerThresholdAngle;
        const float upperThreshold = upperThresholdAngle * upperThresholdAngle;
        var intervalWeight = MathF.Max(0f, MathF.Min(1f, (upperThreshold - squaredAngle) * (1f / (upperThreshold - lowerThreshold))));
        var weightedTa = ta - ta * intervalWeight;
        aMin = intervalWeight * aMin + weightedTa;
        aMax = intervalWeight * aMax + weightedTa;

        var offsetA0 = da * aMin;
        var offsetA1 = da * aMax;
        //Unproject the final interval endpoints from a back onto b:
        //tb0 = (ta0 - daOffsetB) / dadb
        //distance0 = dot(a0 - (offsetB + tb0 * db), normal)
        var dbNormal = ScalarMath.Dot(db, normal);
        var offsetB0 = offsetA0 - offsetB;
        var offsetB1 = offsetA1 - offsetB;
        //inverseDadb can be inf for perpendicular capsules; the perpendicular select below chooses the segment distance instead.
        var inverseDadb = 1f / dadb;
        var projectedTb0 = MathF.Max(bMin, MathF.Min(bMax, (aMin - daOffsetB) * inverseDadb));
        var projectedTb1 = MathF.Max(bMin, MathF.Min(bMax, (aMax - daOffsetB) * inverseDadb));
        var b0Normal = ScalarMath.Dot(offsetB0, normal);
        var b1Normal = ScalarMath.Dot(offsetB1, normal);
        var capsulesArePerpendicular = ScalarMath.LessMask(MathF.Abs(dadb), 1e-7f);
        var distance0 = ScalarMath.Select(capsulesArePerpendicular, distance, b0Normal - dbNormal * projectedTb0);
        var distance1 = ScalarMath.Select(capsulesArePerpendicular, distance, b1Normal - dbNormal * projectedTb1);
        var combinedRadius = a.Radius + b.Radius;
        manifold.Depth0 = combinedRadius - distance0;
        manifold.Depth1 = combinedRadius - distance1;

        //Apply the normal offset to the contact positions.
        var negativeOffsetFromA0 = manifold.Depth0 * 0.5f - a.Radius;
        var negativeOffsetFromA1 = manifold.Depth1 * 0.5f - a.Radius;
        var normalPush0 = normal * negativeOffsetFromA0;
        var normalPush1 = normal * negativeOffsetFromA1;
        manifold.OffsetA0 = offsetA0 + normalPush0;
        manifold.OffsetA1 = offsetA1 + normalPush1;
        manifold.Normal = normal;
        manifold.FeatureId0 = 0;
        manifold.FeatureId1 = 1;
        var minimumAcceptedDepth = -speculativeMargin;
        manifold.Contact0Exists = manifold.Depth0 >= minimumAcceptedDepth;
        manifold.Contact1Exists = manifold.Depth1 >= minimumAcceptedDepth & aMax - aMin > 1e-7f * a.HalfLength;
    }
}
