using BepuPhysics.Collidables;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;

namespace AosBaselines;

using CylinderDepthRefiner = ScalarDepthRefiner<Cylinder, CylinderSupportScalar, Cylinder, CylinderSupportScalar>;

/// <summary>
/// Scalar AoS port of CylinderPairTester, bitwise identical per lane. Works in B's local space like the wide implementation.
/// </summary>
public static class CylinderPairScalarTester
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Dot2(Vector2 a, Vector2 b) => a.X * b.X + a.Y * b.Y;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static Vector2 ProjectOntoCapA(float capCenterBY, Vector3 capCenterA, in Matrix3x3 rA, float inverseNDotAY, Vector3 localNormal, Vector2 point)
    {
        var point3D = new Vector3(point.X, capCenterBY, point.Y);
        return ProjectOntoCapA(capCenterA, rA, inverseNDotAY, localNormal, point3D);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static Vector2 ProjectOntoCapA(Vector3 capCenterA, in Matrix3x3 rA, float inverseNDotAY, Vector3 localNormal, Vector3 point)
    {
        var pointToCapCenterA = capCenterA - point;
        var tDistance = ScalarMath.Dot(pointToCapCenterA, rA.Y);
        var tBOnA = tDistance * inverseNDotAY;
        var projectionOffsetB = localNormal * tBOnA;
        var projectedPoint = point + projectionOffsetB;
        var capCenterAToProjectedPoint = projectedPoint - capCenterA;
        return new Vector2(ScalarMath.Dot(capCenterAToProjectedPoint, rA.X), ScalarMath.Dot(capCenterAToProjectedPoint, rA.Z));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static Vector2 ProjectOntoCapB(float capCenterBY, float inverseLocalNormalY, Vector3 localNormal, Vector3 point)
    {
        var tAOnB = (point.Y - capCenterBY) * inverseLocalNormalY;
        return new Vector2(point.X - localNormal.X * tAOnB, point.Z - localNormal.Z * tAOnB);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void IntersectLineCircle(Vector2 linePosition, Vector2 lineDirection, float radius, out float tMin, out float tMax)
    {
        var a = Dot2(lineDirection, lineDirection);
        var inverseA = 1f / a;
        var b = Dot2(linePosition, lineDirection);
        var c = Dot2(linePosition, linePosition);
        var radiusSquared = radius * radius;
        c -= radiusSquared;
        var tOffset = MathF.Sqrt(float.MaxNative(0f, b * b - a * c)) * inverseA;
        var tBase = -b * inverseA;
        tMin = tBase - tOffset;
        tMax = tBase + tOffset;
        //If the projected line direction is zero, just compress the interval to tBase.
        var useFallback = ScalarMath.LessMask(MathF.Abs(a), 1e-12f);
        tMin = ScalarMath.Select(useFallback, tBase, tMin);
        tMax = ScalarMath.Select(useFallback, tBase, tMax);
    }

    /// <summary>
    /// Two structurally identical line-circle intersections in lanes 0-1. Per-lane operation order matches the scalar
    /// IntersectLineCircle exactly and sqrt/div/mul/add are lane-wise, so results are bitwise identical to two scalar calls.
    /// Lanes 2-3 compute garbage (including 0/0 NaNs) that is never read.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void IntersectLineCircle2(Vector2 positionA, Vector2 directionA, float radiusA, Vector2 positionB, Vector2 directionB, float radiusB,
        out float tMinA, out float tMaxA, out float tMinB, out float tMaxB)
    {
        var posX = Vector128.Create(positionA.X, positionB.X, 0f, 0f);
        var posY = Vector128.Create(positionA.Y, positionB.Y, 0f, 0f);
        var dirX = Vector128.Create(directionA.X, directionB.X, 0f, 0f);
        var dirY = Vector128.Create(directionA.Y, directionB.Y, 0f, 0f);
        var radius = Vector128.Create(radiusA, radiusB, 0f, 0f);
        var a = dirX * dirX + dirY * dirY;
        var inverseA = Vector128<float>.One / a;
        var b = posX * dirX + posY * dirY;
        var c = posX * posX + posY * posY - radius * radius;
        var tOffset = Vector128.Sqrt(Vector128.MaxNative(Vector128<float>.Zero, b * b - a * c)) * inverseA;
        var tBase = -b * inverseA;
        var tMin = tBase - tOffset;
        var tMax = tBase + tOffset;
        //If the projected line direction is zero, just compress the interval to tBase.
        var useFallback = Vector128.LessThan(Vector128.Abs(a), Vector128.Create(1e-12f));
        tMin = Vector128.ConditionalSelect(useFallback, tBase, tMin);
        tMax = Vector128.ConditionalSelect(useFallback, tBase, tMax);
        tMinA = tMin.ToScalar();
        tMaxA = tMax.ToScalar();
        tMinB = tMin.GetElement(1);
        tMaxB = tMax.GetElement(1);
    }

    /// <summary>
    /// Four structurally identical cap projections in lanes 0-3 (all points share the cap plane Y). Per-lane operation order
    /// matches ProjectOntoCapA exactly, including the ordered dot's (x + y) + z association, so results are bitwise identical
    /// to four scalar calls.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void ProjectOntoCapA4(float capCenterBY, Vector3 capCenterA, in Matrix3x3 rA, float inverseNDotAY, Vector3 localNormal,
        Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3,
        out Vector2 projected0, out Vector2 projected1, out Vector2 projected2, out Vector2 projected3)
    {
        var px = Vector128.Create(p0.X, p1.X, p2.X, p3.X);
        var py = Vector128.Create(capCenterBY);
        var pz = Vector128.Create(p0.Y, p1.Y, p2.Y, p3.Y);
        var dx = Vector128.Create(capCenterA.X) - px;
        var dy = Vector128.Create(capCenterA.Y) - py;
        var dz = Vector128.Create(capCenterA.Z) - pz;
        var tDistance = dx * Vector128.Create(rA.Y.X) + dy * Vector128.Create(rA.Y.Y) + dz * Vector128.Create(rA.Y.Z);
        var tBOnA = tDistance * Vector128.Create(inverseNDotAY);
        var projX = px + Vector128.Create(localNormal.X) * tBOnA;
        var projY = py + Vector128.Create(localNormal.Y) * tBOnA;
        var projZ = pz + Vector128.Create(localNormal.Z) * tBOnA;
        var cx = projX - Vector128.Create(capCenterA.X);
        var cy = projY - Vector128.Create(capCenterA.Y);
        var cz = projZ - Vector128.Create(capCenterA.Z);
        var outX = cx * Vector128.Create(rA.X.X) + cy * Vector128.Create(rA.X.Y) + cz * Vector128.Create(rA.X.Z);
        var outY = cx * Vector128.Create(rA.Z.X) + cy * Vector128.Create(rA.Z.Y) + cz * Vector128.Create(rA.Z.Z);
        projected0 = new Vector2(outX.ToScalar(), outY.ToScalar());
        projected1 = new Vector2(outX.GetElement(1), outY.GetElement(1));
        projected2 = new Vector2(outX.GetElement(2), outY.GetElement(2));
        projected3 = new Vector2(outX.GetElement(3), outY.GetElement(3));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static Vector3 FromCapBTo3D(Vector2 contact, float capCenterBY)
    {
        return new Vector3(contact.X, capCenterBY, contact.Y);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void TransformContact(
        in Vector3 contact, in Vector3 localFeaturePositionA, in Vector3 localFeatureNormalA, float inverseFeatureNormalADotLocalNormal,
        in Vector3 localOffsetB, in Matrix3x3 orientationB, float negativeSpeculativeMargin,
        out Vector3 aToContact, out float depth, ref bool contactExists)
    {
        //Project the contact on B along the contact normal to the 'face' of A to compute a per-contact depth.
        var featureOffset = contact - localFeaturePositionA;
        var tDistance = ScalarMath.Dot(featureOffset, localFeatureNormalA);
        depth = tDistance * inverseFeatureNormalADotLocalNormal;
        var localAToContact = contact + localOffsetB;
        Matrix3x3.Transform(localAToContact, orientationB, out aToContact);
        contactExists = contactExists & depth >= negativeSpeculativeMargin;
    }

    /// <summary>
    /// Mirror of CapsuleCylinderTester.GetClosestPointsBetweenSegments.
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
    /// Mirror of CapsuleCylinderTester.GetContactIntervalBetweenSegments.
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
        in Cylinder a, in Cylinder b, float speculativeMargin,
        in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex4ManifoldScalar manifold)
    {
        manifold = default;
        Matrix3x3.CreateFromQuaternion(orientationA, out Matrix3x3 worldRA);
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 worldRB);
        //Work in b's local space.
        ScalarMath.MultiplyByTranspose(worldRA, worldRB, out var rA);
        var localOffsetB = ScalarMath.TransformByTransposed(offsetB, worldRB);
        var localOffsetA = -localOffsetB;

        var length = ScalarMath.Length(localOffsetA);
        var localNormal = localOffsetA * (1f / length);
        if (length < 1e-10f)
        {
            localNormal.X = 0f;
            localNormal.Y = 1f;
            localNormal.Z = 0f;
        }

        var depthThreshold = -speculativeMargin;
        var epsilonScale = float.MinNative(float.MaxNative(a.HalfLength, a.Radius), float.MaxNative(b.HalfLength, b.Radius));
        CylinderDepthRefiner.FindMinimumDepth(
            b, a, localOffsetA, rA, localNormal, epsilonScale * 1e-6f, depthThreshold,
            out var depth, out localNormal, out var closestOnB, maximumIterations: 25);

        if (depth < depthThreshold)
        {
            //The depth is lower than the speculative margin; no contacts.
            return;
        }

        //Contact generation by dominant feature: cap A-cap B, cap A-side B, side A-cap B, side A-side B.
        var nDotAY = ScalarMath.Dot(rA.Y, localNormal);
        var inverseNDotAY = 1f / nDotAY;
        var inverseLocalNormalY = 1f / localNormal.Y;
        var useNegative = ScalarMath.GreaterMask(nDotAY, 0f);
        var capCenterA = rA.Y * ScalarMath.Select(useNegative, -a.HalfLength, a.HalfLength);
        capCenterA += localOffsetA;
        var capCenterBY = ScalarMath.Select(ScalarMath.LessMask(localNormal.Y, 0f), -b.HalfLength, b.HalfLength);

        const float capThreshold = 0.70710678118f;
        var useCapA = MathF.Abs(nDotAY) > capThreshold;
        var useCapB = MathF.Abs(localNormal.Y) > capThreshold;
        Vector3 contact0 = default;
        Vector3 contact1 = default;
        Vector3 contact2 = default;
        Vector3 contact3 = default;
        var useCapCap = useCapA & useCapB;

        //The extreme points along the contact normal are shared between multiple contact generator paths.
        var bToAOffset = localNormal * -depth;
        var extremeA = closestOnB + bToAOffset;
        var extremeAHorizontalOffset = extremeA - localOffsetA;
        var verticalDot = ScalarMath.Dot(extremeAHorizontalOffset, rA.Y);
        var toRemove = rA.Y * verticalDot;
        extremeAHorizontalOffset = extremeAHorizontalOffset - toRemove;

        var extremeB = new Vector2(closestOnB.X, closestOnB.Z);

        //Whole-vector negate-select on the hoisted mask; bitwise identical to the componentwise selects.
        var capFeatureNormalA = ScalarMath.Select(useNegative, -rA.Y, rA.Y);
        //Assume cap-cap to start with; the other branches override.
        var featureNormalA = capFeatureNormalA;
        var featurePositionA = capCenterA;

        if (useCapCap)
        {
            const float parallelThresholdScalar = 0.9999f;
            const float parallelInterpolationMaxScalar = 0.99995f;
            const float inverseParallelInterpolationSpanScalar = 1f / (parallelInterpolationMaxScalar - parallelThresholdScalar);
            var absADot = MathF.Abs(nDotAY);
            var absBDot = MathF.Abs(localNormal.Y);
            var aCapNotParallel = absADot < parallelThresholdScalar;
            var bCapNotParallel = absBDot < parallelThresholdScalar;

            //If both caps are not parallel, we'll use the deepest point on B.
            Vector2 capContact0 = extremeB;

            var bothNotParallel = aCapNotParallel & bCapNotParallel;
            if (!bothNotParallel)
            {
                //The local normal is aligned with at least one of the two cap normals.
                //Note: in the wide version, lanes with bothNotParallel that share a bundle with this path get a replacement blend applied
                //to capContact0 with parallelWeight=0, which is bitwise the identity, so skipping this block for them is exact.
                var capCenterAOnB = ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, capCenterA);
                var horizontalOffsetLength = MathF.Sqrt(capCenterAOnB.X * capCenterAOnB.X + capCenterAOnB.Y * capCenterAOnB.Y);
                var inverseHorizontalOffsetLength = 1f / horizontalOffsetLength;
                var horizontalOffsetDirection = capCenterAOnB * inverseHorizontalOffsetLength;
                var useBothParallelFallback = ScalarMath.LessMask(horizontalOffsetLength, 1e-14f);
                horizontalOffsetDirection = ScalarMath.Select(useBothParallelFallback, new Vector2(1f, 0f), horizontalOffsetDirection);
                var initialLineStart = horizontalOffsetDirection * b.Radius;
                var contact1LineEndpoint = -initialLineStart;
                var contact1LineDirectionOnB = contact1LineEndpoint - initialLineStart;

                //Use a perpendicular line to get a decent cross section of the contact manifold; see the wide implementation's commentary.
                //The second line's endpoints depend only on values available here, so they are computed up front (pure reordering of
                //independent definitions; every defining expression is unchanged) to pack all four cap projections into one 4-wide call.
                var circleIntersectionT = 0.5f * (horizontalOffsetLength + (b.Radius * b.Radius - a.Radius * a.Radius) * inverseHorizontalOffsetLength);
                var secondLineStartT = float.MinNative(horizontalOffsetLength, float.MaxNative(0f, circleIntersectionT));
                var secondLineStartOnB = horizontalOffsetDirection * secondLineStartT;
                var secondLineDirectionOnB = new Vector2(horizontalOffsetDirection.Y, -horizontalOffsetDirection.X);
                var secondLineEndOnB = secondLineStartOnB + secondLineDirectionOnB;

                ProjectOntoCapA4(capCenterBY, capCenterA, rA, inverseNDotAY, localNormal,
                    initialLineStart, contact1LineEndpoint, secondLineStartOnB, secondLineEndOnB,
                    out var lineStartOnA, out var lineEndOnA, out var secondLineStartOnA, out var secondLineEndOnA);

                var lineDirectionOnA = lineEndOnA - lineStartOnA;
                IntersectLineCircle(lineStartOnA, lineDirectionOnA, a.Radius, out var contact1TMinA, out var contact1TMaxA);
                var firstLineTMin = float.MaxNative(contact1TMinA, 0f);
                var firstLineTMax = float.MinNative(contact1TMaxA, 1f);
                capContact0 = contact1LineDirectionOnB * firstLineTMin;
                capContact0 = initialLineStart + capContact0;
                var capContact1 = contact1LineDirectionOnB * firstLineTMax;
                capContact1 = initialLineStart + capContact1;

                var secondLineDirectionOnA = secondLineEndOnA - secondLineStartOnA;
                IntersectLineCircle2(secondLineStartOnA, secondLineDirectionOnA, a.Radius, secondLineStartOnB, secondLineDirectionOnB, b.Radius,
                    out var secondLineTMinA, out var secondLineTMaxA, out var secondLineTMinB, out var secondLineTMaxB);
                var secondLineTMin = float.MaxNative(secondLineTMinA, secondLineTMinB);
                var secondLineTMax = float.MinNative(secondLineTMaxA, secondLineTMaxB);

                var capContact2 = secondLineDirectionOnB * secondLineTMin;
                var capContact3 = secondLineDirectionOnB * secondLineTMax;
                capContact2 = secondLineStartOnB + capContact2;
                capContact3 = secondLineStartOnB + capContact3;

                //The manifold assumed parallel caps; replace one of the four points with the deepest point if necessary.
                var weightAParallel = float.MaxNative(0f, float.MinNative(1f, (absADot - parallelThresholdScalar) * inverseParallelInterpolationSpanScalar));
                var weightBParallel = float.MaxNative(0f, float.MinNative(1f, (absBDot - parallelThresholdScalar) * inverseParallelInterpolationSpanScalar));
                var parallelWeight = weightAParallel * weightBParallel;
                var extremeWeight = 1f - parallelWeight;
                var manifoldCenterToExtremeB = extremeB - secondLineStartOnB;
                var replaceDot0 = horizontalOffsetDirection.X * manifoldCenterToExtremeB.X + horizontalOffsetDirection.Y * manifoldCenterToExtremeB.Y;
                var replaceDot2 = secondLineDirectionOnB.X * manifoldCenterToExtremeB.X + secondLineDirectionOnB.Y * manifoldCenterToExtremeB.Y;
                //Mask selects on whole Vector2s; per-component arithmetic and blend semantics identical to the float selects.
                var replace0Or1 = ScalarMath.GreaterMask(MathF.Abs(replaceDot0), MathF.Abs(replaceDot2));
                var replace0 = ScalarMath.GreaterMask(replaceDot0, 0f) & replace0Or1;
                var replace1 = ScalarMath.LessOrEqualMask(replaceDot0, 0f) & replace0Or1;
                var replace2 = ScalarMath.LessMask(replaceDot2, 0f) & ~replace0Or1;
                var replace3 = ScalarMath.GreaterOrEqualMask(replaceDot2, 0f) & ~replace0Or1;
                var weightedExtremeB = extremeB * extremeWeight;
                capContact0 = ScalarMath.Select(replace0, weightedExtremeB + capContact0 * parallelWeight, capContact0);
                capContact1 = ScalarMath.Select(replace1, weightedExtremeB + capContact1 * parallelWeight, capContact1);
                capContact2 = ScalarMath.Select(replace2, weightedExtremeB + capContact2 * parallelWeight, capContact2);
                capContact3 = ScalarMath.Select(replace3, weightedExtremeB + capContact3 * parallelWeight, capContact3);

                contact1 = FromCapBTo3D(capContact1, capCenterBY);
                contact2 = FromCapBTo3D(capContact2, capCenterBY);
                contact3 = FromCapBTo3D(capContact3, capCenterBY);
                manifold.Contact1Exists = firstLineTMax > firstLineTMin;
                //If 0 and 1 are in the same spot, there aren't going to be any useful additional contacts.
                manifold.Contact2Exists = manifold.Contact1Exists;
                manifold.Contact3Exists = manifold.Contact1Exists & secondLineTMax > secondLineTMin;
            }
            contact0 = FromCapBTo3D(capContact0, capCenterBY);
            manifold.Contact0Exists = true;
        }
        var useCapSide = useCapA != useCapB;
        //The side normal is used in both of the following contact generator cases; the wide version computes it unconditionally.
        var ax = ScalarMath.Dot(rA.X, localNormal);
        var az = ScalarMath.Dot(rA.Z, localNormal);
        var horizontalNormalLengthA = MathF.Sqrt(ax * ax + az * az);
        var inverseHorizontalNormalLengthA = 1f / horizontalNormalLengthA;
        var xScale = ax * inverseHorizontalNormalLengthA;
        var zScale = az * inverseHorizontalNormalLengthA;
        var sideFeatureNormalAX = rA.X * xScale;
        var sideFeatureNormalAZ = rA.Z * zScale;
        var sideFeatureNormalA = sideFeatureNormalAX + sideFeatureNormalAZ;
        //Whole-vector adds/constructions; bitwise identical to the componentwise assembly.
        var sideCenterA = extremeAHorizontalOffset + localOffsetA;
        var sideCenterB = new Vector3(extremeB.X, 0f, extremeB.Y);
        if (useCapSide)
        {
            //Pick a line on the side of one cylinder, the cap on the other, and intersect the projected line with the cap.
            var sideLineEndA = sideCenterA + rA.Y;
            var sideLineEndB = new Vector3(sideCenterB.X, 1f, sideCenterB.Z);
            var projectedLineStartBOnA = ProjectOntoCapA(capCenterA, rA, inverseNDotAY, localNormal, sideCenterB);
            var projectedLineStartAOnB = ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, sideCenterA);
            var projectedLineEndBOnA = ProjectOntoCapA(capCenterA, rA, inverseNDotAY, localNormal, sideLineEndB);
            var projectedLineEndAOnB = ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, sideLineEndA);

            //One bool->mask conversion feeds every select in this block.
            var useCapAMask = ScalarMath.Mask(useCapA);
            var projectedLineStart = ScalarMath.Select(useCapAMask, projectedLineStartBOnA, projectedLineStartAOnB);
            var projectedLineEnd = ScalarMath.Select(useCapAMask, projectedLineEndBOnA, projectedLineEndAOnB);
            var radius = ScalarMath.Select(useCapAMask, a.Radius, b.Radius);
            var sideHalfLength = ScalarMath.Select(useCapAMask, b.HalfLength, a.HalfLength);
            var projectedLineDirection = projectedLineEnd - projectedLineStart;
            IntersectLineCircle(projectedLineStart, projectedLineDirection, radius, out var tMin, out var tMax);
            tMin = float.MinNative(sideHalfLength, float.MaxNative(-sideHalfLength, tMin));
            tMax = float.MinNative(sideHalfLength, tMax);

            //Contacts are on cylinder B: if the cap was on A, scale the side line of B; otherwise use the projected side line of A.
            //Build both whole-vector candidates and blend; per-component arithmetic identical to the old componentwise selects.
            contact0 = ScalarMath.Select(useCapAMask,
                new Vector3(sideCenterB.X, tMin, sideCenterB.Z),
                new Vector3(projectedLineStart.X + tMin * projectedLineDirection.X, capCenterBY, projectedLineStart.Y + tMin * projectedLineDirection.Y));
            contact1 = ScalarMath.Select(useCapAMask,
                new Vector3(sideCenterB.X, tMax, sideCenterB.Z),
                new Vector3(projectedLineStart.X + tMax * projectedLineDirection.X, capCenterBY, projectedLineStart.Y + tMax * projectedLineDirection.Y));
            manifold.Contact0Exists = true;
            manifold.Contact1Exists = tMax > tMin;

            featureNormalA = ScalarMath.Select(useCapAMask, capFeatureNormalA, sideFeatureNormalA);
            featurePositionA = ScalarMath.Select(useCapAMask, capCenterA, sideCenterA);
        }
        var useSideSide = !useCapA & !useCapB;
        if (useSideSide)
        {
            //Similar to capsule-capsule; test the side line of A against the line in the center of B.
            var sideCenterAToLineB = -sideCenterA;
            var horizontalNormalLengthSquaredB = localNormal.X * localNormal.X + localNormal.Z * localNormal.Z;
            var inverseHorizontalNormalLengthSquaredB = 1f / horizontalNormalLengthSquaredB;
            GetContactIntervalBetweenSegments(a.HalfLength, b.HalfLength, rA.Y, localNormal, inverseHorizontalNormalLengthSquaredB, sideCenterAToLineB, out var contactTMin, out var contactTMax);

            contact0 = new Vector3(extremeB.X, contactTMin, extremeB.Y);
            contact1 = new Vector3(extremeB.X, contactTMax, extremeB.Y);
            manifold.Contact0Exists = true;
            manifold.Contact1Exists = contactTMax > contactTMin;
            featureNormalA = sideFeatureNormalA;
            featurePositionA = sideCenterA;
        }
        var featureNormalADotLocalNormal = ScalarMath.Dot(featureNormalA, localNormal);
        //No division guard; the feature normal we picked is never more than 45 degrees away from the local normal.
        var inverseFeatureNormalADotLocalNormal = 1f / featureNormalADotLocalNormal;
        var negativeSpeculativeMargin = -speculativeMargin;
        TransformContact(contact0, featurePositionA, featureNormalA, inverseFeatureNormalADotLocalNormal, localOffsetB, worldRB, negativeSpeculativeMargin, out manifold.OffsetA0, out manifold.Depth0, ref manifold.Contact0Exists);
        TransformContact(contact1, featurePositionA, featureNormalA, inverseFeatureNormalADotLocalNormal, localOffsetB, worldRB, negativeSpeculativeMargin, out manifold.OffsetA1, out manifold.Depth1, ref manifold.Contact1Exists);
        TransformContact(contact2, featurePositionA, featureNormalA, inverseFeatureNormalADotLocalNormal, localOffsetB, worldRB, negativeSpeculativeMargin, out manifold.OffsetA2, out manifold.Depth2, ref manifold.Contact2Exists);
        TransformContact(contact3, featurePositionA, featureNormalA, inverseFeatureNormalADotLocalNormal, localOffsetB, worldRB, negativeSpeculativeMargin, out manifold.OffsetA3, out manifold.Depth3, ref manifold.Contact3Exists);
        Matrix3x3.Transform(localNormal, worldRB, out manifold.Normal);
        //Contact generators all obey a reasonably solid order, so we can use a trivial feature description.
        manifold.FeatureId0 = 0;
        manifold.FeatureId1 = 1;
        manifold.FeatureId2 = 2;
        manifold.FeatureId3 = 3;
    }
}
