using BepuPhysics.Collidables;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace AosBaselines;

/// <summary>
/// Scalar AoS port of CapsuleBoxTester, bitwise identical per lane. Closed-form: box-local segment-vs-edge tests for the three
/// box edge directions plus three face tests, a min-depth select chain, and a representative-face interval clip along the
/// capsule axis; no DepthRefiner and no per-lane iteration. Divisions can produce inf/NaN (parallel axes, capsule axis in the
/// face plane); values are computed unconditionally and then selected, matching the wide ConditionalSelects.
/// The quaternion helpers are verbatim mirrors of the QuaternionWide operations (note that QuaternionWide.Conjugate negates W
/// only, unlike System.Numerics.Quaternion.Conjugate).
/// </summary>
public static class CapsuleBoxScalarTester
{
    /// <summary>
    /// Mirrors QuaternionWide.Conjugate: negates W only.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void Conjugate(in Quaternion quaternion, out Quaternion result)
    {
        result.X = quaternion.X;
        result.Y = quaternion.Y;
        result.Z = quaternion.Z;
        result.W = -quaternion.W;
    }

    /// <summary>
    /// Mirrors QuaternionWide.ConcatenateWithoutOverlap's component order exactly.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void ConcatenateWithoutOverlap(in Quaternion a, in Quaternion b, out Quaternion result)
    {
        result.X = a.W * b.X + a.X * b.W + a.Z * b.Y - a.Y * b.Z;
        result.Y = a.W * b.Y + a.Y * b.W + a.X * b.Z - a.Z * b.X;
        result.Z = a.W * b.Z + a.Z * b.W + a.Y * b.X - a.X * b.Y;
        result.W = a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z;
    }

    /// <summary>
    /// Mirrors QuaternionWide.TransformWithoutOverlap: 2x-product expansion with left-associated 3-term sums.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void TransformWithoutOverlap(in Vector3 v, in Quaternion rotation, out Vector3 result)
    {
        var x2 = rotation.X + rotation.X;
        var y2 = rotation.Y + rotation.Y;
        var z2 = rotation.Z + rotation.Z;
        var xx2 = rotation.X * x2;
        var xy2 = rotation.X * y2;
        var xz2 = rotation.X * z2;
        var yy2 = rotation.Y * y2;
        var yz2 = rotation.Y * z2;
        var zz2 = rotation.Z * z2;
        var wx2 = rotation.W * x2;
        var wy2 = rotation.W * y2;
        var wz2 = rotation.W * z2;
        result.X = v.X * (1f - yy2 - zz2) + v.Y * (xy2 - wz2) + v.Z * (xz2 + wy2);
        result.Y = v.X * (xy2 + wz2) + v.Y * (1f - xx2 - zz2) + v.Z * (yz2 - wx2);
        result.Z = v.X * (xz2 - wy2) + v.Y * (yz2 + wx2) + v.Z * (1f - xx2 - yy2);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void Prepare(in Capsule a, in Box b, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Vector3 localOffsetA, out Vector3 capsuleAxis, out Vector3 edgeCenters)
    {
        Conjugate(orientationB, out var toLocalB);
        TransformWithoutOverlap(offsetB, toLocalB, out localOffsetA);
        localOffsetA = -localOffsetA;
        ConcatenateWithoutOverlap(orientationA, toLocalB, out var boxLocalOrientationA);
        //QuaternionWide.TransformUnitY's arithmetic is identical to TransformUnitXY's y output.
        ScalarMath.TransformUnitXY(boxLocalOrientationA, out _, out capsuleAxis);

        //Get the closest point on the capsule segment to the box center to choose which edge to use.
        var dot = ScalarMath.Dot(localOffsetA, capsuleAxis);
        var clampedDot = MathF.Min(a.HalfLength, MathF.Max(-a.HalfLength, dot));
        var offsetToCapsuleFromBox = capsuleAxis * clampedDot;
        offsetToCapsuleFromBox = localOffsetA - offsetToCapsuleFromBox;
        edgeCenters.X = offsetToCapsuleFromBox.X < 0f ? -b.HalfWidth : b.HalfWidth;
        edgeCenters.Y = offsetToCapsuleFromBox.Y < 0f ? -b.HalfHeight : b.HalfHeight;
        edgeCenters.Z = offsetToCapsuleFromBox.Z < 0f ? -b.HalfLength : b.HalfLength;
    }

    //Built to handle the Z edge hardcoded; callers swizzle arguments to handle X and Y, mirroring the wide code.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void TestBoxEdge(
        float offsetAX, float offsetAY, float offsetAZ,
        float capsuleAxisX, float capsuleAxisY, float capsuleAxisZ,
        float capsuleHalfLength, float boxEdgeCenterX, float boxEdgeCenterY,
        float boxHalfWidth, float boxHalfHeight, float boxHalfLength,
        out float taMin, out float taMax, out Vector3 closestPointOnA,
        out float nX, out float nY, out float nZ,
        out float ta, out float epsilon)
    {
        var abX = boxEdgeCenterX - offsetAX;
        var abY = boxEdgeCenterY - offsetAY;
        var daOffsetB = capsuleAxisX * abX + capsuleAxisY * abY - capsuleAxisZ * offsetAZ;
        //Note potential division by zero; the max mirrors the wide clamp against 1e-15.
        ta = (daOffsetB + offsetAZ * capsuleAxisZ) / MathF.Max(1e-15f, 1f - capsuleAxisZ * capsuleAxisZ);
        var tb = ta * capsuleAxisZ + offsetAZ;

        //Clamp solution to valid regions on edge line segment.
        var absdadb = MathF.Abs(capsuleAxisZ);
        var bOntoAOffset = boxHalfLength * absdadb;
        var aOntoBOffset = capsuleHalfLength * absdadb;
        taMin = MathF.Max(-capsuleHalfLength, MathF.Min(capsuleHalfLength, daOffsetB - bOntoAOffset));
        taMax = MathF.Min(capsuleHalfLength, MathF.Max(-capsuleHalfLength, daOffsetB + bOntoAOffset));
        var bMin = MathF.Max(-boxHalfLength, MathF.Min(boxHalfLength, offsetAZ - aOntoBOffset));
        var bMax = MathF.Min(boxHalfLength, MathF.Max(-boxHalfLength, offsetAZ + aOntoBOffset));
        ta = MathF.Min(MathF.Max(ta, taMin), taMax);
        tb = MathF.Min(MathF.Max(tb, bMin), bMax);

        closestPointOnA.X = ta * capsuleAxisX + offsetAX;
        closestPointOnA.Y = ta * capsuleAxisY + offsetAY;
        closestPointOnA.Z = ta * capsuleAxisZ + offsetAZ;
        nX = closestPointOnA.X - boxEdgeCenterX;
        nY = closestPointOnA.Y - boxEdgeCenterY;
        nZ = closestPointOnA.Z - tb;
        var squaredLength = nX * nX + nY * nY + nZ * nZ;
        //Intersecting axes produce a zero-length normal; fall back to cross(capsuleAxis, boxEdge) = (-capsuleAxisY, capsuleAxisX, 0),
        //and if that's also degenerate (parallel axes), to (1, 0, 0).
        var fallbackSquaredLength = capsuleAxisY * capsuleAxisY + capsuleAxisX * capsuleAxisX;
        epsilon = 1e-10f;
        var useFallback = squaredLength < epsilon;
        var useSecondFallback = useFallback & fallbackSquaredLength < epsilon;
        squaredLength = useSecondFallback ? 1f : useFallback ? fallbackSquaredLength : squaredLength;
        nX = useSecondFallback ? 1f : useFallback ? -capsuleAxisY : nX;
        nY = useSecondFallback ? 0f : useFallback ? capsuleAxisX : nY;
        nZ = useSecondFallback ? 0f : useFallback ? 0f : nZ;

        //Calibrate the normal to point from B to A.
        var calibrationDot = nX * offsetAX + nY * offsetAY + nZ * offsetAZ;
        var shouldNegate = calibrationDot < 0f;
        nX = shouldNegate ? -nX : nX;
        nY = shouldNegate ? -nY : nY;
        nZ = shouldNegate ? -nZ : nZ;

        var inverseLength = 1f / MathF.Sqrt(squaredLength);
        nX *= inverseLength;
        nY *= inverseLength;
        nZ *= inverseLength;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void TestAndRefineBoxEdge(
        float offsetAX, float offsetAY, float offsetAZ,
        float capsuleAxisX, float capsuleAxisY, float capsuleAxisZ,
        float capsuleHalfLength,
        float boxEdgeCenterX, float boxEdgeCenterY,
        float boxHalfWidth, float boxHalfHeight, float boxHalfLength,
        out float ta, out float depth, out float nX, out float nY, out float nZ)
    {
        TestBoxEdge(
            offsetAX, offsetAY, offsetAZ,
            capsuleAxisX, capsuleAxisY, capsuleAxisZ,
            capsuleHalfLength, boxEdgeCenterX, boxEdgeCenterY,
            boxHalfWidth, boxHalfHeight, boxHalfLength,
            out _, out _, out var closestPointOnA,
            out nX, out nY, out nZ,
            out ta, out _);

        //Compute the depth along that normal.
        var boxExtreme = MathF.Abs(nX) * boxHalfWidth + MathF.Abs(nY) * boxHalfHeight + MathF.Abs(nZ) * boxHalfLength;
        var capsuleExtreme = nX * closestPointOnA.X + nY * closestPointOnA.Y + nZ * closestPointOnA.Z;
        depth = boxExtreme - capsuleExtreme;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void TestBoxFace(float offsetAZ,
        float capsuleAxisZ, float capsuleHalfLength,
        float boxHalfLength,
        out float depth, out float normalSign)
    {
        normalSign = offsetAZ > 0f ? 1f : -1f;
        depth = boxHalfLength + MathF.Abs(capsuleAxisZ) * capsuleHalfLength - normalSign * offsetAZ;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void Select(
        ref float depth, ref float ta,
        ref float localNormalX, ref float localNormalY, ref float localNormalZ,
        float depthCandidate, float taCandidate,
        float localNormalCandidateX, float localNormalCandidateY, float localNormalCandidateZ)
    {
        var useCandidate = depthCandidate < depth;
        ta = useCandidate ? taCandidate : ta;
        depth = useCandidate ? depthCandidate : depth;
        localNormalX = useCandidate ? localNormalCandidateX : localNormalX;
        localNormalY = useCandidate ? localNormalCandidateY : localNormalY;
        localNormalZ = useCandidate ? localNormalCandidateZ : localNormalZ;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void Select(
        ref float depth,
        ref float localNormalX, ref float localNormalY, ref float localNormalZ,
        float depthCandidate,
        float localNormalCandidateX, float localNormalCandidateY, float localNormalCandidateZ)
    {
        var useCandidate = depthCandidate < depth;
        depth = useCandidate ? depthCandidate : depth;
        localNormalX = useCandidate ? localNormalCandidateX : localNormalX;
        localNormalY = useCandidate ? localNormalCandidateY : localNormalY;
        localNormalZ = useCandidate ? localNormalCandidateZ : localNormalZ;
    }

    public static void Test(in Capsule a, in Box b, float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex2ManifoldScalar manifold)
    {
        Prepare(a, b, offsetB, orientationA, orientationB, out var localOffsetA, out var capsuleAxis, out var edgeCenters);

        //Swizzle XYZ -> YZX
        Vector3 localNormal;
        TestAndRefineBoxEdge(localOffsetA.Y, localOffsetA.Z, localOffsetA.X,
            capsuleAxis.Y, capsuleAxis.Z, capsuleAxis.X,
            a.HalfLength,
            edgeCenters.Y, edgeCenters.Z,
            b.HalfHeight, b.HalfLength, b.HalfWidth,
            out var ta, out var depth, out localNormal.Y, out localNormal.Z, out localNormal.X);
        //Swizzle XYZ -> ZXY
        TestAndRefineBoxEdge(localOffsetA.Z, localOffsetA.X, localOffsetA.Y,
            capsuleAxis.Z, capsuleAxis.X, capsuleAxis.Y,
            a.HalfLength,
            edgeCenters.Z, edgeCenters.X,
            b.HalfLength, b.HalfWidth, b.HalfHeight,
            out var eyta, out var eyDepth, out var eynZ, out var eynX, out var eynY);
        Select(ref depth, ref ta, ref localNormal.X, ref localNormal.Y, ref localNormal.Z,
            eyDepth, eyta, eynX, eynY, eynZ);
        //Swizzle XYZ -> XYZ
        TestAndRefineBoxEdge(localOffsetA.X, localOffsetA.Y, localOffsetA.Z,
            capsuleAxis.X, capsuleAxis.Y, capsuleAxis.Z,
            a.HalfLength,
            edgeCenters.X, edgeCenters.Y,
            b.HalfWidth, b.HalfHeight, b.HalfLength,
            out var ezta, out var ezDepth, out var eznX, out var eznY, out var eznZ);
        Select(ref depth, ref ta, ref localNormal.X, ref localNormal.Y, ref localNormal.Z,
            ezDepth, ezta, eznX, eznY, eznZ);

        //Face X
        TestBoxFace(localOffsetA.X,
            capsuleAxis.X, a.HalfLength,
            b.HalfWidth,
            out var fxDepth, out var fxn);
        Select(ref depth, ref localNormal.X, ref localNormal.Y, ref localNormal.Z,
            fxDepth, fxn, 0f, 0f);
        //Face Y
        TestBoxFace(localOffsetA.Y,
            capsuleAxis.Y, a.HalfLength,
            b.HalfHeight,
            out var fyDepth, out var fyn);
        Select(ref depth, ref localNormal.X, ref localNormal.Y, ref localNormal.Z,
            fyDepth, 0f, fyn, 0f);
        //Face Z
        TestBoxFace(localOffsetA.Z,
            capsuleAxis.Z, a.HalfLength,
            b.HalfLength,
            out var fzDepth, out var fzn);
        Select(ref depth, ref localNormal.X, ref localNormal.Y, ref localNormal.Z,
            fzDepth, 0f, 0f, fzn);

        //Choose a representative box face based on the collision normal detected above, and compute the interval of intersection
        //along the capsule axis of the box face projected onto the capsule axis.
        var xDot = localNormal.X * fxn;
        var yDot = localNormal.Y * fyn;
        var zDot = localNormal.Z * fzn;
        var useX = xDot > MathF.Max(yDot, zDot);
        var useY = yDot > zDot & !useX;
        var useZ = !useX & !useY;

        //Unproject the capsule center and capsule axis onto the representative face plane.
        var faceNormalDotLocalNormal = useX ? xDot : useY ? yDot : zDot;
        var inverseFaceNormalDotLocalNormal = 1f / MathF.Max(1e-15f, faceNormalDotLocalNormal);
        var capsuleAxisDotFaceNormal = useX ? capsuleAxis.X * fxn : useY ? capsuleAxis.Y * fyn : capsuleAxis.Z * fzn;
        var capsuleCenterDotFaceNormal = useX ? localOffsetA.X * fxn : useY ? localOffsetA.Y * fyn : localOffsetA.Z * fzn;
        var facePlaneOffset = useX ? b.HalfWidth : useY ? b.HalfHeight : b.HalfLength;
        var tAxis = capsuleAxisDotFaceNormal * inverseFaceNormalDotLocalNormal;
        var tCenter = (capsuleCenterDotFaceNormal - facePlaneOffset) * inverseFaceNormalDotLocalNormal;

        //Work in tangent space.
        //Face X uses tangents Y and Z. Face Y uses tangents X and Z. Face Z uses tangents X and Y.
        var axisOffset = localNormal * tAxis;
        var centerOffset = localNormal * tCenter;
        var unprojectedAxis = capsuleAxis - axisOffset;
        var unprojectedCenter = localOffsetA - centerOffset;
        var tangentSpaceAxisX = useX ? unprojectedAxis.Y : unprojectedAxis.X;
        var tangentSpaceAxisY = useZ ? unprojectedAxis.Y : unprojectedAxis.Z;
        var tangentSpaceCenterX = useX ? unprojectedCenter.Y : unprojectedCenter.X;
        var tangentSpaceCenterY = useZ ? unprojectedCenter.Y : unprojectedCenter.Z;
        //Slightly boost the size of the face to avoid minor numerical issues that could block coplanar contacts.
        var epsilonScale = MathF.Min(MathF.Max(b.HalfWidth, MathF.Max(b.HalfHeight, b.HalfLength)), MathF.Max(a.HalfLength, a.Radius));
        var epsilon = epsilonScale * 1e-3f;
        var halfExtentX = epsilon + (useX ? b.HalfHeight : b.HalfWidth);
        var halfExtentY = epsilon + (useZ ? b.HalfHeight : b.HalfLength);

        //Compute interval bounded by edge normals pointing along tangentX.
        //The reciprocals can be infinite and the products NaN; compute unconditionally, then select, mirroring the wide code.
        var inverseAxisX = -1f / tangentSpaceAxisX;
        var inverseAxisY = -1f / tangentSpaceAxisY;
        var tX0 = (tangentSpaceCenterX - halfExtentX) * inverseAxisX;
        var tX1 = (tangentSpaceCenterX + halfExtentX) * inverseAxisX;
        var tY0 = (tangentSpaceCenterY - halfExtentY) * inverseAxisY;
        var tY1 = (tangentSpaceCenterY + halfExtentY) * inverseAxisY;
        var minX = MathF.Min(tX0, tX1);
        var maxX = MathF.Max(tX0, tX1);
        var minY = MathF.Min(tY0, tY1);
        var maxY = MathF.Max(tY0, tY1);
        //Protect against division by zero. If the unprojected capsule is within the slab, use an infinite interval. If it's outside and parallel, use an invalid interval.
        var useFallbackX = MathF.Abs(tangentSpaceAxisX) < 1e-15f;
        var useFallbackY = MathF.Abs(tangentSpaceAxisY) < 1e-15f;
        var centerContainedX = MathF.Abs(tangentSpaceCenterX) <= halfExtentX;
        var centerContainedY = MathF.Abs(tangentSpaceCenterY) <= halfExtentY;
        const float largeNegative = -float.MaxValue;
        const float largePositive = float.MaxValue;
        minX = useFallbackX ? centerContainedX ? largeNegative : largePositive : minX;
        maxX = useFallbackX ? centerContainedX ? largePositive : largeNegative : maxX;
        minY = useFallbackY ? centerContainedY ? largeNegative : largePositive : minY;
        maxY = useFallbackY ? centerContainedY ? largePositive : largeNegative : maxY;

        var faceMin = MathF.Max(minX, minY);
        var faceMax = MathF.Min(maxX, maxY);
        //Clamp the resulting interval to the capsule axis.
        var tMin = MathF.Max(MathF.Min(faceMin, a.HalfLength), -a.HalfLength);
        var tMax = MathF.Max(MathF.Min(faceMax, a.HalfLength), -a.HalfLength);
        var faceIntervalExists = faceMax >= faceMin;
        tMin = faceIntervalExists ? MathF.Min(tMin, ta) : ta;
        tMax = faceIntervalExists ? MathF.Max(tMax, ta) : ta;

        //Each contact may have its own depth; reuse the unprojection from earlier to compute the offset between the points.
        var separationMin = tCenter + tAxis * tMin;
        var separationMax = tCenter + tAxis * tMax;
        manifold.Depth0 = a.Radius - separationMin;
        manifold.Depth1 = a.Radius - separationMax;

        var localA0 = capsuleAxis * tMin;
        var localA1 = capsuleAxis * tMax;

        manifold.FeatureId0 = 0;
        manifold.FeatureId1 = 1;

        //Transform A0, A1, and the normal into world space.
        //Matrix3x3.CreateFromQuaternion and Matrix3x3.Transform were fuzz-verified bitwise-equal to their wide counterparts.
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 orientationMatrixB);
        Matrix3x3.Transform(localNormal, orientationMatrixB, out manifold.Normal);
        Matrix3x3.Transform(localA0, orientationMatrixB, out manifold.OffsetA0);
        Matrix3x3.Transform(localA1, orientationMatrixB, out manifold.OffsetA1);

        //Apply the normal offset to the contact positions.
        var negativeOffsetFromA0 = manifold.Depth0 * 0.5f - a.Radius;
        var negativeOffsetFromA1 = manifold.Depth1 * 0.5f - a.Radius;
        var normalPush0 = manifold.Normal * negativeOffsetFromA0;
        var normalPush1 = manifold.Normal * negativeOffsetFromA1;
        manifold.OffsetA0 = manifold.OffsetA0 + normalPush0;
        manifold.OffsetA1 = manifold.OffsetA1 + normalPush1;

        var minimumAcceptedDepth = -speculativeMargin;
        manifold.Contact0Exists = manifold.Depth0 >= minimumAcceptedDepth;
        manifold.Contact1Exists = manifold.Depth1 >= minimumAcceptedDepth & tMax - tMin > 1e-7f * a.HalfLength;
    }
}
