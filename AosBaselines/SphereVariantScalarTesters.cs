using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace AosBaselines;

using SphereHullDepthRefiner = ScalarDepthRefiner<ConvexHull, HullSupportScalar, Sphere, SphereSupportScalar>;

/// <summary>
/// Scalar counterpart of the orientation-B-only Convex1ContactManifoldWide pair tester interface.
/// Static abstract so generic fuzz/bench plumbing specializes without delegate overhead.
/// </summary>
public interface ISphereVariantScalarTester<TShapeB>
{
    static abstract void Test(in Sphere a, in TShapeB b, float speculativeMargin, in Vector3 offsetB, in Quaternion orientationB, out Convex1ManifoldScalar manifold);
}

/// <summary>Scalar AoS port of SphereCapsuleTester, bitwise identical per lane.</summary>
public struct SphereCapsuleScalarTester : ISphereVariantScalarTester<Capsule>
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Test(in Sphere a, in Capsule b, float speculativeMargin, in Vector3 offsetB, in Quaternion orientationB, out Convex1ManifoldScalar manifold)
    {
        //The contact for a sphere-capsule pair is based on the closest point of the sphere center to the capsule internal line segment.
        ScalarMath.TransformUnitXY(orientationB, out var x, out var y);
        var t = ScalarMath.Dot(y, offsetB);
        t = float.MinNative(b.HalfLength, float.MaxNative(-b.HalfLength, -t));
        var capsuleLocalClosestPointOnLineSegment = y * t;

        var sphereToInternalSegment = offsetB + capsuleLocalClosestPointOnLineSegment;
        var internalDistance = ScalarMath.Length(sphereToInternalSegment);
        //Note that the normal points from B to A by convention. Here, the sphere is A, the capsule is B, so the normalization requires a negation.
        var inverseDistance = -1f / internalDistance;
        manifold.Normal = sphereToInternalSegment * inverseDistance;
        var normalIsValid = internalDistance > 0f;
        //If the center of the sphere is on the internal line segment, choose a direction on the plane defined by the capsule's up vector.
        if (!normalIsValid)
            manifold.Normal = x;
        manifold.Depth = a.Radius + b.Radius - internalDistance;
        manifold.FeatureId = 0;

        var negativeOffsetFromSphere = manifold.Depth * 0.5f - a.Radius;
        manifold.OffsetA = manifold.Normal * negativeOffsetFromSphere;
        manifold.ContactExists = manifold.Depth > -speculativeMargin;
    }
}

/// <summary>Scalar AoS port of SphereBoxTester, bitwise identical per lane.</summary>
public struct SphereBoxScalarTester : ISphereVariantScalarTester<Box>
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Test(in Sphere a, in Box b, float speculativeMargin, in Vector3 offsetB, in Quaternion orientationB, out Convex1ManifoldScalar manifold)
    {
        //Clamp the position of the sphere to the box. Note implicit negation: localOffsetB works due to box symmetry.
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 orientationMatrixB);
        var localOffsetB = ScalarMath.TransformByTransposed(offsetB, orientationMatrixB);
        Vector3 clampedLocalOffsetB;
        clampedLocalOffsetB.X = float.MinNative(float.MaxNative(localOffsetB.X, -b.HalfWidth), b.HalfWidth);
        clampedLocalOffsetB.Y = float.MinNative(float.MaxNative(localOffsetB.Y, -b.HalfHeight), b.HalfHeight);
        clampedLocalOffsetB.Z = float.MinNative(float.MaxNative(localOffsetB.Z, -b.HalfLength), b.HalfLength);
        //Implicit negation to make the normal point from B to A, following convention.
        var outsideNormal = clampedLocalOffsetB - localOffsetB;
        var distance = ScalarMath.Length(outsideNormal);
        var inverseDistance = 1f / distance;
        outsideNormal = outsideNormal * inverseDistance;
        var outsideDepth = a.Radius - distance;

        //If the sphere center is inside the box, then the shortest local axis to exit must be chosen.
        var depthX = b.HalfWidth - MathF.Abs(localOffsetB.X);
        var depthY = b.HalfHeight - MathF.Abs(localOffsetB.Y);
        var depthZ = b.HalfLength - MathF.Abs(localOffsetB.Z);
        var insideDepth = float.MinNative(depthX, float.MinNative(depthY, depthZ));
        //Only one axis may have a nonzero component.
        var useX = insideDepth == depthX;
        var useY = insideDepth == depthY & !useX;
        var useZ = !(useX | useY);
        Vector3 insideNormal;
        insideNormal.X = ScalarMath.Select(useX, ScalarMath.Select(localOffsetB.X < 0f, 1f, -1f), 0f);
        insideNormal.Y = ScalarMath.Select(useY, ScalarMath.Select(localOffsetB.Y < 0f, 1f, -1f), 0f);
        insideNormal.Z = ScalarMath.Select(useZ, ScalarMath.Select(localOffsetB.Z < 0f, 1f, -1f), 0f);

        insideDepth += a.Radius;
        var useInside = distance == 0f;
        var localNormal = ScalarMath.Select(useInside, insideNormal, outsideNormal);
        Matrix3x3.Transform(localNormal, orientationMatrixB, out manifold.Normal);
        manifold.Depth = ScalarMath.Select(useInside, insideDepth, outsideDepth);
        manifold.FeatureId = 0;

        var negativeOffsetFromSphere = manifold.Depth * 0.5f - a.Radius;
        manifold.OffsetA = manifold.Normal * negativeOffsetFromSphere;
        manifold.ContactExists = manifold.Depth > -speculativeMargin;
    }
}

/// <summary>
/// Ternary-select twin of SphereBoxScalarTester for quantifying branch-vs-branchless select cost. Bitwise identical output.
/// </summary>
public struct SphereBoxScalarTesterBranchy : ISphereVariantScalarTester<Box>
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Test(in Sphere a, in Box b, float speculativeMargin, in Vector3 offsetB, in Quaternion orientationB, out Convex1ManifoldScalar manifold)
    {
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 orientationMatrixB);
        var localOffsetB = ScalarMath.TransformByTransposed(offsetB, orientationMatrixB);
        Vector3 clampedLocalOffsetB;
        clampedLocalOffsetB.X = float.MinNative(float.MaxNative(localOffsetB.X, -b.HalfWidth), b.HalfWidth);
        clampedLocalOffsetB.Y = float.MinNative(float.MaxNative(localOffsetB.Y, -b.HalfHeight), b.HalfHeight);
        clampedLocalOffsetB.Z = float.MinNative(float.MaxNative(localOffsetB.Z, -b.HalfLength), b.HalfLength);
        var outsideNormal = clampedLocalOffsetB - localOffsetB;
        var distance = ScalarMath.Length(outsideNormal);
        var inverseDistance = 1f / distance;
        outsideNormal = outsideNormal * inverseDistance;
        var outsideDepth = a.Radius - distance;

        var depthX = b.HalfWidth - MathF.Abs(localOffsetB.X);
        var depthY = b.HalfHeight - MathF.Abs(localOffsetB.Y);
        var depthZ = b.HalfLength - MathF.Abs(localOffsetB.Z);
        var insideDepth = float.MinNative(depthX, float.MinNative(depthY, depthZ));
        var useX = insideDepth == depthX;
        var useY = insideDepth == depthY & !useX;
        var useZ = !(useX | useY);
        Vector3 insideNormal;
        insideNormal.X = useX ? localOffsetB.X < 0f ? 1f : -1f : 0f;
        insideNormal.Y = useY ? localOffsetB.Y < 0f ? 1f : -1f : 0f;
        insideNormal.Z = useZ ? localOffsetB.Z < 0f ? 1f : -1f : 0f;

        insideDepth += a.Radius;
        var useInside = distance == 0f;
        var localNormal = useInside ? insideNormal : outsideNormal;
        Matrix3x3.Transform(localNormal, orientationMatrixB, out manifold.Normal);
        manifold.Depth = useInside ? insideDepth : outsideDepth;
        manifold.FeatureId = 0;

        var negativeOffsetFromSphere = manifold.Depth * 0.5f - a.Radius;
        manifold.OffsetA = manifold.Normal * negativeOffsetFromSphere;
        manifold.ContactExists = manifold.Depth > -speculativeMargin;
    }
}

/// <summary>
/// Packed single-pair sphere-box tester: each Vector3 lives in one Vector128 register instead of three scalar registers,
/// collapsing register pressure (the source of spill-bloated codegen in the scalar version) and instruction count.
/// Every lane's operation sequence matches the scalar/wide versions exactly, so results are bitwise identical.
/// The sphere-center-inside-the-box case is handled in a rarely-taken predictable branch, mirroring the scalar expressions.
/// </summary>
public struct SphereBoxScalarTesterPacked : ISphereVariantScalarTester<Box>
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Test(in Sphere a, in Box b, float speculativeMargin, in Vector3 offsetB, in Quaternion orientationB, out Convex1ManifoldScalar manifold)
    {
        //Matrix construction uses the exact scalar expressions of ScalarMath.CreateFromQuaternion; rows are packed as they're built.
        var qX2 = orientationB.X + orientationB.X;
        var qY2 = orientationB.Y + orientationB.Y;
        var qZ2 = orientationB.Z + orientationB.Z;
        var YY = qY2 * orientationB.Y;
        var ZZ = qZ2 * orientationB.Z;
        var XY = qX2 * orientationB.Y;
        var ZW = qZ2 * orientationB.W;
        var XZ = qX2 * orientationB.Z;
        var YW = qY2 * orientationB.W;
        var XX = qX2 * orientationB.X;
        var XW = qX2 * orientationB.W;
        var YZ = qY2 * orientationB.Z;
        var row0 = Vector128.Create(1f - YY - ZZ, XY + ZW, XZ - YW, 0f);
        var row1 = Vector128.Create(XY - ZW, 1f - XX - ZZ, YZ + XW, 0f);
        var row2 = Vector128.Create(XZ + YW, YZ - XW, 1f - XX - YY, 0f);
        //Columns for the by-transposed transform via 3x3 in-register transpose.
        var unpackLow = Sse.UnpackLow(row0, row1);
        var unpackHigh = Sse.UnpackHigh(row0, row1);
        var col0 = Sse.Shuffle(unpackLow, row2, 0b11_00_01_00);
        var col1 = Sse.Shuffle(unpackLow, row2, 0b11_01_11_10);
        var col2 = Sse.Shuffle(unpackHigh, row2, 0b11_10_01_00);

        //TransformByTransposed: per lane c, (v.X*m[c][0] + v.Y*m[c][1]) + v.Z*m[c][2], matching the scalar mirror's order.
        var localOffsetB = Sse.Add(Sse.Add(
            Sse.Multiply(Vector128.Create(offsetB.X), col0),
            Sse.Multiply(Vector128.Create(offsetB.Y), col1)),
            Sse.Multiply(Vector128.Create(offsetB.Z), col2));
        var halfExtents = Vector128.Create(b.HalfWidth, b.HalfHeight, b.HalfLength, 0f);
        var negatedHalfExtents = Sse.Xor(halfExtents, Vector128.Create(-0.0f));
        //Clamp order matches scalar: Min(Max(offset, -half), half). vminps/vmaxps lane semantics equal the scalar mirrors.
        var clamped = Sse.Min(Sse.Max(localOffsetB, negatedHalfExtents), halfExtents);
        var outsideNormal = Sse.Subtract(clamped, localOffsetB);
        //Ordered dot: (x*x + y*y) + z*z.
        var squared = Sse.Multiply(outsideNormal, outsideNormal);
        var sum01 = Sse.AddScalar(squared, Sse3.MoveHighAndDuplicate(squared));
        var distanceSquared = Sse.AddScalar(sum01, Sse.Shuffle(squared, squared, 0b10_10_10_10)).ToScalar();
        var distance = MathF.Sqrt(distanceSquared);
        var inverseDistance = 1f / distance;
        var localNormal = Sse.Multiply(outsideNormal, Vector128.Create(inverseDistance));
        var depth = a.Radius - distance;

        if (distance == 0f)
        {
            //Sphere center is exactly on the box surface or inside; mirror the scalar inside-normal expressions.
            var localX = localOffsetB.GetElement(0);
            var localY = localOffsetB.GetElement(1);
            var localZ = localOffsetB.GetElement(2);
            var depthX = b.HalfWidth - MathF.Abs(localX);
            var depthY = b.HalfHeight - MathF.Abs(localY);
            var depthZ = b.HalfLength - MathF.Abs(localZ);
            var insideDepth = float.MinNative(depthX, float.MinNative(depthY, depthZ));
            var useX = insideDepth == depthX;
            var useY = insideDepth == depthY & !useX;
            var useZ = !(useX | useY);
            localNormal = Vector128.Create(
                useX ? localX < 0f ? 1f : -1f : 0f,
                useY ? localY < 0f ? 1f : -1f : 0f,
                useZ ? localZ < 0f ? 1f : -1f : 0f,
                0f);
            insideDepth += a.Radius;
            depth = insideDepth;
        }

        //Transform to world: per lane c, (n.X*m[0][c] + n.Y*m[1][c]) + n.Z*m[2][c], matching the scalar mirror's order.
        var worldNormal = Sse.Add(Sse.Add(
            Sse.Multiply(Sse.Shuffle(localNormal, localNormal, 0b00_00_00_00), row0),
            Sse.Multiply(Sse.Shuffle(localNormal, localNormal, 0b01_01_01_01), row1)),
            Sse.Multiply(Sse.Shuffle(localNormal, localNormal, 0b10_10_10_10), row2));
        manifold.Normal = worldNormal.AsVector3();
        manifold.Depth = depth;
        manifold.FeatureId = 0;
        var negativeOffsetFromSphere = depth * 0.5f - a.Radius;
        manifold.OffsetA = Sse.Multiply(worldNormal, Vector128.Create(negativeOffsetFromSphere)).AsVector3();
        manifold.ContactExists = depth > -speculativeMargin;
    }
}

/// <summary>
/// Two-pair software-pipelined variant of the branchy sphere-box tester: statements of two independent pairs are interleaved
/// so the out-of-order scheduler sees independent work at short distance instead of ~130 micro-ops away.
/// Per-pair operation sequences are unchanged, so results remain bitwise identical.
/// </summary>
public static class SphereBoxScalarTesterPipelined
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Test2(
        in SphereVariantCase<Box> pair0, in SphereVariantCase<Box> pair1,
        out Convex1ManifoldScalar manifold0, out Convex1ManifoldScalar manifold1)
    {
        Matrix3x3.CreateFromQuaternion(pair0.OrientationB, out Matrix3x3 rB0);
        Matrix3x3.CreateFromQuaternion(pair1.OrientationB, out Matrix3x3 rB1);
        var localOffsetB0 = ScalarMath.TransformByTransposed(pair0.OffsetB, rB0);
        var localOffsetB1 = ScalarMath.TransformByTransposed(pair1.OffsetB, rB1);
        Vector3 clamped0, clamped1;
        clamped0.X = float.MinNative(float.MaxNative(localOffsetB0.X, -pair0.B.HalfWidth), pair0.B.HalfWidth);
        clamped1.X = float.MinNative(float.MaxNative(localOffsetB1.X, -pair1.B.HalfWidth), pair1.B.HalfWidth);
        clamped0.Y = float.MinNative(float.MaxNative(localOffsetB0.Y, -pair0.B.HalfHeight), pair0.B.HalfHeight);
        clamped1.Y = float.MinNative(float.MaxNative(localOffsetB1.Y, -pair1.B.HalfHeight), pair1.B.HalfHeight);
        clamped0.Z = float.MinNative(float.MaxNative(localOffsetB0.Z, -pair0.B.HalfLength), pair0.B.HalfLength);
        clamped1.Z = float.MinNative(float.MaxNative(localOffsetB1.Z, -pair1.B.HalfLength), pair1.B.HalfLength);
        var outsideNormal0 = clamped0 - localOffsetB0;
        var outsideNormal1 = clamped1 - localOffsetB1;
        var distance0 = ScalarMath.Length(outsideNormal0);
        var distance1 = ScalarMath.Length(outsideNormal1);
        var inverseDistance0 = 1f / distance0;
        var inverseDistance1 = 1f / distance1;
        outsideNormal0 = outsideNormal0 * inverseDistance0;
        outsideNormal1 = outsideNormal1 * inverseDistance1;
        var outsideDepth0 = pair0.A.Radius - distance0;
        var outsideDepth1 = pair1.A.Radius - distance1;

        var depthX0 = pair0.B.HalfWidth - MathF.Abs(localOffsetB0.X);
        var depthX1 = pair1.B.HalfWidth - MathF.Abs(localOffsetB1.X);
        var depthY0 = pair0.B.HalfHeight - MathF.Abs(localOffsetB0.Y);
        var depthY1 = pair1.B.HalfHeight - MathF.Abs(localOffsetB1.Y);
        var depthZ0 = pair0.B.HalfLength - MathF.Abs(localOffsetB0.Z);
        var depthZ1 = pair1.B.HalfLength - MathF.Abs(localOffsetB1.Z);
        var insideDepth0 = float.MinNative(depthX0, float.MinNative(depthY0, depthZ0));
        var insideDepth1 = float.MinNative(depthX1, float.MinNative(depthY1, depthZ1));
        var useX0 = insideDepth0 == depthX0;
        var useX1 = insideDepth1 == depthX1;
        var useY0 = insideDepth0 == depthY0 & !useX0;
        var useY1 = insideDepth1 == depthY1 & !useX1;
        var useZ0 = !(useX0 | useY0);
        var useZ1 = !(useX1 | useY1);
        Vector3 insideNormal0, insideNormal1;
        insideNormal0.X = useX0 ? localOffsetB0.X < 0f ? 1f : -1f : 0f;
        insideNormal1.X = useX1 ? localOffsetB1.X < 0f ? 1f : -1f : 0f;
        insideNormal0.Y = useY0 ? localOffsetB0.Y < 0f ? 1f : -1f : 0f;
        insideNormal1.Y = useY1 ? localOffsetB1.Y < 0f ? 1f : -1f : 0f;
        insideNormal0.Z = useZ0 ? localOffsetB0.Z < 0f ? 1f : -1f : 0f;
        insideNormal1.Z = useZ1 ? localOffsetB1.Z < 0f ? 1f : -1f : 0f;

        insideDepth0 += pair0.A.Radius;
        insideDepth1 += pair1.A.Radius;
        var useInside0 = distance0 == 0f;
        var useInside1 = distance1 == 0f;
        var localNormal0 = useInside0 ? insideNormal0 : outsideNormal0;
        var localNormal1 = useInside1 ? insideNormal1 : outsideNormal1;
        Matrix3x3.Transform(localNormal0, rB0, out manifold0.Normal);
        Matrix3x3.Transform(localNormal1, rB1, out manifold1.Normal);
        manifold0.Depth = useInside0 ? insideDepth0 : outsideDepth0;
        manifold1.Depth = useInside1 ? insideDepth1 : outsideDepth1;
        manifold0.FeatureId = 0;
        manifold1.FeatureId = 0;

        var negativeOffsetFromSphere0 = manifold0.Depth * 0.5f - pair0.A.Radius;
        var negativeOffsetFromSphere1 = manifold1.Depth * 0.5f - pair1.A.Radius;
        manifold0.OffsetA = manifold0.Normal * negativeOffsetFromSphere0;
        manifold1.OffsetA = manifold1.Normal * negativeOffsetFromSphere1;
        manifold0.ContactExists = manifold0.Depth > -pair0.SpeculativeMargin;
        manifold1.ContactExists = manifold1.Depth > -pair1.SpeculativeMargin;
    }
}

/// <summary>Scalar AoS port of SphereTriangleTester, bitwise identical per lane.</summary>
public struct SphereTriangleScalarTester : ISphereVariantScalarTester<Triangle>
{
    static float LengthSquared(in Vector3 v) => v.X * v.X + v.Y * v.Y + v.Z * v.Z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Test(in Sphere a, in Triangle b, float speculativeMargin, in Vector3 offsetB, in Quaternion orientationB, out Convex1ManifoldScalar manifold)
    {
        Unsafe.SkipInit(out manifold);
        //Work in the local space of the triangle, since it's quicker to transform the sphere position than the vertices of the triangle.
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 rB);
        var localOffsetB = ScalarMath.TransformByTransposed(offsetB, rB);

        var ab = b.B - b.A;
        var ac = b.C - b.A;
        //localOffsetA = -localOffsetB, so pa = triangle.A + localOffsetB.
        var pa = b.A + localOffsetB;
        var localTriangleNormal = Vector3.Cross(ab, ac);
        var triangleNormalLength = ScalarMath.Length(localTriangleNormal);
        var inverseTriangleNormalLength = 1f / triangleNormalLength;
        localTriangleNormal = localTriangleNormal * inverseTriangleNormalLength;

        //Edge plane tests are scaled barycentric coordinates; see the wide implementation's commentary.
        var paxab = Vector3.Cross(pa, ab);
        var acxpa = Vector3.Cross(ac, pa);
        var edgePlaneTestAB = ScalarMath.Dot(paxab, localTriangleNormal);
        var edgePlaneTestAC = ScalarMath.Dot(acxpa, localTriangleNormal);
        var edgePlaneTestBC = 1f - (edgePlaneTestAB + edgePlaneTestAC) * inverseTriangleNormalLength;

        var outsideAB = edgePlaneTestAB < 0f;
        var outsideAC = edgePlaneTestAC < 0f;
        var outsideBC = edgePlaneTestBC < 0f;

        var outsideAnyEdge = outsideAB | outsideAC | outsideBC;
        Vector3 localClosestOnTriangle;
        if (outsideAnyEdge)
        {
            //Point is outside the triangle; clamp to the representative edge. The last edge registering an outside result is tested.
            var edgeDirectionCandidate = ScalarMath.Select(outsideAC, ac, ab);
            var edgeDirection = ScalarMath.Select(outsideBC, b.C - b.B, edgeDirectionCandidate);
            var edgeStart = ScalarMath.Select(outsideBC, b.B, b.A);

            var negativeEdgeStartToP = localOffsetB + edgeStart;
            var negativeOffsetDotEdge = ScalarMath.Dot(negativeEdgeStartToP, edgeDirection);
            var edgeDotEdge = ScalarMath.Dot(edgeDirection, edgeDirection);
            var edgeScale = float.MaxNative(0f, float.MinNative(1f, -negativeOffsetDotEdge / edgeDotEdge));
            var pointOnEdge = edgeDirection * edgeScale;
            pointOnEdge = edgeStart + pointOnEdge;

            localClosestOnTriangle = pointOnEdge;
        }
        else
        {
            //p + N * (pa * N) / ||N||^2 = N * (pa * N) / ||N||^2 - (-p)
            var paN = ScalarMath.Dot(localTriangleNormal, pa);
            var offsetToPlane = localTriangleNormal * paN;
            localClosestOnTriangle = offsetToPlane - localOffsetB;
        }

        manifold.FeatureId = outsideAnyEdge ? 0 : MeshReduction.FaceCollisionFlag;

        Matrix3x3.Transform(localClosestOnTriangle, rB, out manifold.OffsetA);
        manifold.OffsetA = manifold.OffsetA + offsetB;
        var distance = ScalarMath.Length(manifold.OffsetA);
        //Note the normal is calibrated to point from B to A.
        var normalScale = -1f / distance;
        manifold.Normal = manifold.OffsetA * normalScale;
        manifold.Depth = a.Radius - distance;
        //Zero distance is treated as a backface non-collision; see wide implementation.
        var faceNormalDotLocalNormal = ScalarMath.Dot(localTriangleNormal, manifold.Normal);
        //ComputeNondegenerateTriangleMask mirror. Note that the wide tester passes ac as the 'ca' argument; only lengths matter.
        var abLengthSquared = LengthSquared(ab);
        var caLengthSquared = LengthSquared(ac);
        var epsilonScale = MathF.Sqrt(float.MaxNative(abLengthSquared, caLengthSquared));
        var degeneracyEpsilon = TriangleWide.DegenerateTriangleEpsilon * epsilonScale;
        var nondegenerate = triangleNormalLength > degeneracyEpsilon;
        manifold.ContactExists =
            distance > 0f & nondegenerate &
            faceNormalDotLocalNormal <= -TriangleWide.BackfaceNormalDotRejectionThreshold &
            manifold.Depth >= -speculativeMargin;
    }
}

/// <summary>Scalar AoS port of SphereCylinderTester, bitwise identical per lane.</summary>
public struct SphereCylinderScalarTester : ISphereVariantScalarTester<Cylinder>
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Test(in Sphere a, in Cylinder b, float speculativeMargin, in Vector3 offsetB, in Quaternion orientationB, out Convex1ManifoldScalar manifold)
    {
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 orientationMatrixB);
        //ComputeSphereToClosest mirror: clamp the sphere position to the cylinder's volume.
        var cylinderLocalOffsetB = ScalarMath.TransformByTransposed(offsetB, orientationMatrixB);
        var cylinderLocalOffsetA = -cylinderLocalOffsetB;
        var horizontalOffsetLength = MathF.Sqrt(cylinderLocalOffsetA.X * cylinderLocalOffsetA.X + cylinderLocalOffsetA.Z * cylinderLocalOffsetA.Z);
        var inverseHorizontalOffsetLength = 1f / horizontalOffsetLength;
        var horizontalClampMultiplier = b.Radius * inverseHorizontalOffsetLength;
        var horizontalClampRequired = horizontalOffsetLength > b.Radius;
        Vector3 clampedSpherePositionLocalB;
        clampedSpherePositionLocalB.X = ScalarMath.Select(horizontalClampRequired, cylinderLocalOffsetA.X * horizontalClampMultiplier, cylinderLocalOffsetA.X);
        clampedSpherePositionLocalB.Y = float.MinNative(b.HalfLength, float.MaxNative(-b.HalfLength, cylinderLocalOffsetA.Y));
        clampedSpherePositionLocalB.Z = ScalarMath.Select(horizontalClampRequired, cylinderLocalOffsetA.Z * horizontalClampMultiplier, cylinderLocalOffsetA.Z);

        var sphereToContactLocalB = clampedSpherePositionLocalB + cylinderLocalOffsetB;
        Matrix3x3.Transform(sphereToContactLocalB, orientationMatrixB, out manifold.OffsetA);

        //If the sphere center is inside the cylinder, then we must compute the fastest way out of the cylinder.
        var absY = MathF.Abs(cylinderLocalOffsetA.Y);
        var depthY = b.HalfLength - absY;
        var horizontalDepth = b.Radius - horizontalOffsetLength;
        var useDepthY = depthY <= horizontalDepth;
        var useTopCapNormal = cylinderLocalOffsetA.Y > 0f;
        Vector3 localInternalNormal;

        var useHorizontalFallback = horizontalOffsetLength <= b.Radius * 1e-5f;
        localInternalNormal.X = ScalarMath.Select(useDepthY, 0f, ScalarMath.Select(useHorizontalFallback, 1f, cylinderLocalOffsetA.X * inverseHorizontalOffsetLength));
        localInternalNormal.Y = ScalarMath.Select(useDepthY, ScalarMath.Select(useTopCapNormal, 1f, -1f), 0f);
        localInternalNormal.Z = ScalarMath.Select(useDepthY, 0f, ScalarMath.Select(useHorizontalFallback, 0f, cylinderLocalOffsetA.Z * inverseHorizontalOffsetLength));

        var contactDistanceFromSphereCenter = ScalarMath.Length(sphereToContactLocalB);
        //Note negation; normal points from B to A by convention.
        var localExternalNormal = sphereToContactLocalB * (-1f / contactDistanceFromSphereCenter);

        //Can't rely on the external normal if the sphere is so close to the surface that the normal isn't numerically computable.
        var useInternal = contactDistanceFromSphereCenter < 1e-7f;
        var localNormal = ScalarMath.Select(useInternal, localInternalNormal, localExternalNormal);

        Matrix3x3.Transform(localNormal, orientationMatrixB, out manifold.Normal);

        manifold.FeatureId = 0;
        manifold.Depth = ScalarMath.Select(useInternal, ScalarMath.Select(useDepthY, depthY, horizontalDepth), -contactDistanceFromSphereCenter) + a.Radius;
        manifold.ContactExists = manifold.Depth >= -speculativeMargin;
    }
}

/// <summary>
/// Ternary-select twin of SphereCylinderScalarTester for quantifying branch-vs-branchless select cost. Bitwise identical output.
/// </summary>
public struct SphereCylinderScalarTesterBranchy : ISphereVariantScalarTester<Cylinder>
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Test(in Sphere a, in Cylinder b, float speculativeMargin, in Vector3 offsetB, in Quaternion orientationB, out Convex1ManifoldScalar manifold)
    {
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 orientationMatrixB);
        var cylinderLocalOffsetB = ScalarMath.TransformByTransposed(offsetB, orientationMatrixB);
        var cylinderLocalOffsetA = -cylinderLocalOffsetB;
        var horizontalOffsetLength = MathF.Sqrt(cylinderLocalOffsetA.X * cylinderLocalOffsetA.X + cylinderLocalOffsetA.Z * cylinderLocalOffsetA.Z);
        var inverseHorizontalOffsetLength = 1f / horizontalOffsetLength;
        var horizontalClampMultiplier = b.Radius * inverseHorizontalOffsetLength;
        var horizontalClampRequired = horizontalOffsetLength > b.Radius;
        Vector3 clampedSpherePositionLocalB;
        clampedSpherePositionLocalB.X = horizontalClampRequired ? cylinderLocalOffsetA.X * horizontalClampMultiplier : cylinderLocalOffsetA.X;
        clampedSpherePositionLocalB.Y = float.MinNative(b.HalfLength, float.MaxNative(-b.HalfLength, cylinderLocalOffsetA.Y));
        clampedSpherePositionLocalB.Z = horizontalClampRequired ? cylinderLocalOffsetA.Z * horizontalClampMultiplier : cylinderLocalOffsetA.Z;

        var sphereToContactLocalB = clampedSpherePositionLocalB + cylinderLocalOffsetB;
        Matrix3x3.Transform(sphereToContactLocalB, orientationMatrixB, out manifold.OffsetA);

        var absY = MathF.Abs(cylinderLocalOffsetA.Y);
        var depthY = b.HalfLength - absY;
        var horizontalDepth = b.Radius - horizontalOffsetLength;
        var useDepthY = depthY <= horizontalDepth;
        var useTopCapNormal = cylinderLocalOffsetA.Y > 0f;
        Vector3 localInternalNormal;

        var useHorizontalFallback = horizontalOffsetLength <= b.Radius * 1e-5f;
        localInternalNormal.X = useDepthY ? 0f : useHorizontalFallback ? 1f : cylinderLocalOffsetA.X * inverseHorizontalOffsetLength;
        localInternalNormal.Y = useDepthY ? useTopCapNormal ? 1f : -1f : 0f;
        localInternalNormal.Z = useDepthY ? 0f : useHorizontalFallback ? 0f : cylinderLocalOffsetA.Z * inverseHorizontalOffsetLength;

        var contactDistanceFromSphereCenter = ScalarMath.Length(sphereToContactLocalB);
        var localExternalNormal = sphereToContactLocalB * (-1f / contactDistanceFromSphereCenter);

        var useInternal = contactDistanceFromSphereCenter < 1e-7f;
        var localNormal = useInternal ? localInternalNormal : localExternalNormal;

        Matrix3x3.Transform(localNormal, orientationMatrixB, out manifold.Normal);

        manifold.FeatureId = 0;
        manifold.Depth = (useInternal ? useDepthY ? depthY : horizontalDepth : -contactDistanceFromSphereCenter) + a.Radius;
        manifold.ContactExists = manifold.Depth >= -speculativeMargin;
    }
}

/// <summary>Scalar AoS port of SphereConvexHullTester, bitwise identical per lane.</summary>
public struct SphereConvexHullScalarTester : ISphereVariantScalarTester<ConvexHull>
{
    public static void Test(in Sphere a, in ConvexHull b, float speculativeMargin, in Vector3 offsetB, in Quaternion orientationB, out Convex1ManifoldScalar manifold)
    {
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 hullOrientation);
        var localOffsetB = ScalarMath.TransformByTransposed(offsetB, hullOrientation);
        var localOffsetA = -localOffsetB;
        var identity = Matrix3x3.Identity;
        var centerDistance = ScalarMath.Length(localOffsetA);
        var initialNormal = localOffsetA * (1f / centerDistance);
        if (centerDistance < 1e-8f)
        {
            initialNormal.X = 0f;
            initialNormal.Y = 1f;
            initialNormal.Z = 0f;
        }
        //EstimateEpsilonScale mirror for the hull; sphere side is just the radius.
        Vector3Wide.ReadSlot(ref b.Points[0], 0, out var firstPoint);
        var hullEpsilonScale = (MathF.Abs(firstPoint.X) + MathF.Abs(firstPoint.Y) + MathF.Abs(firstPoint.Z)) * (1f / 3f);
        var epsilonScale = float.MinNative(a.Radius, hullEpsilonScale);
        SphereHullDepthRefiner.FindMinimumDepth(b, a, localOffsetA, identity, initialNormal, 1e-5f * epsilonScale, -speculativeMargin,
            out var depth, out var localNormal, out var closestOnHull);

        Matrix3x3.Transform(closestOnHull, hullOrientation, out var hullToContact);
        Matrix3x3.Transform(localNormal, hullOrientation, out manifold.Normal);
        manifold.OffsetA = hullToContact + offsetB;

        manifold.FeatureId = 0;
        manifold.Depth = depth;
        manifold.ContactExists = manifold.Depth >= -speculativeMargin;
    }
}
