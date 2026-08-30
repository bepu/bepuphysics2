using BepuPhysics.Collidables;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace AosBaselines;

/// <summary>
/// Scalar AoS port of SpherePairTester, bitwise identical per lane.
/// </summary>
public static class SpherePairScalarTester
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Test(in Sphere a, in Sphere b, float speculativeMargin, in Vector3 offsetB, out Convex1ManifoldScalar manifold)
    {
        var centerDistance = ScalarMath.Length(offsetB);
        //Note the negative 1. By convention, the normal points from B to A.
        var inverseDistance = -1f / centerDistance;
        manifold.Normal.X = offsetB.X * inverseDistance;
        manifold.Normal.Y = offsetB.Y * inverseDistance;
        manifold.Normal.Z = offsetB.Z * inverseDistance;
        var normalIsValid = centerDistance > 0f;
        //Arbitrarily choose the (0,1,0) if the two spheres are in the same position. Any unit length vector is equally valid.
        if (!normalIsValid)
        {
            manifold.Normal.X = 0f;
            manifold.Normal.Y = 1f;
            manifold.Normal.Z = 0f;
        }
        manifold.Depth = a.Radius + b.Radius - centerDistance;

        //The contact position relative to object A is computed as the average of the extreme point along the normal toward the opposing sphere on each sphere, averaged.
        var negativeOffsetFromA = manifold.Depth * 0.5f - a.Radius;
        manifold.OffsetA.X = manifold.Normal.X * negativeOffsetFromA;
        manifold.OffsetA.Y = manifold.Normal.Y * negativeOffsetFromA;
        manifold.OffsetA.Z = manifold.Normal.Z * negativeOffsetFromA;
        manifold.ContactExists = manifold.Depth > -speculativeMargin;
        manifold.FeatureId = 0;
    }
}
