using BepuPhysics.Collidables;
using System.Numerics;

namespace AosBaselines;

/// <summary>
/// Scalar counterpart of the (convex shape)-hull Convex4ContactManifoldWide pair tester interface.
/// Static abstract so generic fuzz/bench plumbing specializes without delegate overhead.
/// </summary>
public interface IShapeHullScalarTester<TShapeA>
{
    static abstract void Test(in TShapeA a, ref ConvexHull b, float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB, out Convex4ManifoldScalar manifold);
}

/// <summary>
/// Scalar equivalent of Convex1ContactManifoldWide for one lane.
/// </summary>
public struct Convex1ManifoldScalar
{
    public Vector3 OffsetA;
    public Vector3 Normal;
    public float Depth;
    public int FeatureId;
    public bool ContactExists;
}

/// <summary>
/// Scalar equivalent of Convex2ContactManifoldWide for one lane.
/// Fields for contacts whose Exists flag is false are undefined, matching the wide convention.
/// </summary>
public struct Convex2ManifoldScalar
{
    public Vector3 OffsetA0;
    public Vector3 OffsetA1;
    public Vector3 Normal;
    public float Depth0;
    public float Depth1;
    public int FeatureId0;
    public int FeatureId1;
    public bool Contact0Exists;
    public bool Contact1Exists;
}

/// <summary>
/// Scalar equivalent of Convex4ContactManifoldWide for one lane.
/// Fields for contacts whose Exists flag is false are undefined, matching the wide convention.
/// </summary>
public struct Convex4ManifoldScalar
{
    public Vector3 OffsetA0;
    public Vector3 OffsetA1;
    public Vector3 OffsetA2;
    public Vector3 OffsetA3;
    public Vector3 Normal;
    public float Depth0;
    public float Depth1;
    public float Depth2;
    public float Depth3;
    public int FeatureId0;
    public int FeatureId1;
    public int FeatureId2;
    public int FeatureId3;
    public bool Contact0Exists;
    public bool Contact1Exists;
    public bool Contact2Exists;
    public bool Contact3Exists;
}
