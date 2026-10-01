using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using System;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public abstract class CollisionBase<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TManifoldWide, TPairTester>
    where TShapeA : unmanaged, IShape where TShapeB : unmanaged, IShape
    where TShapeWideA : unmanaged, IShapeWide<TShapeA> where TShapeWideB : unmanaged, IShapeWide<TShapeB>
    where TPairTester : struct, IPairTester<TShapeWideA, TShapeWideB, TManifoldWide>
    where TManifoldWide : unmanaged, IContactManifoldWide
{
    protected abstract TShapeA CreateShapeA();
    protected abstract TShapeB CreateShapeB();

    protected static bool Colliding(in TShapeA shapeA, in TShapeB shapeB, float speculativeMargin, Vector3 offsetB, Quaternion orientationA, Quaternion orientationB)
    {
        return Collide(shapeA, shapeB, speculativeMargin, offsetB, orientationA, orientationB).Count > 0;
    }

    /// <summary>
    /// Runs the pair tester and returns the manifold of the first lane.
    /// </summary>
    /// <param name="pairCount">Number of lanes the tester is told are active. Every lane holds the same pair, so values above one exercise the multi-lane paths and partially filled bundles.</param>
    protected static ConvexContactManifold Collide(in TShapeA shapeA, in TShapeB shapeB, float speculativeMargin, Vector3 offsetB, Quaternion orientationA, Quaternion orientationB, int pairCount = 1)
    {
        var shapeWideA = default(TShapeWideA);
        shapeWideA.Broadcast(in shapeA);
        var shapeWideB = default(TShapeWideB);
        shapeWideB.Broadcast(in shapeB);
        var speculativeMarginWide = new Vector<float>(speculativeMargin);
        Vector3Wide.Broadcast(offsetB, out var offsetWideB);
        QuaternionWide.Broadcast(orientationA, out var orientationWideA);
        QuaternionWide.Broadcast(orientationB, out var orientationWideB);

        TPairTester.Test(ref shapeWideA, ref shapeWideB, ref speculativeMarginWide, ref offsetWideB, ref orientationWideA, ref orientationWideB, pairCount, out var manifoldWide);

        var manifold = default(ConvexContactManifold);
        manifoldWide.ReadFirst(offsetWideB, ref manifold);

        return manifold;
    }

    protected ConvexContactManifold Collide(Vector3 offsetB, Quaternion orientationA, Quaternion orientationB, float speculativeMargin = 0f, int pairCount = 1)
    {
        return Collide(CreateShapeA(), CreateShapeB(), speculativeMargin, offsetB, orientationA, orientationB, pairCount);
    }

    protected ConvexContactManifold Collide(Vector3 offsetB, float speculativeMargin = 0f)
    {
        return Collide(offsetB, Quaternion.Identity, Quaternion.Identity, speculativeMargin);
    }

    /// <summary>
    /// Checks invariants that every manifold must satisfy: contact count in range, finite values, unit normal, and no contact deeper-negative than the speculative margin.
    /// </summary>
    protected static void AssertValidManifold(in ConvexContactManifold manifold, float speculativeMargin = 0f, float tolerance = 1e-3f)
    {
        Assert.InRange(manifold.Count, 0, 4);
        if (manifold.Count == 0)
            return;
        Assert.True(IsFinite(manifold.Normal), $"Normal is not finite: {manifold.Normal}");
        Assert.True(Math.Abs(manifold.Normal.Length() - 1f) <= tolerance, $"Normal is not unit length: {manifold.Normal}");
        for (int i = 0; i < manifold.Count; ++i)
        {
            manifold.GetContact(i, out var offset, out _, out var depth, out _);
            Assert.True(IsFinite(offset), $"Contact {i} offset is not finite: {offset}");
            Assert.True(float.IsFinite(depth), $"Contact {i} depth is not finite: {depth}");
            Assert.True(depth >= -speculativeMargin - tolerance, $"Contact {i} depth {depth} is beyond the speculative margin {speculativeMargin}");
        }
    }

    public void AssertContactCount(int expectedCount, Vector3 offsetB, Quaternion orientationA, Quaternion orientationB, float speculativeMargin = 0f)
    {
        var manifold = Collide(offsetB, orientationA, orientationB, speculativeMargin);
        AssertValidManifold(manifold, speculativeMargin);
        Assert.True(manifold.Count == expectedCount, $"Expected {expectedCount} contacts, got {manifold.Count}.");
    }

    public void AssertContactCount(int expectedCount, Vector3 offsetB, float speculativeMargin = 0f)
    {
        AssertContactCount(expectedCount, offsetB, Quaternion.Identity, Quaternion.Identity, speculativeMargin);
    }

    /// <summary>
    /// Asserts the manifold is valid, has at least one contact, and that its normal matches the expected direction. The normal points from B to A.
    /// </summary>
    public void AssertNormal(Vector3 expectedNormal, Vector3 offsetB, Quaternion orientationA, Quaternion orientationB, float speculativeMargin = 0f, float tolerance = 1e-3f)
    {
        var manifold = Collide(offsetB, orientationA, orientationB, speculativeMargin);
        AssertValidManifold(manifold, speculativeMargin);
        Assert.True(manifold.Count > 0, "Expected at least one contact.");
        AssertEqual(Vector3.Normalize(expectedNormal), manifold.Normal, tolerance);
    }

    public void AssertNormal(Vector3 expectedNormal, Vector3 offsetB, float speculativeMargin = 0f, float tolerance = 1e-3f)
    {
        AssertNormal(expectedNormal, offsetB, Quaternion.Identity, Quaternion.Identity, speculativeMargin, tolerance);
    }

    /// <summary>
    /// Asserts the deepest contact has the expected depth. Positive depth means penetration; negative means speculative separation.
    /// </summary>
    public void AssertMaximumDepth(float expectedDepth, Vector3 offsetB, Quaternion orientationA, Quaternion orientationB, float speculativeMargin = 0f, float tolerance = 1e-3f)
    {
        var manifold = Collide(offsetB, orientationA, orientationB, speculativeMargin);
        AssertValidManifold(manifold, speculativeMargin);
        Assert.True(manifold.Count > 0, "Expected at least one contact.");
        var maximum = float.MinValue;
        for (int i = 0; i < manifold.Count; ++i)
            maximum = Math.Max(maximum, manifold.GetDepth(i));
        Assert.True(Math.Abs(maximum - expectedDepth) <= tolerance, $"Expected maximum depth {expectedDepth}, actual {maximum}");
    }

    public void AssertMaximumDepth(float expectedDepth, Vector3 offsetB, float speculativeMargin = 0f, float tolerance = 1e-3f)
    {
        AssertMaximumDepth(expectedDepth, offsetB, Quaternion.Identity, Quaternion.Identity, speculativeMargin, tolerance);
    }

    /// <summary>
    /// Asserts that some contact lies at the expected offset from shape A's center.
    /// </summary>
    public void AssertHasContactAt(Vector3 expectedOffsetFromA, Vector3 offsetB, Quaternion orientationA, Quaternion orientationB, float speculativeMargin = 0f, float tolerance = 1e-3f)
    {
        var manifold = Collide(offsetB, orientationA, orientationB, speculativeMargin);
        AssertValidManifold(manifold, speculativeMargin);
        for (int i = 0; i < manifold.Count; ++i)
        {
            if (Vector3.Distance(manifold.GetOffset(i), expectedOffsetFromA) <= tolerance)
                return;
        }
        Assert.Fail($"No contact near {expectedOffsetFromA}; manifold had {manifold.Count} contacts.");
    }

    public void AssertHasContactAt(Vector3 expectedOffsetFromA, Vector3 offsetB, float speculativeMargin = 0f, float tolerance = 1e-3f)
    {
        AssertHasContactAt(expectedOffsetFromA, offsetB, Quaternion.Identity, Quaternion.Identity, speculativeMargin, tolerance);
    }

    /// <summary>
    /// Asserts that the result does not depend
    /// </summary>
    public void AssertBundleConsistent(Vector3 offsetB, Quaternion orientationA, Quaternion orientationB, float speculativeMargin = 0f, float tolerance = 1e-4f)
    {
        var single = Collide(offsetB, orientationA, orientationB, speculativeMargin, 1);
        var full = Collide(offsetB, orientationA, orientationB, speculativeMargin, Vector<float>.Count);
        AssertValidManifold(full, speculativeMargin);
        Assert.Equal(single.Count, full.Count);
        if (single.Count > 0)
            AssertEqual(single.Normal, full.Normal, tolerance);
    }

    /// <summary>
    /// Asserts that the pair collides identically when translated and rotated as a whole (the relative configuration is unchanged), verifying the tester depends only on relative pose.
    /// </summary>
    public void AssertRotationInvariant(Vector3 offsetB, Quaternion orientationA, Quaternion orientationB, Quaternion worldRotation, float speculativeMargin = 0f)
    {
        var expected = Collide(offsetB, orientationA, orientationB, speculativeMargin);
        var rotated = Collide(Vector3.Transform(offsetB, worldRotation), Quaternion.Concatenate(orientationA, worldRotation), Quaternion.Concatenate(orientationB, worldRotation), speculativeMargin);
        AssertValidManifold(rotated, speculativeMargin);
        Assert.Equal(expected.Count, rotated.Count);
        if (expected.Count > 0)
            AssertEqual(Vector3.Transform(expected.Normal, worldRotation), rotated.Normal, 1e-3f);
    }

    public void AssertColliding(Vector3 offsetB, Quaternion orientationA, Quaternion orientationB, float speculativeMargin)
    {
        Assert.True(Colliding(CreateShapeA(), CreateShapeB(), speculativeMargin, offsetB, orientationA, orientationB));
    }

    public void AssertColliding(Vector3 offsetB, Quaternion orientationA, Quaternion orientationB)
    {
        AssertColliding(offsetB, orientationA, orientationB, 0f);
    }

    public void AssertColliding(Vector3 offsetB, float speculativeMargin)
    {
        AssertColliding(offsetB, Quaternion.Identity, Quaternion.Identity, speculativeMargin);
    }

    public void AssertColliding(Vector3 offsetB)
    {
        AssertColliding(offsetB, Quaternion.Identity, Quaternion.Identity, 0f);
    }

    public void AssertSeparated(Vector3 offsetB, Quaternion orientationA, Quaternion orientationB, float speculativeMargin)
    {
        Assert.False(Colliding(CreateShapeA(), CreateShapeB(), speculativeMargin, offsetB, orientationA, orientationB));
    }

    public void AssertSeparated(Vector3 offsetB, Quaternion orientationA, Quaternion orientationB)
    {
        AssertSeparated(offsetB, orientationA, orientationB, 0f);
    }

    public void AssertSeparated(Vector3 offsetB, float speculativeMargin)
    {
        AssertSeparated(offsetB, Quaternion.Identity, Quaternion.Identity, speculativeMargin);
    }

    public void AssertSeparated(Vector3 offsetB)
    {
        AssertSeparated(offsetB, Quaternion.Identity, Quaternion.Identity, 0f);
    }

    protected void AssertEqual(Vector3 expected, Vector3 actual, float tolerance = 1e-4f)
    {
        Assert.True(Vector3.Distance(expected, actual) <= tolerance, $"Expected: {expected}, Actual: {actual}");
    }

    private static bool IsFinite(Vector3 v) => float.IsFinite(v.X) && float.IsFinite(v.Y) && float.IsFinite(v.Z);
}
