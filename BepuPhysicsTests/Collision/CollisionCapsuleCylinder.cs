using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public class CollisionCapsuleCylinder : CollisionBase<Capsule, CapsuleWide, Cylinder, CylinderWide, Convex2ContactManifoldWide, CapsuleCylinderTester>
{
    protected override Capsule CreateShapeA() => new(1, 2);
    protected override Cylinder CreateShapeB() => new(1, 2);

    [Fact]
    public void Concentric_Colliding() => AssertColliding(new Vector3(0, 0, 0));

    [Fact]
    public void OffsetAlongX_Colliding() => AssertColliding(new Vector3(1.5f, 0, 0));

    [Fact]
    public void OffsetAlongX_Separated() => AssertSeparated(new Vector3(2.1f, 0, 0));

    [Fact]
    public void OffsetAlongY_Colliding() => AssertColliding(new Vector3(0, 2.5f, 0));

    [Fact]
    public void OffsetAlongY_Separated() => AssertSeparated(new Vector3(0, 3.1f, 0));

    [Fact]
    public void OffsetAlongZ_Colliding() => AssertColliding(new Vector3(0, 0, 1.5f));

    [Fact]
    public void OffsetAlongZ_Separated() => AssertSeparated(new Vector3(0, 0, 2.1f));

    [Fact]
    public void OffsetAlongXYZ_Colliding() => AssertColliding(new Vector3(1.1f, 1.1f, 1.1f));

    [Fact]
    public void OffsetAlongXYZ_Separated() => AssertSeparated(new Vector3(1.5f, 2.5f, 1.5f));

    [Fact]
    public void Rotated_Colliding()
    {
        var orientationB = Quaternion.CreateFromXAxisAngle(float.Pi * 0.25f);
        AssertColliding(new Vector3(1.5f, 0, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void Rotated_Separated()
    {
        var orientationB = Quaternion.CreateFromXAxisAngle(float.Pi * 0.25f);
        AssertSeparated(new Vector3(2.5f, 0, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void Touching_IsColliding()
    {
        //Capsule A: radius 1, half length 1, axis along local Y. Cylinder B: radius 1, half length 1, axis along local Y. Normals point from B to A.
        AssertColliding(new Vector3(2f - 1e-3f, 0, 0));
        AssertColliding(new Vector3(0, 3f - 1e-3f, 0));
    }

    [Theory]
    [InlineData(1.5f, 0, 0)]
    [InlineData(-1.5f, 0, 0)]
    [InlineData(0, 0, 1.5f)]
    [InlineData(0, 0, -1.5f)]
    [InlineData(1.06f, 0, 1.06f)]
    public void ParallelSideBySide_NormalDepthAndTwoContacts(float offsetX, float offsetY, float offsetZ)
    {
        var offset = new Vector3(offsetX, offsetY, offsetZ);
        var manifold = Collide(offset);
        AssertValidManifold(manifold);
        Assert.Equal(2, manifold.Count);
        AssertEqual(-Vector3.Normalize(offset), manifold.Normal, 1e-3f);
        AssertMaximumDepth(2f - offset.Length(), offset);
        for (int contactIndex = 0; contactIndex < 2; ++contactIndex)
            Assert.InRange(float.Abs(manifold.GetOffset(contactIndex).Y), 1f - 1e-2f, 1f + 1e-2f);
    }

    [Fact]
    public void ParallelSideBySide_ContactsAtOverlapEnds()
    {
        var offset = new Vector3(1.5f, 0, 0);
        AssertHasContactAt(new Vector3(0.5f, 1, 0), offset);
        AssertHasContactAt(new Vector3(0.5f, -1, 0), offset);
    }

    [Fact]
    public void ParallelSideBySide_PartialOverlapAlongAxis_ContactsClipped()
    {
        var offset = new Vector3(1.5f, 1, 0);
        var manifold = Collide(offset);
        AssertValidManifold(manifold);
        Assert.Equal(2, manifold.Count);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal, 1e-3f);
        AssertHasContactAt(new Vector3(0.5f, 0, 0), offset);
        AssertHasContactAt(new Vector3(0.5f, 1, 0), offset);
    }

    [Fact]
    public void CapsuleEndOnCap_NormalAlongAxis()
    {
        //The capsule extends down to y = -2, and the cylinder's upper cap is at y = -1.5, so the penetration depth is 0.5.
        var offset = new Vector3(0, -2.5f, 0);
        var manifold = Collide(offset);
        AssertValidManifold(manifold);
        Assert.InRange(manifold.Count, 1, 2);
        AssertEqual(new Vector3(0, 1, 0), manifold.Normal, 1e-3f);
        AssertMaximumDepth(0.5f, offset);
        AssertNormal(new Vector3(0, -1, 0), -offset);
    }

    [Fact]
    public void CapsuleEndOnCap_OffCenter_StillAxialNormal()
    {
        var offset = new Vector3(0.3f, -2.5f, 0.2f);
        AssertNormal(new Vector3(0, 1, 0), offset);
        AssertMaximumDepth(0.5f, offset);
    }

    [Fact]
    public void CapsuleEndPastCapRim_NormalPointsAwayFromRim()
    {
        //Capsule's lower endpoint at (1.5, 0.0+..). Rim point of cylinder top is (1,1,0) relative to cylinder when capsule endpoint sits at (1.5, 1.5, 0).
        var offset = new Vector3(-1.5f, -0.5f, 0);
        var manifold = Collide(offset);
        AssertValidManifold(manifold);
        Assert.InRange(manifold.Count, 1, 2);
        Assert.True(manifold.Normal.X > 0.5f);
        Assert.True(manifold.Normal.Y >= -1e-3f);
    }

    [Fact]
    public void CapsuleCrossingCylinderSide_Perpendicular_SingleContact()
    {
        var offset = new Vector3(0, 0, 1.5f);
        var manifold = Collide(offset, Quaternion.Identity, CapsuleOrientationAlongXAxis);
        AssertValidManifold(manifold);
        Assert.InRange(manifold.Count, 1, 2);
        AssertEqual(new Vector3(0, 0, -1), manifold.Normal, 1e-3f);
        AssertMaximumDepth(0.5f, offset, Quaternion.Identity, CapsuleOrientationAlongXAxis);
    }

    [Fact]
    public void CapsuleLyingOnCap_ParallelToCapPlane_TwoContacts()
    {
        //Cylinder axis along Y; capsule axis along X lying on the top cap.
        var offset = new Vector3(0, -1.9f, 0);
        var manifold = Collide(offset, CapsuleOrientationAlongXAxis, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(2, manifold.Count);
        AssertEqual(new Vector3(0, 1, 0), manifold.Normal, 1e-3f);
        AssertMaximumDepth(0.1f, offset, CapsuleOrientationAlongXAxis, Quaternion.Identity);
    }

    [Fact]
    public void CapsuleLyingOnCap_AxisAlongZ_TwoContacts()
    {
        var offset = new Vector3(0, -1.9f, 0);
        var manifold = Collide(offset, CapsuleOrientationAlongZAxis, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(2, manifold.Count);
        AssertEqual(new Vector3(0, 1, 0), manifold.Normal, 1e-3f);
    }

    [Fact]
    public void CapsuleLyingOnCap_ShortCapsuleInsideCapRadius_ContactsAtCapsuleEnds()
    {
        var manifold = Collide(new Capsule(0.25f, 1), new Cylinder(2, 2), 0f, new Vector3(0, -1.2f, 0), CapsuleOrientationAlongXAxis, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(2, manifold.Count);
        for (int contactIndex = 0; contactIndex < 2; ++contactIndex)
            Assert.InRange(float.Abs(manifold.GetOffset(contactIndex).X), 0.5f - 1e-2f, 0.5f + 1e-2f);
    }

    [Fact]
    public void CapsuleLyingOnCap_LongCapsuleClippedToCapRadius()
    {
        var manifold = Collide(new Capsule(0.25f, 10), new Cylinder(1, 2), 0f, new Vector3(0, -1.2f, 0), CapsuleOrientationAlongXAxis, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.InRange(manifold.Count, 1, 2);
        for (int contactIndex = 0; contactIndex < manifold.Count; ++contactIndex)
            Assert.InRange(float.Abs(manifold.GetOffset(contactIndex).X), 0f, 1f + 1e-2f);
    }

    [Fact]
    public void CapsuleAntiParallel_SameAsParallel()
    {
        var offset = new Vector3(1.5f, 0.5f, 0);
        var originalManifold = Collide(offset);
        var flippedManifold = Collide(offset, Flipped, Quaternion.Identity);
        AssertValidManifold(flippedManifold);
        Assert.Equal(originalManifold.Count, flippedManifold.Count);
        AssertEqual(originalManifold.Normal, flippedManifold.Normal, 1e-3f);
    }

    [Fact]
    public void CylinderFlipped_SameAsUnflipped()
    {
        var offset = new Vector3(1.5f, 0.5f, 0);
        var originalManifold = Collide(offset);
        var flippedManifold = Collide(offset, Quaternion.Identity, Flipped);
        AssertValidManifold(flippedManifold);
        Assert.Equal(originalManifold.Count, flippedManifold.Count);
        AssertEqual(originalManifold.Normal, flippedManifold.Normal, 1e-3f);
    }

    [Fact]
    public void TiltedCapsule_OutOfPlaneTilt_ReducesToSingleContact()
    {
        var tilt = Quaternion.CreateFromXAxisAngle(0.2f);
        var manifold = Collide(new Vector3(1.5f, 0, 0), tilt, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal, 1e-2f);
    }

    [Fact]
    public void TiltedCapsule_InPlane_ValidManifold()
    {
        var tilt = Quaternion.CreateFromZAxisAngle(0.3f);
        var manifold = Collide(new Vector3(1.6f, 0, 0), tilt, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.InRange(manifold.Count, 1, 2);
    }

    [Fact]
    public void SeparatedWithinMargin_ProducesSpeculativeContacts()
    {
        var offset = new Vector3(2.1f, 0, 0);
        AssertSeparated(offset);
        var manifold = Collide(offset, 0.2f);
        AssertValidManifold(manifold, 0.2f);
        Assert.Equal(2, manifold.Count);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal, 1e-3f);
        AssertMaximumDepth(-0.1f, offset, 0.2f);
    }

    [Fact]
    public void CapSeparatedWithinMargin_ProducesSpeculativeContact()
    {
        var offset = new Vector3(0, -3.1f, 0);
        AssertSeparated(offset);
        AssertColliding(offset, 0.2f);
        AssertNormal(new Vector3(0, 1, 0), offset, 0.2f);
        AssertMaximumDepth(-0.1f, offset, 0.2f);
    }

    [Fact]
    public void SeparatedBeyondMargin_Separated()
    {
        AssertSeparated(new Vector3(2.3f, 0, 0), 0.2f);
        AssertSeparated(new Vector3(0, 3.3f, 0), 0.2f);
        AssertSeparated(new Vector3(0, 0, 2.3f), Quaternion.Identity, CapsuleOrientationAlongXAxis, 0.2f);
    }

    [Fact]
    public void Concentric_HasValidManifold()
    {
        var manifold = Collide(Vector3.Zero);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
        AssertMaximumDepth(2f, Vector3.Zero);
    }

    [Fact]
    public void CapsuleInsideLargeCylinder_ExitsThroughNearestFeature()
    {
        var capsule = new Capsule(0.25f, 1);
        var cylinder = new Cylinder(3, 6);
        var manifold = Collide(capsule, cylinder, 0f, new Vector3(-2.5f, 0, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
        AssertEqual(new Vector3(1, 0, 0), manifold.Normal, 1e-2f);
        manifold = Collide(capsule, cylinder, 0f, new Vector3(0, -2.8f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
        AssertEqual(new Vector3(0, 1, 0), manifold.Normal, 1e-2f);
    }

    [Fact]
    public void CylinderInsideLargeCapsule_ValidManifold()
    {
        var capsule = new Capsule(5, 10);
        var cylinder = new Cylinder(0.5f, 1);
        AssertValidManifold(Collide(capsule, cylinder, 0f, new Vector3(0.2f, 0.3f, 0.1f), Quaternion.Identity, Quaternion.Identity));
    }

    [Fact]
    public void CapsuleAxisThroughCylinder_CrossingAxes_ValidManifold()
    {
        var manifold = Collide(Vector3.Zero, CapsuleOrientationAlongXAxis, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
        manifold = Collide(new Vector3(0.2f, 0.1f, 0), CapsuleOrientationAlongZAxis, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
    }

    [Fact]
    public void ZeroLengthCapsule_BehavesLikeSphere()
    {
        var sphereLike = new Capsule(1, 0);
        var cylinder = new Cylinder(1, 2);
        var manifold = Collide(sphereLike, cylinder, 0f, new Vector3(-1.5f, 0, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        AssertEqual(new Vector3(1, 0, 0), manifold.Normal, 1e-3f);
        manifold = Collide(sphereLike, cylinder, 0f, new Vector3(0, -2.5f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
    }

    [Fact]
    public void DegenerateShapes_ValidManifold()
    {
        var offset = new Vector3(1.2f, 0.3f, 0.1f);
        AssertValidManifold(Collide(new Capsule(1, 1e-4f), new Cylinder(1, 2), 0f, offset, Quaternion.Identity, Quaternion.Identity));
        AssertValidManifold(Collide(new Capsule(1, 2), new Cylinder(1e-3f, 2), 0f, offset, Quaternion.Identity, Quaternion.Identity));
        AssertValidManifold(Collide(new Capsule(1, 2), new Cylinder(1, 1e-3f), 0f, new Vector3(0, 0.9f, 0), CapsuleOrientationAlongXAxis, Quaternion.Identity));
        AssertValidManifold(Collide(new Capsule(0, 2), new Cylinder(1, 2), 0f, offset, Quaternion.Identity, Quaternion.Identity));
    }

    [Fact]
    public void DifferentSizes_DepthAndNormal()
    {
        var capsule = new Capsule(0.5f, 4);
        var cylinder = new Cylinder(2, 6);
        var manifold = Collide(capsule, cylinder, 0f, new Vector3(-2.3f, 0, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(2, manifold.Count);
        AssertEqual(new Vector3(1, 0, 0), manifold.Normal, 1e-3f);
        Assert.Equal(0.2f, manifold.GetDepth(0), 1e-3f);
    }

    [Fact]
    public void RandomOrientations_ValidManifolds()
    {
        var orientationA = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 2, 3)), 1.1f);
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(-2, 1, 0.5f)), 2.3f);
        AssertValidManifold(Collide(new Vector3(0.3f, 0.2f, 0.1f), orientationA, orientationB));
        AssertValidManifold(Collide(new Vector3(1.0f, 0.5f, 0.4f), orientationA, orientationB));
        AssertValidManifold(Collide(Vector3.Zero, orientationA, orientationB));
        AssertValidManifold(Collide(new Vector3(2.2f, 0.5f, 0.4f), orientationA, orientationB, 0.3f), 0.3f);
    }

    [Fact]
    public void BundleResult_MatchesSingleLane()
    {
        var orientationA = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 2, 3)), 1.1f);
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(-2, 1, 0.5f)), 2.3f);
        AssertBundleConsistent(new Vector3(1.5f, 0, 0), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(0, -2.5f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(0, -1.9f, 0), CapsuleOrientationAlongXAxis, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(0.3f, 0.2f, 0.1f), orientationA, orientationB);
        AssertBundleConsistent(new Vector3(2.1f, 0, 0), Quaternion.Identity, Quaternion.Identity, 0.2f);
    }

    [Fact]
    public void RotatingWholeConfiguration_RotatesNormal()
    {
        var world = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0.3f, 1, -0.6f)), 0.9f);
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), 0.4f);
        AssertRotationInvariant(new Vector3(1.5f, 0.2f, 0.1f), Quaternion.Identity, Quaternion.Identity, world);
        AssertRotationInvariant(new Vector3(0.1f, -1.9f, 0.2f), CapsuleOrientationAlongXAxis, Quaternion.Identity, world);
        AssertRotationInvariant(new Vector3(1.4f, 0.3f, 0.2f), Quaternion.Identity, orientationB, world);
    }

    private static readonly Quaternion CapsuleOrientationAlongXAxis = Quaternion.CreateFromZAxisAngle(float.Pi * 0.5f);
    private static readonly Quaternion CapsuleOrientationAlongZAxis = Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f);
    private static readonly Quaternion Flipped = Quaternion.CreateFromXAxisAngle(float.Pi);
}
