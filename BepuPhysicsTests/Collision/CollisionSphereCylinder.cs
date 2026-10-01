using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public class CollisionSphereCylinder : CollisionBase<Sphere, SphereWide, Cylinder, CylinderWide, Convex1ContactManifoldWide, SphereCylinderTester>
{
    protected override Sphere CreateShapeA() => new(1f);
    protected override Cylinder CreateShapeB() => new(1, 2);

    [Fact]
    public void Concentric_Colliding() => AssertColliding(new Vector3(0, 0, 0));

    [Fact]
    public void OffsetAlongX_Colliding() => AssertColliding(new Vector3(1.5f, 0, 0));

    [Fact]
    public void OffsetAlongX_Separated() => AssertSeparated(new Vector3(2.1f, 0, 0));

    [Fact]
    public void OffsetAlongY_Colliding() => AssertColliding(new Vector3(0, 1.5f, 0));

    [Fact]
    public void OffsetAlongY_Separated() => AssertSeparated(new Vector3(0, 2.1f, 0));

    [Fact]
    public void OffsetAlongZ_Colliding() => AssertColliding(new Vector3(0, 0, 1.5f));

    [Fact]
    public void OffsetAlongZ_Separated() => AssertSeparated(new Vector3(0, 0, 2.1f));

    [Fact]
    public void OffsetAlongXYZ_Colliding() => AssertColliding(new Vector3(1.1f, 0.5f, 1.1f));

    [Fact]
    public void OffsetAlongXYZ_Separated() => AssertSeparated(new Vector3(1.5f, 1.5f, 1.5f));

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
        AssertColliding(new Vector3(0, 2f - 1e-3f, 0));
        AssertColliding(new Vector3(2f - 1e-3f, 0, 0));
    }

    [Theory]
    [InlineData(0, 1.5f, 0)]
    [InlineData(0, -1.5f, 0)]
    [InlineData(1.5f, 0, 0)]
    [InlineData(-1.5f, 0, 0)]
    [InlineData(0, 0, 1.5f)]
    [InlineData(0, 0, -1.5f)]
    public void FaceRegions_NormalDepthAndSingleContact(float offsetX, float offsetY, float offsetZ)
    {
        //Cap: sphere reaches 1, cap at 0.5 => depth 0.5. Side: same by symmetry of radius 1.
        var offset = new Vector3(offsetX, offsetY, offsetZ);
        var manifold = Collide(offset);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
        AssertNormal(-Vector3.Normalize(offset), offset);
        AssertMaximumDepth(0.5f, offset);
    }

    [Fact]
    public void Cap_OffCenter_UsesCapNormal()
    {
        var offset = new Vector3(0.3f, 1.5f, -0.2f);
        AssertNormal(new Vector3(0, -1, 0), offset);
        AssertMaximumDepth(0.5f, offset);
    }

    [Fact]
    public void Side_AtDifferentHeight_UsesRadialNormal()
    {
        var offset = new Vector3(1.5f, 0.6f, 0);
        AssertNormal(new Vector3(-1, 0, 0), offset);
        AssertMaximumDepth(0.5f, offset);
    }

    [Fact]
    public void Rim_NormalPointsAwayFromRimPoint()
    {
        //Closest point on the cylinder to the sphere center is the rim point (0.5, 0.5, 0) in world space.
        var offset = new Vector3(1.5f, 1.5f, 0);
        var expectedNormal = Vector3.Normalize(new Vector3(-1, -1, 0));
        AssertNormal(expectedNormal, offset);
        AssertMaximumDepth(1f - float.Sqrt(0.5f), offset);
        AssertSeparated(new Vector3(1.8f, 1.8f, 0));
    }

    [Fact]
    public void Rim_AlongDiagonalInXZ()
    {
        var offset = new Vector3(1.2f, 1.4f, 1.2f);
        var manifold = Collide(offset);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
        Assert.True(manifold.Normal.Y < 0);
        Assert.True(manifold.Normal.X < 0);
        Assert.True(manifold.Normal.Z < 0);
        AssertEqual(new Vector3(manifold.Normal.X, 0, manifold.Normal.Z) * 1f, new Vector3(manifold.Normal.X, 0, manifold.Normal.Z));
        Assert.InRange(float.Abs(manifold.Normal.X - manifold.Normal.Z), 0f, 1e-3f);
    }

    [Fact]
    public void SeparatedWithinMargin_ProducesSpeculativeContact()
    {
        var offset = new Vector3(0, 2.1f, 0);
        AssertSeparated(offset);
        AssertColliding(offset, 0.2f);
        AssertNormal(new Vector3(0, -1, 0), offset, 0.2f);
        AssertMaximumDepth(-0.1f, offset, 0.2f);
        var side = new Vector3(2.1f, 0, 0);
        AssertColliding(side, 0.2f);
        AssertMaximumDepth(-0.1f, side, 0.2f);
    }

    [Fact]
    public void SeparatedBeyondMargin_Separated()
    {
        AssertSeparated(new Vector3(0, 2.3f, 0), 0.2f);
        AssertSeparated(new Vector3(2.3f, 0, 0), 0.2f);
        AssertSeparated(new Vector3(2f, 2f, 0), 0.2f);
    }

    [Fact]
    public void Concentric_HasValidManifold()
    {
        var manifold = Collide(Vector3.Zero);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
    }

    [Fact]
    public void SphereCenterInsideCylinder_ValidManifold()
    {
        var cylinder = new Cylinder(2, 4);
        foreach (var offset in new[] { new Vector3(0.3f, 0, 0), new Vector3(0, 0.5f, 0), new Vector3(0, 0, -0.4f), new Vector3(0.2f, -0.3f, 0.1f) })
        {
            var manifold = Collide(new Sphere(0.5f), cylinder, 0f, offset, Quaternion.Identity, Quaternion.Identity);
            AssertValidManifold(manifold);
            Assert.Equal(1, manifold.Count);
            Assert.True(manifold.GetDepth(0) > 0.5f);
        }
    }

    [Fact]
    public void SphereCenterInsideCylinder_ExitsThroughNearestFeature()
    {
        var sphere = new Sphere(0.5f);
        var cylinder = new Cylinder(2, 4);
        //Sphere center in cylinder-local space: (0, 1.8, 0) => 0.2 from the top cap, 2 from the side. Sphere is above the cylinder center => normal points up.
        var manifold = Collide(sphere, cylinder, 0f, new Vector3(0, -1.8f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        AssertEqual(new Vector3(0, 1, 0), manifold.Normal, 1e-3f);
        Assert.InRange(manifold.GetDepth(0), 0.7f - 1e-3f, 0.7f + 1e-3f);
        //Near the side instead.
        manifold = Collide(sphere, cylinder, 0f, new Vector3(-1.8f, 0, 0), Quaternion.Identity, Quaternion.Identity);
        AssertEqual(new Vector3(1, 0, 0), manifold.Normal, 1e-3f);
        Assert.InRange(manifold.GetDepth(0), 0.7f - 1e-3f, 0.7f + 1e-3f);
    }

    [Fact]
    public void DifferentSizes_DepthAndNormal()
    {
        var sphere = new Sphere(0.5f);
        var cylinder = new Cylinder(2, 6);
        var offset = new Vector3(0, 3.3f, 0);
        var manifold = Collide(sphere, cylinder, 0f, offset, Quaternion.Identity, Quaternion.Identity);
        Assert.Equal(1, manifold.Count);
        AssertEqual(new Vector3(0, -1, 0), manifold.Normal, 1e-3f);
        Assert.InRange(manifold.GetDepth(0), 0.2f - 1e-3f, 0.2f + 1e-3f);
        manifold = Collide(sphere, cylinder, 0f, new Vector3(2.3f, 0, 0), Quaternion.Identity, Quaternion.Identity);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal, 1e-3f);
        Assert.InRange(manifold.GetDepth(0), 0.2f - 1e-3f, 0.2f + 1e-3f);
    }

    [Fact]
    public void LargeSphereSmallCylinder_ValidManifold()
    {
        var manifold = Collide(new Sphere(5), new Cylinder(0.1f, 0.2f), 0f, new Vector3(0, 4.9f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
        AssertEqual(new Vector3(0, -1, 0), manifold.Normal, 1e-3f);
    }

    [Fact]
    public void DegenerateCylinders_ValidManifold()
    {
        foreach (var cylinder in new[] { new Cylinder(1e-3f, 2), new Cylinder(1, 1e-3f), new Cylinder(1e-3f, 1e-3f) })
            AssertValidManifold(Collide(new Sphere(1), cylinder, 0f, new Vector3(0.2f, 0.9f, 0.1f), Quaternion.Identity, Quaternion.Identity));
    }

    [Fact]
    public void ZeroRadiusSphere_ValidManifold()
    {
        AssertValidManifold(Collide(new Sphere(0), new Cylinder(1, 2), 0f, new Vector3(0, 0.9f, 0), Quaternion.Identity, Quaternion.Identity));
    }

    [Fact]
    public void CylinderOnItsSide_CapAndSideSwap()
    {
        //Axis now along X.
        var orientationB = Quaternion.CreateFromZAxisAngle(float.Pi * 0.5f);
        var above = new Vector3(0, 1.5f, 0);
        AssertNormal(new Vector3(0, -1, 0), above, Quaternion.Identity, orientationB);
        AssertMaximumDepth(0.5f, above, Quaternion.Identity, orientationB);
        var alongAxis = new Vector3(1.5f, 0, 0);
        AssertNormal(new Vector3(-1, 0, 0), alongAxis, Quaternion.Identity, orientationB);
        AssertMaximumDepth(0.5f, alongAxis, Quaternion.Identity, orientationB);
        //Sphere beyond the cap region radially but within the cylinder length.
        AssertSeparated(new Vector3(0, 2.1f, 0), Quaternion.Identity, orientationB);
        AssertSeparated(new Vector3(2.1f, 0, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void CylinderFlipped_SameResult()
    {
        var flipped = Quaternion.CreateFromXAxisAngle(float.Pi);
        var offset = new Vector3(0.3f, 1.5f, 0.1f);
        var originalManifold = Collide(offset);
        var flippedManifold = Collide(offset, Quaternion.Identity, flipped);
        AssertEqual(originalManifold.Normal, flippedManifold.Normal, 1e-3f);
        Assert.Equal(originalManifold.GetDepth(0), flippedManifold.GetDepth(0), 1e-3f);
    }

    [Fact]
    public void SphereOrientation_DoesNotMatter()
    {
        var spin = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 2, 3)), 1.3f);
        var offset = new Vector3(1.2f, 1.3f, 0.2f);
        var originalManifold = Collide(offset);
        var rotatedManifold = Collide(offset, spin, Quaternion.Identity);
        AssertEqual(originalManifold.Normal, rotatedManifold.Normal, 1e-3f);
        Assert.Equal(originalManifold.GetDepth(0), rotatedManifold.GetDepth(0), 1e-3f);
    }

    [Fact]
    public void TiltedCylinder_ValidManifold()
    {
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), 0.7f);
        var manifold = Collide(new Vector3(0.5f, 1.2f, 0.3f), Quaternion.Identity, orientationB);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
    }

    [Fact]
    public void BundleResult_MatchesSingleLane()
    {
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(-2, 1, 0.5f)), 2.3f);
        AssertBundleConsistent(new Vector3(0, 1.5f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(1.5f, 1.5f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(0.3f, 0.2f, 0.1f), Quaternion.Identity, orientationB);
        AssertBundleConsistent(new Vector3(0, 2.1f, 0), Quaternion.Identity, Quaternion.Identity, 0.2f);
    }

    [Fact]
    public void RotatingWholeConfiguration_RotatesNormal()
    {
        var world = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0.3f, 1, -0.6f)), 0.9f);
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), 0.3f);
        AssertRotationInvariant(new Vector3(0.1f, 1.5f, 0.2f), Quaternion.Identity, Quaternion.Identity, world);
        AssertRotationInvariant(new Vector3(1.5f, 1.4f, 0.2f), Quaternion.Identity, orientationB, world);
    }
}
