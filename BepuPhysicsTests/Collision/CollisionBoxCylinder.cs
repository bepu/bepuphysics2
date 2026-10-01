using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public class CollisionBoxCylinder : CollisionBase<Box, BoxWide, Cylinder, CylinderWide, Convex4ContactManifoldWide, BoxCylinderTester>
{
    protected override Box CreateShapeA() => new(1, 2, 3);
    protected override Cylinder CreateShapeB() => new(1, 2);

    [Fact]
    public void Concentric_Colliding() => AssertColliding(new Vector3(0, 0, 0));

    [Fact]
    public void OffsetAlongX_Colliding() => AssertColliding(new Vector3(1.25f, 0, 0));

    [Fact]
    public void OffsetAlongX_Separated() => AssertSeparated(new Vector3(1.6f, 0, 0));

    [Fact]
    public void OffsetAlongY_Colliding() => AssertColliding(new Vector3(0, 1.5f, 0));

    [Fact]
    public void OffsetAlongY_Separated() => AssertSeparated(new Vector3(0, 2.1f, 0));

    [Fact]
    public void OffsetAlongZ_Colliding() => AssertColliding(new Vector3(0, 0, 2.25f));

    [Fact]
    public void OffsetAlongZ_Separated() => AssertSeparated(new Vector3(0, 0, 2.6f));

    [Fact]
    public void OffsetAlongXYZ_Colliding() => AssertColliding(new Vector3(0.75f, 0.75f, 1.75f));

    [Fact]
    public void OffsetAlongXYZ_Separated() => AssertSeparated(new Vector3(1.1f, 2.1f, 3.1f));

    [Fact]
    public void Rotated_Colliding()
    {
        var orientationB = Quaternion.CreateFromYAxisAngle(float.Pi * 0.25f);
        AssertColliding(new Vector3(1.25f, 0, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void Rotated_Separated()
    {
        var orientationB = Quaternion.CreateFromYAxisAngle(float.Pi * 0.25f);
        AssertSeparated(new Vector3(2.6f, 0, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void Touching_IsColliding()
    {
        AssertColliding(new Vector3(0, 2f - 1e-3f, 0));
        AssertColliding(new Vector3(1.5f - 1e-3f, 0, 0));
        AssertColliding(new Vector3(0, 0, 2.5f - 1e-3f));
    }

    [Theory]
    [InlineData(1, 0, 0)]
    [InlineData(-1, 0, 0)]
    [InlineData(0, 1, 0)]
    [InlineData(0, -1, 0)]
    [InlineData(0, 0, 1)]
    [InlineData(0, 0, -1)]
    public void AxisAlignedOverlap_NormalAndDepth(float directionX, float directionY, float directionZ)
    {
        var direction = new Vector3(directionX, directionY, directionZ);
        //Reach along each axis: X 1.5, Y 2, Z 2.5.
        var reach = new Vector3(1.5f, 2f, 2.5f);
        var distance = Vector3.Dot(reach, Vector3.Abs(direction)) - 0.25f;
        var offset = direction * distance;
        var manifold = Collide(offset);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
        AssertNormal(-direction, offset);
        AssertMaximumDepth(0.25f, offset);
    }

    [Fact]
    public void CapOnBoxFace_ManyContactsWithinFace()
    {
        var offset = new Vector3(0, 1.5f, 0);
        var manifold = Collide(offset);
        AssertValidManifold(manifold);
        Assert.InRange(manifold.Count, 2, 4);
        AssertNormal(new Vector3(0, -1, 0), offset);
        for (int contactIndex = 0; contactIndex < manifold.Count; ++contactIndex)
        {
            var contact = manifold.GetOffset(contactIndex);
            Assert.InRange(contact.X, -0.5f - 1e-3f, 0.5f + 1e-3f);
            Assert.InRange(contact.Z, -1.5f - 1e-3f, 1.5f + 1e-3f);
            Assert.InRange(manifold.GetDepth(contactIndex), -1e-3f, 0.5f + 1e-3f);
        }
    }

    [Fact]
    public void CylinderBelowBox_NormalPointsUp()
    {
        AssertNormal(new Vector3(0, 1, 0), new Vector3(0, -1.5f, 0));
        AssertMaximumDepth(0.5f, new Vector3(0, -1.5f, 0));
    }

    [Fact]
    public void SmallCylinderOnLargeBoxFace_ContactsOnCapRim()
    {
        var box = new Box(10, 2, 10);
        var cylinder = new Cylinder(0.5f, 1);
        var manifold = Collide(box, cylinder, 0f, new Vector3(0, 1.4f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.InRange(manifold.Count, 3, 4);
        AssertEqual(new Vector3(0, -1, 0), manifold.Normal);
        for (int contactIndex = 0; contactIndex < manifold.Count; ++contactIndex)
        {
            var contact = manifold.GetOffset(contactIndex);
            Assert.InRange(new Vector2(contact.X, contact.Z).Length(), 0f, 0.5f + 1e-3f);
            Assert.InRange(manifold.GetDepth(contactIndex), 0.1f - 1e-3f, 0.1f + 1e-3f);
        }
    }

    [Fact]
    public void SmallBoxOnLargeCylinderCap_ContactsAtBoxCorners()
    {
        var box = new Box(1, 1, 1);
        var cylinder = new Cylinder(5, 2);
        var manifold = Collide(box, cylinder, 0f, new Vector3(0, 1.4f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(4, manifold.Count);
        AssertEqual(new Vector3(0, -1, 0), manifold.Normal);
        for (int contactIndex = 0; contactIndex < 4; ++contactIndex)
        {
            var contact = manifold.GetOffset(contactIndex);
            Assert.InRange(float.Abs(contact.X), 0.5f - 1e-3f, 0.5f + 1e-3f);
            Assert.InRange(float.Abs(contact.Z), 0.5f - 1e-3f, 0.5f + 1e-3f);
        }
    }

    [Fact]
    public void CylinderSideAgainstBoxFace_ContactsOnFace()
    {
        //Cylinder lying on its side (axis along X) resting on the top of the box.
        var orientationB = Quaternion.CreateFromZAxisAngle(float.Pi * 0.5f);
        var offset = new Vector3(0, 1.9f, 0);
        var manifold = Collide(offset, Quaternion.Identity, orientationB);
        AssertValidManifold(manifold);
        Assert.InRange(manifold.Count, 2, 4);
        AssertNormal(new Vector3(0, -1, 0), offset, Quaternion.Identity, orientationB);
        AssertMaximumDepth(0.1f, offset, Quaternion.Identity, orientationB);
    }

    [Fact]
    public void CylinderSideAgainstBoxFace_AxisAlongZ()
    {
        var orientationB = Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f);
        var offset = new Vector3(0, 1.9f, 0);
        var manifold = Collide(offset, Quaternion.Identity, orientationB);
        AssertValidManifold(manifold);
        Assert.InRange(manifold.Count, 1, 4);
        AssertNormal(new Vector3(0, -1, 0), offset, Quaternion.Identity, orientationB);
    }

    [Fact]
    public void CylinderFlipped_SameAsUnflipped()
    {
        var flipped = Quaternion.CreateFromXAxisAngle(float.Pi);
        var offset = new Vector3(0, 1.5f, 0);
        var originalManifold = Collide(offset);
        var flippedManifold = Collide(offset, Quaternion.Identity, flipped);
        AssertValidManifold(flippedManifold);
        Assert.Equal(originalManifold.Count, flippedManifold.Count);
        AssertEqual(originalManifold.Normal, flippedManifold.Normal, 1e-3f);
    }

    [Fact]
    public void RimAgainstBoxEdge_ValidManifold()
    {
        //Cylinder tilted so its rim touches the top edge of the box.
        var orientationB = Quaternion.CreateFromZAxisAngle(float.Pi * 0.25f);
        var offset = new Vector3(0, 1.9f, 0);
        var manifold = Collide(offset, Quaternion.Identity, orientationB);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
    }

    [Fact]
    public void BoxEdgeAgainstCylinderSide_ValidManifold()
    {
        var orientationA = Quaternion.CreateFromYAxisAngle(float.Pi * 0.25f);
        var offset = new Vector3(1.4f, 0, 0);
        var manifold = Collide(offset, orientationA, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
    }

    [Fact]
    public void BoxCornerAgainstCylinderCap_ValidManifold()
    {
        var orientationA = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), 0.9f);
        var manifold = Collide(new Vector3(0, 2.0f, 0), orientationA, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
    }

    [Fact]
    public void SeparatedWithinMargin_ProducesSpeculativeContact()
    {
        var offset = new Vector3(0, 2.1f, 0);
        AssertSeparated(offset);
        AssertColliding(offset, 0.2f);
        AssertNormal(new Vector3(0, -1, 0), offset, 0.2f);
        AssertMaximumDepth(-0.1f, offset, 0.2f);
    }

    [Fact]
    public void SeparatedBeyondMargin_Separated()
    {
        AssertSeparated(new Vector3(0, 2.3f, 0), 0.2f);
        AssertSeparated(new Vector3(1.8f, 0, 0), 0.2f);
        AssertSeparated(new Vector3(0, 0, 2.8f), 0.2f);
    }

    [Fact]
    public void DiagonalNearRim_SeparatedWhenOutsideRadius()
    {
        //Box corner (0.5, *, 1.5) vs cylinder axis at offset; horizontal distance from axis to the box corner must exceed 1.
        AssertSeparated(new Vector3(1.5f, 0, 2.5f));
        AssertColliding(new Vector3(1.0f, 0, 2.0f));
    }

    [Fact]
    public void Concentric_HasValidManifold()
    {
        var manifold = Collide(Vector3.Zero);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
    }

    [Fact]
    public void CylinderInsideBox_ValidManifold()
    {
        var box = new Box(10, 10, 10);
        var cylinder = new Cylinder(0.5f, 1);
        var manifold = Collide(box, cylinder, 0f, new Vector3(0.3f, 0.2f, 0.1f), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
    }

    [Fact]
    public void BoxInsideCylinder_ValidManifold()
    {
        var box = new Box(0.5f, 0.5f, 0.5f);
        var cylinder = new Cylinder(5, 10);
        var manifold = Collide(box, cylinder, 0f, new Vector3(0.3f, 0.2f, 0.1f), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
    }

    [Fact]
    public void ThinCylinder_ValidManifold()
    {
        var cylinder = new Cylinder(1, 1e-3f);
        AssertValidManifold(Collide(new Box(1, 2, 3), cylinder, 0f, new Vector3(0, 0.99f, 0), Quaternion.Identity, Quaternion.Identity));
    }

    [Fact]
    public void NarrowCylinder_ValidManifold()
    {
        var cylinder = new Cylinder(1e-3f, 2);
        AssertValidManifold(Collide(new Box(1, 2, 3), cylinder, 0f, new Vector3(0.2f, 0.5f, 0.2f), Quaternion.Identity, Quaternion.Identity));
    }

    [Fact]
    public void ThinBox_ValidManifold()
    {
        var box = new Box(1, 1e-3f, 3);
        AssertValidManifold(Collide(box, new Cylinder(1, 2), 0f, new Vector3(0, 0.9f, 0), Quaternion.Identity, Quaternion.Identity));
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
        AssertBundleConsistent(new Vector3(0, 1.5f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(1.25f, 0, 0), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(0.3f, 0.2f, 0.1f), orientationA, orientationB);
        AssertBundleConsistent(new Vector3(0, 2.1f, 0), Quaternion.Identity, Quaternion.Identity, 0.2f);
    }

    [Fact]
    public void RotatingWholeConfiguration_RotatesNormal()
    {
        var world = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0.3f, 1, -0.6f)), 0.9f);
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), 0.3f);
        AssertRotationInvariant(new Vector3(0.1f, 1.5f, 0.2f), Quaternion.Identity, Quaternion.Identity, world);
        AssertRotationInvariant(new Vector3(1.25f, 0.1f, 0.2f), Quaternion.Identity, orientationB, world);
    }
}
