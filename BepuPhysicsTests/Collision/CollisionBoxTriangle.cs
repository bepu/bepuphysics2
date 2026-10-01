using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public class CollisionBoxTriangle : CollisionBase<Box, BoxWide, Triangle, TriangleWide, Convex4ContactManifoldWide, BoxTriangleTester>
{
    protected override Box CreateShapeA() => new(1, 2, 3);

    protected override Triangle CreateShapeB() => new(
        new Vector3(-1, 0, -1),
        new Vector3(1, 0, -1),
        new Vector3(-1, 0, 1));

    [Fact]
    public void Concentric_Colliding()
    {
        AssertColliding(new Vector3(0, 0, 0));
    }

    [Fact]
    public void OffsetAlongX_Colliding()
    {
        AssertColliding(new Vector3(0.25f, 0, 0));
    }

    [Fact]
    public void OffsetAlongY_Colliding()
    {
        AssertColliding(new Vector3(0, -0.5f, 0));
    }

    [Fact]
    public void OffsetAlongY_Separated()
    {
        AssertSeparated(new Vector3(0, 1.1f, 0));
    }

    [Fact]
    public void OffsetAlongDiagonal_Separated()
    {
        AssertSeparated(new Vector3(1.6f, 0, 2.1f));
    }

    [Fact]
    public void Rotated_Colliding()
    {
        var orientationB = Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f);
        AssertColliding(new Vector3(0, 0, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void Rotated_Separated()
    {
        var orientationB = Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f);
        AssertSeparated(new Vector3(0, 0, 2.1f), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void Touching_IsColliding() => AssertMaximumDepth(0f, new Vector3(0, -1f, 0), speculativeMargin: 0.01f);

    [Fact]
    public void FrontFace_NormalDepthAndClippedContacts()
    {
        //The triangle lies 0.1 inside the bottom of the box. Its overlap with the box bottom face is a quad.
        var offset = new Vector3(0, -0.9f, 0);
        var manifold = Collide(offset);
        AssertValidManifold(manifold);
        Assert.InRange(manifold.Count, 1, 4);
        AssertEqual(new Vector3(0, 1, 0), manifold.Normal);
        AssertMaximumDepth(0.1f, offset);
        for (int contactIndex = 0; contactIndex < manifold.Count; ++contactIndex)
        {
            var contact = manifold.GetOffset(contactIndex);
            Assert.InRange(contact.X, -0.5f - 1e-3f, 0.5f + 1e-3f);
            Assert.InRange(contact.Z, -1f - 1e-3f, 1f + 1e-3f);
        }
    }

    [Fact]
    public void FrontFace_DeepPenetration_UsesTriangleNormal()
    {
        AssertNormal(new Vector3(0, 1, 0), new Vector3(0, -0.5f, 0));
        AssertMaximumDepth(0.5f, new Vector3(0, -0.5f, 0));
    }

    [Fact]
    public void BoxCenterBehindTriangle_NoContacts()
    {
        //The triangle plane is above the box center, so the box is on the back side even though it penetrates.
        AssertSeparated(new Vector3(0, 0.9f, 0));
        AssertSeparated(new Vector3(0, 0.5f, 0));
        AssertSeparated(new Vector3(0, 0.9f, 0), 0.2f);
    }

    [Fact]
    public void BackfaceWithinSpeculativeMargin_NoContacts()
    {
        //Gap of 0.1 on the back side; speculative contacts must not be generated for the backface either.
        AssertSeparated(new Vector3(0, 1.1f, 0), 0.2f);
    }

    [Fact]
    public void FlippedTriangle_SwapsFrontAndBack()
    {
        //Rotating by 180 degrees about X turns the triangle normal toward -Y.
        AssertSeparated(new Vector3(0, -0.9f, 0), Quaternion.Identity, FlipAboutXAxis);
        AssertColliding(new Vector3(0, 0.9f, 0), Quaternion.Identity, FlipAboutXAxis);
        AssertNormal(new Vector3(0, -1, 0), new Vector3(0, 0.9f, 0), Quaternion.Identity, FlipAboutXAxis);
        AssertMaximumDepth(0.1f, new Vector3(0, 0.9f, 0), Quaternion.Identity, FlipAboutXAxis);
    }

    [Fact]
    public void ReversedWinding_SwapsFrontAndBack()
    {
        var reversed = new Triangle(new Vector3(-1, 0, -1), new Vector3(-1, 0, 1), new Vector3(1, 0, -1));
        Assert.False(Collide(new Box(1, 2, 3), reversed, 0f, new Vector3(0, -0.9f, 0), Quaternion.Identity, Quaternion.Identity).Count > 0);
        var manifold = Collide(new Box(1, 2, 3), reversed, 0f, new Vector3(0, 0.9f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count > 0);
        AssertEqual(new Vector3(0, -1, 0), manifold.Normal);
    }

    [Fact]
    public void StandingTriangle_FrontAndBackAlongZ()
    {
        //Rotating about X by 90 degrees stands the triangle up in the XY plane with its front facing +Z.
        var inFront = new Vector3(0, 0, -1.4f);
        AssertNormal(new Vector3(0, 0, 1), inFront, Quaternion.Identity, StandUpAboutXAxis);
        AssertMaximumDepth(0.1f, inFront, Quaternion.Identity, StandUpAboutXAxis);
        AssertSeparated(new Vector3(0, 0, 1.4f), Quaternion.Identity, StandUpAboutXAxis);
        AssertSeparated(new Vector3(0, 0, 1.6f), Quaternion.Identity, StandUpAboutXAxis, 0.05f);
    }

    [Theory]
    [InlineData(1.6f, 0, 0)]
    [InlineData(-1.6f, 0, 0)]
    [InlineData(0, 1.1f, 0)]
    [InlineData(0, 0, 2.6f)]
    [InlineData(0, 0, -2.6f)]
    [InlineData(1.6f, 0, 2.6f)]
    [InlineData(-1.6f, 1.1f, -2.6f)]
    public void OutsideBoxExtents_Separated(float offsetX, float offsetY, float offsetZ) => AssertSeparated(new Vector3(offsetX, offsetY, offsetZ));

    [Fact]
    public void CornerOfTriangleOverlappingBoxEdge_Colliding()
    {
        //Triangle x extent is [0.4, 2.4], so only a thin sliver overlaps the box (x <= 0.5).
        AssertColliding(new Vector3(1.4f, -0.5f, 0));
        AssertSeparated(new Vector3(1.6f, -0.5f, 0));
    }

    [Fact]
    public void TriangleHypotenuse_DefinesOverlapBoundary()
    {
        //The hypotenuse runs from (1,-1) to (-1,1) in XZ, i.e. x + z = 0. Shifting the triangle by +x moves the line to x + z = 1.5.
        AssertColliding(new Vector3(0.2f, -0.5f, 0));
        //Far past the box along +Z, only the line x + z = offset remains; the box corner (-0.5, 1.5) has x + z = 1.0.
        AssertSeparated(new Vector3(-1.6f, -0.5f, 0.5f));
    }

    [Fact]
    public void LargeTriangle_FourContactsAtBoxFaceCorners()
    {
        var large = new Triangle(new Vector3(-10, 0, -10), new Vector3(30, 0, -10), new Vector3(-10, 0, 30));
        var manifold = Collide(new Box(1, 2, 3), large, 0f, new Vector3(0, -0.9f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(4, manifold.Count);
        AssertEqual(new Vector3(0, 1, 0), manifold.Normal);
        for (int contactIndex = 0; contactIndex < 4; ++contactIndex)
        {
            var contact = manifold.GetOffset(contactIndex);
            Assert.InRange(float.Abs(contact.X), 0.5f - 1e-3f, 0.5f + 1e-3f);
            Assert.InRange(float.Abs(contact.Z), 1.5f - 1e-3f, 1.5f + 1e-3f);
            Assert.InRange(manifold.GetDepth(contactIndex), 0.1f - 1e-3f, 0.1f + 1e-3f);
        }
    }

    [Fact]
    public void LargeTriangle_BackSide_NoContacts()
    {
        var large = new Triangle(new Vector3(-10, 0, -10), new Vector3(30, 0, -10), new Vector3(-10, 0, 30));
        Assert.Equal(0, Collide(new Box(1, 2, 3), large, 0f, new Vector3(0, 0.9f, 0), Quaternion.Identity, Quaternion.Identity).Count);
    }

    [Fact]
    public void SmallTriangleInsideBoxFace_ContactsAtTriangleVertices()
    {
        var smallerTriangle = new Triangle(new Vector3(-0.2f, 0, -0.2f), new Vector3(0.2f, 0, -0.2f), new Vector3(-0.2f, 0, 0.2f));
        var manifold = Collide(new Box(1, 2, 3), smallerTriangle, 0f, new Vector3(0, -0.9f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(3, manifold.Count);
        AssertEqual(new Vector3(0, 1, 0), manifold.Normal);
        for (int contactIndex = 0; contactIndex < manifold.Count; ++contactIndex)
        {
            var contact = manifold.GetOffset(contactIndex);
            Assert.InRange(float.Abs(contact.X), 0.2f - 1e-3f, 0.2f + 1e-3f);
            Assert.InRange(float.Abs(contact.Z), 0.2f - 1e-3f, 0.2f + 1e-3f);
        }
    }

    [Fact]
    public void SeparatedWithinMargin_ProducesSpeculativeContact()
    {
        AssertSeparated(new Vector3(0, -1.1f, 0));
        AssertColliding(new Vector3(0, -1.1f, 0), 0.2f);
        AssertMaximumDepth(-0.1f, new Vector3(0, -1.1f, 0), speculativeMargin: 0.2f);
        AssertNormal(new Vector3(0, 1, 0), new Vector3(0, -1.1f, 0), speculativeMargin: 0.2f);
    }

    [Fact]
    public void SeparatedBeyondMargin_Separated()
    {
        AssertSeparated(new Vector3(0, -1.3f, 0), 0.2f);
        AssertSeparated(new Vector3(1.8f, 0, 0), 0.2f);
        AssertSeparated(new Vector3(0, 0, 2.8f), 0.2f);
    }

    [Fact]
    public void RotatedBox_FrontFaceStillUsesTriangleNormal()
    {
        var orientationA = Quaternion.CreateFromYAxisAngle(float.Pi * 0.25f);
        var offset = new Vector3(0, -0.9f, 0);
        var manifold = Collide(offset, orientationA, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
        AssertEqual(new Vector3(0, 1, 0), manifold.Normal);
        AssertMaximumDepth(0.1f, offset, orientationA, Quaternion.Identity);
        AssertSeparated(new Vector3(0, 0.9f, 0), orientationA, Quaternion.Identity);
    }

    [Fact]
    public void TiltedTriangle_ProducesValidManifold()
    {
        var orientationB = Quaternion.CreateFromZAxisAngle(float.Pi * 0.25f);
        var manifold = Collide(new Vector3(0, -1.2f, 0), Quaternion.Identity, orientationB);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
    }

    [Fact]
    public void TriangleEdgePressingIntoBoxEdge_ProducesValidManifold()
    {
        //Rotate the box so an edge faces the triangle's hypotenuse.
        var orientationA = Quaternion.CreateFromZAxisAngle(float.Pi * 0.25f);
        var manifold = Collide(new Vector3(0, -1.0f, 0), orientationA, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
        AssertNormal(new Vector3(0, 1, 0), new Vector3(0, -1.0f, 0), orientationA, Quaternion.Identity);
    }

    [Fact]
    public void Concentric_HasValidManifold()
    {
        var manifold = Collide(Vector3.Zero);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
    }

    [Fact]
    public void DegenerateTriangle_ProducesNoContacts()
    {
        var box = new Box(1, 2, 3);
        var collinear = new Triangle(new Vector3(-1, 0, 0), new Vector3(0, 0, 0), new Vector3(1, 0, 0));
        var point = new Triangle(Vector3.Zero, Vector3.Zero, Vector3.Zero);
        var twoCoincident = new Triangle(new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(1, 0, 1));
        foreach (var triangle in new[] { collinear, point, twoCoincident })
        {
            var manifold = Collide(box, triangle, 0f, new Vector3(0, -0.5f, 0), Quaternion.Identity, Quaternion.Identity);
            AssertValidManifold(manifold);
            Assert.Equal(0, manifold.Count);
        }
    }

    [Fact]
    public void TinyTriangle_ProducesValidManifold()
    {
        var tiny = new Triangle(new Vector3(-1e-3f, 0, -1e-3f), new Vector3(1e-3f, 0, -1e-3f), new Vector3(-1e-3f, 0, 1e-3f));
        AssertValidManifold(Collide(new Box(1, 2, 3), tiny, 0f, new Vector3(0, -0.9f, 0), Quaternion.Identity, Quaternion.Identity));
    }

    [Fact]
    public void SliverTriangle_ProducesValidManifold()
    {
        var sliver = new Triangle(new Vector3(-1, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 0, 1e-3f));
        AssertValidManifold(Collide(new Box(1, 2, 3), sliver, 0f, new Vector3(0, -0.9f, 0), Quaternion.Identity, Quaternion.Identity));
    }

    [Fact]
    public void RandomOrientations_ValidManifolds()
    {
        var orientationA = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 2, 3)), 1.1f);
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(-2, 1, 0.5f)), 2.3f);
        AssertValidManifold(Collide(new Vector3(0.3f, 0.2f, 0.1f), orientationA, orientationB));
        AssertValidManifold(Collide(new Vector3(1.0f, 0.5f, 0.4f), orientationA, orientationB, 0.3f), 0.3f);
        AssertValidManifold(Collide(Vector3.Zero, orientationA, orientationB));
    }

    [Fact]
    public void BundleResult_MatchesSingleLane()
    {
        var orientationA = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 2, 3)), 1.1f);
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(-2, 1, 0.5f)), 2.3f);
        AssertBundleConsistent(new Vector3(0, -0.9f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(0, 0.9f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(0.3f, 0.2f, 0.1f), orientationA, orientationB);
        AssertBundleConsistent(new Vector3(0, -1.1f, 0), Quaternion.Identity, Quaternion.Identity, 0.2f);
    }

    [Fact]
    public void RotatingWholeConfiguration_RotatesNormal()
    {
        var world = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0.3f, 1, -0.6f)), 0.9f);
        var orientationA = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), 0.3f);
        AssertRotationInvariant(new Vector3(0.1f, -0.9f, 0.2f), Quaternion.Identity, Quaternion.Identity, world);
        AssertRotationInvariant(new Vector3(0.1f, -0.9f, 0.2f), orientationA, Quaternion.Identity, world);
        AssertRotationInvariant(new Vector3(0.1f, 0.9f, 0.2f), Quaternion.Identity, Quaternion.Identity, world);
    }

    private static readonly Quaternion FlipAboutXAxis = Quaternion.CreateFromXAxisAngle(float.Pi);
    private static readonly Quaternion StandUpAboutXAxis = Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f);
}
