using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public class CollisionCapsuleBox : CollisionBase<Capsule, CapsuleWide, Box, BoxWide, Convex2ContactManifoldWide, CapsuleBoxTester>
{
    protected override Capsule CreateShapeA() => new(1, 2);
    protected override Box CreateShapeB() => new(1, 2, 3);

    [Fact]
    public void Concentric_Colliding() => AssertColliding(new Vector3(0, 0, 0));

    [Fact]
    public void OffsetAlongX_Colliding() => AssertColliding(new Vector3(1.25f, 0, 0));

    [Fact]
    public void OffsetAlongX_Separated() => AssertSeparated(new Vector3(1.6f, 0, 0));

    [Fact]
    public void OffsetAlongY_Colliding() => AssertColliding(new Vector3(0, 2.5f, 0));

    [Fact]
    public void OffsetAlongY_Separated() => AssertSeparated(new Vector3(0, 3.1f, 0));

    [Fact]
    public void OffsetAlongZ_Colliding() => AssertColliding(new Vector3(0, 0, 2.25f));

    [Fact]
    public void OffsetAlongZ_Separated() => AssertSeparated(new Vector3(0, 0, 2.6f));

    [Fact]
    public void OffsetAlongXYZ_Colliding() => AssertColliding(new Vector3(0.75f, 1.5f, 1.75f));

    [Fact]
    public void OffsetAlongXYZ_Separated() => AssertSeparated(new Vector3(1.1f, 2.5f, 3.1f));

    [Fact]
    public void Rotated_Colliding()
    {
        var orientationB = Quaternion.CreateFromYAxisAngle(float.Pi * 0.25f);
        AssertColliding(new Vector3(1.5f, 0, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void Rotated_Separated()
    {
        var orientationB = Quaternion.CreateFromYAxisAngle(float.Pi * 0.25f);
        AssertSeparated(new Vector3(2.6f, 0, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void Touching_IsColliding() => AssertMaximumDepth(0f, new Vector3(1.5f, 0, 0), speculativeMargin: 0.01f);

    [Theory]
    [InlineData(1.25f, 0, 0, -1, 0, 0)]
    [InlineData(-1.25f, 0, 0, 1, 0, 0)]
    [InlineData(0, 0, 2.25f, 0, 0, -1)]
    [InlineData(0, 0, -2.25f, 0, 0, 1)]
    public void SideFace_NormalDepth_AndTwoContacts(float offsetX, float offsetY, float offsetZ, float normalX, float normalY, float normalZ)
    {
        //The capsule axis is parallel to the face and fully within its extent, so both segment endpoints generate contacts.
        var offset = new Vector3(offsetX, offsetY, offsetZ);
        var manifold = Collide(offset);
        AssertValidManifold(manifold);
        Assert.Equal(2, manifold.Count);
        AssertEqual(new Vector3(normalX, normalY, normalZ), manifold.Normal);
        AssertMaximumDepth(0.25f, offset);
    }

    [Fact]
    public void SideFace_ContactsAreAtSegmentEndpointsOnFace()
    {
        AssertHasContactAt(new Vector3(0.875f, 1, 0), new Vector3(1.25f, 0, 0));
        AssertHasContactAt(new Vector3(0.875f, -1, 0), new Vector3(1.25f, 0, 0));
    }

    [Fact]
    public void SideFace_CapsuleExtendsPastFace_ContactsAreClipped()
    {
        //Box half height is 0.25 here, so only the part of the capsule axis over the face can generate contacts.
        var manifold = Collide(new Capsule(1, 4), new Box(1, 0.5f, 3), 0f, new Vector3(1.25f, 0, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(2, manifold.Count);
        for (int contactIndex = 0; contactIndex < manifold.Count; ++contactIndex)
            //The tester expands the clip extents by a small epsilon (1e-3 of the epsilon scale), so allow for that.
            Assert.InRange(float.Abs(manifold.GetOffset(contactIndex).Y), 0f, 0.25f + 5e-3f);
    }

    [Theory]
    [InlineData(0, 2.5f, 0, 0, -1, 0, 0.5f)]
    [InlineData(0, -2.5f, 0, 0, 1, 0, 0.5f)]
    public void EndCapOnFace_SingleContact(float offsetX, float offsetY, float offsetZ, float normalX, float normalY, float normalZ, float expectedDepth)
    {
        var offset = new Vector3(offsetX, offsetY, offsetZ);
        var manifold = Collide(offset);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
        AssertEqual(new Vector3(normalX, normalY, normalZ), manifold.Normal);
        AssertMaximumDepth(expectedDepth, offset);
    }

    [Fact]
    public void CapsuleLyingOnBoxTop_TwoContactsClippedToBoxExtent()
    {
        //Capsule along X spans x in [-1, 1], box (half width 0.5) is above it. Contacts are clipped to the box face extent.
        var manifold = Collide(new Vector3(0, 1.9f, 0), CapsuleAlongX, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(2, manifold.Count);
        AssertEqual(new Vector3(0, -1, 0), manifold.Normal);
        AssertMaximumDepth(0.1f, new Vector3(0, 1.9f, 0), CapsuleAlongX, Quaternion.Identity);
        AssertHasContactAt(new Vector3(0.5f, 0.95f, 0), new Vector3(0, 1.9f, 0), CapsuleAlongX, Quaternion.Identity);
        AssertHasContactAt(new Vector3(-0.5f, 0.95f, 0), new Vector3(0, 1.9f, 0), CapsuleAlongX, Quaternion.Identity);
    }

    [Fact]
    public void CapsuleAlongZ_LyingOnBoxSide_TwoContacts()
    {
        var manifold = Collide(new Vector3(1.25f, 0, 0), CapsuleAlongZ, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(2, manifold.Count);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal);
    }

    [Fact]
    public void VerticalBoxEdge_UsesEdgeNormal()
    {
        //The vertical box edge at local (-0.5, y, -1.5) sits at (0.7, y, 0.5) relative to the capsule.
        var offset = new Vector3(1.2f, 0, 2f);
        var expectedNormal = Vector3.Normalize(new Vector3(-0.7f, 0, -0.5f));
        AssertNormal(expectedNormal, offset);
        AssertMaximumDepth(1f - float.Sqrt(0.7f * 0.7f + 0.5f * 0.5f), offset);
    }

    [Fact]
    public void CrossedEdges_SingleContactWithEdgeNormal()
    {
        //A capsule along X passing beside the box's vertical edge: nonparallel edges give one contact.
        var offset = new Vector3(0, 0, 2f);
        var manifold = Collide(offset, CapsuleAlongX, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
    }

    [Fact]
    public void CapsuleEndNearBoxCorner_NormalPointsAwayFromCorner()
    {
        //Capsule top endpoint is at (0, 1, 0). The box corner (-0.5, -1, -1.5) relative to the box center maps to the world corner below.
        var offset = new Vector3(0.8f, 2.4f, 1.9f);
        var manifold = Collide(offset);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
        var cornerToTop = new Vector3(0, 1, 0) - (offset + new Vector3(-0.5f, -1f, -1.5f));
        AssertEqual(Vector3.Normalize(cornerToTop), manifold.Normal, 1e-3f);
        AssertMaximumDepth(1f - cornerToTop.Length(), offset);
    }

    [Fact]
    public void Concentric_HasValidManifold()
    {
        var manifold = Collide(Vector3.Zero);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
        //The capsule core lies within the box, so the shortest exit is through the narrowest box axis (X): half width 0.5 plus radius 1.
        AssertMaximumDepth(1.5f, Vector3.Zero);
    }

    [Fact]
    public void CapsuleSegmentInsideBox_ExitsThroughShortestAxis()
    {
        //Offset the box so the capsule core sits just inside its +Z half.
        var offset = new Vector3(0, 0, -1f);
        var manifold = Collide(offset);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
        Assert.True(manifold.GetDepth(0) > 1f);
    }

    [Fact]
    public void Inside_NormalIsAxisAlignedForAlignedShapes()
    {
        var manifold = Collide(new Vector3(0.1f, 0.2f, 0.3f));
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
        //Half width is the shortest exit, through the face nearest the capsule center, which is -X for a box at +X.
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal);
    }

    [Fact]
    public void SeparatedWithinMargin_ProducesSpeculativeContact()
    {
        AssertSeparated(new Vector3(1.6f, 0, 0));
        AssertColliding(new Vector3(1.6f, 0, 0), 0.2f);
        AssertMaximumDepth(-0.1f, new Vector3(1.6f, 0, 0), speculativeMargin: 0.2f);
        AssertNormal(new Vector3(-1, 0, 0), new Vector3(1.6f, 0, 0), speculativeMargin: 0.2f);
    }

    [Fact]
    public void SeparatedWithinMargin_EndCap_ProducesSpeculativeContact()
    {
        AssertSeparated(new Vector3(0, 3.1f, 0));
        AssertColliding(new Vector3(0, 3.1f, 0), 0.2f);
        AssertMaximumDepth(-0.1f, new Vector3(0, 3.1f, 0), speculativeMargin: 0.2f);
        AssertNormal(new Vector3(0, -1, 0), new Vector3(0, 3.1f, 0), speculativeMargin: 0.2f);
    }

    [Fact]
    public void SeparatedBeyondMargin_Separated()
    {
        AssertSeparated(new Vector3(1.8f, 0, 0), 0.2f);
        AssertSeparated(new Vector3(0, 3.3f, 0), 0.2f);
        AssertSeparated(new Vector3(0, 0, 2.8f), 0.2f);
    }

    [Fact]
    public void SpeculativeContact_EdgeRegion()
    {
        //Gap of 0.1 to the vertical edge along the edge normal.
        var direction = Vector3.Normalize(new Vector3(0.7f, 0, 0.5f));
        var edgePoint = new Vector3(0.7f, 0, 0.5f);
        var offset = edgePoint + direction * 0.1f + new Vector3(0.5f, 0, 1.5f);
        offset = new Vector3(0, 0, 0) + offset;
        var manifold = Collide(offset, 0.2f);
        AssertValidManifold(manifold, 0.2f);
    }

    [Fact]
    public void CapsuleOrientedOppositeAxis_ProducesSameContactCount()
    {
        //A capsule is symmetric; flipping it over must not change the manifold.
        var flipped = Quaternion.CreateFromXAxisAngle(float.Pi);
        var upright = Collide(new Vector3(1.25f, 0, 0));
        var inverted = Collide(new Vector3(1.25f, 0, 0), flipped, Quaternion.Identity);
        AssertValidManifold(inverted);
        Assert.Equal(upright.Count, inverted.Count);
        AssertEqual(upright.Normal, inverted.Normal);
    }

    [Fact]
    public void RotatedBox_FaceContactUsesWorldSpaceNormal()
    {
        //A 90 degree yaw moves the half width (0.5) onto the world Z axis and the half length (1.5) onto the world X axis.
        var orientationB = Quaternion.CreateFromYAxisAngle(float.Pi * 0.5f);
        AssertNormal(new Vector3(0, 0, -1), new Vector3(0, 0, 1.25f), Quaternion.Identity, orientationB);
        AssertMaximumDepth(0.25f, new Vector3(0, 0, 1.25f), Quaternion.Identity, orientationB);
        AssertNormal(new Vector3(-1, 0, 0), new Vector3(1.75f, 0, 0), Quaternion.Identity, orientationB);
        AssertMaximumDepth(0.75f, new Vector3(1.75f, 0, 0), Quaternion.Identity, orientationB);
        AssertSeparated(new Vector3(0, 0, 1.6f), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void TiltedCapsule_AgainstFace_SingleOrTwoContactsWithFaceNormal()
    {
        //Tilting the capsule 45 degrees about Z means only one end is near the face.
        var orientationA = Quaternion.CreateFromZAxisAngle(float.Pi * 0.25f);
        var manifold = Collide(new Vector3(2.2f, 0, 0), orientationA, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal);
    }

    [Fact]
    public void BothOrientationsRandom_ValidManifold()
    {
        var orientationA = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 2, 3)), 1.1f);
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(-2, 1, 0.5f)), 2.3f);
        AssertValidManifold(Collide(new Vector3(1.2f, 0.3f, 0.4f), orientationA, orientationB));
        AssertValidManifold(Collide(new Vector3(0, 0, 0), orientationA, orientationB));
        AssertValidManifold(Collide(new Vector3(2.0f, 0.3f, 0.4f), orientationA, orientationB, 0.3f), 0.3f);
    }

    [Fact]
    public void BundleResult_MatchesSingleLane()
    {
        var orientationA = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 2, 3)), 1.1f);
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(-2, 1, 0.5f)), 2.3f);
        AssertBundleConsistent(new Vector3(1.25f, 0.2f, -0.3f), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(1.2f, 0.3f, 0.4f), orientationA, orientationB);
        AssertBundleConsistent(new Vector3(0, 2.5f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(1.6f, 0, 0), Quaternion.Identity, Quaternion.Identity, 0.2f);
    }

    [Fact]
    public void RotatingWholeConfiguration_RotatesNormal()
    {
        var world = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0.3f, 1, -0.6f)), 0.9f);
        var orientationA = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), 0.7f);
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0, 1, 1)), 0.4f);
        AssertRotationInvariant(new Vector3(1.1f, 0.4f, 0.3f), orientationA, orientationB, world);
        AssertRotationInvariant(new Vector3(1.25f, 0, 0), Quaternion.Identity, Quaternion.Identity, world);
    }

    [Fact]
    public void DifferentSizes_DepthAndNormal()
    {
        var manifold = Collide(new Capsule(0.5f, 2f), new Box(2, 2, 2), 0f, new Vector3(1.25f, 0, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(2, manifold.Count);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal);
        Assert.InRange(manifold.GetDepth(0), 0.25f - 1e-3f, 0.25f + 1e-3f);
    }

    [Fact]
    public void ZeroLengthCapsule_BehavesLikeSphere()
    {
        var manifold = Collide(new Capsule(1f, 0f), new Box(1, 2, 3), 0f, new Vector3(1.25f, 0, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal);
        Assert.InRange(manifold.GetDepth(0), 0.25f - 1e-3f, 0.25f + 1e-3f);
    }

    [Fact]
    public void DegenerateThinBox_StillProducesValidManifold()
    {
        var manifold = Collide(new Capsule(1f, 2f), new Box(0.001f, 2, 2), 0f, new Vector3(0.5f, 0.2f, 0.1f), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
    }

    private static Quaternion CapsuleAlongX => Quaternion.CreateFromZAxisAngle(float.Pi * 0.5f);
    private static Quaternion CapsuleAlongZ => Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f);
}
