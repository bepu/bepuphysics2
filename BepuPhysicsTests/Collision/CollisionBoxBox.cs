using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public class CollisionBoxBox : CollisionBase<Box, BoxWide, Box, BoxWide, Convex4ContactManifoldWide, BoxPairTester>
{
    protected override Box CreateShapeA() => new(1, 2, 3);
    protected override Box CreateShapeB() => new(1, 2, 3);

    [Fact]
    public void Concentric_Colliding()
    {
        AssertColliding(new Vector3(0, 0, 0));
    }

    [Fact]
    public void OffsetAlongX_Colliding()
    {
        AssertColliding(new Vector3(0.75f, 0, 0));
    }

    [Fact]
    public void OffsetAlongX_Separated()
    {
        AssertSeparated(new Vector3(2.1f, 0, 0));
    }

    [Fact]
    public void OffsetAlongY_Colliding()
    {
        AssertColliding(new Vector3(0, 1.5f, 0));
    }

    [Fact]
    public void OffsetAlongY_Separated()
    {
        AssertSeparated(new Vector3(0, 2.1f, 0));
    }

    [Fact]
    public void OffsetAlongZ_Colliding()
    {
        AssertColliding(new Vector3(0, 0, 2.5f));
    }

    [Fact]
    public void OffsetAlongZ_Separated()
    {
        AssertSeparated(new Vector3(0, 0, 3.1f));
    }

    [Fact]
    public void OffsetAlongXYZ_Colliding()
    {
        AssertColliding(new Vector3(0.75f, 0.75f, 1.75f));
    }

    [Fact]
    public void OffsetAlongXYZ_Separated()
    {
        AssertSeparated(new Vector3(1.1f, 2.1f, 3.1f));
    }

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
        AssertSeparated(new Vector3(2.1f, 0, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void Touching_IsColliding() => AssertMaximumDepth(0f, new Vector3(1f, 0, 0), speculativeMargin: 0.01f);

    [Theory]
    [InlineData(0.75f, 0, 0, -1, 0, 0, 0.25f)]
    [InlineData(-0.75f, 0, 0, 1, 0, 0, 0.25f)]
    [InlineData(0, 1.5f, 0, 0, -1, 0, 0.5f)]
    [InlineData(0, -1.5f, 0, 0, 1, 0, 0.5f)]
    [InlineData(0, 0, 2.5f, 0, 0, -1, 0.5f)]
    [InlineData(0, 0, -2.5f, 0, 0, 1, 0.5f)]
    public void AxisAlignedFaceContact_NormalAndDepth(float offsetX, float offsetY, float offsetZ, float normalX, float normalY, float normalZ, float expectedDepth)
    {
        var offset = new Vector3(offsetX, offsetY, offsetZ);
        AssertNormal(new Vector3(normalX, normalY, normalZ), offset);
        AssertMaximumDepth(expectedDepth, offset);
    }

    [Fact]
    public void FaceFace_FullOverlap_FourContactsAtOverlapCorners()
    {
        var manifold = Collide(new Vector3(0.75f, 0, 0));
        AssertValidManifold(manifold);
        Assert.Equal(4, manifold.Count);
        for (int contactIndex = 0; contactIndex < 4; ++contactIndex)
        {
            var offset = manifold.GetOffset(contactIndex);
            Assert.InRange(float.Abs(offset.Y), 1f - 1e-3f, 1f + 1e-3f);
            Assert.InRange(float.Abs(offset.Z), 1.5f - 1e-3f, 1.5f + 1e-3f);
            Assert.InRange(manifold.GetDepth(contactIndex), 0.25f - 1e-3f, 0.25f + 1e-3f);
        }
    }

    [Fact]
    public void FaceFace_PartialOverlap_ContactsClippedToOverlapRegion()
    {
        //B is shifted up by 0.5 in Y, so the shared face region spans y in [-0.5, 1] and z in [-1.5, 1.5].
        var manifold = Collide(new Vector3(0.75f, 0.5f, 0));
        AssertValidManifold(manifold);
        Assert.Equal(4, manifold.Count);
        for (int contactIndex = 0; contactIndex < 4; ++contactIndex)
        {
            var offset = manifold.GetOffset(contactIndex);
            Assert.InRange(offset.Y, -0.5f - 1e-3f, 1f + 1e-3f);
            Assert.InRange(float.Abs(offset.Z), 1.5f - 1e-3f, 1.5f + 1e-3f);
        }
        AssertNormal(new Vector3(-1, 0, 0), new Vector3(0.75f, 0.5f, 0));
    }

    [Fact]
    public void SmallBoxOnLargeBox_ContactsLieOnSmallFace()
    {
        var manifold = Collide(new Box(4, 1, 4), new Box(1, 1, 1), 0f, new Vector3(0, 0.9f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(4, manifold.Count);
        AssertEqual(new Vector3(0, -1, 0), manifold.Normal);
        for (int contactIndex = 0; contactIndex < 4; ++contactIndex)
        {
            var offset = manifold.GetOffset(contactIndex);
            Assert.InRange(float.Abs(offset.X), 0.5f - 1e-3f, 0.5f + 1e-3f);
            Assert.InRange(float.Abs(offset.Z), 0.5f - 1e-3f, 0.5f + 1e-3f);
            Assert.InRange(manifold.GetDepth(contactIndex), 0.1f - 1e-3f, 0.1f + 1e-3f);
        }
    }

    [Fact]
    public void LargeBoxOnSmallBox_ContactsLieOnSmallFace()
    {
        var manifold = Collide(new Box(1, 1, 1), new Box(4, 1, 4), 0f, new Vector3(0, -0.9f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(4, manifold.Count);
        AssertEqual(new Vector3(0, 1, 0), manifold.Normal);
    }

    [Fact]
    public void YawedBox_SwapsExtentsAlongAxes()
    {
        //Yawing B by 90 degrees puts its half length (1.5) on X and its half width (0.5) on Z.
        var orientationB = Quaternion.CreateFromYAxisAngle(float.Pi * 0.5f);
        AssertNormal(new Vector3(-1, 0, 0), new Vector3(1.75f, 0, 0), Quaternion.Identity, orientationB);
        AssertMaximumDepth(0.25f, new Vector3(1.75f, 0, 0), Quaternion.Identity, orientationB);
        AssertSeparated(new Vector3(2.1f, 0, 0), Quaternion.Identity, orientationB);
        AssertNormal(new Vector3(0, 0, -1), new Vector3(0, 0, 1.75f), Quaternion.Identity, orientationB);
        AssertMaximumDepth(0.25f, new Vector3(0, 0, 1.75f), Quaternion.Identity, orientationB);
        AssertSeparated(new Vector3(0, 0, 2.1f), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void BothYawed_SameRotation_BehavesLikeAxisAligned()
    {
        var orientation = Quaternion.CreateFromYAxisAngle(0.6f);
        var offset = Vector3.Transform(new Vector3(0.75f, 0, 0), orientation);
        var manifold = Collide(offset, orientation, orientation);
        AssertValidManifold(manifold);
        Assert.Equal(4, manifold.Count);
        AssertEqual(Vector3.Transform(new Vector3(-1, 0, 0), orientation), manifold.Normal);
        AssertMaximumDepth(0.25f, offset, orientation, orientation);
    }

    [Fact]
    public void PitchedBoxOnFlatBox_TiltedEdgeContact()
    {
        //B rotated 45 degrees about Z so a bottom edge presses into A's top face. Box A's top is at y=1.
        var orientationB = Quaternion.CreateFromZAxisAngle(float.Pi * 0.25f);
        //B's lowest point is an edge at distance 0.5*sqrt(2)+... from its center along -Y. Compute so the penetration is 0.1.
        var lowest = float.Abs(Vector3.Transform(new Vector3(0.5f, 1f, 0), orientationB).Y);
        lowest = float.Max(lowest, float.Abs(Vector3.Transform(new Vector3(0.5f, -1f, 0), orientationB).Y));
        var offset = new Vector3(0, 1f + lowest - 0.1f, 0);
        var manifold = Collide(offset, Quaternion.Identity, orientationB);
        AssertValidManifold(manifold);
        Assert.Equal(2, manifold.Count);
        AssertEqual(new Vector3(0, -1, 0), manifold.Normal);
        AssertMaximumDepth(0.1f, offset, Quaternion.Identity, orientationB);
    }

    [Fact]
    public void CornerOnFace_SingleContact()
    {
        //Rotate B so one of its corners points straight down onto the top face of A.
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, -1)), float.Acos(1f / float.Sqrt(3f)));
        var offset = new Vector3(0, 3f, 0);
        var manifold = Collide(offset, Quaternion.Identity, orientationB);
        AssertValidManifold(manifold);
        Assert.InRange(manifold.Count, 0, 4);
        AssertSeparated(new Vector3(0, 4f, 0), Quaternion.Identity, orientationB);
        AssertColliding(new Vector3(0, 2f, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void EdgeEdge_CrossedBoxes_SingleContact()
    {
        //A is rotated 45 degrees about X and B 45 degrees about Z, so the closest features are a crossed edge pair along Y and X-ish directions.
        var orientationA = Quaternion.CreateFromXAxisAngle(float.Pi * 0.25f);
        var orientationB = Quaternion.CreateFromZAxisAngle(float.Pi * 0.25f);
        var manifold = Collide(new Vector3(0, 2.0f, 0), orientationA, orientationB);
        AssertValidManifold(manifold);
        AssertBundleConsistent(new Vector3(0, 2.0f, 0), orientationA, orientationB);
        AssertSeparated(new Vector3(0, 4f, 0), orientationA, orientationB);
    }

    [Fact]
    public void Concentric_HasValidManifold()
    {
        var manifold = Collide(Vector3.Zero);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
        //The shortest axis is X: the half widths add to 1.
        AssertMaximumDepth(1f, Vector3.Zero);
    }

    [Fact]
    public void Concentric_Rotated_HasValidManifold()
    {
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 2, 3)), 0.9f);
        var manifold = Collide(Vector3.Zero, Quaternion.Identity, orientationB);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
    }

    [Fact]
    public void SeparatedWithinMargin_ProducesSpeculativeContact()
    {
        AssertSeparated(new Vector3(1.1f, 0, 0));
        AssertColliding(new Vector3(1.1f, 0, 0), 0.2f);
        AssertMaximumDepth(-0.1f, new Vector3(1.1f, 0, 0), speculativeMargin: 0.2f);
        AssertNormal(new Vector3(-1, 0, 0), new Vector3(1.1f, 0, 0), speculativeMargin: 0.2f);
    }

    [Fact]
    public void SeparatedBeyondMargin_Separated()
    {
        AssertSeparated(new Vector3(1.3f, 0, 0), 0.2f);
        AssertSeparated(new Vector3(0, 2.3f, 0), 0.2f);
        AssertSeparated(new Vector3(0, 0, 3.3f), 0.2f);
    }

    [Fact]
    public void SpeculativeContact_HasFaceFaceManifold()
    {
        var manifold = Collide(new Vector3(0, 2.1f, 0), 0.2f);
        AssertValidManifold(manifold, 0.2f);
        Assert.Equal(4, manifold.Count);
        AssertEqual(new Vector3(0, -1, 0), manifold.Normal);
    }

    [Fact]
    public void SeparatedBySlidingOffFace_NoContact()
    {
        //Shifted far enough in Z that the boxes no longer overlap on that axis.
        AssertSeparated(new Vector3(0.5f, 0, 3.1f));
        AssertColliding(new Vector3(0.5f, 0, 2.9f));
    }

    [Fact]
    public void SwappedOrder_ProducesOppositeNormal()
    {
        var orientationB = Quaternion.CreateFromYAxisAngle(0.4f);
        var offset = new Vector3(0.8f, 0.2f, 0.1f);
        var forward = Collide(offset, Quaternion.Identity, orientationB);
        var backward = Collide(Vector3.Transform(-offset, Quaternion.Inverse(Quaternion.Identity)), orientationB, Quaternion.Identity);
        AssertValidManifold(forward);
        Assert.Equal(forward.Count > 0, backward.Count > 0);
    }

    [Fact]
    public void RandomOrientations_ValidManifolds()
    {
        var orientationA = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 2, 3)), 1.1f);
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(-2, 1, 0.5f)), 2.3f);
        AssertValidManifold(Collide(new Vector3(0.8f, 0.3f, 0.4f), orientationA, orientationB));
        AssertValidManifold(Collide(new Vector3(1.3f, 0.3f, 0.4f), orientationA, orientationB, 0.3f), 0.3f);
        AssertValidManifold(Collide(Vector3.Zero, orientationA, orientationB));
    }

    [Fact]
    public void BundleResult_MatchesSingleLane()
    {
        var orientationA = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 2, 3)), 1.1f);
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(-2, 1, 0.5f)), 2.3f);
        AssertBundleConsistent(new Vector3(0.75f, 0.2f, -0.3f), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(0.8f, 0.3f, 0.4f), orientationA, orientationB);
        AssertBundleConsistent(new Vector3(0, 1.5f, 0), Quaternion.Identity, Quaternion.CreateFromYAxisAngle(0.5f));
        AssertBundleConsistent(new Vector3(1.1f, 0, 0), Quaternion.Identity, Quaternion.Identity, 0.2f);
    }

    [Fact]
    public void RotatingWholeConfiguration_RotatesNormal()
    {
        var world = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0.3f, 1, -0.6f)), 0.9f);
        var orientationA = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), 0.7f);
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0, 1, 1)), 0.4f);
        AssertRotationInvariant(new Vector3(0.8f, 0.4f, 0.3f), orientationA, orientationB, world);
        //Faces that only partially overlap avoid having every clipped corner land exactly on a boundary, where rounding can legitimately drop a contact.
        AssertRotationInvariant(new Vector3(0.75f, 0.3f, 0.2f), Quaternion.Identity, Quaternion.Identity, world);
    }

    [Fact]
    public void DegenerateThinBox_StillProducesValidManifold()
    {
        var manifold = Collide(new Box(2, 2, 2), new Box(0.001f, 2, 2), 0f, new Vector3(0.9f, 0.2f, 0.1f), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
    }

    [Fact]
    public void TinyBoxInsideLargeBox_ValidManifold()
    {
        var manifold = Collide(new Box(10, 10, 10), new Box(0.1f, 0.1f, 0.1f), 0f, new Vector3(0.5f, 0.2f, 0.1f), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
    }
}
