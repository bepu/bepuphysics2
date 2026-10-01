using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public class CollisionCapsuleCapsule : CollisionBase<Capsule, CapsuleWide, Capsule, CapsuleWide, Convex2ContactManifoldWide, CapsulePairTester>
{
    protected override Capsule CreateShapeA() => new(1, 2);
    protected override Capsule CreateShapeB() => new(1, 2);

    [Fact]
    public void Concentric_Colliding()
    {
        AssertColliding(new Vector3(0, 0, 0));
    }

    [Fact]
    public void OffsetAlongX_Colliding()
    {
        AssertColliding(new Vector3(1.5f, 0, 0));
    }

    [Fact]
    public void OffsetAlongX_Separated()
    {
        AssertSeparated(new Vector3(2.1f, 0, 0));
    }

    [Fact]
    public void OffsetAlongY_Colliding()
    {
        AssertColliding(new Vector3(0, 3.5f, 0));
    }

    [Fact]
    public void OffsetAlongY_Separated()
    {
        AssertSeparated(new Vector3(0, 4.1f, 0));
    }

    [Fact]
    public void OffsetAlongZ_Colliding()
    {
        AssertColliding(new Vector3(0, 0, 1.5f));
    }

    [Fact]
    public void OffsetAlongZ_Separated()
    {
        AssertSeparated(new Vector3(0, 0, 2.1f));
    }

    [Fact]
    public void OffsetAlongXYZ_Colliding()
    {
        AssertColliding(new Vector3(1.1f, 1.1f, 1.1f));
    }

    [Fact]
    public void OffsetAlongXYZ_Separated()
    {
        AssertSeparated(new Vector3(1.5f, 2.5f, 1.5f));
    }

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
        AssertColliding(new Vector3(2f - 1e-3f, 0, 0));
        AssertColliding(new Vector3(0, 4f - 1e-3f, 0));
    }

    [Fact]
    public void Parallel_SideBySide_TwoContactsAtSegmentEnds()
    {
        var offset = new Vector3(1.5f, 0, 0);
        var manifold = Collide(offset);
        AssertValidManifold(manifold);
        Assert.Equal(2, manifold.Count);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal);
        AssertMaximumDepth(0.5f, offset);
        AssertHasContactAt(new Vector3(0.75f, 1, 0), offset);
        AssertHasContactAt(new Vector3(0.75f, -1, 0), offset);
        for (int contactIndex = 0; contactIndex < 2; ++contactIndex)
            Assert.Equal(0.5f, manifold.GetDepth(contactIndex), 1e-3f);
    }

    [Theory]
    [InlineData(1.5f, 0, 0)]
    [InlineData(-1.5f, 0, 0)]
    [InlineData(0, 0, 1.5f)]
    [InlineData(0, 0, -1.5f)]
    [InlineData(1.06f, 0, 1.06f)]
    public void Parallel_AnySideDirection_TwoContactsAndNormalPointsToA(float offsetX, float offsetY, float offsetZ)
    {
        var offset = new Vector3(offsetX, offsetY, offsetZ);
        var manifold = Collide(offset);
        AssertValidManifold(manifold);
        Assert.Equal(2, manifold.Count);
        AssertEqual(-Vector3.Normalize(offset), manifold.Normal, 1e-3f);
        AssertMaximumDepth(2f - offset.Length(), offset);
    }

    [Fact]
    public void Parallel_PartialOverlap_ContactsClippedToSharedInterval()
    {
        var offset = new Vector3(1.5f, 1, 0);
        var manifold = Collide(offset);
        AssertValidManifold(manifold);
        Assert.Equal(2, manifold.Count);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal);
        AssertHasContactAt(new Vector3(0.75f, 0, 0), offset);
        AssertHasContactAt(new Vector3(0.75f, 1, 0), offset);
    }

    [Fact]
    public void Parallel_EndsJustTouchingInInterval_SingleContact()
    {
        var offset = new Vector3(1.5f, 2, 0);
        var manifold = Collide(offset);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal);
        AssertMaximumDepth(0.5f, offset);
        AssertHasContactAt(new Vector3(0.75f, 1, 0), offset);
    }

    [Fact]
    public void Parallel_EndToEnd_SingleContactAndAxialNormal()
    {
        var offset = new Vector3(0, 3.5f, 0);
        var manifold = Collide(offset);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
        AssertEqual(new Vector3(0, -1, 0), manifold.Normal);
        AssertMaximumDepth(0.5f, offset);
        AssertHasContactAt(new Vector3(0, 1.75f, 0), offset);
        AssertNormal(new Vector3(0, 1, 0), -offset);
    }

    [Fact]
    public void Parallel_OffsetBeyondEndDiagonal_NormalFromEndpoints()
    {
        //A end (0,1,0), B end (1.5,2.5,0).
        var offset = new Vector3(1.5f, 3.5f, 0);
        AssertSeparated(offset);
        var manifold = Collide(offset, 0.7f);
        AssertValidManifold(manifold, 0.7f);
        Assert.Equal(1, manifold.Count);
        AssertEqual(Vector3.Normalize(new Vector3(-1, -1, 0)), manifold.Normal, 1e-3f);
        AssertMaximumDepth(2f - float.Sqrt(4.5f), offset, 0.7f);
    }

    [Fact]
    public void AntiParallel_SameAsParallel()
    {
        var offset = new Vector3(1.5f, 0.5f, 0);
        var originalManifold = Collide(offset);
        var flippedManifold = Collide(offset, Quaternion.Identity, Flipped);
        AssertValidManifold(flippedManifold);
        Assert.Equal(originalManifold.Count, flippedManifold.Count);
        AssertEqual(originalManifold.Normal, flippedManifold.Normal, 1e-3f);
        Assert.Equal(originalManifold.GetDepth(0), flippedManifold.GetDepth(0), 1e-3f);
    }

    [Fact]
    public void Perpendicular_Skew_SingleContactAtSegmentClosestPoints()
    {
        var offset = new Vector3(0, 0, 1.5f);
        var manifold = Collide(offset, Quaternion.Identity, CapsuleOrientationAlongXAxis);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
        AssertEqual(new Vector3(0, 0, -1), manifold.Normal);
        AssertMaximumDepth(0.5f, offset, Quaternion.Identity, CapsuleOrientationAlongXAxis);
        AssertHasContactAt(new Vector3(0, 0, 0.75f), offset, Quaternion.Identity, CapsuleOrientationAlongXAxis);
    }

    [Fact]
    public void Perpendicular_Skew_ContactFollowsClosestPoint()
    {
        //B runs along X at y=0.5, z=1.5; closest point on A is (0,0.5,0).
        var offset = new Vector3(0.3f, 0.5f, 1.5f);
        var manifold = Collide(offset, Quaternion.Identity, CapsuleOrientationAlongXAxis);
        Assert.Equal(1, manifold.Count);
        AssertEqual(new Vector3(0, 0, -1), manifold.Normal);
        AssertHasContactAt(new Vector3(0, 0.5f, 0.75f), offset, Quaternion.Identity, CapsuleOrientationAlongXAxis);
    }

    [Fact]
    public void Perpendicular_TBoneIntoSide_Colliding()
    {
        AssertColliding(new Vector3(0, 0.9f, 1.9f), Quaternion.Identity, CapsuleOrientationAlongXAxis);
        AssertSeparated(new Vector3(0, 0.9f, 2.1f), Quaternion.Identity, CapsuleOrientationAlongXAxis);
    }

    [Fact]
    public void Perpendicular_EndOfBBeyondEndOfA_ClampedToEndpoints()
    {
        //B along X offset so its end region is nearest A's end point (0,1,0).
        var offset = new Vector3(2.5f, 1.5f, 0);
        AssertColliding(offset, Quaternion.Identity, CapsuleOrientationAlongXAxis);
        var manifold = Collide(offset, Quaternion.Identity, CapsuleOrientationAlongXAxis);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
    }

    [Fact]
    public void BothAlongX_SideBySideAlongY_TwoContacts()
    {
        var offset = new Vector3(0, 1.5f, 0);
        var manifold = Collide(offset, CapsuleOrientationAlongXAxis, CapsuleOrientationAlongXAxis);
        AssertValidManifold(manifold);
        Assert.Equal(2, manifold.Count);
        AssertEqual(new Vector3(0, -1, 0), manifold.Normal);
        AssertMaximumDepth(0.5f, offset, CapsuleOrientationAlongXAxis, CapsuleOrientationAlongXAxis);
    }

    [Fact]
    public void BothAlongZ_SideBySideAlongX_TwoContacts()
    {
        var offset = new Vector3(1.5f, 0, 0);
        var manifold = Collide(offset, CapsuleOrientationAlongZAxis, CapsuleOrientationAlongZAxis);
        AssertValidManifold(manifold);
        Assert.Equal(2, manifold.Count);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal);
    }

    [Fact]
    public void SlightTilt_InCoplanarPlane_KeepsTwoContacts()
    {
        var tilt = Quaternion.CreateFromZAxisAngle(0.005f);
        var manifold = Collide(new Vector3(1.5f, 0, 0), Quaternion.Identity, tilt);
        AssertValidManifold(manifold);
        Assert.Equal(2, manifold.Count);
    }

    [Fact]
    public void LargeTiltOutOfPlane_ReducesToSingleContact()
    {
        var tilt = Quaternion.CreateFromXAxisAngle(0.2f);
        var offset = new Vector3(1.5f, 0, 0);
        var manifold = Collide(offset, Quaternion.Identity, tilt);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal, 1e-3f);
    }

    [Fact]
    public void TiltInPlane_ValidManifold()
    {
        var tilt = Quaternion.CreateFromZAxisAngle(0.3f);
        var manifold = Collide(new Vector3(1.6f, 0, 0), Quaternion.Identity, tilt);
        AssertValidManifold(manifold);
        Assert.InRange(manifold.Count, 1, 2);
    }

    [Fact]
    public void Concentric_UsesLocalXOfAAsFallbackNormal()
    {
        var manifold = Collide(Vector3.Zero);
        AssertValidManifold(manifold);
        AssertEqual(new Vector3(1, 0, 0), manifold.Normal);
        AssertMaximumDepth(2f, Vector3.Zero);
        var rotated = Quaternion.CreateFromYAxisAngle(float.Pi * 0.5f);
        manifold = Collide(Vector3.Zero, rotated, rotated);
        AssertEqual(Vector3.Transform(Vector3.UnitX, rotated), manifold.Normal, 1e-3f);
    }

    [Fact]
    public void CrossingAxes_Intersecting_ValidManifold()
    {
        var manifold = Collide(Vector3.Zero, Quaternion.Identity, CapsuleOrientationAlongXAxis);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
        AssertEqual(new Vector3(1, 0, 0), manifold.Normal, 1e-3f);
        AssertMaximumDepth(2f, Vector3.Zero, Quaternion.Identity, CapsuleOrientationAlongXAxis);
    }

    [Fact]
    public void SwappedOrder_ProducesOppositeNormal()
    {
        var orientationA = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 2, 3)), 0.8f);
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(-2, 1, 0.5f)), 1.7f);
        var offset = new Vector3(0.9f, 0.4f, 0.7f);
        var forward = Collide(offset, orientationA, orientationB);
        var swapped = Collide(Vector3.Transform(-offset, Quaternion.Identity), orientationB, orientationA);
        AssertValidManifold(forward);
        AssertValidManifold(swapped);
        AssertEqual(forward.Normal, -swapped.Normal, 1e-3f);
    }

    [Fact]
    public void SeparatedWithinMargin_ProducesSpeculativeContacts()
    {
        var offset = new Vector3(2.1f, 0, 0);
        AssertSeparated(offset);
        var manifold = Collide(offset, 0.2f);
        AssertValidManifold(manifold, 0.2f);
        Assert.Equal(2, manifold.Count);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal);
        AssertMaximumDepth(-0.1f, offset, 0.2f);
    }

    [Fact]
    public void SeparatedBeyondMargin_Separated()
    {
        AssertSeparated(new Vector3(2.3f, 0, 0), 0.2f);
        AssertSeparated(new Vector3(0, 4.3f, 0), 0.2f);
        AssertSeparated(new Vector3(0, 0, 2.3f), Quaternion.Identity, CapsuleOrientationAlongXAxis, 0.2f);
    }

    [Fact]
    public void DifferentRadiiAndLengths_DepthAndContacts()
    {
        var largerCapsule = new Capsule(0.5f, 4);
        var smallerCapsule = new Capsule(0.25f, 2);
        var manifold = Collide(largerCapsule, smallerCapsule, 0f, new Vector3(0.5f, 0, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(2, manifold.Count);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal);
        for (int contactIndex = 0; contactIndex < 2; ++contactIndex)
        {
            Assert.Equal(0.25f, manifold.GetDepth(contactIndex), 1e-3f);
            //Contacts lie within B's extent, not A's.
            Assert.InRange(float.Abs(manifold.GetOffset(contactIndex).Y), 1f - 1e-3f, 1f + 1e-3f);
        }
    }

    [Fact]
    public void ZeroLengthCapsules_BehaveLikeSpheres()
    {
        var sphereLike = new Capsule(1, 0);
        var manifold = Collide(sphereLike, sphereLike, 0f, new Vector3(1.5f, 0, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal);
        Assert.Equal(0.5f, manifold.GetDepth(0), 1e-3f);
    }

    [Fact]
    public void ZeroLengthAAgainstRegularB_ContactOnBSegment()
    {
        var manifold = Collide(new Capsule(0.5f, 0), new Capsule(1, 2), 0f, new Vector3(1, 0.5f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal);
        Assert.Equal(0.5f, manifold.GetDepth(0), 1e-3f);
    }

    [Fact]
    public void ZeroLengthBAgainstRegularA_ContactOnASegment()
    {
        var manifold = Collide(new Capsule(1, 2), new Capsule(0.5f, 0), 0f, new Vector3(1, 0.5f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal);
        Assert.Equal(0.5f, manifold.GetDepth(0), 1e-3f);
    }

    [Fact]
    public void VeryShortCapsules_ValidManifold()
    {
        var tiny = new Capsule(1, 1e-4f);
        AssertValidManifold(Collide(tiny, tiny, 0f, new Vector3(1.5f, 0, 0), Quaternion.Identity, Quaternion.Identity));
        AssertValidManifold(Collide(tiny, new Capsule(1, 2), 0f, new Vector3(1.5f, 0, 0), Quaternion.Identity, CapsuleOrientationAlongZAxis));
    }

    [Fact]
    public void ZeroRadiusCapsules_ValidManifold()
    {
        var segment = new Capsule(0, 2);
        AssertValidManifold(Collide(segment, segment, 0f, new Vector3(0, 0, 0.001f), Quaternion.Identity, CapsuleOrientationAlongXAxis));
        AssertValidManifold(Collide(segment, segment, 0f, Vector3.Zero, Quaternion.Identity, Quaternion.Identity));
    }

    [Fact]
    public void LongCapsuleCrossedOverShortCapsule_ValidManifold()
    {
        var longCapsule = new Capsule(0.5f, 20);
        var shortCapsule = new Capsule(0.5f, 1);
        var manifold = Collide(longCapsule, shortCapsule, 0f, new Vector3(0.3f, 0.5f, 0.6f), Quaternion.Identity, CapsuleOrientationAlongXAxis);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
        AssertEqual(new Vector3(0, 0, -1), manifold.Normal, 1e-3f);
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
        AssertBundleConsistent(new Vector3(0, 3.5f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(0, 0, 1.5f), Quaternion.Identity, CapsuleOrientationAlongXAxis);
        AssertBundleConsistent(new Vector3(0.3f, 0.2f, 0.1f), orientationA, orientationB);
        AssertBundleConsistent(new Vector3(2.1f, 0, 0), Quaternion.Identity, Quaternion.Identity, 0.2f);
    }

    [Fact]
    public void RotatingWholeConfiguration_RotatesNormal()
    {
        var world = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0.3f, 1, -0.6f)), 0.9f);
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), 0.4f);
        AssertRotationInvariant(new Vector3(1.5f, 0.2f, 0.1f), Quaternion.Identity, Quaternion.Identity, world);
        AssertRotationInvariant(new Vector3(0.2f, 0.5f, 1.5f), Quaternion.Identity, CapsuleOrientationAlongXAxis, world);
        AssertRotationInvariant(new Vector3(1.4f, 0.3f, 0.2f), Quaternion.Identity, orientationB, world);
    }

    private static readonly Quaternion CapsuleOrientationAlongXAxis = Quaternion.CreateFromZAxisAngle(float.Pi * 0.5f);
    private static readonly Quaternion CapsuleOrientationAlongZAxis = Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f);
    private static readonly Quaternion Flipped = Quaternion.CreateFromXAxisAngle(float.Pi);
}
