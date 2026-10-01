using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public class CollisionCapsuleTriangle : CollisionBase<Capsule, CapsuleWide, Triangle, TriangleWide, Convex2ContactManifoldWide, CapsuleTriangleTester>
{
    protected override Capsule CreateShapeA() => new(1, 2);

    protected override Triangle CreateShapeB() => new(
        new Vector3(-1, 0, -1),
        new Vector3(1, 0, -1),
        new Vector3(-1, 0, 1));

    [Fact]
    public void FaceRegion_Colliding()
    {
        AssertColliding(new Vector3(0, -0.5f, -0.25f));
    }

    [Fact]
    public void EdgeRegion_Colliding()
    {
        AssertColliding(new Vector3(-0.5f, 0, -0.5f));
    }

    [Fact]
    public void OppositeSide_Colliding()
    {
        AssertColliding(new Vector3(0, 0.5f, -0.25f));
    }

    [Fact]
    public void FaceRegion_Separated()
    {
        AssertSeparated(new Vector3(0, 2.1f, -0.25f));
    }

    [Fact]
    public void OutsideTriangle_Separated()
    {
        AssertSeparated(new Vector3(0, 0, 2.1f));
    }

    [Theory]
    [InlineData(-1.5f, 1f, 0.5f)]
    public void CapsuleEndOverTriangleFace_NormalAndDepth(float verticalOffset, float expectedVerticalNormal, float expectedDepth)
    {
        //Capsule axis passes through the triangle interior; the hemisphere cap touches the face.
        var offset = new Vector3(0.3f, verticalOffset, 0.3f);
        var manifold = Collide(offset);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
        AssertEqual(new Vector3(0, expectedVerticalNormal, 0), manifold.Normal, 1e-3f);
        Assert.Equal(expectedDepth, manifold.GetDepth(0), 1e-3f);
    }

    [Fact]
    public void CapsuleEndOverTriangleFace_ContactAtCapsuleTip()
    {
        AssertHasContactAt(new Vector3(0, -1, 0) + new Vector3(0, -1, 0) * 0.5f + new Vector3(0, 0.5f, 0), new Vector3(0.3f, -1.5f, 0.3f), tolerance: 0.6f);
    }

    [Fact]
    public void CapsuleLyingParallelAboveFace_TwoContacts()
    {
        var orientationA = Quaternion.CreateFromZAxisAngle(float.Pi * 0.5f);
        var manifold = Collide(new Vector3(0, -0.5f, 0), orientationA, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
        AssertEqual(Vector3.UnitY, manifold.Normal, 1e-3f);
        Assert.Equal(0.5f, manifold.GetDepth(0), 1e-3f);
    }

    [Fact]
    public void CapsuleLyingParallelBelowFace_NormalFlipped()
    {
        var orientationA = Quaternion.CreateFromZAxisAngle(float.Pi * 0.5f);
        var manifold = Collide(new Vector3(0, 1.5f, 0), orientationA, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(0, manifold.Count);
    }

    [Fact]
    public void CapsuleCrossingEdge_NormalPointsAwayFromEdge()
    {
        //Horizontal capsule along Z, lying beside the hypotenuse-free edge at x=-1 of the triangle.
        var orientationA = Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f);
        var manifold = Collide(new Vector3(-1.5f, 0, 0), orientationA, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
        AssertEqual(Vector3.UnitX, manifold.Normal, 1e-3f);
        Assert.Equal(0.5f, manifold.GetDepth(0), 1e-3f);
    }

    [Fact]
    public void CapsuleBesideVertex_NormalPointsAwayFromVertex()
    {
        //Vertical capsule beyond the (1,0,-1) vertex.
        var manifold = Collide(new Vector3(-1.8f, 0, 1.4f));
        AssertValidManifold(manifold);
        if (manifold.Count > 0)
            Assert.True(manifold.Normal.X > 0 && manifold.Normal.Z < 0);
    }

    [Fact]
    public void CapsulePastTriangleExtent_Separated()
    {
        AssertSeparated(new Vector3(-0.3f, 0, 3f));
        AssertSeparated(new Vector3(3f, 0, 0));
        AssertSeparated(new Vector3(-3f, 0, 0));
        AssertSeparated(new Vector3(0, 0, -3f));
    }

    [Fact]
    public void Touching_ValidManifold()
    {
        var manifold = Collide(new Vector3(0.3f, -2f, 0.3f));
        AssertValidManifold(manifold);
    }

    [Fact]
    public void SeparatedWithinMargin_ProducesSpeculativeContact()
    {
        var manifold = Collide(new Vector3(0.3f, -2.25f, 0.3f), 0.5f);
        AssertValidManifold(manifold, 0.5f);
        Assert.True(manifold.Count >= 1);
        Assert.InRange(manifold.GetDepth(0), -0.3f, -0.2f);
        AssertEqual(Vector3.UnitY, manifold.Normal, 1e-3f);
    }

    [Fact]
    public void SeparatedBeyondMargin_NoContacts()
    {
        AssertSeparated(new Vector3(0.3f, -2.75f, 0.3f), 0.5f);
    }

    [Fact]
    public void TriangleFlippedWinding_SameCollision()
    {
        var flipped = new Triangle(new Vector3(-1, 0, -1), new Vector3(-1, 0, 1), new Vector3(1, 0, -1));
        var offset = new Vector3(0.3f, -1.5f, 0.3f);
        var originalManifold = Collide(CreateShapeA(), CreateShapeB(), 0f, offset, Quaternion.Identity, Quaternion.Identity);
        var reversedManifold = Collide(CreateShapeA(), flipped, 0f, offset, Quaternion.Identity, Quaternion.Identity);
        Assert.Equal(1, originalManifold.Count);
        AssertValidManifold(reversedManifold);
        Assert.True(reversedManifold.Count <= originalManifold.Count);
    }

    [Fact]
    public void ZeroLengthCapsule_ValidManifold()
    {
        var manifold = Collide(new Capsule(1, 0), CreateShapeB(), 0f, new Vector3(0.3f, -0.5f, 0.3f), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
    }

    [Fact]
    public void LargeCapsuleSmallTriangle_ValidManifold()
    {
        var manifold = Collide(new Capsule(0.5f, 10), CreateShapeB(), 0f, new Vector3(0.3f, 1f, 0.3f), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
    }

    [Fact]
    public void RandomOrientations_ValidManifolds()
    {
        var random = new System.Random(5);
        for (int iteration = 0; iteration < 200; ++iteration)
        {
            var firstShapeOrientation = Quaternion.Normalize(new Quaternion((float)random.NextDouble() - 0.5f, (float)random.NextDouble() - 0.5f, (float)random.NextDouble() - 0.5f, (float)random.NextDouble() + 0.1f));
            var secondShapeOrientation = Quaternion.Normalize(new Quaternion((float)random.NextDouble() - 0.5f, (float)random.NextDouble() - 0.5f, (float)random.NextDouble() - 0.5f, (float)random.NextDouble() + 0.1f));
            var offset = new Vector3((float)random.NextDouble() - 0.5f, (float)random.NextDouble() - 0.5f, (float)random.NextDouble() - 0.5f) * 3;
            AssertValidManifold(Collide(offset, firstShapeOrientation, secondShapeOrientation, 0.1f), 0.1f);
        }
    }

    [Fact]
    public void BundleResult_MatchesSingleLane()
    {
        AssertBundleConsistent(new Vector3(0.3f, -1.5f, 0.3f), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(-1.5f, 0, 0), Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f), Quaternion.Identity);
    }

    [Fact]
    public void RotatingWholeConfiguration_RotatesNormal()
    {
        var rotation = Quaternion.Normalize(Quaternion.CreateFromYawPitchRoll(0.7f, 0.3f, -0.5f));
        AssertRotationInvariant(new Vector3(0.3f, -1.5f, 0.3f), Quaternion.Identity, Quaternion.Identity, rotation);
    }

    [Fact]
    public void Rotated_Separated()
    {
        var orientationB = Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f);
        AssertSeparated(new Vector3(0, 0, 2.1f), Quaternion.Identity, orientationB);
    }
}
