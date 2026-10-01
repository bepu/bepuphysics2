using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public class CollisionCylinderCylinder : CollisionBase<Cylinder, CylinderWide, Cylinder, CylinderWide, Convex4ContactManifoldWide, CylinderPairTester>
{
    protected override Cylinder CreateShapeA() => new(1, 2);
    protected override Cylinder CreateShapeB() => new(1, 2);

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
        AssertColliding(new Vector3(1.1f, 0.5f, 1.1f));
    }

    [Fact]
    public void OffsetAlongXYZ_Separated()
    {
        AssertSeparated(new Vector3(1.5f, 1.5f, 1.5f));
    }

    [Fact]
    public void Rotated_Colliding()
    {
        var orientationB = Quaternion.CreateFromXAxisAngle(float.Pi * 0.25f);
        AssertColliding(new Vector3(1.5f, 0, 0), Quaternion.Identity, orientationB);
    }

    [Theory]
    [InlineData(1.5f, -1f)]
    [InlineData(-1.5f, 1f)]
    public void StackedCapOnCap_NormalDepthAndContacts(float verticalOffset, float expectedVerticalNormal)
    {
        var manifold = Collide(new Vector3(0, verticalOffset, 0));
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
        AssertEqual(new Vector3(0, expectedVerticalNormal, 0), manifold.Normal, 1e-3f);
        AssertMaximumDepth(0.5f, new Vector3(0, verticalOffset, 0));
    }

    [Fact]
    public void StackedCapOnCap_OffsetPartialOverlap_AxialNormal()
    {
        var manifold = Collide(new Vector3(0.5f, 1.5f, 0.25f));
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
        AssertEqual(new Vector3(0, -1, 0), manifold.Normal, 1e-2f);
    }

    [Theory]
    [InlineData(1.5f, 0, -1f, 0)]
    [InlineData(-1.5f, 0, 1f, 0)]
    [InlineData(0, 1.5f, 0, -1f)]
    [InlineData(0, -1.5f, 0, 1f)]
    public void ParallelSideBySide_NormalDepthAndTwoContacts(float offsetX, float offsetZ, float normalX, float normalZ)
    {
        var offset = new Vector3(offsetX, 0, offsetZ);
        var manifold = Collide(offset);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 2);
        AssertEqual(new Vector3(normalX, 0, normalZ), manifold.Normal, 1e-3f);
        AssertMaximumDepth(0.5f, offset);
    }

    [Fact]
    public void ParallelSideBySide_VerticallyShifted_ValidManifold()
    {
        var manifold = Collide(new Vector3(1.5f, 1f, 0));
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal, 1e-2f);
    }

    [Fact]
    public void ParallelSideBySide_ShiftedBeyondLength_Separated()
    {
        AssertSeparated(new Vector3(1.5f, 2.1f, 0));
    }

    [Fact]
    public void ParallelSideBySide_Touching_ValidManifold()
    {
        AssertValidManifold(Collide(new Vector3(2f, 0, 0)));
        AssertValidManifold(Collide(new Vector3(0, 2f, 0)));
    }

    [Fact]
    public void PerpendicularAxes_SideAgainstCap_ValidManifold()
    {
        var orientationB = Quaternion.CreateFromZAxisAngle(float.Pi * 0.5f);
        foreach (var horizontalOffset in new[] { 1.5f, 1.9f, 1.99f, -1.5f })
            AssertValidManifold(Collide(new Vector3(horizontalOffset, 0, 0), Quaternion.Identity, orientationB));
        AssertSeparated(new Vector3(2.1f, 0, 0), Quaternion.Identity, orientationB);
        AssertSeparated(new Vector3(0, 2.1f, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void PerpendicularAxes_CrossingLikeX_ValidManifold()
    {
        var orientationB = Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f);
        foreach (var verticalOffset in new[] { 0f, 1f, 1.5f, 1.9f, 2.1f })
            AssertValidManifold(Collide(new Vector3(0, verticalOffset, 0), Quaternion.Identity, orientationB));
        AssertSeparated(new Vector3(0, 2.1f, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void PerpendicularAxes_SideContactOnLyingCylinder_NormalIsVertical()
    {
        //B lies on its side atop A's cap. A's cap is the nearest feature.
        var manifold = Collide(new Vector3(0, 1.8f, 0), Quaternion.Identity, Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f));
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
        AssertEqual(new Vector3(0, -1, 0), manifold.Normal, 1e-3f);
    }

    [Fact]
    public void RimToRim_AxesTilted_ValidManifold()
    {
        foreach (var angle in new[] { 0.1f, 0.4f, 0.785f, 1.2f, 1.5f })
            foreach (var verticalOffset in new[] { 1.5f, 1.8f })
                AssertValidManifold(Collide(new Vector3(0.2f, verticalOffset, 0), Quaternion.Identity, Quaternion.CreateFromXAxisAngle(angle)));
    }

    [Fact]
    public void SkewedAxes_ValidManifold()
    {
        var orientationA = Quaternion.Normalize(Quaternion.CreateFromYawPitchRoll(0.3f, 0.5f, 0.7f));
        var orientationB = Quaternion.Normalize(Quaternion.CreateFromYawPitchRoll(-0.6f, 1.1f, 0.2f));
        foreach (var horizontalOffset in new[] { 0.5f, 1f, 1.5f, 2f })
            AssertValidManifold(Collide(new Vector3(horizontalOffset, 0.3f, -0.2f), orientationA, orientationB, 0.1f), 0.1f);
    }

    [Fact]
    public void Concentric_ValidManifold()
    {
        AssertValidManifold(Collide(Vector3.Zero));
        AssertValidManifold(Collide(Vector3.Zero, Quaternion.Identity, Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f)));
        AssertValidManifold(Collide(Vector3.Zero, Quaternion.Identity, Quaternion.CreateFromXAxisAngle(0.3f)));
    }

    [Fact]
    public void SecondCylinderFlipped_SameAsUnflipped()
    {
        var originalManifold = Collide(new Vector3(1.5f, 0, 0));
        var flippedManifold = Collide(new Vector3(1.5f, 0, 0), Quaternion.Identity, Quaternion.CreateFromXAxisAngle(float.Pi));
        Assert.Equal(originalManifold.Count > 0, flippedManifold.Count > 0);
        AssertValidManifold(flippedManifold);
        if (flippedManifold.Count > 0)
            AssertEqual(originalManifold.Normal, flippedManifold.Normal, 1e-3f);
    }

    [Fact]
    public void OrderSwapped_NormalFlips()
    {
        var offset = new Vector3(1.5f, 0.2f, 0);
        var firstThenSecondManifold = Collide(offset);
        var secondThenFirstManifold = Collide(CreateShapeB(), CreateShapeA(), 0f, -offset, Quaternion.Identity, Quaternion.Identity);
        Assert.Equal(firstThenSecondManifold.Count > 0, secondThenFirstManifold.Count > 0);
        if (firstThenSecondManifold.Count > 0)
            AssertEqual(-firstThenSecondManifold.Normal, secondThenFirstManifold.Normal, 1e-3f);
    }

    [Fact]
    public void SeparatedWithinMargin_ProducesSpeculativeContacts()
    {
        foreach (var offset in new[] { new Vector3(2.25f, 0, 0), new Vector3(0, 2.25f, 0), new Vector3(0, 0, -2.25f) })
        {
            var manifold = Collide(offset, 0.5f);
            AssertValidManifold(manifold, 0.5f);
            Assert.True(manifold.Count >= 1);
            Assert.InRange(manifold.GetDepth(0), -0.3f, -0.2f);
            AssertEqual(-Vector3.Normalize(offset), manifold.Normal, 1e-3f);
        }
    }

    [Fact]
    public void SeparatedBeyondMargin_NoContacts()
    {
        foreach (var offset in new[] { new Vector3(2.75f, 0, 0), new Vector3(0, 2.75f, 0), new Vector3(0, 0, 2.75f) })
            AssertSeparated(offset, 0.5f);
    }

    [Fact]
    public void DifferentSizes_DepthAndNormal()
    {
        var largerCylinder = new Cylinder(2, 4);
        var smallerCylinder = new Cylinder(0.5f, 1);
        var manifold = Collide(largerCylinder, smallerCylinder, 0f, new Vector3(2.25f, 0, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal, 1e-3f);
        Assert.Equal(0.25f, manifold.GetDepth(0), 1e-3f);

        manifold = Collide(largerCylinder, smallerCylinder, 0f, new Vector3(0, 2.25f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.True(manifold.Count >= 1);
        AssertEqual(new Vector3(0, -1, 0), manifold.Normal, 1e-3f);
        Assert.Equal(0.25f, manifold.GetDepth(0), 1e-3f);
    }

    [Fact]
    public void SmallCylinderInsideLarge_ValidManifold()
    {
        var large = new Cylinder(5, 10);
        var smallerCylinder = new Cylinder(0.5f, 1);
        foreach (var offset in new[] { Vector3.Zero, new Vector3(1, 1, 1), new Vector3(4.8f, 0, 0), new Vector3(0, 4.8f, 0) })
        {
            AssertValidManifold(Collide(large, smallerCylinder, 0.1f, offset, Quaternion.Identity, Quaternion.CreateFromXAxisAngle(0.4f)), 0.1f);
            AssertValidManifold(Collide(smallerCylinder, large, 0.1f, offset, Quaternion.CreateFromXAxisAngle(0.4f), Quaternion.Identity), 0.1f);
        }
    }

    [Fact]
    public void DegenerateShapes_ValidManifold()
    {
        var shapes = new[] { new Cylinder(0, 2), new Cylinder(1, 0), new Cylinder(0, 0), new Cylinder(1e-4f, 1e-4f) };
        foreach (var degenerate in shapes)
            foreach (var offset in new[] { Vector3.Zero, new Vector3(0.5f, 0, 0), new Vector3(0, 0.5f, 0), new Vector3(1.05f, 0.3f, 0) })
            {
                AssertValidManifold(Collide(degenerate, CreateShapeB(), 0.1f, offset, Quaternion.Identity, Quaternion.Identity), 0.1f);
                AssertValidManifold(Collide(CreateShapeA(), degenerate, 0.1f, offset, Quaternion.Identity, Quaternion.CreateFromXAxisAngle(0.5f)), 0.1f);
            }
    }

    [Fact]
    public void RandomOrientations_ValidManifolds()
    {
        var random = new System.Random(13);
        for (int iteration = 0; iteration < 500; ++iteration)
        {
            var offset = new Vector3((float)random.NextDouble() - 0.5f, (float)random.NextDouble() - 0.5f, (float)random.NextDouble() - 0.5f) * 4;
            AssertValidManifold(Collide(offset, Quaternion.CreateRandom(random), Quaternion.CreateRandom(random), 0.1f), 0.1f);
        }
    }

    [Fact]
    public void BundleResult_MatchesSingleLane()
    {
        AssertBundleConsistent(new Vector3(1.5f, 0, 0), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(0, 1.5f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(0.2f, 1.5f, 0), Quaternion.Identity, Quaternion.CreateFromXAxisAngle(0.5f));
        AssertBundleConsistent(new Vector3(0, 1.5f, 0), Quaternion.Identity, Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f), 0.1f);
    }

    [Fact]
    public void RotatingWholeConfiguration_RotatesNormal()
    {
        var rotation = Quaternion.Normalize(Quaternion.CreateFromYawPitchRoll(0.7f, 0.3f, -0.5f));
        var offset = new Vector3(0, 1.5f, 0);
        var expected = Collide(offset);
        var rotated = Collide(Vector3.Transform(offset, rotation), rotation, rotation);
        AssertValidManifold(rotated);
        Assert.True(expected.Count > 0 && rotated.Count > 0);
        AssertEqual(Vector3.Transform(expected.Normal, rotation), rotated.Normal, 1e-3f);
    }

    [Fact]
    public void Rotated_Separated()
    {
        var orientationB = Quaternion.CreateFromXAxisAngle(float.Pi * 0.25f);
        AssertSeparated(new Vector3(2.5f, 0, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void Rotated_SeparatedAlongAxes()
    {
        var orientationB = Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f);
        AssertSeparated(new Vector3(0, 0, 2.1f), Quaternion.Identity, orientationB);
        AssertSeparated(new Vector3(0, 0, -2.1f), Quaternion.Identity, orientationB);
        AssertSeparated(new Vector3(2.1f, 0, 0), Quaternion.Identity, orientationB);
    }

}
