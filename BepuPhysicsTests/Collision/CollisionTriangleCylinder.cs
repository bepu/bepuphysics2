using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public class CollisionTriangleCylinder : CollisionBase<Triangle, TriangleWide, Cylinder, CylinderWide, Convex4ContactManifoldWide, TriangleCylinderTester>
{
    protected override Triangle CreateShapeA() => new(
        new Vector3(-1, 0, -1),
        new Vector3(1, 0, -1),
        new Vector3(-1, 0, 1));

    protected override Cylinder CreateShapeB() => new(1, 2);

    [Fact]
    public void OffsetAlongDiagonal_Colliding()
    {
        AssertColliding(new Vector3(0.25f, 0, 0.25f));
    }

    [Fact]
    public void OffsetAlongX_Colliding()
    {
        AssertColliding(new Vector3(0.5f, 0, 0));
    }

    [Fact]
    public void OffsetAlongY_Colliding()
    {
        AssertColliding(new Vector3(0, 0.5f, 0));
    }

    [Fact]
    public void OffsetAlongY_Separated()
    {
        AssertSeparated(new Vector3(0, 1.1f, 0));
    }

    [Fact]
    public void OffsetAlongDiagonal_Separated()
    {
        AssertSeparated(new Vector3(1.6f, 0, 1.6f));
    }

    [Fact]
    public void Rotated_Colliding()
    {
        var orientationB = Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f);
        AssertColliding(new Vector3(0, 0, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void Concentric_ValidManifold()
    {
        AssertValidManifold(Collide(Vector3.Zero));
    }

    [Fact]
    public void AxisThroughTriangle_ManyOffsets_ValidManifold()
    {
        for (float verticalOffset = -2.5f; verticalOffset <= 2.5f; verticalOffset += 0.25f)
            AssertValidManifold(Collide(new Vector3(0, verticalOffset, 0), 0.1f), 0.1f);
    }

    [Fact]
    public void FarAway_NoContacts()
    {
        foreach (var offset in new[] { new Vector3(5, 0, 0), new Vector3(-5, 0, 0), new Vector3(0, 0, 5), new Vector3(0, 0, -5), new Vector3(0, 5, 0), new Vector3(0, -5, 0) })
        {
            Assert.Equal(0, Collide(offset).Count);
            Assert.Equal(0, Collide(offset, 0.5f).Count);
        }
    }

    [Fact]
    public void CylinderAboveAndBelow_BeyondHalfLength_Separated()
    {
        AssertSeparated(new Vector3(0, 1.5f, 0));
        AssertSeparated(new Vector3(0, -1.5f, 0));
    }

    [Fact]
    public void SeparatedWithinMargin_ValidSpeculative()
    {
        foreach (var verticalOffset in new[] { 1.25f, -1.25f })
        {
            var manifold = Collide(new Vector3(0, verticalOffset, 0), 0.5f);
            AssertValidManifold(manifold, 0.5f);
            if (manifold.Count > 0)
                Assert.True(float.Abs(float.Abs(manifold.Normal.Y) - 1) < 1e-3f);
        }
    }

    [Fact]
    public void SeparatedBeyondMargin_NoContacts()
    {
        Assert.Equal(0, Collide(new Vector3(0, 2f, 0), 0.5f).Count);
        Assert.Equal(0, Collide(new Vector3(0, -2f, 0), 0.5f).Count);
    }

    [Fact]
    public void CylinderCapFlat_OnTriangleFace_ValidManifold()
    {
        foreach (var verticalOffset in new[] { 0.9f, 0.99f, -0.9f, -0.99f })
            AssertValidManifold(Collide(new Vector3(0.2f, verticalOffset, 0.2f)));
    }

    [Fact]
    public void CylinderSideOnTriangle_AxisParallelToFace_ValidManifold()
    {
        var orientationB = Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f);
        foreach (var verticalOffset in new[] { 0.9f, 0.99f, -0.9f, -0.99f, 0.5f, 0f })
            AssertValidManifold(Collide(new Vector3(0, verticalOffset, 0), Quaternion.Identity, orientationB));
    }

    [Fact]
    public void CylinderSideOnTriangle_AxisAlongXAndZ_ValidManifold()
    {
        foreach (var orientation in new[] { Quaternion.CreateFromZAxisAngle(float.Pi * 0.5f), Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f) })
            foreach (var verticalOffset in new[] { 0.5f, 0.99f, -0.5f })
                AssertValidManifold(Collide(new Vector3(0.1f, verticalOffset, 0.1f), Quaternion.Identity, orientation));
    }

    [Fact]
    public void CylinderBeyondTriangleEdges_Separated()
    {
        AssertSeparated(new Vector3(-3f, 0, 0));
        AssertSeparated(new Vector3(0, 0, -3f));
        AssertSeparated(new Vector3(2.5f, 0, 2.5f));
    }

    [Fact]
    public void CylinderRimNearTriangleEdge_ValidManifold()
    {
        foreach (var horizontalOffset in new[] { 1.5f, 1.9f, 2.1f })
            AssertValidManifold(Collide(new Vector3(horizontalOffset, 0.5f, -1f), 0.1f), 0.1f);
        foreach (var depthOffset in new[] { 1.5f, 1.9f, 2.1f })
            AssertValidManifold(Collide(new Vector3(-1f, 0.5f, depthOffset), 0.1f), 0.1f);
    }

    [Fact]
    public void CylinderAtVertices_ValidManifold()
    {
        foreach (var triangleVertex in new[] { new Vector3(-1, 0, -1), new Vector3(1, 0, -1), new Vector3(-1, 0, 1) })
            foreach (var verticalOffset in new[] { 0.5f, 0f, -0.5f })
                AssertValidManifold(Collide(triangleVertex + new Vector3(0, verticalOffset, 0), 0.1f), 0.1f);
    }

    [Fact]
    public void TiltedCylinder_ValidManifold()
    {
        foreach (var angle in new[] { 0.1f, 0.5f, 0.78f, 1.2f, 1.5f, 2f, 3f })
            AssertValidManifold(Collide(new Vector3(0.2f, 0.5f, 0.2f), Quaternion.Identity, Quaternion.CreateFromXAxisAngle(angle), 0.1f), 0.1f);
    }

    [Fact]
    public void CylinderFlipped_ValidManifold()
    {
        AssertValidManifold(Collide(new Vector3(0.2f, 0.9f, 0.2f), Quaternion.Identity, Quaternion.CreateFromXAxisAngle(float.Pi)));
        AssertValidManifold(Collide(new Vector3(0.2f, -0.9f, 0.2f), Quaternion.Identity, Quaternion.CreateFromXAxisAngle(float.Pi)));
    }

    [Fact]
    public void RotatedTriangle_ValidManifold()
    {
        foreach (var angle in new[] { 0.5f, float.Pi * 0.5f, float.Pi })
            AssertValidManifold(Collide(new Vector3(0.2f, 0.5f, 0.2f), Quaternion.CreateFromXAxisAngle(angle), Quaternion.Identity, 0.1f), 0.1f);
    }

    [Fact]
    public void DegenerateShapes_ValidManifold()
    {
        var offsets = new[] { Vector3.Zero, new Vector3(0, 0.5f, 0), new Vector3(0.5f, 0, 0) };
        var degenerateTriangle = new Triangle(Vector3.Zero, Vector3.Zero, Vector3.Zero);
        var lineTriangle = new Triangle(new Vector3(-1, 0, 0), Vector3.Zero, new Vector3(1, 0, 0));
        foreach (var offset in offsets)
        {
            AssertValidManifold(Collide(degenerateTriangle, CreateShapeB(), 0.1f, offset, Quaternion.Identity, Quaternion.Identity), 0.1f);
            AssertValidManifold(Collide(lineTriangle, CreateShapeB(), 0.1f, offset, Quaternion.Identity, Quaternion.Identity), 0.1f);
            AssertValidManifold(Collide(CreateShapeA(), new Cylinder(0, 2), 0.1f, offset, Quaternion.Identity, Quaternion.Identity), 0.1f);
            AssertValidManifold(Collide(CreateShapeA(), new Cylinder(1, 0), 0.1f, offset, Quaternion.Identity, Quaternion.Identity), 0.1f);
        }
    }

    [Fact]
    public void DifferentSizes_ValidManifold()
    {
        var largeTriangle = new Triangle(new Vector3(-10, 0, -10), new Vector3(10, 0, -10), new Vector3(-10, 0, 10));
        AssertValidManifold(Collide(largeTriangle, new Cylinder(0.1f, 0.2f), 0.1f, new Vector3(0, 0.05f, 0), Quaternion.Identity, Quaternion.Identity), 0.1f);
        AssertValidManifold(Collide(CreateShapeA(), new Cylinder(10, 0.2f), 0.1f, new Vector3(0, 0.05f, 0), Quaternion.Identity, Quaternion.Identity), 0.1f);
        AssertValidManifold(Collide(CreateShapeA(), new Cylinder(0.1f, 20), 0.1f, new Vector3(0, 0.05f, 0), Quaternion.Identity, Quaternion.CreateFromXAxisAngle(0.7f)), 0.1f);
    }

    [Fact]
    public void RandomOrientations_ValidManifolds()
    {
        var random = new System.Random(11);
        for (int iteration = 0; iteration < 300; ++iteration)
        {
            var offset = new Vector3((float)random.NextDouble() - 0.5f, (float)random.NextDouble() - 0.5f, (float)random.NextDouble() - 0.5f) * 3;
            AssertValidManifold(Collide(offset, Quaternion.CreateRandom(random), Quaternion.CreateRandom(random), 0.1f), 0.1f);
        }
    }

    [Fact]
    public void BundleResult_MatchesSingleLane()
    {
        AssertBundleConsistent(new Vector3(0.2f, 0.5f, 0.2f), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(0, 0.5f, 0), Quaternion.Identity, Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f), 0.1f);
    }

    [Fact]
    public void RotatingWholeConfiguration_RotatesNormal()
    {
        var rotation = Quaternion.Normalize(Quaternion.CreateFromYawPitchRoll(0.7f, 0.3f, -0.5f));
        AssertRotationInvariant(new Vector3(0.2f, 0.5f, 0.2f), Quaternion.Identity, Quaternion.Identity, rotation);
    }

    [Fact]
    public void Rotated_Separated()
    {
        var orientationB = Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f);
        AssertSeparated(new Vector3(0, 0, 2.1f), Quaternion.Identity, orientationB);
    }

}
