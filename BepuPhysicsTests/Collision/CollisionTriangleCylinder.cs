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
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.UnitX, float.Pi * 0.5f);
        AssertColliding(new Vector3(0, 0, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void Rotated_Separated()
    {
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.UnitX, float.Pi * 0.5f);
        AssertSeparated(new Vector3(0, 0, 2.1f), Quaternion.Identity, orientationB);
    }
}
