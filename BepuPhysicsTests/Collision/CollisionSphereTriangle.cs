using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public class CollisionSphereTriangle : CollisionBase<Sphere, SphereWide, Triangle, TriangleWide, Convex1ContactManifoldWide, SphereTriangleTester>
{
    protected override Sphere CreateShapeA() => new(1f);

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
        AssertColliding(new Vector3(0, -0.5f, -0.25f));
    }

    [Fact]
    public void FaceRegion_Separated()
    {
        AssertSeparated(new Vector3(0, 1.1f, -0.25f));
    }

    [Fact]
    public void OutsideTriangle_Separated()
    {
        AssertSeparated(new Vector3(0, 0, 2.1f));
    }

    [Fact]
    public void Rotated_Separated()
    {
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.UnitX, float.Pi * 0.5f);
        AssertSeparated(new Vector3(0, 0, 2.1f), Quaternion.Identity, orientationB);
    }
}
