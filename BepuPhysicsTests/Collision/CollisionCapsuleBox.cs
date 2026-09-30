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
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.UnitY, float.Pi * 0.25f);
        AssertColliding(new Vector3(1.5f, 0, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void Rotated_Separated()
    {
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.UnitY, float.Pi * 0.25f);
        AssertSeparated(new Vector3(2.6f, 0, 0), Quaternion.Identity, orientationB);
    }
}
