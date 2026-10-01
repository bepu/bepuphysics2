using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public class CollisionTriangleCone : CollisionBase<Triangle, TriangleWide, Cone, ConeWide, Convex4ContactManifoldWide, TriangleConeTester>
{
    protected override Triangle CreateShapeA() => new(
        new Vector3(-1, 0, -1),
        new Vector3(1, 0, -1),
        new Vector3(-1, 0, 1));
    protected override Cone CreateShapeB() => new() { Radius = 1, Height = 2 };
}