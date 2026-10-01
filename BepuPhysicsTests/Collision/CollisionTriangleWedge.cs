using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public class CollisionTriangleWedge : CollisionBase<Triangle, TriangleWide, Wedge, WedgeWide, Convex4ContactManifoldWide, TriangleWedgeTester>
{
    protected override Triangle CreateShapeA() => new(
        new Vector3(-1, 0, -1),
        new Vector3(1, 0, -1),
        new Vector3(-1, 0, 1));
    protected override Wedge CreateShapeB() => new() { Width = 2, Height = 2, HalfLength = 1 };
}