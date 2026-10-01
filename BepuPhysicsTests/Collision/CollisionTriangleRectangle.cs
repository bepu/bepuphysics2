using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public class CollisionTriangleRectangle : CollisionBase<Triangle, TriangleWide, Rectangle, RectangleWide, Convex4ContactManifoldWide, TriangleRectangleTester>
{
    protected override Triangle CreateShapeA() => new(
        new Vector3(-1, 0, -1),
        new Vector3(1, 0, -1),
        new Vector3(-1, 0, 1));
    protected override Rectangle CreateShapeB() => new() { HalfWidth = 1, HalfLength = 1 };
}