using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionBoxRectangle : CollisionBase<Box, BoxWide, Rectangle, RectangleWide, Convex4ContactManifoldWide, BoxRectangleTester>
{
    protected override Box CreateShapeA() => new(1, 2, 3);
    protected override Rectangle CreateShapeB() => new() { HalfWidth = 1, HalfLength = 1 };
}