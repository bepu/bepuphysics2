using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionRectangleRectangle : CollisionBase<Rectangle, RectangleWide, Rectangle, RectangleWide, Convex4ContactManifoldWide, RectanglePairTester>
{
    protected override Rectangle CreateShapeA() => new() { HalfWidth = 1, HalfLength = 1 };
    protected override Rectangle CreateShapeB() => new() { HalfWidth = 1, HalfLength = 1 };
}