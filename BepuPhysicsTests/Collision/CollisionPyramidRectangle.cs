using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionPyramidRectangle : CollisionBase<Pyramid, PyramidWide, Rectangle, RectangleWide, Convex4ContactManifoldWide, PyramidRectangleTester>
{
    protected override Pyramid CreateShapeA() => new() { HalfWidth = 1, HalfLength = 1, Height = 2 };
    protected override Rectangle CreateShapeB() => new() { HalfWidth = 1, HalfLength = 1 };
}