using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionWedgeRectangle : CollisionBase<Wedge, WedgeWide, Rectangle, RectangleWide, Convex4ContactManifoldWide, WedgeRectangleTester>
{
    protected override Wedge CreateShapeA() => new() { Width = 2, Height = 2, HalfLength = 1 };
    protected override Rectangle CreateShapeB() => new() { HalfWidth = 1, HalfLength = 1 };
}