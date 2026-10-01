using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionConeRectangle : CollisionBase<Cone, ConeWide, Rectangle, RectangleWide, Convex4ContactManifoldWide, ConeRectangleTester>
{
    protected override Cone CreateShapeA() => new() { Radius = 1, Height = 2 };
    protected override Rectangle CreateShapeB() => new() { HalfWidth = 1, HalfLength = 1 };
}