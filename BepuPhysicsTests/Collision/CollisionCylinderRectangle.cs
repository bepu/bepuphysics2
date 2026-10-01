using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionCylinderRectangle : CollisionBase<Cylinder, CylinderWide, Rectangle, RectangleWide, Convex4ContactManifoldWide, CylinderRectangleTester>
{
    protected override Cylinder CreateShapeA() => new(1, 2);
    protected override Rectangle CreateShapeB() => new() { HalfWidth = 1, HalfLength = 1 };
}