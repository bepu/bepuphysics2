using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionSphereRectangle : CollisionBase<Sphere, SphereWide, Rectangle, RectangleWide, Convex1ContactManifoldWide, SphereRectangleTester>
{
    protected override Sphere CreateShapeA() => new(1f);
    protected override Rectangle CreateShapeB() => new() { HalfWidth = 1, HalfLength = 1 };
}