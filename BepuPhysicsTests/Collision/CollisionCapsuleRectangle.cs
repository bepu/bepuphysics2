using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionCapsuleRectangle : CollisionBase<Capsule, CapsuleWide, Rectangle, RectangleWide, Convex2ContactManifoldWide, CapsuleRectangleTester>
{
    protected override Capsule CreateShapeA() => new(1, 2);
    protected override Rectangle CreateShapeB() => new() { HalfWidth = 1, HalfLength = 1 };
}