using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionCapsuleWedge : CollisionBase<Capsule, CapsuleWide, Wedge, WedgeWide, Convex2ContactManifoldWide, CapsuleWedgeTester>
{
    protected override Capsule CreateShapeA() => new(1, 2);
    protected override Wedge CreateShapeB() => new() { Width = 2, Height = 2, HalfLength = 1 };
}