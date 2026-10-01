using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionCapsuleCone : CollisionBase<Capsule, CapsuleWide, Cone, ConeWide, Convex2ContactManifoldWide, CapsuleConeTester>
{
    protected override Capsule CreateShapeA() => new(1, 2);
    protected override Cone CreateShapeB() => new() { Radius = 1, Height = 2 };
}