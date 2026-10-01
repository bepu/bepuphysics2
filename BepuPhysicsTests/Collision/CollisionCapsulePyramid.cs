using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionCapsulePyramid : CollisionBase<Capsule, CapsuleWide, Pyramid, PyramidWide, Convex2ContactManifoldWide, CapsulePyramidTester>
{
    protected override Capsule CreateShapeA() => new(1, 2);
    protected override Pyramid CreateShapeB() => new() { HalfWidth = 1, HalfLength = 1, Height = 2 };
}