using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionSphereCone : CollisionBase<Sphere, SphereWide, Cone, ConeWide, Convex1ContactManifoldWide, SphereConeTester>
{
    protected override Sphere CreateShapeA() => new(1f);
    protected override Cone CreateShapeB() => new() { Radius = 1, Height = 2 };
}