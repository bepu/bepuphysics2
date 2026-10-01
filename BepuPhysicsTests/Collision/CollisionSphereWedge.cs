using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionSphereWedge : CollisionBase<Sphere, SphereWide, Wedge, WedgeWide, Convex1ContactManifoldWide, SphereWedgeTester>
{
    protected override Sphere CreateShapeA() => new(1f);
    protected override Wedge CreateShapeB() => new() { Width = 2, Height = 2, HalfLength = 1 };
}