using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionBoxCone : CollisionBase<Box, BoxWide, Cone, ConeWide, Convex4ContactManifoldWide, BoxConeTester>
{
    protected override Box CreateShapeA() => new(1, 2, 3);
    protected override Cone CreateShapeB() => new() { Radius = 1, Height = 2 };
}