using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionBoxWedge : CollisionBase<Box, BoxWide, Wedge, WedgeWide, Convex4ContactManifoldWide, BoxWedgeTester>
{
    protected override Box CreateShapeA() => new(1, 2, 3);
    protected override Wedge CreateShapeB() => new() { Width = 2, Height = 2, HalfLength = 1 };
}