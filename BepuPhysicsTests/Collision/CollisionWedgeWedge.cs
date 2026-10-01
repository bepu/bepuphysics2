using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionWedgeWedge : CollisionBase<Wedge, WedgeWide, Wedge, WedgeWide, Convex4ContactManifoldWide, WedgePairTester>
{
    protected override Wedge CreateShapeA() => new() { Width = 2, Height = 2, HalfLength = 1 };
    protected override Wedge CreateShapeB() => new() { Width = 2, Height = 2, HalfLength = 1 };
}