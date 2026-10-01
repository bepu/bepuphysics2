using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionConeWedge : CollisionBase<Cone, ConeWide, Wedge, WedgeWide, Convex4ContactManifoldWide, ConeWedgeTester>
{
    protected override Cone CreateShapeA() => new() { Radius = 1, Height = 2 };
    protected override Wedge CreateShapeB() => new() { Width = 2, Height = 2, HalfLength = 1 };
}