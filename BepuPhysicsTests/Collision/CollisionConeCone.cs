using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionConeCone : CollisionBase<Cone, ConeWide, Cone, ConeWide, Convex4ContactManifoldWide, ConePairTester>
{
    protected override Cone CreateShapeA() => new() { Radius = 1, Height = 2 };
    protected override Cone CreateShapeB() => new() { Radius = 1, Height = 2 };
}