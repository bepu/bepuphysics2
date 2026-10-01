using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionPyramidWedge : CollisionBase<Pyramid, PyramidWide, Wedge, WedgeWide, Convex4ContactManifoldWide, PyramidWedgeTester>
{
    protected override Pyramid CreateShapeA() => new() { HalfWidth = 1, HalfLength = 1, Height = 2 };
    protected override Wedge CreateShapeB() => new() { Width = 2, Height = 2, HalfLength = 1 };
}