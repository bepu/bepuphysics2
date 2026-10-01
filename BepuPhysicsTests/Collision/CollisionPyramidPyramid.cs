using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionPyramidPyramid : CollisionBase<Pyramid, PyramidWide, Pyramid, PyramidWide, Convex4ContactManifoldWide, PyramidPairTester>
{
    protected override Pyramid CreateShapeA() => new() { HalfWidth = 1, HalfLength = 1, Height = 2 };
    protected override Pyramid CreateShapeB() => new() { HalfWidth = 1, HalfLength = 1, Height = 2 };
}