using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionBoxPyramid : CollisionBase<Box, BoxWide, Pyramid, PyramidWide, Convex4ContactManifoldWide, BoxPyramidTester>
{
    protected override Box CreateShapeA() => new(1, 2, 3);
    protected override Pyramid CreateShapeB() => new() { HalfWidth = 1, HalfLength = 1, Height = 2 };
}
