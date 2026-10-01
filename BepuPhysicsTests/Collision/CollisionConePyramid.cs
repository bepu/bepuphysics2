using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionConePyramid : CollisionBase<Cone, ConeWide, Pyramid, PyramidWide, Convex4ContactManifoldWide, ConePyramidTester>
{
    protected override Cone CreateShapeA() => new() { Radius = 1, Height = 2 };
    protected override Pyramid CreateShapeB() => new() { HalfWidth = 1, HalfLength = 1, Height = 2 };
}