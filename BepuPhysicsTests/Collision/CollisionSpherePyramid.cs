using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionSpherePyramid : CollisionBase<Sphere, SphereWide, Pyramid, PyramidWide, Convex1ContactManifoldWide, SpherePyramidTester>
{
    protected override Sphere CreateShapeA() => new(1f);
    protected override Pyramid CreateShapeB() => new() { HalfWidth = 1, HalfLength = 1, Height = 2 };
}