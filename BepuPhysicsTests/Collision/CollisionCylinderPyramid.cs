using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionCylinderPyramid : CollisionBase<Cylinder, CylinderWide, Pyramid, PyramidWide, Convex4ContactManifoldWide, CylinderPyramidTester>
{
    protected override Cylinder CreateShapeA() => new(1, 2);
    protected override Pyramid CreateShapeB() => new() { HalfWidth = 1, HalfLength = 1, Height = 2 };
}