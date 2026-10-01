using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public class CollisionTrianglePyramid : CollisionBase<Triangle, TriangleWide, Pyramid, PyramidWide, Convex4ContactManifoldWide, TrianglePyramidTester>
{
    protected override Triangle CreateShapeA() => new(
        new Vector3(-1, 0, -1),
        new Vector3(1, 0, -1),
        new Vector3(-1, 0, 1));
    protected override Pyramid CreateShapeB() => new() { HalfWidth = 1, HalfLength = 1, Height = 2 };
}