using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionCylinderCone : CollisionBase<Cylinder, CylinderWide, Cone, ConeWide, Convex4ContactManifoldWide, CylinderConeTester>
{
    protected override Cylinder CreateShapeA() => new(1, 2);
    protected override Cone CreateShapeB() => new() { Radius = 1, Height = 2 };
}