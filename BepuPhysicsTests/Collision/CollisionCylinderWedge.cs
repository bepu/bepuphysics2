using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysicsTests.Collision;

public class CollisionCylinderWedge : CollisionBase<Cylinder, CylinderWide, Wedge, WedgeWide, Convex4ContactManifoldWide, CylinderWedgeTester>
{
    protected override Cylinder CreateShapeA() => new(1, 2);
    protected override Wedge CreateShapeB() => new() { Width = 2, Height = 2, HalfLength = 1 };
}