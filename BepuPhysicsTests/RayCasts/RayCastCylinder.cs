using BepuPhysics.Collidables;
using System.Numerics;

namespace BepuPhysicsTests.RayCasts;

public class RayCastCylinder : RayCastBase<Cylinder, CylinderWide>
{
    protected override Cylinder CreateShape() => new Cylinder(1, 2);

    [Fact]
    public void RayPointingAtCylindricalSide_ShouldHit()
    {
        var localOrigin = new Vector3(0, 0, -5);
        var localDirection = new Vector3(0, 0, 1);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAtTopCap_ShouldHit()
    {
        var localOrigin = new Vector3(0, 5, 0);
        var localDirection = new Vector3(0, -1, 0);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAtBottomCap_ShouldHit()
    {
        var localOrigin = new Vector3(0, -5, 0);
        var localDirection = new Vector3(0, 1, 0);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAwayFromCylinder_ShouldNotHit()
    {
        var localOrigin = new Vector3(0, 0, -5);
        var localDirection = new Vector3(0, 0, -1);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingNextToCylinder_ShouldNotHit()
    {
        var localOrigin = new Vector3(1.1f, 0, -5);
        var localDirection = new Vector3(0, 0, 1);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAboveCylinder_ShouldNotHit()
    {
        var localOrigin = new Vector3(0, 2, 0);
        var localDirection = new Vector3(0, 1, 0);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }
}
