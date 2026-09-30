using BepuPhysics.Collidables;
using System.Numerics;

namespace BepuPhysicsTests.RayCasts;

public class RayCastSphere : RayCastBase<Sphere, SphereWide>
{
    protected override Sphere CreateShape() => new Sphere(1);

    [Fact]
    public void RayPointingAtSphereCenter_ShouldHit()
    {
        var localOrigin = new Vector3(0, 0, -5);
        var localDirection = new Vector3(0, 0, 1);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAtTopOfSphere_ShouldHit()
    {
        var localOrigin = new Vector3(0, 5, 0);
        var localDirection = new Vector3(0, -1, 0);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAtBottomOfSphere_ShouldHit()
    {
        var localOrigin = new Vector3(0, -5, 0);
        var localDirection = new Vector3(0, 1, 0);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAtSphereAtAnAngle_ShouldHit()
    {
        var localOrigin = new Vector3(3, 3, 0);
        var localDirection = new Vector3(-1, -1, 0);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAwayFromSphere_ShouldNotHit()
    {
        var localOrigin = new Vector3(0, 0, -5);
        var localDirection = new Vector3(0, 0, -1);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingNextToSphere_ShouldNotHit()
    {
        var localOrigin = new Vector3(1.1f, 0, -5);
        var localDirection = new Vector3(0, 0, 1);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAboveSphere_ShouldNotHit()
    {
        var localOrigin = new Vector3(0, 2, 0);
        var localDirection = new Vector3(0, 1, 0);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }
}
