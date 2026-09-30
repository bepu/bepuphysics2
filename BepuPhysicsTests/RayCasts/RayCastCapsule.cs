using BepuPhysics.Collidables;
using System.Numerics;

namespace BepuPhysicsTests.RayCasts;

public class RayCastCapsule : RayCastBase<Capsule, CapsuleWide>
{
    protected override Capsule CreateShape() => new Capsule(1, 2);

    [Fact]
    public void RayPointingAtCylindricalSection_ShouldHit()
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
    public void RayPointingAwayFromCapsule_ShouldNotHit()
    {
        var localOrigin = new Vector3(0, 0, -5);
        var localDirection = new Vector3(0, 0, -1);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingNextToCapsule_ShouldNotHit()
    {
        var localOrigin = new Vector3(1.1f, 0, -5);
        var localDirection = new Vector3(0, 0, 1);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAboveCapsule_ShouldNotHit()
    {
        var localOrigin = new Vector3(0, 3, 0);
        var localDirection = new Vector3(0, 1, 0);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }
}
