using BepuPhysics.Collidables;
using System.Numerics;

namespace BepuPhysicsTests.RayCasts;

public class RayCastCone : RayCastBase<Cone, ConeWide>
{
    protected override Cone CreateShape() => new Cone
    {
        Radius = 1,
        Height = 2
    };

    [Fact]
    public void RayPointingAtBase_ShouldHit()
    {
        var localOrigin = new Vector3(0, -2, 0);
        var localDirection = new Vector3(0, 1, 0);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAtSide_ShouldHit()
    {
        var localOrigin = new Vector3(3, 0.5f, 0);
        var localDirection = new Vector3(-1, 0, 0);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingStraightDownUnderneath_ShouldNotHit()
    {
        var localOrigin = new Vector3(0, -2, 0);
        var localDirection = new Vector3(0, -1, 0);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingToTheSideFromUnderneath_ShouldNotHit()
    {
        var localOrigin = new Vector3(0, -2, 0);
        var localDirection = new Vector3(1, 0, 0);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingNextToSide_ShouldNotHit()
    {
        var localOrigin = new Vector3(3, 0.5f, 0.6f);
        var localDirection = new Vector3(-1, 0, 0);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAboveApex_ShouldNotHit()
    {
        var localOrigin = new Vector3(0.25f, 2, 0);
        var localDirection = new Vector3(0, 1, 0);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }
}
