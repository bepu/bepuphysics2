using BepuPhysics.Collidables;
using System.Numerics;

namespace BepuPhysicsTests.RayCasts;

public class RayCastWedge : RayCastBase<Wedge, WedgeWide>
{
    protected override Wedge CreateShape() => new Wedge
    {
        Width = 2,
        Height = 2,
        HalfLength = 1
    };

    [Fact]
    public void RayPointingAtNegativeXSide_ShouldHit()
    {
        var localOrigin = new Vector3(-2, 0, 0);
        var localDirection = new Vector3(1, 0, 0);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAtNegativeYSide_ShouldHit()
    {
        var localOrigin = new Vector3(0, -2, 0);
        var localDirection = new Vector3(0, 1, 0);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAtHypotenuse_ShouldHit()
    {
        var localOrigin = new Vector3(2, 2, 0);
        var localDirection = new Vector3(-1, -1, 0);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAtPositiveZSide_ShouldHit()
    {
        var localOrigin = new Vector3(0, 0, 3);
        var localDirection = new Vector3(0, 0, -1);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAtNegativeZSide_ShouldHit()
    {
        var localOrigin = new Vector3(0, 0, -3);
        var localDirection = new Vector3(0, 0, 1);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAwayFromNegativeXSide_ShouldNotHit()
    {
        var localOrigin = new Vector3(-2, 0, 0);
        var localDirection = new Vector3(-1, 0, 0);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingBelowNegativeYSide_ShouldNotHit()
    {
        var localOrigin = new Vector3(0, -2, 0);
        var localDirection = new Vector3(0, -1, 0);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAwayFromHypotenuse_ShouldNotHit()
    {
        var localOrigin = new Vector3(2, 2, 0);
        var localDirection = new Vector3(1, 1, 0);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingNextToHypotenuse_ShouldNotHit()
    {
        var localOrigin = new Vector3(2, 0, 0);
        var localDirection = new Vector3(0, 1, 0);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }
}
