using BepuPhysics.Collidables;
using System.Numerics;

namespace BepuPhysicsTests.RayCasts;

public class RayCastPyramid : RayCastBase<Pyramid, PyramidWide>
{
    protected override Pyramid CreateShape() => new Pyramid
    {
        HalfWidth = 1,
        HalfLength = 1,
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
    public void RayPointingAtPositiveXSide_ShouldHit()
    {
        var localOrigin = new Vector3(3, 0.5f, 0);
        var localDirection = new Vector3(-1, 0, 0);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAtNegativeXSide_ShouldHit()
    {
        var localOrigin = new Vector3(-3, 0.5f, 0);
        var localDirection = new Vector3(1, 0, 0);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAtPositiveZSide_ShouldHit()
    {
        var localOrigin = new Vector3(0, 0.5f, 3);
        var localDirection = new Vector3(0, 0, -1);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAtNegativeZSide_ShouldHit()
    {
        var localOrigin = new Vector3(0, 0.5f, -3);
        var localDirection = new Vector3(0, 0, 1);
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
        var localOrigin = new Vector3(3, 0.5f, 1.1f);
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
