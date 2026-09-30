using BepuPhysics.Collidables;
using System.Numerics;

namespace BepuPhysicsTests.RayCasts;

public class RayCastBox : RayCastBase<Box, BoxWide>
{
    protected override Box CreateShape() => new Box(1, 2, 3);

    [Fact]
    public void RayPointingAtBoxCenter_ShouldHit()
    {
        var localOrigin = new Vector3(0, 0, -5);
        var localDirection = new Vector3(0, 0, 1);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAtTopOfBox_ShouldHit()
    {
        var localOrigin = new Vector3(0, 5, 0);
        var localDirection = new Vector3(0, -1, 0);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAtSideOfBox_ShouldHit()
    {
        var localOrigin = new Vector3(3, 0, 0);
        var localDirection = new Vector3(-1, 0, 0);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAwayFromBox_ShouldNotHit()
    {
        var localOrigin = new Vector3(0, 0, -5);
        var localDirection = new Vector3(0, 0, -1);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingNextToBox_ShouldNotHit()
    {
        var localOrigin = new Vector3(0.6f, 0, -5);
        var localDirection = new Vector3(0, 0, 1);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }
}
