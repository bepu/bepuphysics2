using BepuPhysics.Collidables;
using System.Numerics;

namespace BepuPhysicsTests.RayCasts;

public class RayCastRectangle : RayCastBase<Rectangle, RectangleWide>
{
    protected override Rectangle CreateShape() => new Rectangle { Width = 1, Length = 3 };

    [Fact]
    public void RayPointingAtRectangleCenter_ShouldHit()
    {
        var localOrigin = new Vector3(0, 5, 0);
        var localDirection = new Vector3(0, -1, 0);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAwayFromRectangle_ShouldNotHit()
    {
        var localOrigin = new Vector3(0, 5, 0);
        var localDirection = new Vector3(0, 1, 0);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAtRectangleFromWrongSide_ShouldNotHit()
    {
        var localOrigin = new Vector3(0, -5, 0);
        var localDirection = new Vector3(0, 1, 0);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }
}
