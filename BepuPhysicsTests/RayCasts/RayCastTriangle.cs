using BepuPhysics.Collidables;
using System.Numerics;

namespace BepuPhysicsTests.RayCasts;

public class RayCastTriangle : RayCastBase<Triangle, TriangleWide>
{
    protected override Triangle CreateShape() => new Triangle(
        new Vector3(-1, 0, -1),
        new Vector3(1, 0, -1),
        new Vector3(-1, 0, 1));

    [Fact]
    public void RayPointingAtTriangleFront_ShouldHit()
    {
        var localOrigin = new Vector3(-0.25f, 5, -0.25f);
        var localDirection = new Vector3(0, -1, 0);
        AssertRayHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAtTriangleBack_ShouldNotHit()
    {
        var localOrigin = new Vector3(-0.25f, -5, -0.25f);
        var localDirection = new Vector3(0, 1, 0);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingAwayFromTriangle_ShouldNotHit()
    {
        var localOrigin = new Vector3(-0.25f, 5, -0.25f);
        var localDirection = new Vector3(0, 1, 0);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }

    [Fact]
    public void RayPointingOutsideTriangle_ShouldNotHit()
    {
        var localOrigin = new Vector3(0.9f, 5, 0.9f);
        var localDirection = new Vector3(0, -1, 0);
        AssertRayNotHit_Local(localOrigin, localDirection);
    }
}
