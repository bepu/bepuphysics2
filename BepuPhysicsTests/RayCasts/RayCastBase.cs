using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using System.Numerics;

namespace BepuPhysicsTests.RayCasts;

public abstract class RayCastBase<TShape, TShapeWide>
    where TShape : unmanaged, IConvexShape
    where TShapeWide : unmanaged, IShapeWide<TShape>
{
    protected abstract TShape CreateShape();

    private bool RayTest_Individual(in RigidPose pose, Vector3 origin, Vector3 direction)
    {
        return CreateShape().RayTest(pose, origin, direction, out _, out _);
    }

    private bool RayTest_Vector(in RigidPose pose, Vector3 origin, Vector3 direction)
    {
        var shapeWide = default(TShapeWide);
        shapeWide.Broadcast(CreateShape());

        RigidPoseWide.Broadcast(pose, out RigidPoseWide poseWide);
        Vector3Wide.Broadcast(origin, out Vector3Wide originWide);
        Vector3Wide.Broadcast(direction, out Vector3Wide directionWide);
        var rayWide = new RayWide()
        {
            Origin = originWide,
            Direction = directionWide
        };

        shapeWide.RayTest(ref poseWide, ref rayWide, out var intersectedWide, out _, out _);
        return intersectedWide[0] != 0;
    }

    public void AssertRayHit(in RigidPose pose, Vector3 origin, Vector3 direction)
    {
        Assert.True(RayTest_Individual(pose, origin, direction));
        Assert.True(RayTest_Vector(pose, origin, direction));
    }

    public void AssertRayHit_Local(in RigidPose pose, Vector3 localOrigin, Vector3 localDirection)
    {
        var origin = pose.Position + QuaternionEx.Transform(localOrigin, pose.Orientation);
        var direction = QuaternionEx.Transform(localDirection, pose.Orientation);
        AssertRayHit(pose, origin, direction);
    }

    public void AssertRayHit_Local(Vector3 localOrigin, Vector3 localDirection)
    {
        AssertRayHit(RigidPose.Identity, localOrigin, localDirection);
    }

    public void AssertRayNotHit(in RigidPose pose, Vector3 origin, Vector3 direction)
    {
        Assert.False(RayTest_Individual(pose, origin, direction));
        Assert.False(RayTest_Vector(pose, origin, direction));
    }

    public void AssertRayNotHit_Local(in RigidPose pose, Vector3 localOrigin, Vector3 localDirection)
    {
        var origin = pose.Position + QuaternionEx.Transform(localOrigin, pose.Orientation);
        var direction = QuaternionEx.Transform(localDirection, pose.Orientation);
        AssertRayNotHit(pose, origin, direction);
    }

    public void AssertRayNotHit_Local(Vector3 localOrigin, Vector3 localDirection)
    {
        AssertRayNotHit(RigidPose.Identity, localOrigin, localDirection);
    }
}
