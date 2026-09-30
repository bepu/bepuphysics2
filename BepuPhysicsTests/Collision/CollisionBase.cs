using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public abstract class CollisionBase<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TManifoldWide, TPairTester>
    where TShapeA : unmanaged, IShape where TShapeB : unmanaged, IShape
    where TShapeWideA : unmanaged, IShapeWide<TShapeA> where TShapeWideB : unmanaged, IShapeWide<TShapeB>
    where TPairTester : struct, IPairTester<TShapeWideA, TShapeWideB, TManifoldWide>
    where TManifoldWide : unmanaged, IContactManifoldWide
{
    protected abstract TShapeA CreateShapeA();
    protected abstract TShapeB CreateShapeB();

    protected static bool Colliding(in TShapeA shapeA, in TShapeB shapeB, float speculativeMargin, Vector3 offsetB, Quaternion orientationA, Quaternion orientationB)
    {
        var shapeWideA = default(TShapeWideA);
        shapeWideA.Broadcast(in shapeA);
        var shapeWideB = default(TShapeWideB);
        shapeWideB.Broadcast(in shapeB);
        var speculativeMarginWide = new Vector<float>(speculativeMargin);
        Vector3Wide.Broadcast(offsetB, out var offsetWideB);
        QuaternionWide.Broadcast(orientationA, out var orientationWideA);
        QuaternionWide.Broadcast(orientationB, out var orientationWideB);

        TPairTester.Test(ref shapeWideA, ref shapeWideB, ref speculativeMarginWide, ref offsetWideB, ref orientationWideA, ref orientationWideB, 1, out var manifoldWide);

        var manifold = default(ConvexContactManifold);
        manifoldWide.ReadFirst(offsetWideB, ref manifold);

        return manifold.Count > 0;
    }

    public void AssertColliding(Vector3 offsetB, Quaternion orientationA, Quaternion orientationB, float speculativeMargin)
    {
        Assert.True(Colliding(CreateShapeA(), CreateShapeB(), speculativeMargin, offsetB, orientationA, orientationB));
    }

    public void AssertColliding(Vector3 offsetB, Quaternion orientationA, Quaternion orientationB)
    {
        AssertColliding(offsetB, orientationA, orientationB, 0f);
    }

    public void AssertColliding(Vector3 offsetB, float speculativeMargin)
    {
        AssertColliding(offsetB, Quaternion.Identity, Quaternion.Identity, speculativeMargin);
    }

    public void AssertColliding(Vector3 offsetB)
    {
        AssertColliding(offsetB, Quaternion.Identity, Quaternion.Identity, 0f);
    }

    public void AssertSeparated(Vector3 offsetB, Quaternion orientationA, Quaternion orientationB, float speculativeMargin)
    {
        Assert.False(Colliding(CreateShapeA(), CreateShapeB(), speculativeMargin, offsetB, orientationA, orientationB));
    }

    public void AssertSeparated(Vector3 offsetB, Quaternion orientationA, Quaternion orientationB)
    {
        AssertSeparated(offsetB, orientationA, orientationB, 0f);
    }

    public void AssertSeparated(Vector3 offsetB, float speculativeMargin)
    {
        AssertSeparated(offsetB, Quaternion.Identity, Quaternion.Identity, speculativeMargin);
    }

    public void AssertSeparated(Vector3 offsetB)
    {
        AssertSeparated(offsetB, Quaternion.Identity, Quaternion.Identity, 0f);
    }
}
