using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    //Individual pair testers are designed to be used outside of the narrow phase. They need to be usable for queries and such, so all necessary data must be gathered externally.
    public struct SphereTriangleTester : IPairTester<SphereWide, TriangleWide, Convex1ContactManifoldWide>
    {
        public void Test(ref SphereWide a, ref TriangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, out Convex1ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Test(ref SphereWide a, ref TriangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, out Convex1ContactManifoldWide manifold)
        {
            manifold = new Convex1ContactManifoldWide();
        }

        public void Test(ref SphereWide a, ref TriangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, out Convex1ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }
    
    public class SphereTriangleCollisionTask : CollisionTask
    {
        public SphereTriangleCollisionTask()
        {
            BatchSize = 32;
            ShapeTypeIndexA = default(Triangle).TypeId;
            ShapeTypeIndexB = default(Box).TypeId;
        }

        public unsafe override void ExecuteBatch<TCallbacks>(ref UntypedList batch, ref CollisionBatcher<TCallbacks> batcher)
        {
            ConvexCollisionTaskCommon.ExecuteBatch
                <TCallbacks,
                Sphere, SphereWide, Triangle, TriangleWide, OneOrientationTestPairWide<Sphere, SphereWide, Triangle, TriangleWide>,
                Convex1ContactManifoldWide, SphereTriangleTester>(ref batch, ref batcher);
        }
    }
}
