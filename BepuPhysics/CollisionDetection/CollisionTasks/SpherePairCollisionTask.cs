using BepuPhysics.Collidables;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    //Individual pair testers are designed to be used outside of the narrow phase. They need to be usable for queries and such, so all necessary data must be gathered externally.
    public struct SpherePairTester : IPairTester<SphereWide, SphereWide, Convex1ContactManifoldWide>
    {
        public void Test(ref SphereWide a, ref SphereWide b, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, out Convex1ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref SphereWide a, ref SphereWide b, ref Vector3Wide offsetB, ref QuaternionWide orientationB, out Convex1ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Test(ref SphereWide a, ref SphereWide b, ref Vector3Wide offsetB, out Convex1ContactManifoldWide manifold)
        {
            Vector3Wide.Length(ref offsetB, out var centerDistance);
            //Note the negative 1. By convention, the normal points from B to A.
            var inverseDistance = new Vector<float>(-1f) / centerDistance;
            Vector3Wide.Scale(ref offsetB, ref inverseDistance, out manifold.Normal);
            var normalIsValid = Vector.GreaterThan(centerDistance, Vector<float>.Zero);
            //Arbitrarily choose the (0,1,0) if the two spheres are in the same position. Any unit length vector is equally valid.
            manifold.Normal.X = Vector.ConditionalSelect(normalIsValid, manifold.Normal.X, Vector<float>.Zero);
            manifold.Normal.Y = Vector.ConditionalSelect(normalIsValid, manifold.Normal.Y, Vector<float>.One);
            manifold.Normal.Z = Vector.ConditionalSelect(normalIsValid, manifold.Normal.Z, Vector<float>.Zero);
            manifold.Depth = a.Radius + b.Radius - centerDistance;

            //The contact position relative to object A is computed as the average of the extreme point along the normal toward the opposing sphere on each sphere, averaged.
            var negativeOffsetFromA = manifold.Depth * 0.5f - a.Radius;
            Vector3Wide.Scale(ref manifold.Normal, ref negativeOffsetFromA, out manifold.OffsetA0);
        }
    }

    public class SpherePairCollisionTask : CollisionTask
    {
        public SpherePairCollisionTask()
        {
            BatchSize = 32;
            ShapeTypeIndexA = default(Sphere).TypeId;
            ShapeTypeIndexB = ShapeTypeIndexA;
        }
        
        public unsafe override void ExecuteBatch<TContinuations, TFilters>(ref UntypedList batch, ref StreamingBatcher batcher, ref TContinuations continuations, ref TFilters filters)
        {
            CollisionTaskCommon.ExecuteBatch
                <TContinuations, TFilters, 
                Sphere, SphereWide, Sphere, SphereWide, NoOrientationTestPairWide<Sphere, SphereWide, Sphere, SphereWide>,
                Convex1ContactManifoldWide, SpherePairTester>(ref batch, ref batcher, ref continuations, ref filters);
        }
    }
}
