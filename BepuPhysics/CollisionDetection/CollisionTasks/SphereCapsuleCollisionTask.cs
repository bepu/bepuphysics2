using BepuPhysics.Collidables;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    //Individual pair testers are designed to be used outside of the narrow phase. They need to be usable for queries and such, so all necessary data must be gathered externally.
    public struct SphereCapsuleTester : IPairTester<SphereWide, CapsuleWide, Convex1ContactManifoldWide>
    {
        public void Test(ref SphereWide a, ref CapsuleWide b, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, out Convex1ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Test(ref SphereWide a, ref CapsuleWide b, ref Vector3Wide offsetB, ref QuaternionWide orientationB, out Convex1ContactManifoldWide manifold)
        {
            //The contact for a sphere-capsule pair is based on the closest point of the sphere center to the capsule internal line segment.
            QuaternionWide.TransformUnitXY(ref orientationB, out var x, out var y);
            Vector3Wide.Dot(ref y, ref offsetB, out var t);
            t = Vector.Min(b.HalfLength, Vector.Max(-b.HalfLength, -t));
            Vector3Wide.Scale(ref y, ref t, out var capsuleLocalClosestPointOnLineSegment);

            Vector3Wide.Add(ref offsetB, ref capsuleLocalClosestPointOnLineSegment, out var sphereToInternalSegment);
            Vector3Wide.Length(ref sphereToInternalSegment, out var internalDistance);
            //Note that the normal points from B to A by convention. Here, the sphere is A, the capsule is B, so the normalization requires a negation.
            var inverseDistance = new Vector<float>(-1f) / internalDistance;
            Vector3Wide.Scale(ref sphereToInternalSegment, ref inverseDistance, out manifold.Normal);
            var normalIsValid = Vector.GreaterThan(internalDistance, Vector<float>.Zero);
            //If the center of the sphere is on the internal line segment, then choose a direction on the plane defined by the capsule's up vector.
            //We computed one such candidate earlier. Note that we could usually get away with choosing a completely arbitrary direction, but 
            //going through the extra effort to compute a true local horizontal direction avoids some nasty corner case surprises if a user is trying
            //to do something like manually resolving collisions or other query-based logic.
            //A cheaper option would be to simply use the y axis as the normal. That's known to be suboptimal, but if we don't guarantee minimum penetration depth, that's totally fine.
            //My guess is that computing x will be so cheap as to be irrelevant.
            Vector3Wide.ConditionalSelect(ref normalIsValid, ref manifold.Normal, ref x, out manifold.Normal);
            manifold.Depth = a.Radius + b.Radius - internalDistance;

            //The contact position relative to object A (the sphere) is computed as the average of the extreme point along the normal toward the opposing shape on each shape, averaged.
            //For capsule-sphere, this can be computed from the normal and depth.
            var negativeOffsetFromSphere = manifold.Depth * 0.5f - a.Radius;
            Vector3Wide.Scale(ref manifold.Normal, ref negativeOffsetFromSphere, out manifold.OffsetA0);
        }

        public void Test(ref SphereWide a, ref CapsuleWide b, ref Vector3Wide offsetB, out Convex1ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }

    public class SphereCapsuleCollisionTask : CollisionTask
    {
        public SphereCapsuleCollisionTask()
        {
            BatchSize = 32;
            ShapeTypeIndexA = default(Sphere).TypeId;
            ShapeTypeIndexB = default(Capsule).TypeId;
        }


        //Every single collision task type will mirror this general layout.
        public unsafe override void ExecuteBatch<TContinuations, TFilters>(ref UntypedList batch, ref StreamingBatcher batcher, ref TContinuations continuations, ref TFilters filters)
        {
            CollisionTaskCommon.ExecuteBatch
                <TContinuations, TFilters,
                Sphere, SphereWide, Capsule, CapsuleWide, OneOrientationTestPairWide<Sphere, SphereWide, Capsule, CapsuleWide>,
                Convex1ContactManifoldWide, SphereCapsuleTester>(ref batch, ref batcher, ref continuations, ref filters);
        }
    }
}
