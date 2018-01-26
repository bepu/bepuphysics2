using BepuPhysics.Collidables;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    //Individual pair testers are designed to be used outside of the narrow phase. They need to be usable for queries and such, so all necessary data must be gathered externally.
    public struct SpherePairTester
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Test(
            ref Vector<float> radiiA, ref Vector<float> radiiB,
            ref Vector3Wide relativePositionB,
            out Vector3Wide relativeContactPosition, out Vector3Wide contactNormal, out Vector<float> depth)
        {
            Vector3Wide.Length(ref relativePositionB, out var centerDistance);
            //Note the negative 1. By convention, the normal points from B to A.
            var inverseDistance = new Vector<float>(-1f) / centerDistance;
            Vector3Wide.Scale(ref relativePositionB, ref inverseDistance, out contactNormal);
            var normalIsValid = Vector.GreaterThan(centerDistance, Vector<float>.Zero);
            //Arbitrarily choose the (0,1,0) if the two spheres are in the same position. Any unit length vector is equally valid.
            contactNormal.X = Vector.ConditionalSelect(normalIsValid, contactNormal.X, Vector<float>.Zero);
            contactNormal.Y = Vector.ConditionalSelect(normalIsValid, contactNormal.Y, Vector<float>.One);
            contactNormal.Z = Vector.ConditionalSelect(normalIsValid, contactNormal.Z, Vector<float>.Zero);
            depth = radiiA + radiiB - centerDistance;

            //The contact position relative to object A is computed as the average of the extreme point along the normal toward the opposing sphere on each sphere, averaged.
            var negativeOffsetFromA = depth * 0.5f - radiiA;
            Vector3Wide.Scale(ref contactNormal, ref negativeOffsetFromA, out relativeContactPosition);
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


        //Every single collision task type will mirror this general layout.
        public unsafe override void ExecuteBatch<TContinuations, TFilters>(ref UntypedList batch, ref StreamingBatcher batcher, ref TContinuations continuations, ref TFilters filters)
        {
            ref var start = ref Unsafe.As<byte, RigidPair<Sphere, Sphere>>(ref batch.Buffer[0]);
            var manifolds = stackalloc ContactManifold[Vector<float>.Count];
            var trustMeThisManifoldIsTotallyInitialized = &manifolds;
            //Note that this is hoisted out of the loop. The notification function is not allowed to modify the manifold passed in, so we can do it once up front.
            for (int i = 0; i < Vector<float>.Count; ++i)
            {
                manifolds[i].SetConvexityAndCount(1, true);
            }
            Vector<float> radiiA;
            Vector<float> radiiB;
            Vector3Wide positionA;
            Vector3Wide positionB;
            Vector3Wide contactNormal, contactPosition, relativePosition;
            Vector<float> depth;

            for (int i = 0; i < batch.Count; i += Vector<float>.Count)
            {
                ref var bundleStart = ref Unsafe.Add(ref start, i);
                int countInBundle = batch.Count - i;
                if (countInBundle > Vector<float>.Count)
                    countInBundle = Vector<float>.Count;

                //TODO: In the future, once we have some more codegen options, we should try to change this- probably into an intrinsic.
                //Or maybe an explicit AOS->(AOSOA|SOA) transition during add time- in other words, we never store the AOS representation.
                //(That's still a gather, just moving it around a bit in the hope that it is more centralized.)
                GatherScatter.Gather<float, RigidPair<Sphere, Sphere>>(ref radiiA, ref bundleStart.A.Radius, countInBundle);
                GatherScatter.Gather<float, RigidPair<Sphere, Sphere>>(ref radiiB, ref bundleStart.B.Radius, countInBundle);
                GatherScatter.Gather<float, RigidPair<Sphere, Sphere>>(ref positionA.X, ref bundleStart.Shared.PoseA.Position.X, countInBundle);
                GatherScatter.Gather<float, RigidPair<Sphere, Sphere>>(ref positionA.Y, ref bundleStart.Shared.PoseA.Position.Y, countInBundle);
                GatherScatter.Gather<float, RigidPair<Sphere, Sphere>>(ref positionA.Z, ref bundleStart.Shared.PoseA.Position.Z, countInBundle);
                GatherScatter.Gather<float, RigidPair<Sphere, Sphere>>(ref positionB.X, ref bundleStart.Shared.PoseB.Position.X, countInBundle);
                GatherScatter.Gather<float, RigidPair<Sphere, Sphere>>(ref positionB.Y, ref bundleStart.Shared.PoseB.Position.Y, countInBundle);
                GatherScatter.Gather<float, RigidPair<Sphere, Sphere>>(ref positionB.Z, ref bundleStart.Shared.PoseB.Position.Z, countInBundle);

                Vector3Wide.Subtract(ref positionB, ref positionA, out relativePosition);
                SpherePairTester.Test(ref radiiA, ref radiiB, ref relativePosition, out contactPosition, out contactNormal, out depth);
                
                GatherScatter.Scatter<float, ContactManifold>(ref relativePosition.X, ref manifolds->OffsetB.X, countInBundle);
                GatherScatter.Scatter<float, ContactManifold>(ref relativePosition.Y, ref manifolds->OffsetB.Y, countInBundle);
                GatherScatter.Scatter<float, ContactManifold>(ref relativePosition.Z, ref manifolds->OffsetB.Z, countInBundle);
                GatherScatter.Scatter<float, ContactManifold>(ref contactPosition.X, ref manifolds->Offset0.X, countInBundle);
                GatherScatter.Scatter<float, ContactManifold>(ref contactPosition.Y, ref manifolds->Offset0.Y, countInBundle);
                GatherScatter.Scatter<float, ContactManifold>(ref contactPosition.Z, ref manifolds->Offset0.Z, countInBundle);
                GatherScatter.Scatter<float, ContactManifold>(ref depth, ref manifolds->Depth0, countInBundle);
                GatherScatter.Scatter<float, ContactManifold>(ref contactNormal.X, ref manifolds->Normal0.X, countInBundle);
                GatherScatter.Scatter<float, ContactManifold>(ref contactNormal.Y, ref manifolds->Normal0.Y, countInBundle);
                GatherScatter.Scatter<float, ContactManifold>(ref contactNormal.Z, ref manifolds->Normal0.Z, countInBundle);
                for (int j = 0; j < countInBundle; ++j)
                {
                    continuations.Notify(Unsafe.Add(ref bundleStart, j).Shared.Continuation, manifolds + j);
                    Debug.Assert(manifolds[j].ContactCount == 1 && manifolds[j].Convex, "The notify function should not modify the provided manifold reference.");
                }
            }
        }

    }
}
