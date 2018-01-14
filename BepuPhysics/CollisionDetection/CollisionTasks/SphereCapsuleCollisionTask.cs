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
    public struct SphereCapsuleTester
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Test(
            ref Vector<float> sphere, ref CapsuleWide capsule,
            ref Vector3Wide capsuleToSphere, ref QuaternionWide capsuleOrientation,
            out Vector3Wide relativeContactPosition, out Vector3Wide contactNormal, out Vector<float> depth)
        {
            //The contact for a sphere-capsule pair is based on the closest point of the sphere center to the capsule internal line segment.
            QuaternionWide.TransformUnitXY(ref capsuleOrientation, out var x, out var y);
            Vector3Wide.Dot(ref y, ref capsuleToSphere, out var t);
            t = Vector.Min(capsule.HalfLength, Vector.Max(-capsule.HalfLength, t));
            Vector3Wide.Scale(ref y, ref t, out var closestPointOnLineSegment);

            Vector3Wide.Subtract(ref capsuleToSphere, ref closestPointOnLineSegment, out var offset);
            //Note that the normal points from B to A by convention. Here, the sphere is A, the capsule is B.
            Vector3Wide.Length(ref offset, out var internalDistance);
            var inverseDistance = Vector<float>.One / internalDistance;            
            Vector3Wide.Scale(ref offset, ref inverseDistance, out contactNormal);
            var normalIsValid = Vector.GreaterThan(internalDistance, Vector<float>.Zero);
            //If the center of the sphere is on the internal line segment, then choose a direction on the plane defined by the capsule's up vector.
            //We computed one such candidate earlier. Note that we could usually get away with choosing a completely arbitrary direction, but 
            //going through the extra effort to compute a true local horizontal direction avoids some nasty corner case surprises if a user is trying
            //to do something like manually resolving collisions or other query-based logic.
            //A cheaper option would be to simply use the y axis as the normal. That's known to be suboptimal, but if we don't guarantee minimum penetration depth, that's totally fine.
            //My guess is that computing x will be so cheap as to be irrelevant.
            Vector3Wide.ConditionalSelect(ref normalIsValid, ref contactNormal, ref x, out contactNormal);
            depth = sphere + capsule.Radius - internalDistance;

            //The contact position relative to object A is computed as the average of the extreme point along the normal toward the opposing shape on each shape, averaged.
            //In other words:
            //relativeContactPosition = (capsuleToSphere + (-normal) * sphereRadius + closestPointOnLineSegment + normal * capsuleRadius) / 2
            //relativeContactPosition = (capsuleToSphere + closestPointOnLineSegment + normal * (capsuleRadius - sphereRadius)) / 2
            var radiusDifference = capsule.Radius - sphere;
            Vector3Wide.Scale(ref contactNormal, ref radiusDifference, out relativeContactPosition);
            Vector3Wide.Add(ref relativeContactPosition, ref capsuleToSphere, out relativeContactPosition);
            Vector3Wide.Add(ref relativeContactPosition, ref closestPointOnLineSegment, out relativeContactPosition);
            var scale = new Vector<float>(0.5f);
            Vector3Wide.Scale(ref relativeContactPosition, ref scale, out relativeContactPosition);
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
            ref var start = ref Unsafe.As<byte, RigidPair<Sphere, Capsule>>(ref batch.Buffer[0]);
            var manifolds = stackalloc ContactManifold[Vector<float>.Count];
            var trustMeThisManifoldIsTotallyInitialized = &manifolds;
            //Note that this is hoisted out of the loop. The notification function is not allowed to modify the manifold passed in, so we can do it once up front.
            for (int i = 0; i < Vector<float>.Count; ++i)
            {
                manifolds[i].SetConvexityAndCount(1, true);
            }
            Vector<float> spheres;
            CapsuleWide capsules;
            Vector3Wide spherePosition;
            Vector3Wide capsulePosition;
            QuaternionWide capsuleOrientation;
            Vector3Wide contactNormal, contactPosition, capsuleToSphere;
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
                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref spheres, ref bundleStart.A.Radius, countInBundle);
                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref capsules.Radius, ref bundleStart.B.Radius, countInBundle);
                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref capsules.HalfLength, ref bundleStart.B.HalfLength, countInBundle);
                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref spherePosition.X, ref bundleStart.Shared.PoseA.Position.X, countInBundle);
                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref spherePosition.Y, ref bundleStart.Shared.PoseA.Position.Y, countInBundle);
                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref spherePosition.Z, ref bundleStart.Shared.PoseA.Position.Z, countInBundle);
                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref capsulePosition.X, ref bundleStart.Shared.PoseB.Position.X, countInBundle);
                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref capsulePosition.Y, ref bundleStart.Shared.PoseB.Position.Y, countInBundle);
                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref capsulePosition.Z, ref bundleStart.Shared.PoseB.Position.Z, countInBundle);
                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref capsuleOrientation.X, ref bundleStart.Shared.PoseB.Orientation.X, countInBundle);
                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref capsuleOrientation.Y, ref bundleStart.Shared.PoseB.Orientation.Y, countInBundle);
                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref capsuleOrientation.Z, ref bundleStart.Shared.PoseB.Orientation.Z, countInBundle);
                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref capsuleOrientation.W, ref bundleStart.Shared.PoseB.Orientation.W, countInBundle);

                Vector3Wide.Subtract(ref spherePosition, ref capsulePosition, out capsuleToSphere);
                SphereCapsuleTester.Test(ref spheres, ref capsules, ref capsuleToSphere, ref capsuleOrientation, out contactPosition, out contactNormal, out depth);

                GatherScatter.Scatter<float, ContactManifold>(ref capsuleToSphere.X, ref manifolds->OffsetB.X, countInBundle);
                GatherScatter.Scatter<float, ContactManifold>(ref capsuleToSphere.Y, ref manifolds->OffsetB.Y, countInBundle);
                GatherScatter.Scatter<float, ContactManifold>(ref capsuleToSphere.Z, ref manifolds->OffsetB.Z, countInBundle);
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
