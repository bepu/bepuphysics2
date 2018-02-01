//using BepuPhysics.Collidables;
//using System;
//using System.Collections.Generic;
//using System.Diagnostics;
//using System.Numerics;
//using System.Runtime.CompilerServices;
//using System.Runtime.InteropServices;
//using System.Text;

//namespace BepuPhysics.CollisionDetection.CollisionTasks
//{
//    //Individual pair testers are designed to be used outside of the narrow phase. They need to be usable for queries and such, so all necessary data must be gathered externally.
//    public struct CapsulePairTester
//    {
//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static void Test(
//            ref CapsuleWide a, ref CapsuleWide b,
//            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
//            out ConvexContact2ManifoldWide manifold)
//        {
//            //The closest points between two capsules are the closest points of their internal line segments, plus their radius offset.
//            //If you were dealing with two ideal rigid capsules, it's possible to generate two contacts only when their line segments are perfectly parallel.
//            //However, in practice, there are many times where it's beneficial for stability to be a little more aggressive about generating the second contact.
//            //Consider the case where two capsules are deeply penetrating with not-entirely-parallel axes, or when two capsules are approaching each other at high speed.
//            //In the penetrating case, two contacts help separate the objects in a smooth way rather than ping-ponging the contact position all over the place.
//            //In the separating-but-approaching case, two contacts give the speculative solver a way to stop inappropriate angular impulses that a single contact would permit.

//            //Both of these are primarily concerns in the context of mostly-coplanar capsules.
//            //If the line segments are considerably off-plane, then there's really no point in having two contacts, and choosing reasonable ones becomes difficult.
  

//            //For the single contact case, find the closest point between the two capsule internal line segments.



//            //The contact for a sphere-capsule pair is based on the closest point of the sphere center to the capsule internal line segment.
//            QuaternionWide.TransformUnitXY(ref capsuleOrientation, out var x, out var y);
//            Vector3Wide.Dot(ref y, ref sphereToCapsule, out var t);
//            t = Vector.Min(capsule.HalfLength, Vector.Max(-capsule.HalfLength, -t));
//            Vector3Wide.Scale(ref y, ref t, out var capsuleLocalClosestPointOnLineSegment);

//            Vector3Wide.Add(ref sphereToCapsule, ref capsuleLocalClosestPointOnLineSegment, out var sphereToInternalSegment);
//            Vector3Wide.Length(ref sphereToInternalSegment, out var internalDistance);
//            //Note that the normal points from B to A by convention. Here, the sphere is A, the capsule is B, so the normalization requires a negation.
//            var inverseDistance = new Vector<float>(-1f) / internalDistance;
//            Vector3Wide.Scale(ref sphereToInternalSegment, ref inverseDistance, out contactNormal);
//            var normalIsValid = Vector.GreaterThan(internalDistance, Vector<float>.Zero);
//            //If the center of the sphere is on the internal line segment, then choose a direction on the plane defined by the capsule's up vector.
//            //We computed one such candidate earlier. Note that we could usually get away with choosing a completely arbitrary direction, but 
//            //going through the extra effort to compute a true local horizontal direction avoids some nasty corner case surprises if a user is trying
//            //to do something like manually resolving collisions or other query-based logic.
//            //A cheaper option would be to simply use the y axis as the normal. That's known to be suboptimal, but if we don't guarantee minimum penetration depth, that's totally fine.
//            //My guess is that computing x will be so cheap as to be irrelevant.
//            Vector3Wide.ConditionalSelect(ref normalIsValid, ref contactNormal, ref x, out contactNormal);
//            depth = sphere + capsule.Radius - internalDistance;

//            //The contact position relative to object A (the sphere) is computed as the average of the extreme point along the normal toward the opposing shape on each shape, averaged.
//            //For capsule-sphere, this can be computed from the normal and depth.
//            var negativeOffsetFromSphere = depth * 0.5f - sphere;
//            Vector3Wide.Scale(ref contactNormal, ref negativeOffsetFromSphere, out relativeContactPosition);
//        }
//    }

//    public class CapsulePairCollisionTask : CollisionTask
//    {
//        public CapsulePairCollisionTask()
//        {
//            BatchSize = 32;
//            ShapeTypeIndexA = default(Capsule).TypeId;
//            ShapeTypeIndexB = default(Capsule).TypeId;
//        }


//        //Every single collision task type will mirror this general layout.
//        public unsafe override void ExecuteBatch<TContinuations, TFilters>(ref UntypedList batch, ref StreamingBatcher batcher, ref TContinuations continuations, ref TFilters filters)
//        {
//            ref var start = ref Unsafe.As<byte, RigidPair<Sphere, Capsule>>(ref batch.Buffer[0]);
//            var manifolds = stackalloc ContactManifold[Vector<float>.Count];
//            var trustMeThisManifoldIsTotallyInitialized = &manifolds;
//            //Note that this is hoisted out of the loop. The notification function is not allowed to modify the manifold passed in, so we can do it once up front.
//            for (int i = 0; i < Vector<float>.Count; ++i)
//            {
//                manifolds[i].SetConvexityAndCount(1, true);
//            }
//            Vector<float> spheres;
//            CapsuleWide capsules;
//            Vector3Wide spherePosition;
//            Vector3Wide capsulePosition;
//            QuaternionWide capsuleOrientation;
//            Vector3Wide contactNormal, contactPosition, offsetB;
//            Vector<float> depth;
//            Vector<int> flipMask;

//            for (int i = 0; i < batch.Count; i += Vector<float>.Count)
//            {
//                ref var bundleStart = ref Unsafe.Add(ref start, i);
//                int countInBundle = batch.Count - i;
//                if (countInBundle > Vector<float>.Count)
//                    countInBundle = Vector<float>.Count;

//                //TODO: In the future, once we have some more codegen options, we should try to change this- probably into an intrinsic.
//                //Or maybe an explicit AOS->(AOSOA|SOA) transition during add time- in other words, we never store the AOS representation.
//                //(That's still a gather, just moving it around a bit in the hope that it is more centralized.)
//                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref spheres, ref bundleStart.A.Radius, countInBundle);
//                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref capsules.Radius, ref bundleStart.B.Radius, countInBundle);
//                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref capsules.HalfLength, ref bundleStart.B.HalfLength, countInBundle);
//                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref spherePosition.X, ref bundleStart.Shared.PoseA.Position.X, countInBundle);
//                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref spherePosition.Y, ref bundleStart.Shared.PoseA.Position.Y, countInBundle);
//                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref spherePosition.Z, ref bundleStart.Shared.PoseA.Position.Z, countInBundle);
//                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref capsulePosition.X, ref bundleStart.Shared.PoseB.Position.X, countInBundle);
//                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref capsulePosition.Y, ref bundleStart.Shared.PoseB.Position.Y, countInBundle);
//                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref capsulePosition.Z, ref bundleStart.Shared.PoseB.Position.Z, countInBundle);
//                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref capsuleOrientation.X, ref bundleStart.Shared.PoseB.Orientation.X, countInBundle);
//                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref capsuleOrientation.Y, ref bundleStart.Shared.PoseB.Orientation.Y, countInBundle);
//                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref capsuleOrientation.Z, ref bundleStart.Shared.PoseB.Orientation.Z, countInBundle);
//                GatherScatter.Gather<float, RigidPair<Sphere, Capsule>>(ref capsuleOrientation.W, ref bundleStart.Shared.PoseB.Orientation.W, countInBundle);
//                GatherScatter.Gather<int, RigidPair<Sphere, Capsule>>(ref flipMask, ref bundleStart.Shared.FlipMask, countInBundle);

//                Vector3Wide.Subtract(ref capsulePosition, ref spherePosition, out offsetB);
//                SphereCapsuleTester.Test(ref spheres, ref capsules, ref offsetB, ref capsuleOrientation, out contactPosition, out contactNormal, out depth);

//                //Flip back any contacts associated with pairs which had to be flipped for shape order.
//                Vector3Wide.Negate(ref contactNormal, out var flippedNormal);
//                Vector3Wide.Subtract(ref contactPosition, ref offsetB, out var flippedContactPosition);
//                Vector3Wide.Negate(ref offsetB, out var flippedOffsetB);
//                Vector3Wide.ConditionalSelect(ref flipMask, ref flippedNormal, ref contactNormal, out contactNormal);
//                Vector3Wide.ConditionalSelect(ref flipMask, ref flippedContactPosition, ref contactPosition, out contactPosition);
//                Vector3Wide.ConditionalSelect(ref flipMask, ref flippedOffsetB, ref offsetB, out offsetB);

//                GatherScatter.Scatter<float, ContactManifold>(ref offsetB.X, ref manifolds->OffsetB.X, countInBundle);
//                GatherScatter.Scatter<float, ContactManifold>(ref offsetB.Y, ref manifolds->OffsetB.Y, countInBundle);
//                GatherScatter.Scatter<float, ContactManifold>(ref offsetB.Z, ref manifolds->OffsetB.Z, countInBundle);
//                GatherScatter.Scatter<float, ContactManifold>(ref contactPosition.X, ref manifolds->Offset0.X, countInBundle);
//                GatherScatter.Scatter<float, ContactManifold>(ref contactPosition.Y, ref manifolds->Offset0.Y, countInBundle);
//                GatherScatter.Scatter<float, ContactManifold>(ref contactPosition.Z, ref manifolds->Offset0.Z, countInBundle);
//                GatherScatter.Scatter<float, ContactManifold>(ref depth, ref manifolds->Depth0, countInBundle);
//                GatherScatter.Scatter<float, ContactManifold>(ref contactNormal.X, ref manifolds->Normal0.X, countInBundle);
//                GatherScatter.Scatter<float, ContactManifold>(ref contactNormal.Y, ref manifolds->Normal0.Y, countInBundle);
//                GatherScatter.Scatter<float, ContactManifold>(ref contactNormal.Z, ref manifolds->Normal0.Z, countInBundle);
//                for (int j = 0; j < countInBundle; ++j)
//                {
//                    continuations.Notify(Unsafe.Add(ref bundleStart, j).Shared.Continuation, manifolds + j);
//                    Debug.Assert(manifolds[j].ContactCount == 1 && manifolds[j].Convex, "The notify function should not modify the provided manifold reference.");
//                }
//            }

//        }
//    }
//}
