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
    public struct CapsulePairTester
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Test(
            ref CapsuleWide a, ref CapsuleWide b,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out ConvexContact2ManifoldWide manifold)
        {
            //Compute the closest points between the two line segments. No clamping to begin with.
            //We want to minimize distance = ||(a + da * ta) - (b + db * tb)||.
            //Taking the derivative with respect to ta and doing some algebra (taking into account ||da|| == ||db|| == 1) to solve for ta yields:
            //ta = (da * (b - a) + (db * (a - b)) * (da * db)) / (1 - ((da * db) * (da * db))        
            QuaternionWide.TransformUnitY(ref orientationA, out var da);
            QuaternionWide.TransformUnitY(ref orientationB, out var db);
            Vector3Wide.Dot(ref da, ref offsetB, out var daOffsetB);
            Vector3Wide.Dot(ref db, ref offsetB, out var dbOffsetB);
            Vector3Wide.Dot(ref da, ref db, out var dadb);
            //Note potential division by zero when the capsule axes are parallel. We conditional select on dadb later to avoid keeping bad results.
            var ta = (daOffsetB - dbOffsetB * dadb) / (Vector<float>.One - dadb * dadb);
            //tb = ta * (da * db) - db * (b - a)
            var tb = ta * dadb - dbOffsetB;

            //We cannot simply clamp the ta and tb values to the capsule line segments. Instead, project each line segment onto the other line segment, clamping against the target's interval.
            //That new clamped projected interval is the valid solution space on that line segment. We can clamp the t value by that interval to get the correctly bounded solution.
            //The projected intervals are:
            //B onto A: +-BHalfLength * (da * db) + da * offsetB
            //A onto B: +-AHalfLength * (da * db) - db * offsetB
            var absdadb = Vector.Abs(dadb);
            var bOntoAOffset = b.HalfLength * absdadb;
            var aOntoBOffset = a.HalfLength * absdadb;
            var aMin = Vector.Max(-a.HalfLength, daOffsetB - bOntoAOffset);
            var aMax = Vector.Min(a.HalfLength, daOffsetB + bOntoAOffset);
            var bMin = Vector.Max(-b.HalfLength, -bOntoAOffset - dbOffsetB);
            var bMax = Vector.Min(b.HalfLength, bOntoAOffset - dbOffsetB);
            ta = Vector.Min(Vector.Max(ta, aMin), aMax);
            tb = Vector.Min(Vector.Max(tb, bMin), bMax);

            //In the event that the axes are parallel, we select the midpoints of the potential solution region as the closest points.
            var parallel = Vector.LessThan(absdadb, new Vector<float>(1e-7f));
            var half = new Vector<float>(0.5f);
            ta = Vector.ConditionalSelect(parallel, half * (aMin + aMax), ta);
            tb = Vector.ConditionalSelect(parallel, half * (bMin + bMax), tb);

            Vector3Wide.Scale(ref da, ref ta, out var closestPointOnA);
            Vector3Wide.Scale(ref db, ref tb, out var closestPointOnB);
            Vector3Wide.Add(ref closestPointOnB, ref offsetB, out closestPointOnB);
            //Note that normals are calibrated to point from B to A by convention.
            Vector3Wide.Subtract(ref closestPointOnA, ref closestPointOnB, out manifold.ContactNormal);
            Vector3Wide.Length(ref manifold.ContactNormal, out var normalLength);
            var inverseLength = Vector<float>.One / normalLength;
            Vector3Wide.Scale(ref manifold.ContactNormal, ref inverseLength, out manifold.ContactNormal);

            manifold.Count = Vector<int>.One;
        }
    }

    public class CapsulePairCollisionTask : CollisionTask
    {
        public CapsulePairCollisionTask()
        {
            BatchSize = 32;
            ShapeTypeIndexA = default(Capsule).TypeId;
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
            Vector3Wide contactNormal, contactPosition, offsetB;
            Vector<float> depth;
            Vector<int> flipMask;

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
                GatherScatter.Gather<int, RigidPair<Sphere, Capsule>>(ref flipMask, ref bundleStart.Shared.FlipMask, countInBundle);

                Vector3Wide.Subtract(ref capsulePosition, ref spherePosition, out offsetB);
                SphereCapsuleTester.Test(ref spheres, ref capsules, ref offsetB, ref capsuleOrientation, out contactPosition, out contactNormal, out depth);

                //Flip back any contacts associated with pairs which had to be flipped for shape order.
                Vector3Wide.Negate(ref contactNormal, out var flippedNormal);
                Vector3Wide.Subtract(ref contactPosition, ref offsetB, out var flippedContactPosition);
                Vector3Wide.Negate(ref offsetB, out var flippedOffsetB);
                Vector3Wide.ConditionalSelect(ref flipMask, ref flippedNormal, ref contactNormal, out contactNormal);
                Vector3Wide.ConditionalSelect(ref flipMask, ref flippedContactPosition, ref contactPosition, out contactPosition);
                Vector3Wide.ConditionalSelect(ref flipMask, ref flippedOffsetB, ref offsetB, out offsetB);

                GatherScatter.Scatter<float, ContactManifold>(ref offsetB.X, ref manifolds->OffsetB.X, countInBundle);
                GatherScatter.Scatter<float, ContactManifold>(ref offsetB.Y, ref manifolds->OffsetB.Y, countInBundle);
                GatherScatter.Scatter<float, ContactManifold>(ref offsetB.Z, ref manifolds->OffsetB.Z, countInBundle);
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
