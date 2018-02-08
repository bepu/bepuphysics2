using BepuPhysics.Collidables;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    //Individual pair testers are designed to be used outside of the narrow phase. They need to be usable for queries and such, so all necessary data must be gathered externally.
    public struct CapsulePairTester : IPairTester<CapsuleWide, CapsuleWide, Convex1ContactManifoldWide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Test(
            ref CapsuleWide a, ref CapsuleWide b,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Convex1ContactManifoldWide manifold)
        {
            //Compute the closest points between the two line segments. No clamping to begin with.
            //We want to minimize distance = ||(a + da * ta) - (b + db * tb)||.
            //Taking the derivative with respect to ta and doing some algebra (taking into account ||da|| == ||db|| == 1) to solve for ta yields:
            //ta = (da * (b - a) + (db * (a - b)) * (da * db)) / (1 - ((da * db) * (da * db))        
            QuaternionWide.TransformUnitXY(ref orientationA, out var xa, out var da);
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
            var aMin = Vector.Max(-a.HalfLength, Vector.Min(a.HalfLength, daOffsetB - bOntoAOffset));
            var aMax = Vector.Min(a.HalfLength, Vector.Max(-a.HalfLength, daOffsetB + bOntoAOffset));
            var bMin = Vector.Max(-b.HalfLength, Vector.Min(b.HalfLength, -bOntoAOffset - dbOffsetB));
            var bMax = Vector.Min(b.HalfLength, Vector.Max(-b.HalfLength, bOntoAOffset - dbOffsetB));
            ta = Vector.Min(Vector.Max(ta, aMin), aMax);
            tb = Vector.Min(Vector.Max(tb, bMin), bMax);

            //In the event that the axes are parallel, we select the midpoints of the potential solution region as the closest points.
            var parallel = Vector.GreaterThan(absdadb, new Vector<float>(1f - 1e-7f));
            var half = new Vector<float>(0.5f);
            ta = Vector.ConditionalSelect(parallel, half * (aMin + aMax), ta);
            tb = Vector.ConditionalSelect(parallel, half * (bMin + bMax), tb);

            Vector3Wide.Scale(ref da, ref ta, out var closestPointOnA);
            Vector3Wide.Scale(ref db, ref tb, out var closestPointOnB);
            Vector3Wide.Add(ref closestPointOnB, ref offsetB, out closestPointOnB);
            //Note that normals are calibrated to point from B to A by convention.
            Vector3Wide.Subtract(ref closestPointOnA, ref closestPointOnB, out manifold.Normal);
            Vector3Wide.Length(ref manifold.Normal, out var distance);
            var inverseDistance = Vector<float>.One / distance;
            Vector3Wide.Scale(ref manifold.Normal, ref inverseDistance, out manifold.Normal);
            //In the event that the line segments are touching, the normal doesn't exist and we need an alternative. Any direction along the local horizontal (XZ) plane of either capsule
            //is valid. (Normals along the local Y axes are not guaranteed to be as quick of a path to separation due to nonzero line length.)
            var normalIsValid = Vector.GreaterThan(distance, new Vector<float>(1e-7f));
            Vector3Wide.ConditionalSelect(ref normalIsValid, ref manifold.Normal, ref xa, out manifold.Normal);

            manifold.Depth = a.Radius + b.Radius - distance;
            var negativeOffsetFromA = manifold.Depth * 0.5f - a.Radius;
            Vector3Wide.Scale(ref manifold.Normal, ref negativeOffsetFromA, out manifold.OffsetA0);
            Vector3Wide.Add(ref manifold.OffsetA0, ref closestPointOnA, out manifold.OffsetA0);

            //TODO: In the future, you may want to actually take advantage of two contacts in the parallel or coplanar axes case. That can help avoid instabilities that could arise from
            //a contact jumping across the length of the segment.
            //This is primarily useful in some pretty weird corner cases, so we're ignoring it for the early versions.
            //As far as an implementation goes: replace parallel case midpoint contact with two contacts at the borders of the B-on-A region. Can generalize to nonparallel but coplanar case:
            //depth of each contact computed by unprojecting the B-on-A endpoints back onto B, and then dotting those offsets with the normal.
            //(That unprojection process is just two line-plane tests with shared calculations.)
        }

        public void Test(ref CapsuleWide a, ref CapsuleWide b, ref Vector3Wide offsetB, ref QuaternionWide orientationB, out Convex1ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref CapsuleWide a, ref CapsuleWide b, ref Vector3Wide offsetB, out Convex1ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
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
            CollisionTaskCommon.ExecuteBatch
                <TContinuations, TFilters,
                Capsule, CapsuleWide, Capsule, CapsuleWide, UnflippableTestPairWide<Capsule, CapsuleWide, Capsule, CapsuleWide>,
                Convex1ContactManifoldWide, CapsulePairTester>(ref batch, ref batcher, ref continuations, ref filters);
        }
    }
}
