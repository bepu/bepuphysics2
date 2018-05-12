using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    public interface ISupportFinder<TShape, TShapeWide> where TShape : IConvexShape where TShapeWide : IShapeWide<TShape>
    {
        void ComputeSupport(ref TShapeWide shape, ref Matrix3x3Wide orientation, ref Vector3Wide direction, out Vector3Wide support);
    }

    public struct GJKDistanceTester<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB> : IPairDistanceTester<TShapeWideA, TShapeWideB>
        where TShapeA : IConvexShape where TShapeB : IConvexShape
        where TShapeWideA : IShapeWide<TShapeA> where TShapeWideB : IShapeWide<TShapeB>
        where TSupportFinderA : struct, ISupportFinder<TShapeA, TShapeWideA>
        where TSupportFinderB : struct, ISupportFinder<TShapeB, TShapeWideB>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void SampleMinkowskiDifference(
            ref TShapeWideA a, ref Matrix3x3Wide rA, ref TSupportFinderA supportFinderA,
            ref TShapeWideB b, ref Matrix3x3Wide rB, ref TSupportFinderB supportFinderB, ref Vector3Wide offsetB, ref Vector3Wide direction,
            out Vector3Wide supportA, out Vector3Wide support)
        {
            supportFinderA.ComputeSupport(ref a, ref rA, ref direction, out supportA);
            Vector3Wide.Negate(ref direction, out var negatedDirection);
            supportFinderA.ComputeSupport(ref a, ref rA, ref negatedDirection, out var supportB);
            Vector3Wide.Add(ref supportB, ref offsetB, out supportB);
            Vector3Wide.Subtract(ref supportA, ref supportB, out support);

        }

        struct Simplex
        {
            public Vector3Wide V0;
            public Vector3Wide V1;
            public Vector3Wide V2;
            public Vector3Wide V3;
            //Contributing points from one object are tracked to create the closest points.
            public Vector3Wide V0A;
            public Vector3Wide V1A;
            public Vector3Wide V2A;
            public Vector3Wide V3A;
            public Vector<int> Count;
        }

       [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void Append(ref Vector<int> mask, ref Simplex simplex, ref Vector3Wide vA, ref Vector3Wide v)
        {
            for (int i = 0; i < Vector<float>.Count; ++i)
            {
                if(mask[i] == 0)
                {
                    ref var count = ref Unsafe.Add(ref GatherScatter.GetFirst(ref simplex.Count), i);
                    ref var vTargetSlot = ref GatherScatter.GetOffsetInstance(ref Unsafe.Add(ref simplex.V0, count), i);
                    GatherScatter.GetFirst(ref vTargetSlot.X) = v.X[i];
                    GatherScatter.GetFirst(ref vTargetSlot.Y) = v.Y[i];
                    GatherScatter.GetFirst(ref vTargetSlot.Z) = v.Z[i];
                    ref var vATargetSlot = ref GatherScatter.GetOffsetInstance(ref Unsafe.Add(ref simplex.V0A, count), i);
                    GatherScatter.GetFirst(ref vATargetSlot.X) = vA.X[i];
                    GatherScatter.GetFirst(ref vATargetSlot.Y) = vA.Y[i];
                    GatherScatter.GetFirst(ref vATargetSlot.Z) = vA.Z[i];
                    ++count;
                }
            }
        }

        void FindClosestPoint(ref Simplex simplex, out Vector3Wide closestA, out Vector3Wide closest)
        {
            //There are four possible cases to consider for the simplex: point, line, triangle, and tetrahedron.
        }

        public void Test(ref TShapeWideA a, ref TShapeWideB b, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Vector<int> intersected, out Vector<float> distance, out Vector3Wide closestA, out Vector3Wide normal)
        {
            Matrix3x3Wide.CreateFromQuaternion(ref orientationA, out var rA);
            Matrix3x3Wide.CreateFromQuaternion(ref orientationB, out var rB);
            var supportFinderA = default(TSupportFinderA);
            var supportFinderB = default(TSupportFinderB);
            Simplex simplex;
            //TODO: It would be pretty easy to initialize to a triangle or tetrahedron. Might be worth it.
            SampleMinkowskiDifference(ref a, ref rA, ref supportFinderA, ref b, ref rB, ref supportFinderB, ref offsetB, ref offsetB, out simplex.V0A, out simplex.V0);
            simplex.Count = new Vector<int>(1);

            //GJK is a pretty branchy algorithm that doesn't map perfectly to widely vectorized implementations. We'll be using an SPMD model- if any SIMD lane needs to continue 
            //working, all lanes continue to work, and the unnecessary lane results are discarded.
            //In the context of the engine, distance tests are often quite coherent (by virtue of being used with sweeps), so we shouldn't often see pathologically low occupancy.
            var mask = Vector<int>.Zero;
            intersected = Vector<int>.Zero;
            while (true)
            {
                FindClosestPoint(ref simplex, out var simplexClosestA, out var simplexClosest);

                var simplexContainsOrigin = Vector.BitwiseAnd(
                    Vector.BitwiseAnd(
                        Vector.Equals(simplexClosest.X, Vector<float>.Zero),
                        Vector.Equals(simplexClosest.Y, Vector<float>.Zero)),
                    Vector.Equals(simplexClosest.Z, Vector<float>.Zero));
                //For any lane that has not yet terminated, and which has an intersecting simplex, terminate with intersection.
                intersected = Vector.BitwiseOr(intersected, Vector.AndNot(simplexContainsOrigin, mask));
                mask = Vector.BitwiseOr(mask, simplexContainsOrigin);
                if (Vector.EqualsAll(mask, -Vector<int>.One))
                    break;

                Vector3Wide.Negate(ref simplexClosest, out var sampleDirection);
                SampleMinkowskiDifference(ref a, ref rA, ref supportFinderA, ref b, ref rB, ref supportFinderB, ref offsetB, ref sampleDirection, out var vA, out var v);

                //If the newly sampled is no better than the simplex-identified closest point, the lane is done.
                Vector3Wide.Subtract(ref v, ref simplexClosest, out var progressOffset);
                Vector3Wide.Dot(ref progressOffset, ref sampleDirection, out var progress);
                var noProgressMade = Vector.LessThanOrEqual(progress, Vector<float>.Zero);
                //Any still-executing lane that failed to make progress is now verified to be nonintersecting.
                //Note that the default initialized state is nonintersecting, so we don't have to set the flag explicitly here.
                //Note that lanes which previously terminated are excluded from closest/normal updates.      
                Vector3Wide.ConditionalSelect(ref mask, ref closestA, ref simplexClosestA, out closestA);
                Vector3Wide.ConditionalSelect(ref mask, ref normal, ref simplexClosest, out normal);
                mask = Vector.BitwiseOr(mask, noProgressMade);
                if (Vector.EqualsAll(mask, -Vector<int>.One))
                    break;

                Append(ref mask, ref simplex, ref vA, ref v);
            }
            //The normal gathered during the loop was the unnormalized offset.
            //Note that this normalization will cover lanes which weren't separated- that's fine. Valid lanes are described by the intersected flag.
            Vector3Wide.Normalize(ref normal, out normal);
        }
    }
}
