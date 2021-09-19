using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public unsafe struct OverlapQueryForPair
    {
        public void* Container;
        public Vector3 Min;
        public Vector3 Max;
    }

    public unsafe interface IBoundsQueryableCompound
    {
        unsafe void FindLocalOverlaps<TOverlaps, TSubpairOverlaps>(ref Buffer<OverlapQueryForPair> pairs, BufferPool pool, Shapes shapes, ref TOverlaps overlaps)
            where TOverlaps : struct, ICollisionTaskOverlaps<TSubpairOverlaps>
            where TSubpairOverlaps : struct, ICollisionTaskSubpairOverlaps;

        unsafe void FindLocalOverlaps<TOverlaps>(in Vector3 min, in Vector3 max, in Vector3 sweep, float maximumT, BufferPool pool, Shapes shapes, void* overlaps)
            where TOverlaps : ICollisionTaskSubpairOverlaps;
    }
    public interface IOverlapTestingOptions
    {
        /// <summary>
        /// Returns true if pairs should epsilon-expand the query bounds for the sake of a later reduction process, like MeshReduction.
        /// This helps avoid situations where contacts that are just barely contained within the bounding box can get filtered out incorrectly by the MeshReduction's heuristics.
        /// </summary>
        bool EpsilonExpandBounds { get; }
    }
    public struct UseEpsilonBoundsExpansion : IOverlapTestingOptions { public readonly bool EpsilonExpandBounds => true; }
    public struct DontUseEpsilonBoundsExpansion : IOverlapTestingOptions { public readonly bool EpsilonExpandBounds => false; }
    public interface IConvexCompoundOverlapFinder
    {
        void FindLocalOverlaps<TOverlapTestingOptions>(ref Buffer<BoundsTestedPair> pairs, int pairCount, BufferPool pool, Shapes shapes, float dt, out ConvexCompoundTaskOverlaps overlaps) where TOverlapTestingOptions : unmanaged, IOverlapTestingOptions;
    }

    public struct ConvexCompoundOverlapFinder<TConvex, TConvexWide, TCompound> : IConvexCompoundOverlapFinder
        where TConvex : struct, IConvexShape
        where TConvexWide : struct, IShapeWide<TConvex>
        where TCompound : struct, IBoundsQueryableCompound
    {
        public unsafe void FindLocalOverlaps<TOverlapTestingOptions>(ref Buffer<BoundsTestedPair> pairs, int pairCount, BufferPool pool, Shapes shapes, float dt, out ConvexCompoundTaskOverlaps overlaps) where TOverlapTestingOptions : unmanaged, IOverlapTestingOptions
        {
            overlaps = new ConvexCompoundTaskOverlaps(pool, pairCount);
            ref var pairsToTest = ref overlaps.subpairQueries;
            Vector3Wide offsetB = default;
            QuaternionWide orientationA = default;
            QuaternionWide orientationB = default;
            Vector3Wide relativeLinearVelocityA = default;
            Vector3Wide angularVelocityA = default;
            Vector3Wide angularVelocityB = default;
            Vector<float> maximumAllowedExpansion = default;
            TConvexWide convexWide = default;
            Unsafe.SkipInit(out TOverlapTestingOptions overlapTestingOptions);
            if (convexWide.InternalAllocationSize > 0)
            {
                var memory = stackalloc byte[convexWide.InternalAllocationSize];
                convexWide.Initialize(new RawBuffer(memory, convexWide.InternalAllocationSize));
            }
            for (int i = 0; i < pairCount; i += Vector<float>.Count)
            {
                var count = pairCount - i;
                if (count > Vector<float>.Count)
                    count = Vector<float>.Count;

                //Compute the local bounding boxes using wide operations for the expansion work.
                //Doing quite a bit of gather work (and still quite a bit of scalar work). Very possible that a scalar path could win. TODO: test that.
                //TODO: Now that we're free of NS2.0, the transpose could be intrinsified.
                for (int j = 0; j < count; ++j)
                {
                    var pairIndex = i + j;
                    ref var pair = ref pairs[pairIndex];
                    pairsToTest[pairIndex].Container = pair.B;
                    Vector3Wide.WriteFirst(pair.OffsetB, ref GatherScatter.GetOffsetInstance(ref offsetB, j));
                    QuaternionWide.WriteFirst(pair.OrientationA, ref GatherScatter.GetOffsetInstance(ref orientationA, j));
                    QuaternionWide.WriteFirst(pair.OrientationB, ref GatherScatter.GetOffsetInstance(ref orientationB, j));
                    Vector3Wide.WriteFirst(pair.RelativeLinearVelocityA, ref GatherScatter.GetOffsetInstance(ref relativeLinearVelocityA, j));
                    Vector3Wide.WriteFirst(pair.AngularVelocityA, ref GatherScatter.GetOffsetInstance(ref angularVelocityA, j));
                    Vector3Wide.WriteFirst(pair.AngularVelocityB, ref GatherScatter.GetOffsetInstance(ref angularVelocityB, j));
                    Unsafe.Add(ref Unsafe.As<Vector<float>, float>(ref maximumAllowedExpansion), j) = pair.MaximumExpansion;

                    convexWide.WriteSlot(j, Unsafe.AsRef<TConvex>(pair.A));
                }

                QuaternionWide.Conjugate(orientationB, out var inverseOrientationB);
                QuaternionWide.TransformWithoutOverlap(offsetB, inverseOrientationB, out var localOffsetB);
                QuaternionWide.ConcatenateWithoutOverlap(orientationA, inverseOrientationB, out var localOrientationA);
                QuaternionWide.TransformWithoutOverlap(relativeLinearVelocityA, inverseOrientationB, out var localRelativeLinearVelocityA);

                convexWide.GetBounds(ref localOrientationA, count, out var maximumRadius, out var maximumAngularExpansion, out var min, out var max);

                Vector3Wide.Negate(localOffsetB, out var localPositionA);
                BoundingBoxHelpers.ExpandLocalBoundingBoxes(ref min, ref max, Vector<float>.Zero, localPositionA, localRelativeLinearVelocityA, angularVelocityA, angularVelocityB, dt,
                    maximumRadius, maximumAngularExpansion, maximumAllowedExpansion);
                if (overlapTestingOptions.EpsilonExpandBounds)
                {
                    BoundingBoxHelpers.EpsilonExpandLocalBoundingBoxes(maximumRadius, ref min, ref max);
                }

                for (int j = 0; j < count; ++j)
                {
                    ref var pairToTest = ref pairsToTest[i + j];
                    Vector3Wide.ReadSlot(ref min, j, out pairToTest.Min);
                    Vector3Wide.ReadSlot(ref max, j, out pairToTest.Max);
                }
            }
            //The choice of instance here is irrelevant.
            Unsafe.AsRef<TCompound>(pairsToTest[0].Container).FindLocalOverlaps<ConvexCompoundTaskOverlaps, ConvexCompoundOverlaps>(ref pairsToTest, pool, shapes, ref overlaps);

        }

    }
}
