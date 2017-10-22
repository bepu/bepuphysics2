using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Constraints
{
    /// <summary>
    /// A constraint's body references. Stored separately from the iteration data since it is accessed by both the prestep and solve.
    /// Two address streams isn't much of a problem for prefetching.
    /// </summary>
    public struct TwoBodyReferences
    {
        public Vector<int> IndexA;
        public Vector<int> IndexB;
    }

    /// <summary>
    /// Prestep, warm start and solve iteration functions for a constraint type.
    /// </summary>
    /// <typeparam name="TPrestepData">Type of the prestep data used by the constraint.</typeparam>
    /// <typeparam name="TAccumulatedImpulse">Type of the accumulated impulses used by the constraint.</typeparam>
    /// <typeparam name="TProjection">Type of the projection to input.</typeparam>
    public interface IConstraintFunctions<TPrestepData, TProjection, TAccumulatedImpulse>
    {
        void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref TPrestepData prestepData, out TProjection projection);
        void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref TProjection projection, ref TAccumulatedImpulse accumulatedImpulse);
        void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref TProjection projection, ref TAccumulatedImpulse accumulatedImpulse);
    }

    //Not a big fan of complex generic-filled inheritance hierarchies, but this is the shortest evolutionary step to removing duplicates.
    //There are some other options if this inheritance hierarchy gets out of control.
    /// <summary>
    /// Shared implementation across all two body constraints.
    /// </summary>
    public abstract class TwoBodyTypeBatch<TPrestepData, TProjection, TAccumulatedImpulse, TConstraintFunctions>
        : TypeBatch<TwoBodyReferences, TPrestepData, TProjection, TAccumulatedImpulse>
        where TConstraintFunctions : struct, IConstraintFunctions<TPrestepData, TProjection, TAccumulatedImpulse>
    {
        public sealed override int BodiesPerConstraint => 2;

        public sealed override void EnumerateConnectedBodyIndices<TEnumerator>(int indexInTypeBatch, ref TEnumerator enumerator)
        {
            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var constraintBundleIndex, out var constraintInnerIndex);

            ref var indexA = ref GatherScatter.Get(ref BodyReferences[constraintBundleIndex].IndexA, constraintInnerIndex);
            ref var indexB = ref Unsafe.Add(ref indexA, Vector<int>.Count);

            //Note that the variables are ref locals! This is important for correctness, because every execution of LoopBody could result in a swap.
            //Ref locals aren't the only solution, but if you ever change this, make sure you account for the potential mutation in the enumerator.
            enumerator.LoopBody(indexA);
            enumerator.LoopBody(indexB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetSortKey(int constraintIndex, ref Buffer<TwoBodyReferences> bodyReferences)
        {
            BundleIndexing.GetBundleIndices(constraintIndex, out var bundleIndex, out var innerIndex);
            ref var bundleReferences = ref bodyReferences[bundleIndex];
            //We sort based on the body references within the constraint. 
            //Sort based on the smaller body index in a constraint. Note that it is impossible for there to be two references to the same body within a constraint batch, 
            //so there's no need to worry about the case where the comparison is equal.
            ref var indexA = ref GatherScatter.Get(ref bundleReferences.IndexA, innerIndex);
            ref var indexB = ref Unsafe.Add(ref indexA, Vector<int>.Count);
            return indexA < indexB ? indexA : indexB;
            //TODO: It is conceivable that another sorting key heuristic would beat this one. This completely ignores the second connection and makes it very unlikely
            //that it could converge to whatever globally optimal layout exists. It's a little tricky to come up with good heuristics, though- many will end up 
            //batching constraints which relate to wildly different bodies. Sorting by the minimum at least guarantees that two adjacent constraints will be as close as they can be
            //in one way. 

            //In practice, we approach within about 5-10% of the optimum using the above sorting heuristic and the current incremental body optimizer.

            //It's not immediately clear that ANY local comparison based sort will be able to do as well as some form of global optimizer that maximizes
            //the 'closeness', which drops to zero once accesses would leave the cache line. This is made more complicated by the AOSOA layout- most likely
            //such a heuristic would need to score based on whether the bodies are in the same bundle. So, for example, two constraints get one 'close' point for each 
            //shared body bundle.
            //(Since you would only be optimizing within type batches, the fact that different types have different body counts wouldn't be an issue. They would
            //only ever be compared against other constraints of the same type.)
            //Even if you can't guarantee the next constraint is going to have bodies that are in cache, if you can generally lift the number of constraints
            //that end up used quite a few times in L1/L2, there is probably a nice benefit to be had. That would suggest 'wider' optimizations rather than bundle-specific ones.
            //All of these global techniques get into some nasty O complexities, but there may be heuristics which can approach them- sort of like BVH SAH sweep builders.
            //Especially incremental ones, like the refinement we use in the dynamic BVH broadphase.
        }


        internal sealed override void GenerateSortKeysAndCopyReferences(
            int bundleStart, int localBundleStart, int bundleCount,
            int constraintStart, int localConstraintStart, int constraintCount,
            ref int firstSortKey, ref int firstSourceIndex, ref RawBuffer bodyReferencesCache)
        {
            for (int i = 0; i < constraintCount; ++i)
            {
                Unsafe.Add(ref firstSourceIndex, i) = localConstraintStart + i;
                Unsafe.Add(ref firstSortKey, i) = GetSortKey(constraintStart + i, ref BodyReferences);
            }
            var typedBodyReferencesCache = bodyReferencesCache.As<TwoBodyReferences>();
            BodyReferences.CopyTo(bundleStart, ref typedBodyReferencesCache, localBundleStart, bundleCount);
        }


        internal sealed override void CopyToCache(
            int bundleStart, int localBundleStart, int bundleCount,
            int constraintStart, int localConstraintStart, int constraintCount,
            ref Buffer<int> indexToHandleCache, ref RawBuffer prestepCache, ref RawBuffer accumulatedImpulsesCache)
        {
            IndexToHandle.CopyTo(constraintStart, ref indexToHandleCache, localConstraintStart, constraintCount);
            var typedPrestepCache = prestepCache.As<TPrestepData>();
            var typedAccumulatedImpulsesCache = accumulatedImpulsesCache.As<TAccumulatedImpulse>();
            PrestepData.CopyTo(bundleStart, ref typedPrestepCache, localBundleStart, bundleCount);
            AccumulatedImpulses.CopyTo(bundleStart, ref typedAccumulatedImpulsesCache, localBundleStart, bundleCount);
        }


        internal sealed override void Regather(int constraintStart, int constraintCount, ref int firstSourceIndex,
            ref Buffer<int> indexToHandleCache, ref RawBuffer bodyReferencesCache, ref RawBuffer prestepCache, ref RawBuffer accumulatedImpulsesCache,
            ref Buffer<ConstraintLocation> handlesToConstraints)
        {
            var typedBodyReferencesCache = bodyReferencesCache.As<TwoBodyReferences>();
            var typedPrestepCache = prestepCache.As<TPrestepData>();
            var typedAccumulatedImpulsesCache = accumulatedImpulsesCache.As<TAccumulatedImpulse>();
            for (int i = 0; i < constraintCount; ++i)
            {
                var sourceIndex = Unsafe.Add(ref firstSourceIndex, i);
                var targetIndex = constraintStart + i;
                //Note that we do not bother checking whether the source and target are the same.
                //The cost of the branch is large enough in comparison to the frequency of its usefulness that it only helps in practically static situations.
                //Also, its maximum benefit is quite small.
                BundleIndexing.GetBundleIndices(sourceIndex, out var sourceBundle, out var sourceInner);
                BundleIndexing.GetBundleIndices(targetIndex, out var targetBundle, out var targetInner);

                Move(ref typedBodyReferencesCache[sourceBundle], ref typedPrestepCache[sourceBundle], ref typedAccumulatedImpulsesCache[sourceBundle],
                    sourceInner, indexToHandleCache[sourceIndex],
                    targetBundle, targetInner, targetIndex, ref handlesToConstraints);

            }
        }

        internal override sealed void VerifySortRegion(int bundleStartIndex, int constraintCount, ref Buffer<int> sortedKeys, ref Buffer<int> sortedSourceIndices)
        {
            var previousKey = -1;
            var baseIndex = bundleStartIndex << BundleIndexing.VectorShift;
            for (int i = 0; i < constraintCount; ++i)
            {
                var sourceIndex = sortedSourceIndices[i];
                var targetIndex = baseIndex + i;
                var key = GetSortKey(baseIndex + i, ref BodyReferences);
                Debug.Assert(key > previousKey, "After the sort and swap completes, all constraints should be in order.");
                Debug.Assert(key == sortedKeys[i], "After the swap goes through, the rederived sort keys should match the previously sorted ones.");
                previousKey = key;

            }
        }
        internal override int GetBodyIndexInstanceCount(int bodyIndex)
        {
            //This is a pure debug function; performance does not matter.
            int count = 0;
            for (int i = 0; i < BundleCount; ++i)
            {
                var bundleSize = Math.Min(Vector<float>.Count, constraintCount - (i << BundleIndexing.VectorShift));
                for (int j = 0; j < bundleSize; ++j)
                {
                    if (GatherScatter.Get(ref BodyReferences[i].IndexA, j) == bodyIndex)
                        ++count;
                    Debug.Assert(count <= 1);
                    if (GatherScatter.Get(ref BodyReferences[i].IndexB, j) == bodyIndex)
                        ++count;
                    Debug.Assert(count <= 1);
                }
            }
            return count;
        }

        //The following covers the common loop logic for all two body constraints. Each iteration invokes the warm start function type.
        //This abstraction should, in theory, have zero overhead if the implementation of the interface is in a struct with aggressive inlining.

        //By providing the overrides at this level, the concrete implementation (assuming it inherits from one of the prestep-providing variants)
        //only has to specify *type* arguments associated with the interface-implementing struct-delegates. It's going to look very strange, but it's low overhead
        //and minimizes per-type duplication.

  
        public override void Prestep(Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            ref var prestepBase = ref PrestepData[0];
            ref var bodyReferencesBase = ref BodyReferences[0];
            ref var projectionBase = ref Projection[0];
            var function = default(TConstraintFunctions);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref Unsafe.Add(ref prestepBase, i);
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                function.Prestep(bodies, ref Unsafe.Add(ref bodyReferencesBase, i), GetCountInBundle(i), dt, inverseDt, ref prestep, out projection);
            }
        }

        public override void WarmStart(ref Buffer<BodyVelocity> bodyVelocities, int startBundle, int exclusiveEndBundle)
        {
            ref var bodyReferencesBase = ref BodyReferences[0];
            ref var accumulatedImpulsesBase = ref AccumulatedImpulses[0];
            ref var projectionBase = ref Projection[0];
            var function = default(TConstraintFunctions);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                int count = GetCountInBundle(i);
                GatherScatter.GatherVelocities(ref bodyVelocities, ref bodyReferences, count, out var wsvA, out var wsvB);
                function.WarmStart(ref wsvA, ref wsvB, ref projection, ref accumulatedImpulses);
                GatherScatter.ScatterVelocities(ref bodyVelocities, ref bodyReferences, count, ref wsvA, ref wsvB);
            }
        }

        public override void SolveIteration(ref Buffer<BodyVelocity> bodyVelocities, int startBundle, int exclusiveEndBundle)
        {
            ref var projectionBase = ref Projection[0];
            ref var bodyReferencesBase = ref BodyReferences[0];
            ref var accumulatedImpulsesBase = ref AccumulatedImpulses[0];
            var function = default(TConstraintFunctions);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                int count = GetCountInBundle(i);
                GatherScatter.GatherVelocities(ref bodyVelocities, ref bodyReferences, count, out var wsvA, out var wsvB);
                function.Solve(ref wsvA, ref wsvB, ref projection, ref accumulatedImpulses);
                GatherScatter.ScatterVelocities(ref bodyVelocities, ref bodyReferences, count, ref wsvA, ref wsvB);
            }
        }

    }
}
