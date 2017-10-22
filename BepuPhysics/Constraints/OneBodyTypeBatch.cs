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
    /// Prestep, warm start and solve iteration functions for a constraint type.
    /// </summary>
    /// <typeparam name="TPrestepData">Type of the prestep data used by the constraint.</typeparam>
    /// <typeparam name="TAccumulatedImpulse">Type of the accumulated impulses used by the constraint.</typeparam>
    /// <typeparam name="TProjection">Type of the projection to input.</typeparam>
    public interface IOneBodyConstraintFunctions<TPrestepData, TProjection, TAccumulatedImpulse>
    {
        void Prestep(Bodies bodies, ref Vector<int> bodyReferences, int count, float dt, float inverseDt, ref TPrestepData prestepData, out TProjection projection);
        void WarmStart(ref BodyVelocities velocity, ref TProjection projection, ref TAccumulatedImpulse accumulatedImpulse);
        void Solve(ref BodyVelocities velocity, ref TProjection projection, ref TAccumulatedImpulse accumulatedImpulse);
    }

    //Not a big fan of complex generic-filled inheritance hierarchies, but this is the shortest evolutionary step to removing duplicates.
    //There are some other options if this inheritance hierarchy gets out of control.
    /// <summary>
    /// Shared implementation across all two body constraints.
    /// </summary>
    public abstract class OneBodyTypeBatch<TPrestepData, TProjection, TAccumulatedImpulse, TConstraintFunctions>
        : TypeBatch<Vector<int>, TPrestepData, TProjection, TAccumulatedImpulse>
        where TConstraintFunctions : struct, IOneBodyConstraintFunctions<TPrestepData, TProjection, TAccumulatedImpulse>
    {
        public sealed override int BodiesPerConstraint => 1;

        public sealed override void EnumerateConnectedBodyIndices<TEnumerator>(int indexInTypeBatch, ref TEnumerator enumerator)
        {
            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var constraintBundleIndex, out var constraintInnerIndex);
            enumerator.LoopBody(GatherScatter.Get(ref BodyReferences[constraintBundleIndex], constraintInnerIndex));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetSortKey(int constraintIndex, ref Buffer<Vector<int>> bodyReferences)
        {
            BundleIndexing.GetBundleIndices(constraintIndex, out var bundleIndex, out var innerIndex);
            //We sort based on the body references within the constraint. 
            //Note that it is impossible for there to be two references to the same body within a constraint batch, 
            //so there's no need to worry about the case where the comparison is equal.
            return GatherScatter.Get(ref bodyReferences[bundleIndex], innerIndex);
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
            var typedBodyReferencesCache = bodyReferencesCache.As<Vector<int>>();
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
            var typedBodyReferencesCache = bodyReferencesCache.As<Vector<int>>();
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
                    if (GatherScatter.Get(ref BodyReferences[i], j) == bodyIndex)
                        ++count;
                    Debug.Assert(count <= 1);
                    if (GatherScatter.Get(ref BodyReferences[i], j) == bodyIndex)
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
                GatherScatter.GatherVelocities(ref bodyVelocities, ref bodyReferences, count, out var wsvA);
                function.WarmStart(ref wsvA, ref projection, ref accumulatedImpulses);
                GatherScatter.ScatterVelocities(ref bodyVelocities, ref bodyReferences, count, ref wsvA);
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
                GatherScatter.GatherVelocities(ref bodyVelocities, ref bodyReferences, count, out var wsvA);
                function.Solve(ref wsvA, ref projection, ref accumulatedImpulses);
                GatherScatter.ScatterVelocities(ref bodyVelocities, ref bodyReferences, count, ref wsvA);
            }
        }

    }
}
