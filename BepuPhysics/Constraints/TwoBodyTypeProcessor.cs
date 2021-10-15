using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;
using System.Runtime.Intrinsics.Arm;

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
    /// Prestep, warm start and solve iteration functions for a two body constraint type.
    /// </summary>
    /// <typeparam name="TPrestepData">Type of the prestep data used by the constraint.</typeparam>
    /// <typeparam name="TAccumulatedImpulse">Type of the accumulated impulses used by the constraint.</typeparam>
    /// <typeparam name="TProjection">Type of the projection to input.</typeparam>
    public interface ITwoBodyConstraintFunctions<TPrestepData, TProjection, TAccumulatedImpulse>
    {
        void Prestep(in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide ab, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref TPrestepData prestepData, out TProjection projection);
        void WarmStart(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB, ref TProjection projection, ref TAccumulatedImpulse accumulatedImpulse);
        void Solve(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB, ref TProjection projection, ref TAccumulatedImpulse accumulatedImpulse);

        void WarmStart2(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB,
            ref TPrestepData prestep, ref TAccumulatedImpulse accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB);
        void Solve2(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt,
            ref TPrestepData prestep, ref TAccumulatedImpulse accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB);

        void UpdateForNewPose(
            in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in BodyVelocityWide wsvA,
            in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, in BodyVelocityWide wsvB,
            in Vector<float> dt, in TAccumulatedImpulse accumulatedImpulses, ref TPrestepData prestep);
    }

    /// <summary>
    /// Prestep, warm start, solve iteration, and incremental contact update functions for a two body contact constraint type.
    /// </summary>
    /// <typeparam name="TPrestepData">Type of the prestep data used by the constraint.</typeparam>
    /// <typeparam name="TAccumulatedImpulse">Type of the accumulated impulses used by the constraint.</typeparam>
    /// <typeparam name="TProjection">Type of the projection to input.</typeparam>
    public interface IContactConstraintFunctions<TPrestepData, TProjection, TAccumulatedImpulse> : ITwoBodyConstraintFunctions<TPrestepData, TProjection, TAccumulatedImpulse>
    {
        void IncrementallyUpdateContactData(in Vector<float> dt, in BodyVelocityWide velocityA, in BodyVelocityWide velocityB, ref TPrestepData prestepData);
    }

    //Not a big fan of complex generic-filled inheritance hierarchies, but this is the shortest evolutionary step to removing duplicates.
    //There are some other options if this inheritance hierarchy gets out of control.
    /// <summary>
    /// Shared implementation across all two body constraints.
    /// </summary>
    public abstract class TwoBodyTypeProcessor<TPrestepData, TProjection, TAccumulatedImpulse, TConstraintFunctions,
        TWarmStartAccessFilterA, TWarmStartAccessFilterB, TSolveAccessFilterA, TSolveAccessFilterB>
        : TypeProcessor<TwoBodyReferences, TPrestepData, TProjection, TAccumulatedImpulse>
        where TPrestepData : unmanaged where TProjection : unmanaged where TAccumulatedImpulse : unmanaged
        where TConstraintFunctions : unmanaged, ITwoBodyConstraintFunctions<TPrestepData, TProjection, TAccumulatedImpulse>
        where TWarmStartAccessFilterA : unmanaged, IBodyAccessFilter
        where TWarmStartAccessFilterB : unmanaged, IBodyAccessFilter
        where TSolveAccessFilterA : unmanaged, IBodyAccessFilter
        where TSolveAccessFilterB : unmanaged, IBodyAccessFilter
    {
        protected sealed override int InternalBodiesPerConstraint => 2;

        public sealed override void EnumerateConnectedBodyIndices<TEnumerator>(ref TypeBatch typeBatch, int indexInTypeBatch, ref TEnumerator enumerator)
        {
            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var constraintBundleIndex, out var constraintInnerIndex);
            ref var indexA = ref GatherScatter.Get(ref Buffer<TwoBodyReferences>.Get(ref typeBatch.BodyReferences, constraintBundleIndex).IndexA, constraintInnerIndex);
            ref var indexB = ref Unsafe.Add(ref indexA, Vector<int>.Count);
            //Note that the variables are ref locals! This is important for correctness, because every execution of LoopBody could result in a swap.
            //Ref locals aren't the only solution, but if you ever change this, make sure you account for the potential mutation in the enumerator.
            enumerator.LoopBody(indexA);
            enumerator.LoopBody(indexB);
        }
        struct TwoBodySortKeyGenerator : ISortKeyGenerator<TwoBodyReferences>
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int GetSortKey(int constraintIndex, ref Buffer<TwoBodyReferences> bodyReferences)
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
        }


        internal sealed override void GenerateSortKeysAndCopyReferences(
            ref TypeBatch typeBatch,
            int bundleStart, int localBundleStart, int bundleCount,
            int constraintStart, int localConstraintStart, int constraintCount,
            ref int firstSortKey, ref int firstSourceIndex, ref RawBuffer bodyReferencesCache)
        {
            GenerateSortKeysAndCopyReferences<TwoBodySortKeyGenerator>(
                ref typeBatch,
                bundleStart, localBundleStart, bundleCount,
                constraintStart, localConstraintStart, constraintCount,
                ref firstSortKey, ref firstSourceIndex, ref bodyReferencesCache);
        }

        internal sealed override void VerifySortRegion(ref TypeBatch typeBatch, int bundleStartIndex, int constraintCount, ref Buffer<int> sortedKeys, ref Buffer<int> sortedSourceIndices)
        {
            VerifySortRegion<TwoBodySortKeyGenerator>(ref typeBatch, bundleStartIndex, constraintCount, ref sortedKeys, ref sortedSourceIndices);
        }



        //The following covers the common loop logic for all two body constraints. Each iteration invokes the warm start function type.
        //This abstraction should, in theory, have zero overhead if the implementation of the interface is in a struct with aggressive inlining.

        //By providing the overrides at this level, the concrete implementation (assuming it inherits from one of the prestep-providing variants)
        //only has to specify *type* arguments associated with the interface-implementing struct-delegates. It's going to look very strange, but it's low overhead
        //and minimizes per-type duplication.


        public unsafe override void Prestep(ref TypeBatch typeBatch, Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            ref var prestepBase = ref Unsafe.AsRef<TPrestepData>(typeBatch.PrestepData.Memory);
            ref var bodyReferencesBase = ref Unsafe.AsRef<TwoBodyReferences>(typeBatch.BodyReferences.Memory);
            ref var projectionBase = ref Unsafe.AsRef<TProjection>(typeBatch.Projection.Memory);
            var function = default(TConstraintFunctions);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref Unsafe.Add(ref prestepBase, i);
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var references = ref Unsafe.Add(ref bodyReferencesBase, i);
                bodies.GatherState(ref references, out var orientationA, out var wsvA, out var inertiaA, out var ab, out var orientationB, out var wsvB, out var inertiaB);
                function.Prestep(orientationA, inertiaA, ab, orientationB, inertiaB, dt, inverseDt, ref prestep, out projection);
            }
        }

        public unsafe override void WarmStart(ref TypeBatch typeBatch, Bodies bodies, int startBundle, int exclusiveEndBundle)
        {
            ref var bodyReferencesBase = ref Unsafe.AsRef<TwoBodyReferences>(typeBatch.BodyReferences.Memory);
            ref var accumulatedImpulsesBase = ref Unsafe.AsRef<TAccumulatedImpulse>(typeBatch.AccumulatedImpulses.Memory);
            ref var projectionBase = ref Unsafe.AsRef<TProjection>(typeBatch.Projection.Memory);
            var function = default(TConstraintFunctions);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                ref var references = ref Unsafe.Add(ref bodyReferencesBase, i);
                bodies.GatherState(ref references, out var orientationA, out var wsvA, out var inertiaA, out var ab, out var orientationB, out var wsvB, out var inertiaB);
                function.WarmStart(ref wsvA, ref wsvB, ref projection, ref accumulatedImpulses);
                bodies.ScatterVelocities<AccessAll>(ref wsvA, ref references.IndexA);
                bodies.ScatterVelocities<AccessAll>(ref wsvB, ref references.IndexB);

            }
        }

        public unsafe override void SolveIteration(ref TypeBatch typeBatch, Bodies bodies, int startBundle, int exclusiveEndBundle)
        {
            ref var bodyReferencesBase = ref Unsafe.AsRef<TwoBodyReferences>(typeBatch.BodyReferences.Memory);
            ref var accumulatedImpulsesBase = ref Unsafe.AsRef<TAccumulatedImpulse>(typeBatch.AccumulatedImpulses.Memory);
            ref var projectionBase = ref Unsafe.AsRef<TProjection>(typeBatch.Projection.Memory);
            var function = default(TConstraintFunctions);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                ref var references = ref Unsafe.Add(ref bodyReferencesBase, i);
                bodies.GatherState(ref references, out var orientationA, out var wsvA, out var inertiaA, out var ab, out var orientationB, out var wsvB, out var inertiaB);
                function.Solve(ref wsvA, ref wsvB, ref projection, ref accumulatedImpulses);
                bodies.ScatterVelocities<AccessAll>(ref wsvA, ref references.IndexA);
                bodies.ScatterVelocities<AccessAll>(ref wsvB, ref references.IndexB);
            }
        }

        //public const int WarmStartPrefetchDistance = 8;
        //public const int SolvePrefetchDistance = 4;
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //static unsafe void Prefetch(void* address)
        //{
        //    if (Sse.IsSupported)
        //    {
        //        Sse.Prefetch0(address);
        //        //Sse.Prefetch0((byte*)address + 64);
        //        //TODO: prefetch should grab cache line pair anyway, right? not much reason to explicitly do more?
        //    }
        //    //TODO: ARM?
        //}
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //static unsafe void PrefetchBundle(SolverState* stateBase, ref TwoBodyReferences references, int countInBundle)
        //{
        //    var indicesA = (int*)Unsafe.AsPointer(ref references.IndexA);
        //    var indicesB = (int*)Unsafe.AsPointer(ref references.IndexB);
        //    for (int i = 0; i < countInBundle; ++i)
        //    {
        //        var indexA = indicesA[i];
        //        var indexB = indicesA[i];
        //        Prefetch(stateBase + indexA);
        //        Prefetch(stateBase + indexB);
        //    }
        //}

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //[Conditional("PREFETCH")]
        //public unsafe static void EarlyPrefetch(int prefetchDistance, ref TypeBatch typeBatch, ref Buffer<TwoBodyReferences> references, ref Buffer<SolverState> states, int startBundleIndex, int exclusiveEndBundleIndex)
        //{
        //    exclusiveEndBundleIndex = Math.Min(exclusiveEndBundleIndex, startBundleIndex + prefetchDistance);
        //    var lastBundleIndex = exclusiveEndBundleIndex - 1;
        //    for (int i = startBundleIndex; i < lastBundleIndex; ++i)
        //    {
        //        PrefetchBundle(states.Memory, ref references[i], Vector<float>.Count);
        //    }
        //    var countInBundle = GetCountInBundle(ref typeBatch, lastBundleIndex);
        //    PrefetchBundle(states.Memory, ref references[lastBundleIndex], countInBundle);
        //}

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //[Conditional("PREFETCH")]
        //public unsafe static void Prefetch(int prefetchDistance, ref TypeBatch typeBatch, ref Buffer<TwoBodyReferences> references, ref Buffer<SolverState> states, int bundleIndex, int exclusiveEndBundleIndex)
        //{
        //    var targetIndex = bundleIndex + prefetchDistance;
        //    if (targetIndex < exclusiveEndBundleIndex)
        //    {
        //        PrefetchBundle(states.Memory, ref references[targetIndex], GetCountInBundle(ref typeBatch, targetIndex));
        //    }
        //}



        public unsafe override void WarmStart2<TIntegratorCallbacks, TBatchIntegrationMode, TAllowPoseIntegration>(
            ref TypeBatch typeBatch, ref Buffer<IndexSet> integrationFlags, Bodies bodies, ref TIntegratorCallbacks integratorCallbacks,
            float dt, float inverseDt, int startBundle, int exclusiveEndBundle, int workerIndex)
        {
            var prestepBundles = typeBatch.PrestepData.As<TPrestepData>();
            var bodyReferencesBundles = typeBatch.BodyReferences.As<TwoBodyReferences>();
            var accumulatedImpulsesBundles = typeBatch.AccumulatedImpulses.As<TAccumulatedImpulse>();
            Unsafe.SkipInit(out TConstraintFunctions function);
            ref var states = ref bodies.ActiveSet.SolverStates;
            //EarlyPrefetch(WarmStartPrefetchDistance, ref typeBatch, ref bodyReferencesBundles, ref states, startBundle, exclusiveEndBundle);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref prestepBundles[i];
                ref var accumulatedImpulses = ref accumulatedImpulsesBundles[i];
                ref var references = ref bodyReferencesBundles[i];
                //Prefetch(WarmStartPrefetchDistance, ref typeBatch, ref bodyReferencesBundles, ref states, i, exclusiveEndBundle);
                GatherAndIntegrate<TIntegratorCallbacks, TBatchIntegrationMode, TWarmStartAccessFilterA, TAllowPoseIntegration>(bodies, ref integratorCallbacks, ref integrationFlags, 0, dt, workerIndex, i, ref references.IndexA,
                    out var positionA, out var orientationA, out var wsvA, out var inertiaA);
                GatherAndIntegrate<TIntegratorCallbacks, TBatchIntegrationMode, TWarmStartAccessFilterB, TAllowPoseIntegration>(bodies, ref integratorCallbacks, ref integrationFlags, 1, dt, workerIndex, i, ref references.IndexB,
                    out var positionB, out var orientationB, out var wsvB, out var inertiaB);

                //if (typeof(TAllowPoseIntegration) == typeof(AllowPoseIntegration))
                //    function.UpdateForNewPose(positionA, orientationA, inertiaA, wsvA, positionB, orientationB, inertiaB, wsvB, new Vector<float>(dt), accumulatedImpulses, ref prestep);

                function.WarmStart2(positionA, orientationA, inertiaA, positionB, orientationB, inertiaB, ref prestep, ref accumulatedImpulses, ref wsvA, ref wsvB);

                if (typeof(TBatchIntegrationMode) == typeof(BatchShouldNeverIntegrate))
                {
                    bodies.ScatterVelocities<TWarmStartAccessFilterA>(ref wsvA, ref references.IndexA);
                    bodies.ScatterVelocities<TWarmStartAccessFilterB>(ref wsvB, ref references.IndexB);
                }
                else
                {
                    //This batch has some integrators, which means that every bundle is going to gather all velocities.
                    //(We don't make per-bundle determinations about this to avoid an extra branch and instruction complexity, and the difference is very small.)
                    bodies.ScatterVelocities<AccessAll>(ref wsvA, ref references.IndexA);
                    bodies.ScatterVelocities<AccessAll>(ref wsvB, ref references.IndexB);
                }

            }
        }

        public unsafe override void SolveStep2(ref TypeBatch typeBatch, Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            var prestepBundles = typeBatch.PrestepData.As<TPrestepData>();
            var bodyReferencesBundles = typeBatch.BodyReferences.As<TwoBodyReferences>();
            var accumulatedImpulsesBundles = typeBatch.AccumulatedImpulses.As<TAccumulatedImpulse>();
            Unsafe.SkipInit(out TConstraintFunctions function);
            ref var motionStates = ref bodies.ActiveSet.SolverStates;
            //EarlyPrefetch(SolvePrefetchDistance, ref typeBatch, ref bodyReferencesBundles, ref motionStates, startBundle, exclusiveEndBundle);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref prestepBundles[i];
                ref var accumulatedImpulses = ref accumulatedImpulsesBundles[i];
                ref var references = ref bodyReferencesBundles[i];
                //Prefetch(SolvePrefetchDistance, ref typeBatch, ref bodyReferencesBundles, ref motionStates, i, exclusiveEndBundle);
                bodies.GatherState<TSolveAccessFilterA>(ref references.IndexA, true, out var positionA, out var orientationA, out var wsvA, out var inertiaA);
                bodies.GatherState<TSolveAccessFilterB>(ref references.IndexB, true, out var positionB, out var orientationB, out var wsvB, out var inertiaB);

                function.Solve2(positionA, orientationA, inertiaA, positionB, orientationB, inertiaB, dt, inverseDt, ref prestep, ref accumulatedImpulses, ref wsvA, ref wsvB);

                bodies.ScatterVelocities<TSolveAccessFilterA>(ref wsvA, ref references.IndexA);
                bodies.ScatterVelocities<TSolveAccessFilterB>(ref wsvB, ref references.IndexB);
            }
        }
    }

    public abstract class TwoBodyContactTypeProcessor<TPrestepData, TProjection, TAccumulatedImpulse, TConstraintFunctions>
        : TwoBodyTypeProcessor<TPrestepData, TProjection, TAccumulatedImpulse, TConstraintFunctions, AccessNoOrientation, AccessNoPose, AccessNoOrientation, AccessNoPose>
        where TPrestepData : unmanaged where TProjection : unmanaged where TAccumulatedImpulse : unmanaged
        where TConstraintFunctions : unmanaged, IContactConstraintFunctions<TPrestepData, TProjection, TAccumulatedImpulse>
    {
        public unsafe override void IncrementallyUpdateContactData(ref TypeBatch typeBatch, Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            ref var prestepBase = ref Unsafe.AsRef<TPrestepData>(typeBatch.PrestepData.Memory);
            ref var bodyReferencesBase = ref Unsafe.AsRef<TwoBodyReferences>(typeBatch.BodyReferences.Memory);
            var function = default(TConstraintFunctions);
            var dtWide = new Vector<float>(dt);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref Unsafe.Add(ref prestepBase, i);
                ref var references = ref Unsafe.Add(ref bodyReferencesBase, i);
                bodies.GatherState<AccessOnlyVelocity>(ref references.IndexA, true, out _, out _, out var wsvA, out _);
                bodies.GatherState<AccessOnlyVelocity>(ref references.IndexB, true, out _, out _, out var wsvB, out _);
                function.IncrementallyUpdateContactData(dtWide, wsvA, wsvB, ref prestep);
            }
        }
    }
}
