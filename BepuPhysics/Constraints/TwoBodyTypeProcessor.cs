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
    public interface IConstraintFunctions<TPrestepData, TProjection, TAccumulatedImpulse>
    {
        void Prestep(in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide ab, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref TPrestepData prestepData, out TProjection projection);
        void WarmStart(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB, ref TProjection projection, ref TAccumulatedImpulse accumulatedImpulse);
        void Solve(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB, ref TProjection projection, ref TAccumulatedImpulse accumulatedImpulse);
    }

    /// <summary>
    /// Prestep, warm start, solve iteration, and incremental contact update functions for a two body contact constraint type.
    /// </summary>
    /// <typeparam name="TPrestepData">Type of the prestep data used by the constraint.</typeparam>
    /// <typeparam name="TAccumulatedImpulse">Type of the accumulated impulses used by the constraint.</typeparam>
    /// <typeparam name="TProjection">Type of the projection to input.</typeparam>
    public interface IContactConstraintFunctions<TPrestepData, TProjection, TAccumulatedImpulse> : IConstraintFunctions<TPrestepData, TProjection, TAccumulatedImpulse>
    {
        void IncrementallyUpdateContactData(in Vector<float> dt, in BodyVelocityWide velocityA, in BodyVelocityWide velocityB, ref TPrestepData prestepData);
    }

    //Not a big fan of complex generic-filled inheritance hierarchies, but this is the shortest evolutionary step to removing duplicates.
    //There are some other options if this inheritance hierarchy gets out of control.
    /// <summary>
    /// Shared implementation across all two body constraints.
    /// </summary>
    public abstract class TwoBodyTypeProcessor<TPrestepData, TProjection, TAccumulatedImpulse, TConstraintFunctions>
        : TypeProcessor<TwoBodyReferences, TPrestepData, TProjection, TAccumulatedImpulse>
        where TPrestepData : unmanaged where TProjection : unmanaged where TAccumulatedImpulse : unmanaged
        where TConstraintFunctions : unmanaged, IConstraintFunctions<TPrestepData, TProjection, TAccumulatedImpulse>
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
                var count = GetCountInBundle(ref typeBatch, i);
                bodies.GatherState(ref references, count, out var orientationA, out var wsvA, out var inertiaA, out var ab, out var orientationB, out var wsvB, out var inertiaB);
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
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                int count = GetCountInBundle(ref typeBatch, i);
                bodies.GatherState(ref bodyReferences, count, out var orientationA, out var wsvA, out var inertiaA, out var ab, out var orientationB, out var wsvB, out var inertiaB);
                function.WarmStart(ref wsvA, ref wsvB, ref projection, ref accumulatedImpulses);
                bodies.ScatterVelocities(ref wsvA, ref wsvB, ref bodyReferences, count);

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
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                int count = GetCountInBundle(ref typeBatch, i);
                bodies.GatherState(ref bodyReferences, count, out var orientationA, out var wsvA, out var inertiaA, out var ab, out var orientationB, out var wsvB, out var inertiaB);
                function.Solve(ref wsvA, ref wsvB, ref projection, ref accumulatedImpulses);
                bodies.ScatterVelocities(ref wsvA, ref wsvB, ref bodyReferences, count);
            }
        }

        public unsafe override void JacobiPrestep(ref TypeBatch typeBatch, Bodies bodies, ref FallbackBatch jacobiBatch, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
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
                var count = GetCountInBundle(ref typeBatch, i);
                bodies.GatherState(ref references, count, out var orientationA, out var wsvA, out var inertiaA, out var ab, out var orientationB, out var wsvB, out var inertiaB);
                //Jacobi batches split affected bodies into multiple pieces to guarantee convergence.
                jacobiBatch.GetJacobiScaleForBodies(ref references, count, out var jacobiScaleA, out var jacobiScaleB);
                Symmetric3x3Wide.Scale(inertiaA.InverseInertiaTensor, jacobiScaleA, out inertiaA.InverseInertiaTensor);
                inertiaA.InverseMass *= jacobiScaleA;
                Symmetric3x3Wide.Scale(inertiaB.InverseInertiaTensor, jacobiScaleB, out inertiaB.InverseInertiaTensor);
                inertiaB.InverseMass *= jacobiScaleB;
                function.Prestep(orientationA, inertiaA, ab, orientationB, inertiaB, dt, inverseDt, ref prestep, out projection);
            }
        }
        public unsafe override void JacobiWarmStart(ref TypeBatch typeBatch, Bodies bodies, ref FallbackTypeBatchResults jacobiResults, int startBundle, int exclusiveEndBundle)
        {
            ref var bodyReferencesBase = ref Unsafe.AsRef<TwoBodyReferences>(typeBatch.BodyReferences.Memory);
            ref var accumulatedImpulsesBase = ref Unsafe.AsRef<TAccumulatedImpulse>(typeBatch.AccumulatedImpulses.Memory);
            ref var projectionBase = ref Unsafe.AsRef<TProjection>(typeBatch.Projection.Memory);
            var function = default(TConstraintFunctions);
            ref var jacobiResultsBundlesA = ref jacobiResults.GetVelocitiesForBody(0);
            ref var jacobiResultsBundlesB = ref jacobiResults.GetVelocitiesForBody(1);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                int count = GetCountInBundle(ref typeBatch, i);
                ref var wsvA = ref jacobiResultsBundlesA[i];
                ref var wsvB = ref jacobiResultsBundlesB[i];
                bodies.GatherState(ref bodyReferences, count, out var orientationA, out wsvA, out var inertiaA, out var ab, out var orientationB, out wsvB, out var inertiaB);
                function.WarmStart(ref wsvA, ref wsvB, ref projection, ref accumulatedImpulses);
            }
        }
        public unsafe override void JacobiSolveIteration(ref TypeBatch typeBatch, Bodies bodies, ref FallbackTypeBatchResults jacobiResults, int startBundle, int exclusiveEndBundle)
        {
            ref var bodyReferencesBase = ref Unsafe.AsRef<TwoBodyReferences>(typeBatch.BodyReferences.Memory);
            ref var accumulatedImpulsesBase = ref Unsafe.AsRef<TAccumulatedImpulse>(typeBatch.AccumulatedImpulses.Memory);
            ref var projectionBase = ref Unsafe.AsRef<TProjection>(typeBatch.Projection.Memory);
            var function = default(TConstraintFunctions);
            ref var jacobiResultsBundlesA = ref jacobiResults.GetVelocitiesForBody(0);
            ref var jacobiResultsBundlesB = ref jacobiResults.GetVelocitiesForBody(1);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                int count = GetCountInBundle(ref typeBatch, i);
                ref var wsvA = ref jacobiResultsBundlesA[i];
                ref var wsvB = ref jacobiResultsBundlesB[i];
                bodies.GatherState(ref bodyReferences, count, out var orientationA, out wsvA, out var inertiaA, out var ab, out var orientationB, out wsvB, out var inertiaB);
                function.Solve(ref wsvA, ref wsvB, ref projection, ref accumulatedImpulses);
            }
        }

        public unsafe override void SolveStep(ref TypeBatch typeBatch, Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            ref var prestepBase = ref Unsafe.AsRef<TPrestepData>(typeBatch.PrestepData.Memory);
            ref var bodyReferencesBase = ref Unsafe.AsRef<TwoBodyReferences>(typeBatch.BodyReferences.Memory);
            ref var accumulatedImpulsesBase = ref Unsafe.AsRef<TAccumulatedImpulse>(typeBatch.AccumulatedImpulses.Memory);
            ref var projectionBase = ref Unsafe.AsRef<TProjection>(typeBatch.Projection.Memory);
            var function = default(TConstraintFunctions);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref Unsafe.Add(ref prestepBase, i);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                ref var references = ref Unsafe.Add(ref bodyReferencesBase, i);
                var count = GetCountInBundle(ref typeBatch, i);
                bodies.GatherState(ref references, count, out var orientationA, out var wsvA, out var inertiaA, out var ab, out var orientationB, out var wsvB, out var inertiaB);
                function.Prestep(orientationA, inertiaA, ab, orientationB, inertiaB, dt, inverseDt, ref prestep, out var projection);
                function.WarmStart(ref wsvA, ref wsvB, ref projection, ref accumulatedImpulses);
                function.Solve(ref wsvA, ref wsvB, ref projection, ref accumulatedImpulses);
                bodies.ScatterVelocities(ref wsvA, ref wsvB, ref bodyReferences, count);
            }
        }

        public unsafe override void JacobiSolveStep(ref TypeBatch typeBatch, Bodies bodies, ref FallbackBatch jacobiBatch, ref FallbackTypeBatchResults jacobiResults, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            ref var prestepBase = ref Unsafe.AsRef<TPrestepData>(typeBatch.PrestepData.Memory);
            ref var bodyReferencesBase = ref Unsafe.AsRef<TwoBodyReferences>(typeBatch.BodyReferences.Memory);
            ref var accumulatedImpulsesBase = ref Unsafe.AsRef<TAccumulatedImpulse>(typeBatch.AccumulatedImpulses.Memory);
            ref var projectionBase = ref Unsafe.AsRef<TProjection>(typeBatch.Projection.Memory);
            ref var motionStates = ref bodies.ActiveSet.MotionStates;
            var function = default(TConstraintFunctions);
            ref var jacobiResultsBundlesA = ref jacobiResults.GetVelocitiesForBody(0);
            ref var jacobiResultsBundlesB = ref jacobiResults.GetVelocitiesForBody(1);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref Unsafe.Add(ref prestepBase, i);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                ref var references = ref Unsafe.Add(ref bodyReferencesBase, i);
                var count = GetCountInBundle(ref typeBatch, i);
                ref var wsvA = ref jacobiResultsBundlesA[i];
                ref var wsvB = ref jacobiResultsBundlesB[i];
                bodies.GatherState(ref references, count, out var orientationA, out wsvA, out var inertiaA, out var ab, out var orientationB, out wsvB, out var inertiaB);
                //Jacobi batches split affected bodies into multiple pieces to guarantee convergence.
                jacobiBatch.GetJacobiScaleForBodies(ref references, count, out var jacobiScaleA, out var jacobiScaleB);
                Symmetric3x3Wide.Scale(inertiaA.InverseInertiaTensor, jacobiScaleA, out inertiaA.InverseInertiaTensor);
                inertiaA.InverseMass *= jacobiScaleA;
                Symmetric3x3Wide.Scale(inertiaB.InverseInertiaTensor, jacobiScaleB, out inertiaB.InverseInertiaTensor);
                inertiaB.InverseMass *= jacobiScaleB;
                function.Prestep(orientationA, inertiaA, ab, orientationB, inertiaB, dt, inverseDt, ref prestep, out var projection);
                function.WarmStart(ref wsvA, ref wsvB, ref projection, ref accumulatedImpulses);
                function.Solve(ref wsvA, ref wsvB, ref projection, ref accumulatedImpulses);
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static unsafe void Prefetch(void* address)
        {
            if (Sse.IsSupported)
            {
                Sse.Prefetch0(address);
            }
            //TODO: ARM?
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static unsafe void PrefetchBundle(MotionState* motionBase, BodyInertias* inertiaBase, ref TwoBodyReferences references, int countInBundle)
        {
            var indicesA = (int*)Unsafe.AsPointer(ref references.IndexA);
            var indicesB = (int*)Unsafe.AsPointer(ref references.IndexB);
            for (int i = 0; i < countInBundle; ++i)
            {
                var indexA = indicesA[i];
                var indexB = indicesA[i];
                Prefetch(motionBase + indexA);
                Prefetch(inertiaBase + indexA);
                Prefetch(motionBase + indexB);
                Prefetch(inertiaBase + indexB);
            }
        }

        const int warmStartPrefetchDistance = 8;
        const int solvePrefetchDistance = 4;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static void EarlyPrefetch(int prefetchDistance, ref TypeBatch typeBatch, ref Buffer<TwoBodyReferences> references, ref Buffer<MotionState> states, ref Buffer<BodyInertias> inertias, int startBundleIndex, int exclusiveEndBundleIndex)
        {
            exclusiveEndBundleIndex = Math.Min(exclusiveEndBundleIndex, startBundleIndex + prefetchDistance);
            var lastBundleIndex = exclusiveEndBundleIndex - 1;
            for (int i = startBundleIndex; i < lastBundleIndex; ++i)
            {
                PrefetchBundle(states.Memory, inertias.Memory, ref references[i], Vector<float>.Count);
            }
            var countInBundle = GetCountInBundle(ref typeBatch, lastBundleIndex);
            PrefetchBundle(states.Memory, inertias.Memory, ref references[lastBundleIndex], countInBundle);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static void Prefetch(int prefetchDistance, ref TypeBatch typeBatch, ref Buffer<TwoBodyReferences> references, ref Buffer<MotionState> states, ref Buffer<BodyInertias> inertias, int bundleIndex, int exclusiveEndBundleIndex)
        {
            var targetIndex = bundleIndex + prefetchDistance;
            if (targetIndex < exclusiveEndBundleIndex)
            {
                PrefetchBundle(states.Memory, inertias.Memory, ref references[targetIndex], GetCountInBundle(ref typeBatch, targetIndex));
            }
        }

        public enum BundleIntegrationMode
        {
            None = 0,
            Partial = 1,
            All = 2
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static BundleIntegrationMode BundleShouldIntegrate(int bundleIndex, in IndexSet integrationFlags, out Vector<int> integrationMask)
        {
            Debug.Assert(Vector<float>.Count <= 32, "Wait, what? The integration mask isn't big enough to handle a vector this big.");
            var constraintStartIndex = bundleIndex * Vector<float>.Count;
            var flagBundleIndex = constraintStartIndex >> 6;
            var flagInnerIndex = constraintStartIndex - (flagBundleIndex << 6);
            var flagMask = (1 << Vector<float>.Count) - 1;
            var scalarIntegrationMask = ((int)(integrationFlags.Flags[flagBundleIndex] >> flagInnerIndex)) & flagMask;
            if (scalarIntegrationMask == flagMask)
            {
                //No need to carefully expand a bitstring into a vector mask if we know that a single broadcast will suffice.
                integrationMask = new Vector<int>(-1);
                return BundleIntegrationMode.All;
            }
            else if (scalarIntegrationMask > 0)
            {
                if (Vector<int>.Count == 4 || Vector<int>.Count == 8)
                {
                    Vector<int> selectors;
                    if (Vector<int>.Count == 8)
                    {
                        selectors = Vector256.Create(1, 2, 4, 8, 16, 32, 64, 128).AsVector();
                    }
                    else
                    {
                        selectors = Vector128.Create(1, 2, 4, 8).AsVector();
                    }
                    var scalarBroadcast = new Vector<int>(scalarIntegrationMask);
                    var selected = Vector.BitwiseAnd(selectors, scalarBroadcast);
                    integrationMask = Vector.Equals(selected, selectors);
                }
                else
                {
                    //This is not a good implementation, but I don't know of any target platforms that will hit this.
                    //TODO: AVX512 being enabled by the runtime could force this path to be taken; it'll require an update!
                    Debug.Assert(Vector<int>.Count < 16, "The vector path assumes that AVX512 is not supported, so this is hitting a fallback path.");
                    Span<int> mask = stackalloc int[Vector<int>.Count];
                    for (int i = 0; i < Vector<int>.Count; ++i)
                    {
                        mask[i] = (scalarIntegrationMask & (1 << i)) > 0 ? -1 : 0;
                    }
                    integrationMask = new Vector<int>(mask);
                }
                return BundleIntegrationMode.Partial;
            }
            integrationMask = default;
            return BundleIntegrationMode.None;
        }

        public static unsafe void IntegratePoseAndVelocity<TIntegratorCallbacks>(
            ref TIntegratorCallbacks integratorCallbacks, ref Vector<int> bodyIndices, int count, in BodyInertiaWide localInertia, float dt, Vector<int> integrationMask,
            ref Vector3Wide position, ref QuaternionWide orientation, ref BodyVelocityWide velocity,
            int workerIndex,
            out BodyInertiaWide inertia)
            where TIntegratorCallbacks : struct, IPoseIntegratorCallbacks
        {
            //Note that we integrate pose, then velocity.
            //We only use this function where we can guarantee that the external-to-timestep view of velocities and poses looks like the frame starts on a velocity integration and ends on a pose integration.
            //This ensures that velocities set externally are still solved before being integrated.
            //So, the solver runs velocity integration alone on the first substep. All later substeps then run pose + velocity, and then after the last substep, a final pose integration.
            //This is equivalent in ordering to running each substep as velocity, warmstart, solve, pose integration, but just shifting the execution context.
            var newPosition = position + velocity.Linear * new Vector<float>(dt);
            //Note that we only take results for slots which actually need integration. Reintegration would be an error.
            Vector3Wide.ConditionalSelect(integrationMask, newPosition, position, out position);
            QuaternionWide newOrientation;
            inertia.InverseMass = localInertia.InverseMass;
            var previousVelocity = velocity;
            if (integratorCallbacks.AngularIntegrationMode == AngularIntegrationMode.ConserveMomentum)
            {
                var previousOrientation = orientation;
                PoseIntegration.Integrate(orientation, velocity.Angular, new Vector<float>(dt * 0.5f), out newOrientation);
                QuaternionWide.ConditionalSelect(integrationMask, newOrientation, orientation, out orientation);
                PoseIntegration.RotateInverseInertia(localInertia.InverseInertiaTensor, orientation, out inertia.InverseInertiaTensor);
                PoseIntegration.IntegrateAngularVelocityConserveMomentum(previousOrientation, localInertia, inertia, ref velocity.Angular);
            }
            else if (integratorCallbacks.AngularIntegrationMode == AngularIntegrationMode.ConserveMomentumWithGyroscopicTorque)
            {
                PoseIntegration.Integrate(orientation, velocity.Angular, new Vector<float>(dt * 0.5f), out newOrientation);
                QuaternionWide.ConditionalSelect(integrationMask, newOrientation, orientation, out orientation);
                PoseIntegration.RotateInverseInertia(localInertia.InverseInertiaTensor, orientation, out inertia.InverseInertiaTensor);
                PoseIntegration.IntegrateAngularVelocityConserveMomentumWithGyroscopicTorque(orientation, localInertia, inertia, ref velocity.Angular, dt);
            }
            else
            {
                PoseIntegration.Integrate(orientation, velocity.Angular, new Vector<float>(dt * 0.5f), out newOrientation);
                QuaternionWide.ConditionalSelect(integrationMask, newOrientation, orientation, out orientation);
                PoseIntegration.RotateInverseInertia(localInertia.InverseInertiaTensor, orientation, out inertia.InverseInertiaTensor);
            }
            integratorCallbacks.IntegrateVelocity(new ReadOnlySpan<int>(Unsafe.AsPointer(ref bodyIndices), count), position, orientation, localInertia, integrationMask, workerIndex, new Vector<float>(dt), ref velocity);
            //It would be annoying to make the user handle masking velocity writes to inactive lanes, so we handle it internally.
            Vector3Wide.ConditionalSelect(integrationMask, velocity.Linear, previousVelocity.Linear, out velocity.Linear);
            Vector3Wide.ConditionalSelect(integrationMask, velocity.Angular, previousVelocity.Angular, out velocity.Angular);
        }

        public static unsafe void IntegratePoseAndVelocity<TIntegratorCallbacks>(
            ref TIntegratorCallbacks integratorCallbacks, ref Vector<int> bodyIndices, int count, in BodyInertiaWide localInertia, float dt,
            ref Vector3Wide position, ref QuaternionWide orientation, ref BodyVelocityWide velocity,
            int workerIndex,
            out BodyInertiaWide inertia)
            where TIntegratorCallbacks : struct, IPoseIntegratorCallbacks
        {
            //This is identical to the other IntegratePoseAndVelocity, but it avoids any masking because we know ahead of time that the entire bundle is integrating.
            position += velocity.Linear * new Vector<float>(dt);
            inertia.InverseMass = localInertia.InverseMass;
            if (integratorCallbacks.AngularIntegrationMode == AngularIntegrationMode.ConserveMomentum)
            {
                var previousOrientation = orientation;
                PoseIntegration.Integrate(orientation, velocity.Angular, new Vector<float>(dt * 0.5f), out orientation);
                PoseIntegration.RotateInverseInertia(localInertia.InverseInertiaTensor, orientation, out inertia.InverseInertiaTensor);
                PoseIntegration.IntegrateAngularVelocityConserveMomentum(previousOrientation, localInertia, inertia, ref velocity.Angular);
            }
            else if (integratorCallbacks.AngularIntegrationMode == AngularIntegrationMode.ConserveMomentumWithGyroscopicTorque)
            {
                PoseIntegration.Integrate(orientation, velocity.Angular, new Vector<float>(dt * 0.5f), out orientation);
                PoseIntegration.RotateInverseInertia(localInertia.InverseInertiaTensor, orientation, out inertia.InverseInertiaTensor);
                PoseIntegration.IntegrateAngularVelocityConserveMomentumWithGyroscopicTorque(orientation, localInertia, inertia, ref velocity.Angular, dt);
            }
            else
            {
                PoseIntegration.Integrate(orientation, velocity.Angular, new Vector<float>(dt * 0.5f), out orientation);
                PoseIntegration.RotateInverseInertia(localInertia.InverseInertiaTensor, orientation, out inertia.InverseInertiaTensor);
            }
            integratorCallbacks.IntegrateVelocity(new ReadOnlySpan<int>(Unsafe.AsPointer(ref bodyIndices), count), position, orientation, localInertia, new Vector<int>(-1), workerIndex, new Vector<float>(dt), ref velocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static unsafe void IntegrateUniformly<TIntegratorCallbacks>(
            Bodies bodies, ref TIntegratorCallbacks integratorCallbacks, float dt, int workerIndex, ref Vector<int> bodyIndices, int count, ref Vector<int> integrationMask,
            ref Vector3Wide position, ref QuaternionWide orientation, ref BodyVelocityWide velocity, out BodyInertiaWide inertia) where TIntegratorCallbacks : struct, IPoseIntegratorCallbacks
        {
            bodies.GatherLocalInertia(ref bodyIndices, count, out var localInertia);
            IntegratePoseAndVelocity(ref integratorCallbacks, ref bodyIndices, count, localInertia, dt, ref position, ref orientation, ref velocity, workerIndex, out inertia);
            bodies.ScatterPoseAndInertia<BatchShouldAlwaysIntegrate>(ref position, ref orientation, ref inertia, ref bodyIndices, count, ref integrationMask);
        }
        public static unsafe void GatherAndIntegrate<TIntegratorCallbacks, TBatchIntegrationMode>(
            Bodies bodies, ref TIntegratorCallbacks integratorCallbacks, ref IndexSet integrationFlags, float dt, int workerIndex, int bundleIndex,
            ref Vector<int> bodyIndices, int count, out Vector3Wide position, out QuaternionWide orientation, out BodyVelocityWide velocity, out BodyInertiaWide inertia)
            where TIntegratorCallbacks : struct, IPoseIntegratorCallbacks
            where TBatchIntegrationMode : struct, IBatchIntegrationMode
        {
            bodies.GatherMotionState(ref bodyIndices, count, out position, out orientation, out velocity);
            //These type tests are compile time constants and will be specialized.
            if (typeof(TBatchIntegrationMode) == typeof(BatchShouldAlwaysIntegrate))
            {
                var integrationMask = new Vector<int>(-1);
                IntegrateUniformly(bodies, ref integratorCallbacks, dt, workerIndex, ref bodyIndices, count, ref integrationMask, ref position, ref orientation, ref velocity, out inertia);
            }
            else if (typeof(TBatchIntegrationMode) == typeof(BatchShouldNeverIntegrate))
            {
                bodies.GatherWorldInertia(ref bodyIndices, count, out inertia);
            }
            else
            {
                Debug.Assert(typeof(TBatchIntegrationMode) == typeof(BatchShouldConditionallyIntegrate));
                //This executes in warmstart, and warmstarts are typically quite simple from an instruction stream perspective.
                //Having a dynamically chosen codepath is unlikely to cause instruction fetching issues.
                switch (BundleShouldIntegrate(bundleIndex, integrationFlags, out var integrationMask))
                {
                    case BundleIntegrationMode.All:
                        {
                            IntegrateUniformly(bodies, ref integratorCallbacks, dt, workerIndex, ref bodyIndices, count, ref integrationMask, ref position, ref orientation, ref velocity, out inertia);
                        }
                        break;
                    case BundleIntegrationMode.Partial:
                        {
                            //Note that if we take this codepath, the integration routine will reconstruct the world inertias from local inertia given the current pose.
                            //The changes to pose and velocity for integration inactive lanes will be masked out, so it'll just be identical to the world inertia if we had gathered it.
                            //Given that we're running the instructions in a bundle to build it, there's no reason to go out of our way to gather the world inertia.
                            bodies.GatherLocalInertia(ref bodyIndices, count, out var localInertia);
                            IntegratePoseAndVelocity(ref integratorCallbacks, ref bodyIndices, count, localInertia, dt, integrationMask, ref position, ref orientation, ref velocity, workerIndex, out inertia);
                            //The scatter will be able to ignore any lanes which have a zeroed integration mask.
                            bodies.ScatterPoseAndInertia<BatchShouldConditionallyIntegrate>(ref position, ref orientation, ref inertia, ref bodyIndices, count, ref integrationMask);
                        }
                        break;
                    default:
                        {
                            bodies.GatherWorldInertia(ref bodyIndices, count, out inertia);
                        }
                        break;
                }
            }
        }


        public unsafe override void WarmStart2<TIntegratorCallbacks, TBatchIntegrationMode>(
            ref TypeBatch typeBatch, ref Buffer<IndexSet> integrationFlags, Bodies bodies, ref TIntegratorCallbacks integratorCallbacks, float dt, float inverseDt, int startBundle, int exclusiveEndBundle, int workerIndex)
        {
            var prestepBundles = typeBatch.PrestepData.As<TPrestepData>();
            var bodyReferencesBundles = typeBatch.BodyReferences.As<TwoBodyReferences>();
            var accumulatedImpulsesBundles = typeBatch.AccumulatedImpulses.As<TAccumulatedImpulse>();
            var function = default(TConstraintFunctions);
            ref var motionStates = ref bodies.ActiveSet.MotionStates;
            ref var inertias = ref bodies.ActiveSet.Inertias;
            EarlyPrefetch(warmStartPrefetchDistance, ref typeBatch, ref bodyReferencesBundles, ref motionStates, ref inertias, startBundle, exclusiveEndBundle);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref prestepBundles[i];
                ref var accumulatedImpulses = ref accumulatedImpulsesBundles[i];
                ref var references = ref bodyReferencesBundles[i];
                var count = GetCountInBundle(ref typeBatch, i);
                Prefetch(warmStartPrefetchDistance, ref typeBatch, ref bodyReferencesBundles, ref motionStates, ref inertias, i, exclusiveEndBundle);
                GatherAndIntegrate<TIntegratorCallbacks, TBatchIntegrationMode>(bodies, ref integratorCallbacks, ref integrationFlags[0], dt, workerIndex, i, ref references.IndexA, count,
                    out var positionA, out var orientationA, out var wsvA, out var inertiaA);
                GatherAndIntegrate<TIntegratorCallbacks, TBatchIntegrationMode>(bodies, ref integratorCallbacks, ref integrationFlags[1], dt, workerIndex, i, ref references.IndexB, count,
                    out var positionB, out var orientationB, out var wsvB, out var inertiaB);
                var ab = positionB - positionA;
                if (typeof(TConstraintFunctions) == typeof(WeldFunctions))
                {
                    default(WeldFunctions).WarmStart2(orientationA, inertiaA, ab, orientationB, inertiaB,
                        Unsafe.As<TPrestepData, WeldPrestepData>(ref prestep), Unsafe.As<TAccumulatedImpulse, WeldAccumulatedImpulses>(ref accumulatedImpulses), ref wsvA, ref wsvB);
                }
                else if (typeof(TConstraintFunctions) == typeof(BallSocketFunctions))
                {
                    default(BallSocketFunctions).WarmStart2(orientationA, inertiaA, ab, orientationB, inertiaB,
                        Unsafe.As<TPrestepData, BallSocketPrestepData>(ref prestep), Unsafe.As<TAccumulatedImpulse, Vector3Wide>(ref accumulatedImpulses), ref wsvA, ref wsvB);
                }
                else
                {
                    function.Prestep(orientationA, inertiaA, ab, orientationB, inertiaB, dt, inverseDt, ref prestep, out var projection);
                    function.WarmStart(ref wsvA, ref wsvB, ref projection, ref accumulatedImpulses);
                }
                bodies.ScatterVelocities(ref wsvA, ref wsvB, ref references, count);
            }
        }


        public unsafe override void SolveStep2(ref TypeBatch typeBatch, Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            var prestepBundles = typeBatch.PrestepData.As<TPrestepData>();
            var bodyReferencesBundles = typeBatch.BodyReferences.As<TwoBodyReferences>();
            var accumulatedImpulsesBundles = typeBatch.AccumulatedImpulses.As<TAccumulatedImpulse>();
            var function = default(TConstraintFunctions);
            ref var motionStates = ref bodies.ActiveSet.MotionStates;
            ref var inertias = ref bodies.ActiveSet.Inertias;
            EarlyPrefetch(solvePrefetchDistance, ref typeBatch, ref bodyReferencesBundles, ref motionStates, ref inertias, startBundle, exclusiveEndBundle);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref prestepBundles[i];
                ref var accumulatedImpulses = ref accumulatedImpulsesBundles[i];
                ref var references = ref bodyReferencesBundles[i];
                var count = GetCountInBundle(ref typeBatch, i);
                Prefetch(solvePrefetchDistance, ref typeBatch, ref bodyReferencesBundles, ref motionStates, ref inertias, i, exclusiveEndBundle);
                bodies.GatherState(ref references, count, out var orientationA, out var wsvA, out var inertiaA, out var ab, out var orientationB, out var wsvB, out var inertiaB);
                if (typeof(TConstraintFunctions) == typeof(WeldFunctions))
                {
                    default(WeldFunctions).Solve2(orientationA, inertiaA, ab, orientationB, inertiaB, dt, inverseDt,
                        Unsafe.As<TPrestepData, WeldPrestepData>(ref prestep), ref Unsafe.As<TAccumulatedImpulse, WeldAccumulatedImpulses>(ref accumulatedImpulses), ref wsvA, ref wsvB);
                }
                else if (typeof(TConstraintFunctions) == typeof(BallSocketFunctions))
                {
                    default(BallSocketFunctions).Solve2(orientationA, inertiaA, ab, orientationB, inertiaB, dt, inverseDt,
                        Unsafe.As<TPrestepData, BallSocketPrestepData>(ref prestep), ref Unsafe.As<TAccumulatedImpulse, Vector3Wide>(ref accumulatedImpulses), ref wsvA, ref wsvB);
                }
                else
                {
                    function.Prestep(orientationA, inertiaA, ab, orientationB, inertiaB, dt, inverseDt, ref prestep, out var projection);
                    function.Solve(ref wsvA, ref wsvB, ref projection, ref accumulatedImpulses);
                }
                bodies.ScatterVelocities(ref wsvA, ref wsvB, ref references, count);
            }
        }

        public unsafe override void Prestep2(ref TypeBatch typeBatch, Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            ref var prestepBase = ref Unsafe.AsRef<TPrestepData>(typeBatch.PrestepData.Memory);
            ref var bodyReferencesBase = ref Unsafe.AsRef<TwoBodyReferences>(typeBatch.BodyReferences.Memory);
            ref var projectionBase = ref Unsafe.AsRef<TProjection>(typeBatch.Projection.Memory);
            ref var accumulatedImpulsesBase = ref Unsafe.AsRef<TAccumulatedImpulse>(typeBatch.AccumulatedImpulses.Memory);
            var function = default(TConstraintFunctions);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref Unsafe.Add(ref prestepBase, i);
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                ref var references = ref Unsafe.Add(ref bodyReferencesBase, i);
                var count = GetCountInBundle(ref typeBatch, i);
                bodies.GatherState(ref references, count, out var orientationA, out var wsvA, out var inertiaA, out var ab, out var orientationB, out var wsvB, out var inertiaB);
                function.Prestep(orientationA, inertiaA, ab, orientationB, inertiaB, dt, inverseDt, ref prestep, out projection);
                function.WarmStart(ref wsvA, ref wsvB, ref projection, ref accumulatedImpulses);
                bodies.ScatterVelocities(ref wsvA, ref wsvB, ref references, count);
            }
        }
        public override void JacobiPrestep2(ref TypeBatch typeBatch, Bodies bodies, ref FallbackBatch jacobiBatch, ref FallbackTypeBatchResults jacobiResults, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            throw new NotImplementedException();
        }
    }

    public abstract class TwoBodyContactTypeProcessor<TPrestepData, TProjection, TAccumulatedImpulse, TConstraintFunctions>
        : TwoBodyTypeProcessor<TPrestepData, TProjection, TAccumulatedImpulse, TConstraintFunctions>
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
                var count = GetCountInBundle(ref typeBatch, i);
                bodies.GatherState(ref references, count, out var orientationA, out var wsvA, out var inertiaA, out var ab, out var orientationB, out var wsvB, out var inertiaB);
                function.IncrementallyUpdateContactData(dtWide, wsvA, wsvB, ref prestep);
            }
        }
    }
}
