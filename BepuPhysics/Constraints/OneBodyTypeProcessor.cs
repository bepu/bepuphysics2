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
    public interface IOneBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse>
    {
        void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA,
            ref TPrestepData prestep, ref TAccumulatedImpulse accumulatedImpulses, ref BodyVelocityWide wsvA);
        void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, float dt, float inverseDt,
            ref TPrestepData prestep, ref TAccumulatedImpulse accumulatedImpulses, ref BodyVelocityWide wsvA);

        /// <summary>
        /// Gets whether this constraint type requires incremental updates for each substep taken beyond the first.
        /// </summary>
        bool RequiresIncrementalSubstepUpdates { get; }
        void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide velocity, ref TPrestepData prestepData);
    }

    //Not a big fan of complex generic-filled inheritance hierarchies, but this is the shortest evolutionary step to removing duplicates.
    //There are some other options if this inheritance hierarchy gets out of control.
    /// <summary>
    /// Shared implementation across all one body constraints.
    /// </summary>
    public abstract class OneBodyTypeProcessor<TPrestepData, TAccumulatedImpulse, TConstraintFunctions,
        TWarmStartAccessFilterA, TSolveAccessFilterA>
        : TypeProcessor<Vector<int>, TPrestepData, TAccumulatedImpulse>
        where TPrestepData : unmanaged where TAccumulatedImpulse : unmanaged
        where TConstraintFunctions : unmanaged, IOneBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse>
        where TWarmStartAccessFilterA : unmanaged, IBodyAccessFilter
        where TSolveAccessFilterA : unmanaged, IBodyAccessFilter
    {
        protected sealed override int InternalBodiesPerConstraint => 1;

        struct OneBodySortKeyGenerator : ISortKeyGenerator<Vector<int>>
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int GetSortKey(int constraintIndex, ref Buffer<Vector<int>> bodyReferences)
            {
                BundleIndexing.GetBundleIndices(constraintIndex, out var bundleIndex, out var innerIndex);
                //We sort based on the body references within the constraint. 
                //Note that it is impossible for there to be two references to the same body within a constraint batch, 
                //so there's no need to worry about the case where the comparison is equal.
                return GatherScatter.Get(ref bodyReferences[bundleIndex], innerIndex);
            }
        }

        internal sealed override void GenerateSortKeysAndCopyReferences(
            ref TypeBatch typeBatch,
            int bundleStart, int localBundleStart, int bundleCount,
            int constraintStart, int localConstraintStart, int constraintCount,
            ref int firstSortKey, ref int firstSourceIndex, ref Buffer<byte> bodyReferencesCache)
        {
            GenerateSortKeysAndCopyReferences<OneBodySortKeyGenerator>(
                ref typeBatch,
                bundleStart, localBundleStart, bundleCount,
                constraintStart, localConstraintStart, constraintCount,
                ref firstSortKey, ref firstSourceIndex, ref bodyReferencesCache);
        }

        internal sealed override void VerifySortRegion(ref TypeBatch typeBatch, int bundleStartIndex, int constraintCount, ref Buffer<int> sortedKeys, ref Buffer<int> sortedSourceIndices)
        {
            VerifySortRegion<OneBodySortKeyGenerator>(ref typeBatch, bundleStartIndex, constraintCount, ref sortedKeys, ref sortedSourceIndices);
        }


        //The following covers the common loop logic for all one body constraints. Each iteration invokes the warm start function type.
        //This abstraction should, in theory, have zero overhead if the implementation of the interface is in a struct with aggressive inlining.

        //By providing the overrides at this level, the concrete implementation (assuming it inherits from one of the prestep-providing variants)
        //only has to specify *type* arguments associated with the interface-implementing struct-delegates. It's going to look very strange, but it's low overhead
        //and minimizes per-type duplication.

        public unsafe override void WarmStart<TIntegratorCallbacks, TBatchIntegrationMode, TAllowPoseIntegration>(
            ref TypeBatch typeBatch, ref Buffer<IndexSet> integrationFlags, Bodies bodies, ref TIntegratorCallbacks integratorCallbacks,
            float dt, float inverseDt, int startBundle, int exclusiveEndBundle, int workerIndex)
        {
            var prestepBundles = typeBatch.PrestepData.As<TPrestepData>();
            var bodyReferencesBundles = typeBatch.BodyReferences.As<Vector<int>>();
            var accumulatedImpulsesBundles = typeBatch.AccumulatedImpulses.As<TAccumulatedImpulse>();
            var function = default(TConstraintFunctions);
            ref var states = ref bodies.ActiveSet.SolverStates;
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref prestepBundles[i];
                ref var accumulatedImpulses = ref accumulatedImpulsesBundles[i];
                ref var references = ref bodyReferencesBundles[i];
                GatherAndIntegrate<TIntegratorCallbacks, TBatchIntegrationMode, TWarmStartAccessFilterA, TAllowPoseIntegration>(bodies, ref integratorCallbacks, ref integrationFlags, 0, dt, workerIndex, i, ref references,
                    out var positionA, out var orientationA, out var wsvA, out var inertiaA);

                //if (typeof(TAllowPoseIntegration) == typeof(AllowPoseIntegration))
                //    function.UpdateForNewPose(positionA, orientationA, inertiaA, wsvA, new Vector<float>(dt), accumulatedImpulses, ref prestep);

                function.WarmStart(positionA, orientationA, inertiaA, ref prestep, ref accumulatedImpulses, ref wsvA);

                if (typeof(TBatchIntegrationMode) == typeof(BatchShouldNeverIntegrate))
                {
                    bodies.ScatterVelocities<TWarmStartAccessFilterA>(ref wsvA, ref references);
                }
                else
                {
                    //This batch has some integrators, which means that every bundle is going to gather all velocities.
                    //(We don't make per-bundle determinations about this to avoid an extra branch and instruction complexity, and the difference is very small.)
                    bodies.ScatterVelocities<AccessAll>(ref wsvA, ref references);
                }
            }
        }

        public unsafe override void Solve(ref TypeBatch typeBatch, Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            var prestepBundles = typeBatch.PrestepData.As<TPrestepData>();
            var bodyReferencesBundles = typeBatch.BodyReferences.As<Vector<int>>();
            var accumulatedImpulsesBundles = typeBatch.AccumulatedImpulses.As<TAccumulatedImpulse>();
            var function = default(TConstraintFunctions);
            ref var motionStates = ref bodies.ActiveSet.SolverStates;
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref prestepBundles[i];
                ref var accumulatedImpulses = ref accumulatedImpulsesBundles[i];
                ref var references = ref bodyReferencesBundles[i];
                bodies.GatherState<TSolveAccessFilterA>(references, true, out var positionA, out var orientationA, out var wsvA, out var inertiaA);

                function.Solve(positionA, orientationA, inertiaA, dt, inverseDt, ref prestep, ref accumulatedImpulses, ref wsvA);

                bodies.ScatterVelocities<TSolveAccessFilterA>(ref wsvA, ref references);
            }
        }

        public override bool RequiresIncrementalSubstepUpdates => default(TConstraintFunctions).RequiresIncrementalSubstepUpdates;
        public unsafe override void IncrementallyUpdateForSubstep(ref TypeBatch typeBatch, Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            var prestepBundles = typeBatch.PrestepData.As<TPrestepData>();
            var bodyReferencesBundles = typeBatch.BodyReferences.As<Vector<int>>();
            var function = default(TConstraintFunctions);
            var dtWide = new Vector<float>(dt);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref prestepBundles[i];
                ref var references = ref bodyReferencesBundles[i];
                bodies.GatherState<AccessOnlyVelocity>(references, true, out _, out _, out var wsvA, out _);
                function.IncrementallyUpdateForSubstep(dtWide, wsvA, ref prestep);
            }
        }
    }

    public abstract class OneBodyContactTypeProcessor<TPrestepData, TAccumulatedImpulse, TConstraintFunctions>
        : OneBodyTypeProcessor<TPrestepData, TAccumulatedImpulse, TConstraintFunctions, AccessNoPose, AccessNoPose>
        where TPrestepData : unmanaged where TAccumulatedImpulse : unmanaged
        where TConstraintFunctions : unmanaged, IOneBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse>
    {

    }

}
