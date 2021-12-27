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
    /// </summary>
    public struct ThreeBodyReferences
    {
        public Vector<int> IndexA;
        public Vector<int> IndexB;
        public Vector<int> IndexC;
    }

    /// <summary>
    /// Prestep, warm start and solve iteration functions for a three body constraint type.
    /// </summary>
    /// <typeparam name="TPrestepData">Type of the prestep data used by the constraint.</typeparam>
    /// <typeparam name="TAccumulatedImpulse">Type of the accumulated impulses used by the constraint.</typeparam>
    public interface IThreeBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse>
    {
        void WarmStart(
            in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA,
            in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB,
            in Vector3Wide positionC, in QuaternionWide orientationC, in BodyInertiaWide inertiaC,
            ref TPrestepData prestep, ref TAccumulatedImpulse accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB, ref BodyVelocityWide wsvC);
        void Solve(
            in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA,
            in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB,
            in Vector3Wide positionC, in QuaternionWide orientationC, in BodyInertiaWide inertiaC, float dt, float inverseDt,
            ref TPrestepData prestep, ref TAccumulatedImpulse accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB, ref BodyVelocityWide wsvC);

        /// <summary>
        /// Gets whether this constraint type requires incremental updates for each substep taken beyond the first.
        /// </summary>
        bool RequiresIncrementalSubstepUpdates { get; }
        void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, in BodyVelocityWide wsvC, ref TPrestepData prestepData);
    }

    /// <summary>
    /// Shared implementation across all three body constraints.
    /// </summary>
    public abstract class ThreeBodyTypeProcessor<TPrestepData, TAccumulatedImpulse, TConstraintFunctions,
        TWarmStartAccessFilterA, TWarmStartAccessFilterB, TWarmStartAccessFilterC, TSolveAccessFilterA, TSolveAccessFilterB, TSolveAccessFilterC>
        : TypeProcessor<ThreeBodyReferences, TPrestepData, TAccumulatedImpulse>
        where TPrestepData : unmanaged where TAccumulatedImpulse : unmanaged
        where TConstraintFunctions : unmanaged, IThreeBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse>
        where TWarmStartAccessFilterA : unmanaged, IBodyAccessFilter
        where TWarmStartAccessFilterB : unmanaged, IBodyAccessFilter
        where TWarmStartAccessFilterC : unmanaged, IBodyAccessFilter
        where TSolveAccessFilterA : unmanaged, IBodyAccessFilter
        where TSolveAccessFilterB : unmanaged, IBodyAccessFilter
        where TSolveAccessFilterC : unmanaged, IBodyAccessFilter
    {
        protected sealed override int InternalBodiesPerConstraint => 3;

        struct ThreeBodySortKeyGenerator : ISortKeyGenerator<ThreeBodyReferences>
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int GetSortKey(int constraintIndex, ref Buffer<ThreeBodyReferences> bodyReferences)
            {
                BundleIndexing.GetBundleIndices(constraintIndex, out var bundleIndex, out var innerIndex);
                ref var bundleReferences = ref bodyReferences[bundleIndex];
                //We sort based on the body references within the constraint. 
                //Sort based on the smallest body index in a constraint. Note that it is impossible for there to be two references to the same body within a constraint batch, 
                //so there's no need to worry about the case where the comparison is equal.
                ref var bundle = ref bodyReferences[bundleIndex];
                //Avoiding some branches and scalar work. Could do better with platform intrinsics on some architectures.
                var bundleMin = Vector.Min(Vector.Min(bundle.IndexA, bundle.IndexB), bundle.IndexC);
                return GatherScatter.Get(ref bundleMin, innerIndex);
                //TODO: Note that we could do quite a bit better by generating sort keys in a fully vectorized way. Would require some changes in the caller, but it's doable-
                //these sorts are done across contiguous regions, after all.
                //Not a huge deal regardless.
            }
        }


        internal sealed override void GenerateSortKeysAndCopyReferences(
            ref TypeBatch typeBatch,
            int bundleStart, int localBundleStart, int bundleCount,
            int constraintStart, int localConstraintStart, int constraintCount,
            ref int firstSortKey, ref int firstSourceIndex, ref Buffer<byte> bodyReferencesCache)
        {
            GenerateSortKeysAndCopyReferences<ThreeBodySortKeyGenerator>(
                ref typeBatch,
                bundleStart, localBundleStart, bundleCount,
                constraintStart, localConstraintStart, constraintCount,
                ref firstSortKey, ref firstSourceIndex, ref bodyReferencesCache);
        }

        internal sealed override void VerifySortRegion(ref TypeBatch typeBatch, int bundleStartIndex, int constraintCount, ref Buffer<int> sortedKeys, ref Buffer<int> sortedSourceIndices)
        {
            VerifySortRegion<ThreeBodySortKeyGenerator>(ref typeBatch, bundleStartIndex, constraintCount, ref sortedKeys, ref sortedSourceIndices);
        }
       
        public unsafe override void WarmStart<TIntegratorCallbacks, TBatchIntegrationMode, TAllowPoseIntegration>(
            ref TypeBatch typeBatch, ref Buffer<IndexSet> integrationFlags, Bodies bodies, ref TIntegratorCallbacks integratorCallbacks,
            float dt, float inverseDt, int startBundle, int exclusiveEndBundle, int workerIndex)
        {
            var prestepBundles = typeBatch.PrestepData.As<TPrestepData>();
            var bodyReferencesBundles = typeBatch.BodyReferences.As<ThreeBodyReferences>();
            var accumulatedImpulsesBundles = typeBatch.AccumulatedImpulses.As<TAccumulatedImpulse>();
            var function = default(TConstraintFunctions);
            ref var states = ref bodies.ActiveSet.SolverStates;
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref prestepBundles[i];
                ref var accumulatedImpulses = ref accumulatedImpulsesBundles[i];
                ref var references = ref bodyReferencesBundles[i];
                GatherAndIntegrate<TIntegratorCallbacks, TBatchIntegrationMode, TWarmStartAccessFilterA, TAllowPoseIntegration>(bodies, ref integratorCallbacks, ref integrationFlags, 0, dt, workerIndex, i, ref references.IndexA,
                    out var positionA, out var orientationA, out var wsvA, out var inertiaA);
                GatherAndIntegrate<TIntegratorCallbacks, TBatchIntegrationMode, TWarmStartAccessFilterB, TAllowPoseIntegration>(bodies, ref integratorCallbacks, ref integrationFlags, 1, dt, workerIndex, i, ref references.IndexB,
                    out var positionB, out var orientationB, out var wsvB, out var inertiaB);
                GatherAndIntegrate<TIntegratorCallbacks, TBatchIntegrationMode, TWarmStartAccessFilterC, TAllowPoseIntegration>(bodies, ref integratorCallbacks, ref integrationFlags, 2, dt, workerIndex, i, ref references.IndexC,
                    out var positionC, out var orientationC, out var wsvC, out var inertiaC);

                //if (typeof(TAllowPoseIntegration) == typeof(AllowPoseIntegration))
                //    function.UpdateForNewPose(positionA, orientationA, inertiaA, wsvA, positionB, orientationB, inertiaB, wsvB, positionC, orientationC, inertiaC, wsvC, new Vector<float>(dt), accumulatedImpulses, ref prestep);

                function.WarmStart(positionA, orientationA, inertiaA, positionB, orientationB, inertiaB, positionC, orientationC, inertiaC, ref prestep, ref accumulatedImpulses, ref wsvA, ref wsvB, ref wsvC);

                if (typeof(TBatchIntegrationMode) == typeof(BatchShouldNeverIntegrate))
                {
                    bodies.ScatterVelocities<TWarmStartAccessFilterA>(ref wsvA, ref references.IndexA);
                    bodies.ScatterVelocities<TWarmStartAccessFilterB>(ref wsvB, ref references.IndexB);
                    bodies.ScatterVelocities<TWarmStartAccessFilterC>(ref wsvC, ref references.IndexC);
                }
                else
                {
                    //This batch has some integrators, which means that every bundle is going to gather all velocities.
                    //(We don't make per-bundle determinations about this to avoid an extra branch and instruction complexity, and the difference is very small.)
                    bodies.ScatterVelocities<AccessAll>(ref wsvA, ref references.IndexA);
                    bodies.ScatterVelocities<AccessAll>(ref wsvB, ref references.IndexB);
                    bodies.ScatterVelocities<AccessAll>(ref wsvC, ref references.IndexC);
                }

            }
        }

        public unsafe override void Solve(ref TypeBatch typeBatch, Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            var prestepBundles = typeBatch.PrestepData.As<TPrestepData>();
            var bodyReferencesBundles = typeBatch.BodyReferences.As<ThreeBodyReferences>();
            var accumulatedImpulsesBundles = typeBatch.AccumulatedImpulses.As<TAccumulatedImpulse>();
            var function = default(TConstraintFunctions);
            ref var motionStates = ref bodies.ActiveSet.SolverStates;
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref prestepBundles[i];
                ref var accumulatedImpulses = ref accumulatedImpulsesBundles[i];
                ref var references = ref bodyReferencesBundles[i];
                bodies.GatherState<TSolveAccessFilterA>(references.IndexA, true, out var positionA, out var orientationA, out var wsvA, out var inertiaA);
                bodies.GatherState<TSolveAccessFilterB>(references.IndexB, true, out var positionB, out var orientationB, out var wsvB, out var inertiaB);
                bodies.GatherState<TSolveAccessFilterC>(references.IndexC, true, out var positionC, out var orientationC, out var wsvC, out var inertiaC);

                function.Solve(positionA, orientationA, inertiaA, positionB, orientationB, inertiaB, positionC, orientationC, inertiaC, dt, inverseDt, ref prestep, ref accumulatedImpulses, ref wsvA, ref wsvB, ref wsvC);

                bodies.ScatterVelocities<TSolveAccessFilterA>(ref wsvA, ref references.IndexA);
                bodies.ScatterVelocities<TSolveAccessFilterB>(ref wsvB, ref references.IndexB);
                bodies.ScatterVelocities<TSolveAccessFilterC>(ref wsvC, ref references.IndexC);
            }
        }

        public override bool RequiresIncrementalSubstepUpdates => default(TConstraintFunctions).RequiresIncrementalSubstepUpdates;
        public unsafe override void IncrementallyUpdateForSubstep(ref TypeBatch typeBatch, Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            var prestepBundles = typeBatch.PrestepData.As<TPrestepData>();
            var bodyReferencesBundles = typeBatch.BodyReferences.As<ThreeBodyReferences>();
            var function = default(TConstraintFunctions);
            var dtWide = new Vector<float>(dt);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref prestepBundles[i];
                ref var references = ref bodyReferencesBundles[i];
                bodies.GatherState<AccessOnlyVelocity>(references.IndexA, true, out _, out _, out var wsvA, out _);
                bodies.GatherState<AccessOnlyVelocity>(references.IndexB, true, out _, out _, out var wsvB, out _);
                bodies.GatherState<AccessOnlyVelocity>(references.IndexC, true, out _, out _, out var wsvC, out _);
                function.IncrementallyUpdateForSubstep(dtWide, wsvA, wsvB, wsvC, ref prestep);
            }
        }
    }
}
