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
    public struct FourBodyReferences
    {
        public Vector<int> IndexA;
        public Vector<int> IndexB;
        public Vector<int> IndexC;
        public Vector<int> IndexD;
    }

    /// <summary>
    /// Prestep, warm start and solve iteration functions for a four body constraint type.
    /// </summary>
    /// <typeparam name="TPrestepData">Type of the prestep data used by the constraint.</typeparam>
    /// <typeparam name="TAccumulatedImpulse">Type of the accumulated impulses used by the constraint.</typeparam>
    public interface IFourBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse>
    {
        void WarmStart(
            in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA,
            in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB,
            in Vector3Wide positionC, in QuaternionWide orientationC, in BodyInertiaWide inertiaC,
            in Vector3Wide positionD, in QuaternionWide orientationD, in BodyInertiaWide inertiaD,
            ref TPrestepData prestep, ref TAccumulatedImpulse accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB, ref BodyVelocityWide wsvC, ref BodyVelocityWide wsvD);
        void Solve(
            in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA,
            in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB,
            in Vector3Wide positionC, in QuaternionWide orientationC, in BodyInertiaWide inertiaC,
            in Vector3Wide positionD, in QuaternionWide orientationD, in BodyInertiaWide inertiaD, float dt, float inverseDt,
            ref TPrestepData prestep, ref TAccumulatedImpulse accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB, ref BodyVelocityWide wsvC, ref BodyVelocityWide wsvD);

        /// <summary>
        /// Gets whether this constraint type requires incremental updates for each substep taken beyond the first.
        /// </summary>
        bool RequiresIncrementalSubstepUpdates { get; }
        void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, in BodyVelocityWide wsvC, in BodyVelocityWide wsvD, ref TPrestepData prestepData);
    }

    /// <summary>
    /// Shared implementation across all four body constraints.
    /// </summary>
    public abstract class FourBodyTypeProcessor<TPrestepData, TAccumulatedImpulse, TConstraintFunctions,
        TWarmStartAccessFilterA, TWarmStartAccessFilterB, TWarmStartAccessFilterC, TWarmStartAccessFilterD, TSolveAccessFilterA, TSolveAccessFilterB, TSolveAccessFilterC, TSolveAccessFilterD>
        : TypeProcessor<FourBodyReferences, TPrestepData, TAccumulatedImpulse>
        where TPrestepData : unmanaged where TAccumulatedImpulse : unmanaged
        where TConstraintFunctions : unmanaged, IFourBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse>
        where TWarmStartAccessFilterA : unmanaged, IBodyAccessFilter
        where TWarmStartAccessFilterB : unmanaged, IBodyAccessFilter
        where TWarmStartAccessFilterC : unmanaged, IBodyAccessFilter
        where TWarmStartAccessFilterD : unmanaged, IBodyAccessFilter
        where TSolveAccessFilterA : unmanaged, IBodyAccessFilter
        where TSolveAccessFilterB : unmanaged, IBodyAccessFilter
        where TSolveAccessFilterC : unmanaged, IBodyAccessFilter
        where TSolveAccessFilterD : unmanaged, IBodyAccessFilter
    {
        protected sealed override int InternalBodiesPerConstraint => 4;

        struct FourBodySortKeyGenerator : ISortKeyGenerator<FourBodyReferences>
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int GetSortKey(int constraintIndex, ref Buffer<FourBodyReferences> bodyReferences)
            {
                BundleIndexing.GetBundleIndices(constraintIndex, out var bundleIndex, out var innerIndex);
                ref var bundleReferences = ref bodyReferences[bundleIndex];
                //We sort based on the body references within the constraint. 
                //Sort based on the smallest body index in a constraint. Note that it is impossible for there to be two references to the same body within a constraint batch, 
                //so there's no need to worry about the case where the comparison is equal.
                ref var bundle = ref bodyReferences[bundleIndex];
                //Avoiding some branches and scalar work. Could do better with platform intrinsics on some architectures.
                var bundleMin = Vector.Min(Vector.Min(bundle.IndexA, bundle.IndexB), Vector.Min(bundle.IndexC, bundle.IndexD));
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
            GenerateSortKeysAndCopyReferences<FourBodySortKeyGenerator>(
                ref typeBatch,
                bundleStart, localBundleStart, bundleCount,
                constraintStart, localConstraintStart, constraintCount,
                ref firstSortKey, ref firstSourceIndex, ref bodyReferencesCache);
        }

        internal sealed override void VerifySortRegion(ref TypeBatch typeBatch, int bundleStartIndex, int constraintCount, ref Buffer<int> sortedKeys, ref Buffer<int> sortedSourceIndices)
        {
            VerifySortRegion<FourBodySortKeyGenerator>(ref typeBatch, bundleStartIndex, constraintCount, ref sortedKeys, ref sortedSourceIndices);
        }

        public unsafe override void WarmStart<TIntegratorCallbacks, TBatchIntegrationMode, TAllowPoseIntegration>(
            ref TypeBatch typeBatch, ref Buffer<IndexSet> integrationFlags, Bodies bodies, ref TIntegratorCallbacks integratorCallbacks,
            float dt, float inverseDt, int startBundle, int exclusiveEndBundle, int workerIndex)
        {
            var prestepBundles = typeBatch.PrestepData.As<TPrestepData>();
            var bodyReferencesBundles = typeBatch.BodyReferences.As<FourBodyReferences>();
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
                GatherAndIntegrate<TIntegratorCallbacks, TBatchIntegrationMode, TWarmStartAccessFilterD, TAllowPoseIntegration>(bodies, ref integratorCallbacks, ref integrationFlags, 3, dt, workerIndex, i, ref references.IndexD,
                    out var positionD, out var orientationD, out var wsvD, out var inertiaD);

                //if (typeof(TAllowPoseIntegration) == typeof(AllowPoseIntegration))
                //    function.UpdateForNewPose(positionA, orientationA, inertiaA, wsvA, positionB, orientationB, inertiaB, wsvB, positionC, orientationC, inertiaC, wsvC, positionD, orientationD, inertiaD, wsvD, new Vector<float>(dt), accumulatedImpulses, ref prestep);

                function.WarmStart(positionA, orientationA, inertiaA, positionB, orientationB, inertiaB, positionC, orientationC, inertiaC, positionD, orientationD, inertiaD, ref prestep, ref accumulatedImpulses, ref wsvA, ref wsvB, ref wsvC, ref wsvD);

                if (typeof(TBatchIntegrationMode) == typeof(BatchShouldNeverIntegrate))
                {
                    bodies.ScatterVelocities<TWarmStartAccessFilterA>(ref wsvA, ref references.IndexA);
                    bodies.ScatterVelocities<TWarmStartAccessFilterB>(ref wsvB, ref references.IndexB);
                    bodies.ScatterVelocities<TWarmStartAccessFilterC>(ref wsvC, ref references.IndexC);
                    bodies.ScatterVelocities<TWarmStartAccessFilterD>(ref wsvD, ref references.IndexD);
                }
                else
                {
                    //This batch has some integrators, which means that every bundle is going to gather all velocities.
                    //(We don't make per-bundle determinations about this to avoid an extra branch and instruction complexity, and the difference is very small.)
                    bodies.ScatterVelocities<AccessAll>(ref wsvA, ref references.IndexA);
                    bodies.ScatterVelocities<AccessAll>(ref wsvB, ref references.IndexB);
                    bodies.ScatterVelocities<AccessAll>(ref wsvC, ref references.IndexC);
                    bodies.ScatterVelocities<AccessAll>(ref wsvD, ref references.IndexD);
                }

            }
        }

        public unsafe override void Solve(ref TypeBatch typeBatch, Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            var prestepBundles = typeBatch.PrestepData.As<TPrestepData>();
            var bodyReferencesBundles = typeBatch.BodyReferences.As<FourBodyReferences>();
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
                bodies.GatherState<TSolveAccessFilterD>(references.IndexD, true, out var positionD, out var orientationD, out var wsvD, out var inertiaD);

                function.Solve(positionA, orientationA, inertiaA, positionB, orientationB, inertiaB, positionC, orientationC, inertiaC, positionD, orientationD, inertiaD, dt, inverseDt, ref prestep, ref accumulatedImpulses, ref wsvA, ref wsvB, ref wsvC, ref wsvD);

                bodies.ScatterVelocities<TSolveAccessFilterA>(ref wsvA, ref references.IndexA);
                bodies.ScatterVelocities<TSolveAccessFilterB>(ref wsvB, ref references.IndexB);
                bodies.ScatterVelocities<TSolveAccessFilterC>(ref wsvC, ref references.IndexC);
                bodies.ScatterVelocities<TSolveAccessFilterD>(ref wsvD, ref references.IndexD);
            }
        }

        public override bool RequiresIncrementalSubstepUpdates => default(TConstraintFunctions).RequiresIncrementalSubstepUpdates;
        public unsafe override void IncrementallyUpdateForSubstep(ref TypeBatch typeBatch, Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            var prestepBundles = typeBatch.PrestepData.As<TPrestepData>();
            var bodyReferencesBundles = typeBatch.BodyReferences.As<FourBodyReferences>();
            var function = default(TConstraintFunctions);
            var dtWide = new Vector<float>(dt);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref prestepBundles[i];
                ref var references = ref bodyReferencesBundles[i];
                bodies.GatherState<AccessOnlyVelocity>(references.IndexA, true, out _, out _, out var wsvA, out _);
                bodies.GatherState<AccessOnlyVelocity>(references.IndexB, true, out _, out _, out var wsvB, out _);
                bodies.GatherState<AccessOnlyVelocity>(references.IndexC, true, out _, out _, out var wsvC, out _);
                bodies.GatherState<AccessOnlyVelocity>(references.IndexD, true, out _, out _, out var wsvD, out _);
                function.IncrementallyUpdateForSubstep(dtWide, wsvA, wsvB, wsvC, wsvD, ref prestep);
            }
        }

    }
}
