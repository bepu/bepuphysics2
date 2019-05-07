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
    /// <typeparam name="TProjection">Type of the projection to input.</typeparam>
    public interface IFourBodyConstraintFunctions<TPrestepData, TProjection, TAccumulatedImpulse>
    {
        void Prestep(Bodies bodies, ref FourBodyReferences bodyReferences, int count, float dt, float inverseDt,
            ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref BodyInertias inertiaC, ref BodyInertias inertiaD, ref TPrestepData prestepData, out TProjection projection);
        void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref BodyVelocities velocityC, ref BodyVelocities velocityD, ref TProjection projection, ref TAccumulatedImpulse accumulatedImpulse);
        void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref BodyVelocities velocityC, ref BodyVelocities velocityD, ref TProjection projection, ref TAccumulatedImpulse accumulatedImpulse);
    }

    /// <summary>
    /// Shared implementation across all four body constraints.
    /// </summary>
    public abstract class FourBodyTypeProcessor<TPrestepData, TProjection, TAccumulatedImpulse, TConstraintFunctions>
        : TypeProcessor<FourBodyReferences, TPrestepData, TProjection, TAccumulatedImpulse>
        where TPrestepData : struct where TProjection : struct where TAccumulatedImpulse : struct
        where TConstraintFunctions : struct, IFourBodyConstraintFunctions<TPrestepData, TProjection, TAccumulatedImpulse>
    {
        protected sealed override int InternalBodiesPerConstraint => 4;

        public sealed unsafe override void EnumerateConnectedBodyIndices<TEnumerator>(ref TypeBatch typeBatch, int indexInTypeBatch, ref TEnumerator enumerator)
        {
            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var constraintBundleIndex, out var constraintInnerIndex);
            ref var indices = ref GatherScatter.GetOffsetInstance(ref Buffer<FourBodyReferences>.Get(typeBatch.BodyReferences.Memory, constraintBundleIndex), constraintInnerIndex);
            //Note that we hold a reference to the indices. That's important if the loop body mutates indices.
            enumerator.LoopBody(GatherScatter.GetFirst(ref indices.IndexA));
            enumerator.LoopBody(GatherScatter.GetFirst(ref indices.IndexB));
            enumerator.LoopBody(GatherScatter.GetFirst(ref indices.IndexC));
            enumerator.LoopBody(GatherScatter.GetFirst(ref indices.IndexD));
        }
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
            ref int firstSortKey, ref int firstSourceIndex, ref RawBuffer bodyReferencesCache)
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

        public unsafe override void Prestep(ref TypeBatch typeBatch, Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            ref var prestepBase = ref Unsafe.AsRef<TPrestepData>(typeBatch.PrestepData.Memory);
            ref var bodyReferencesBase = ref Unsafe.AsRef<FourBodyReferences>(typeBatch.BodyReferences.Memory);
            ref var projectionBase = ref Unsafe.AsRef<TProjection>(typeBatch.Projection.Memory);
            var function = default(TConstraintFunctions);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref Unsafe.Add(ref prestepBase, i);
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var references = ref Unsafe.Add(ref bodyReferencesBase, i);
                var count = GetCountInBundle(ref typeBatch, i);
                bodies.GatherInertia(ref references, count, out var inertiaA, out var inertiaB, out var inertiaC, out var inertiaD);
                function.Prestep(bodies, ref references, count, dt, inverseDt, ref inertiaA, ref inertiaB, ref inertiaC, ref inertiaD, ref prestep, out projection);
            }
        }

        public unsafe override void WarmStart(ref TypeBatch typeBatch, ref Buffer<BodyVelocity> bodyVelocities, int startBundle, int exclusiveEndBundle)
        {
            ref var bodyReferencesBase = ref Unsafe.AsRef<FourBodyReferences>(typeBatch.BodyReferences.Memory);
            ref var accumulatedImpulsesBase = ref Unsafe.AsRef<TAccumulatedImpulse>(typeBatch.AccumulatedImpulses.Memory);
            ref var projectionBase = ref Unsafe.AsRef<TProjection>(typeBatch.Projection.Memory);
            var function = default(TConstraintFunctions);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                int count = GetCountInBundle(ref typeBatch, i);
                Bodies.GatherVelocities(ref bodyVelocities, ref bodyReferences, count, out var wsvA, out var wsvB, out var wsvC, out var wsvD);
                function.WarmStart(ref wsvA, ref wsvB, ref wsvC, ref wsvD, ref projection, ref accumulatedImpulses);
                Bodies.ScatterVelocities(ref wsvA, ref wsvB, ref wsvC, ref wsvD, ref bodyVelocities, ref bodyReferences, count);

            }
        }

        public unsafe override void SolveIteration(ref TypeBatch typeBatch, ref Buffer<BodyVelocity> bodyVelocities, int startBundle, int exclusiveEndBundle)
        {
            ref var bodyReferencesBase = ref Unsafe.AsRef<FourBodyReferences>(typeBatch.BodyReferences.Memory);
            ref var accumulatedImpulsesBase = ref Unsafe.AsRef<TAccumulatedImpulse>(typeBatch.AccumulatedImpulses.Memory);
            ref var projectionBase = ref Unsafe.AsRef<TProjection>(typeBatch.Projection.Memory);
            var function = default(TConstraintFunctions);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                int count = GetCountInBundle(ref typeBatch, i);
                Bodies.GatherVelocities(ref bodyVelocities, ref bodyReferences, count, out var wsvA, out var wsvB, out var wsvC, out var wsvD);
                function.Solve(ref wsvA, ref wsvB, ref wsvC, ref wsvD, ref projection, ref accumulatedImpulses);
                Bodies.ScatterVelocities(ref wsvA, ref wsvB, ref wsvC, ref wsvD, ref bodyVelocities, ref bodyReferences, count);
            }
        }

        public unsafe override void JacobiPrestep(ref TypeBatch typeBatch, Bodies bodies, ref FallbackBatch jacobiBatch, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            ref var prestepBase = ref Unsafe.AsRef<TPrestepData>(typeBatch.PrestepData.Memory);
            ref var bodyReferencesBase = ref Unsafe.AsRef<FourBodyReferences>(typeBatch.BodyReferences.Memory);
            ref var projectionBase = ref Unsafe.AsRef<TProjection>(typeBatch.Projection.Memory);
            var function = default(TConstraintFunctions);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref Unsafe.Add(ref prestepBase, i);
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var references = ref Unsafe.Add(ref bodyReferencesBase, i);
                var count = GetCountInBundle(ref typeBatch, i);
                bodies.GatherInertia(ref references, count, out var inertiaA, out var inertiaB, out var inertiaC, out var inertiaD);
                //Jacobi batches split affected bodies into multiple pieces to guarantee convergence.
                jacobiBatch.GetJacobiScaleForBodies(ref references, count, out var jacobiScaleA, out var jacobiScaleB, out var jacobiScaleC, out var jacobiScaleD);
                Symmetric3x3Wide.Scale(inertiaA.InverseInertiaTensor, jacobiScaleA, out inertiaA.InverseInertiaTensor);
                inertiaA.InverseMass *= jacobiScaleA;
                Symmetric3x3Wide.Scale(inertiaB.InverseInertiaTensor, jacobiScaleB, out inertiaB.InverseInertiaTensor);
                inertiaB.InverseMass *= jacobiScaleB;
                Symmetric3x3Wide.Scale(inertiaC.InverseInertiaTensor, jacobiScaleC, out inertiaC.InverseInertiaTensor);
                inertiaC.InverseMass *= jacobiScaleC;
                Symmetric3x3Wide.Scale(inertiaD.InverseInertiaTensor, jacobiScaleD, out inertiaD.InverseInertiaTensor);
                inertiaD.InverseMass *= jacobiScaleD;
                function.Prestep(bodies, ref references, count, dt, inverseDt, ref inertiaA, ref inertiaB, ref inertiaC, ref inertiaD, ref prestep, out projection);
            }
        }
        public unsafe override void JacobiWarmStart(ref TypeBatch typeBatch, ref Buffer<BodyVelocity> bodyVelocities, ref FallbackTypeBatchResults jacobiResults, int startBundle, int exclusiveEndBundle)
        {
            ref var bodyReferencesBase = ref Unsafe.AsRef<FourBodyReferences>(typeBatch.BodyReferences.Memory);
            ref var accumulatedImpulsesBase = ref Unsafe.AsRef<TAccumulatedImpulse>(typeBatch.AccumulatedImpulses.Memory);
            ref var projectionBase = ref Unsafe.AsRef<TProjection>(typeBatch.Projection.Memory);
            var function = default(TConstraintFunctions);
            ref var jacobiResultsBundlesA = ref jacobiResults.GetVelocitiesForBody(0);
            ref var jacobiResultsBundlesB = ref jacobiResults.GetVelocitiesForBody(1);
            ref var jacobiResultsBundlesC = ref jacobiResults.GetVelocitiesForBody(2);
            ref var jacobiResultsBundlesD = ref jacobiResults.GetVelocitiesForBody(3);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                int count = GetCountInBundle(ref typeBatch, i);
                ref var wsvA = ref jacobiResultsBundlesA[i];
                ref var wsvB = ref jacobiResultsBundlesB[i];
                ref var wsvC = ref jacobiResultsBundlesC[i];
                ref var wsvD = ref jacobiResultsBundlesD[i];
                Bodies.GatherVelocities(ref bodyVelocities, ref bodyReferences, count, out wsvA, out wsvB, out wsvC, out wsvD);
                function.WarmStart(ref wsvA, ref wsvB, ref wsvC, ref wsvD, ref projection, ref accumulatedImpulses);
            }
        }
        public unsafe override void JacobiSolveIteration(ref TypeBatch typeBatch, ref Buffer<BodyVelocity> bodyVelocities, ref FallbackTypeBatchResults jacobiResults, int startBundle, int exclusiveEndBundle)
        {
            ref var bodyReferencesBase = ref Unsafe.AsRef<FourBodyReferences>(typeBatch.BodyReferences.Memory);
            ref var accumulatedImpulsesBase = ref Unsafe.AsRef<TAccumulatedImpulse>(typeBatch.AccumulatedImpulses.Memory);
            ref var projectionBase = ref Unsafe.AsRef<TProjection>(typeBatch.Projection.Memory);
            var function = default(TConstraintFunctions);
            ref var jacobiResultsBundlesA = ref jacobiResults.GetVelocitiesForBody(0);
            ref var jacobiResultsBundlesB = ref jacobiResults.GetVelocitiesForBody(1);
            ref var jacobiResultsBundlesC = ref jacobiResults.GetVelocitiesForBody(2);
            ref var jacobiResultsBundlesD = ref jacobiResults.GetVelocitiesForBody(3);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                int count = GetCountInBundle(ref typeBatch, i);
                ref var wsvA = ref jacobiResultsBundlesA[i];
                ref var wsvB = ref jacobiResultsBundlesB[i];
                ref var wsvC = ref jacobiResultsBundlesC[i];
                ref var wsvD = ref jacobiResultsBundlesD[i];
                Bodies.GatherVelocities(ref bodyVelocities, ref bodyReferences, count, out wsvA, out wsvB, out wsvC, out wsvD);
                function.Solve(ref wsvA, ref wsvB, ref wsvC, ref wsvD, ref projection, ref accumulatedImpulses);
            }
        }

    }
}
