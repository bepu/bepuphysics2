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
        void Prestep(Bodies bodies, ref Vector<int> bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertia, ref TPrestepData prestepData, out TProjection projection);
        void WarmStart(ref BodyVelocities velocity, ref TProjection projection, ref TAccumulatedImpulse accumulatedImpulse);
        void Solve(ref BodyVelocities velocity, ref TProjection projection, ref TAccumulatedImpulse accumulatedImpulse);
    }

    /// <summary>
    /// Prestep, warm start, solve iteration, and incremental contact update functions for a one body contact constraint type.
    /// </summary>
    /// <typeparam name="TPrestepData">Type of the prestep data used by the constraint.</typeparam>
    /// <typeparam name="TAccumulatedImpulse">Type of the accumulated impulses used by the constraint.</typeparam>
    /// <typeparam name="TProjection">Type of the projection to input.</typeparam>
    public interface IOneBodyContactConstraintFunctions<TPrestepData, TProjection, TAccumulatedImpulse> : IOneBodyConstraintFunctions<TPrestepData, TProjection, TAccumulatedImpulse>
    {
        void IncrementallyUpdateContactData(in Vector<float> dt, in BodyVelocities velocity, ref TPrestepData prestepData);
    }

    //Not a big fan of complex generic-filled inheritance hierarchies, but this is the shortest evolutionary step to removing duplicates.
    //There are some other options if this inheritance hierarchy gets out of control.
    /// <summary>
    /// Shared implementation across all one body constraints.
    /// </summary>
    public abstract class OneBodyTypeProcessor<TPrestepData, TProjection, TAccumulatedImpulse, TConstraintFunctions>
        : TypeProcessor<Vector<int>, TPrestepData, TProjection, TAccumulatedImpulse>
        where TPrestepData : struct where TProjection : struct where TAccumulatedImpulse : struct
        where TConstraintFunctions : struct, IOneBodyConstraintFunctions<TPrestepData, TProjection, TAccumulatedImpulse>
    {
        protected sealed override int InternalBodiesPerConstraint => 1;

        public sealed override void EnumerateConnectedBodyIndices<TEnumerator>(ref TypeBatch typeBatch, int indexInTypeBatch, ref TEnumerator enumerator)
        {
            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var constraintBundleIndex, out var constraintInnerIndex);
            enumerator.LoopBody(GatherScatter.Get(ref Buffer<Vector<int>>.Get(ref typeBatch.BodyReferences, constraintBundleIndex), constraintInnerIndex));
        }


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
            ref int firstSortKey, ref int firstSourceIndex, ref RawBuffer bodyReferencesCache)
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
        public unsafe override void Prestep(ref TypeBatch typeBatch, Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            ref var prestepBase = ref Unsafe.AsRef<TPrestepData>(typeBatch.PrestepData.Memory);
            ref var bodyReferencesBase = ref Unsafe.AsRef<Vector<int>>(typeBatch.BodyReferences.Memory);
            ref var projectionBase = ref Unsafe.AsRef<TProjection>(typeBatch.Projection.Memory);
            var function = default(TConstraintFunctions);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref Unsafe.Add(ref prestepBase, i);
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var references = ref Unsafe.Add(ref bodyReferencesBase, i);
                var count = GetCountInBundle(ref typeBatch, i);
                bodies.GatherInertia(ref references, count, out var inertiaA);
                function.Prestep(bodies, ref references, count, dt, inverseDt, ref inertiaA, ref prestep, out projection);
            }
        }

        public unsafe override void WarmStart(ref TypeBatch typeBatch, ref Buffer<BodyVelocity> bodyVelocities, int startBundle, int exclusiveEndBundle)
        {
            ref var bodyReferencesBase = ref Unsafe.AsRef<Vector<int>>(typeBatch.BodyReferences.Memory);
            ref var accumulatedImpulsesBase = ref Unsafe.AsRef<TAccumulatedImpulse>(typeBatch.AccumulatedImpulses.Memory);
            ref var projectionBase = ref Unsafe.AsRef<TProjection>(typeBatch.Projection.Memory);
            var function = default(TConstraintFunctions);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                int count = GetCountInBundle(ref typeBatch, i);
                Bodies.GatherVelocities(ref bodyVelocities, ref bodyReferences, count, out var wsvA);
                function.WarmStart(ref wsvA, ref projection, ref accumulatedImpulses);
                Bodies.ScatterVelocities(ref wsvA, ref bodyVelocities, ref bodyReferences, count);
            }
        }

        public unsafe override void SolveIteration(ref TypeBatch typeBatch, ref Buffer<BodyVelocity> bodyVelocities, int startBundle, int exclusiveEndBundle)
        {
            ref var bodyReferencesBase = ref Unsafe.AsRef<Vector<int>>(typeBatch.BodyReferences.Memory);
            ref var accumulatedImpulsesBase = ref Unsafe.AsRef<TAccumulatedImpulse>(typeBatch.AccumulatedImpulses.Memory);
            ref var projectionBase = ref Unsafe.AsRef<TProjection>(typeBatch.Projection.Memory);
            var function = default(TConstraintFunctions);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                int count = GetCountInBundle(ref typeBatch, i);
                Bodies.GatherVelocities(ref bodyVelocities, ref bodyReferences, count, out var wsvA);
                function.Solve(ref wsvA, ref projection, ref accumulatedImpulses);
                Bodies.ScatterVelocities(ref wsvA, ref bodyVelocities, ref bodyReferences, count);
            }
        }

        public unsafe override void JacobiPrestep(ref TypeBatch typeBatch, Bodies bodies, ref FallbackBatch jacobiBatch, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            ref var prestepBase = ref Unsafe.AsRef<TPrestepData>(typeBatch.PrestepData.Memory);
            ref var bodyReferencesBase = ref Unsafe.AsRef<Vector<int>>(typeBatch.BodyReferences.Memory);
            ref var projectionBase = ref Unsafe.AsRef<TProjection>(typeBatch.Projection.Memory);
            var function = default(TConstraintFunctions);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref Unsafe.Add(ref prestepBase, i);
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var references = ref Unsafe.Add(ref bodyReferencesBase, i);
                var count = GetCountInBundle(ref typeBatch, i);
                bodies.GatherInertia(ref references, count, out var inertia);
                //Jacobi batches split affected bodies into multiple pieces to guarantee convergence.
                jacobiBatch.GetJacobiScaleForBodies(ref references, count, out var jacobiScale);
                Symmetric3x3Wide.Scale(inertia.InverseInertiaTensor, jacobiScale, out inertia.InverseInertiaTensor);
                inertia.InverseMass *= jacobiScale;
                function.Prestep(bodies, ref references, count, dt, inverseDt, ref inertia, ref prestep, out projection);
            }
        }
        public unsafe override void JacobiWarmStart(ref TypeBatch typeBatch, ref Buffer<BodyVelocity> bodyVelocities, ref FallbackTypeBatchResults jacobiResults, int startBundle, int exclusiveEndBundle)
        {
            ref var bodyReferencesBase = ref Unsafe.AsRef<Vector<int>>(typeBatch.BodyReferences.Memory);
            ref var accumulatedImpulsesBase = ref Unsafe.AsRef<TAccumulatedImpulse>(typeBatch.AccumulatedImpulses.Memory);
            ref var projectionBase = ref Unsafe.AsRef<TProjection>(typeBatch.Projection.Memory);
            var function = default(TConstraintFunctions);
            ref var jacobiResultsBundlesA = ref jacobiResults.GetVelocitiesForBody(0);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                int count = GetCountInBundle(ref typeBatch, i);
                ref var wsvA = ref jacobiResultsBundlesA[i];
                Bodies.GatherVelocities(ref bodyVelocities, ref bodyReferences, count, out wsvA);
                function.WarmStart(ref wsvA, ref projection, ref accumulatedImpulses);
            }
        }
        public unsafe override void JacobiSolveIteration(ref TypeBatch typeBatch, ref Buffer<BodyVelocity> bodyVelocities, ref FallbackTypeBatchResults jacobiResults, int startBundle, int exclusiveEndBundle)
        {
            ref var bodyReferencesBase = ref Unsafe.AsRef<Vector<int>>(typeBatch.BodyReferences.Memory);
            ref var accumulatedImpulsesBase = ref Unsafe.AsRef<TAccumulatedImpulse>(typeBatch.AccumulatedImpulses.Memory);
            ref var projectionBase = ref Unsafe.AsRef<TProjection>(typeBatch.Projection.Memory);
            var function = default(TConstraintFunctions);
            ref var jacobiResultsBundlesA = ref jacobiResults.GetVelocitiesForBody(0);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                int count = GetCountInBundle(ref typeBatch, i);
                ref var wsvA = ref jacobiResultsBundlesA[i];
                Bodies.GatherVelocities(ref bodyVelocities, ref bodyReferences, count, out wsvA);
                function.Solve(ref wsvA, ref projection, ref accumulatedImpulses);
            }
        }

    }

    public abstract class OneBodyContactTypeProcessor<TPrestepData, TProjection, TAccumulatedImpulse, TConstraintFunctions>
        : OneBodyTypeProcessor<TPrestepData, TProjection, TAccumulatedImpulse, TConstraintFunctions>
        where TPrestepData : struct where TProjection : struct where TAccumulatedImpulse : struct
        where TConstraintFunctions : struct, IOneBodyContactConstraintFunctions<TPrestepData, TProjection, TAccumulatedImpulse>
    {
        public unsafe override void IncrementallyUpdateContactData(ref TypeBatch typeBatch, Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            ref var prestepBase = ref Unsafe.AsRef<TPrestepData>(typeBatch.PrestepData.Memory);
            ref var bodyReferencesBase = ref Unsafe.AsRef<Vector<int>>(typeBatch.BodyReferences.Memory);
            ref var bodyVelocities = ref bodies.ActiveSet.Velocities;
            var function = default(TConstraintFunctions);
            var dtWide = new Vector<float>(dt);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref Unsafe.Add(ref prestepBase, i);
                ref var references = ref Unsafe.Add(ref bodyReferencesBase, i);
                var count = GetCountInBundle(ref typeBatch, i);
                Bodies.GatherVelocities(ref bodyVelocities, ref references, count, out var velocityA);
                function.IncrementallyUpdateContactData(dtWide, velocityA, ref prestep);
            }
        }
    }

}
