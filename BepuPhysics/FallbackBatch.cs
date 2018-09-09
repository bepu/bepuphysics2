using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics
{
    /// <summary>
    /// Marks a type as able to provide information about a jacobi fallback batch.
    /// </summary>
    public interface IJacobiBatchInformation
    {
        int GetJacobiConstraintCountForBody(int bodyIndex);

    }
    public struct FallbackTypeBatchResults
    {
        public Buffer<Buffer<BodyVelocities>> BodyVelocities;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Buffer<BodyVelocities> GetVelocitiesForBody(int slotIndex)
        {
            return ref BodyVelocities[slotIndex];
        }
    }

    /// <summary>
    /// Contains constraints that could not belong to any lower constraint batch due to their involved bodies. All of the contained constraints will be solved using a fallback solver that
    /// trades rigidity for parallelism.
    /// </summary>
    public struct FallbackBatch
    {
        public struct FallbackReference
        {
            public int ConstraintHandle;
            public int TypeIndex;
            public int IndexInTypeBatch;
            public int IndexInConstraint;
        }

        //Every body in the fallback batch must track what constraints are associated with it. These tables must be maintained as constraints are added and removed.
        //Note that this dictionary contains active set body *indices* while active, but body *handles* when associated with an inactive set.
        //This is consistent with the body references stored by active/inactive constraints.
        //Note that this is a dictionary of *sets*. This is because fallback batches are expected to be used in pathological cases where there are many constraints associated with
        //a single body. There are likely to be too many constraints for list-based containment/removal to be faster than the set implementation.
        QuickDictionary<int, QuickSet<FallbackReference, Buffer<FallbackReference>, Buffer<int>, FallbackReferenceComparer>,
            Buffer<int>, Buffer<QuickSet<FallbackReference, Buffer<FallbackReference>, Buffer<int>, FallbackReferenceComparer>>, Buffer<int>, PrimitiveComparer<int>> bodyConstraintReferences;
        //(but is this really ENOUGH generics?)

        interface IBodyReferenceGetter
        {
            int GetBodyReference(Bodies bodies, int handle);
        }
        struct ActiveSetGetter : IBodyReferenceGetter
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int GetBodyReference(Bodies bodies, int bodyHandle)
            {
                ref var bodyLocation = ref bodies.HandleToLocation[bodyHandle];
                Debug.Assert(bodyLocation.SetIndex == 0, "When creating a fallback batch for the active set, all bodies associated with it must be active.");
                return bodyLocation.Index;
            }
        }
        struct InactiveSetGetter : IBodyReferenceGetter
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int GetBodyReference(Bodies bodies, int bodyHandle)
            {
                ref var bodyLocation = ref bodies.HandleToLocation[bodyHandle];
                Debug.Assert(bodyLocation.SetIndex == 0, "When creating a fallback batch for the active set, all bodies associated with it must be active.");
                return bodyLocation.Index;
            }
        }

        struct FallbackReferenceComparer : IEqualityComparerRef<FallbackReference>
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool Equals(ref FallbackReference a, ref FallbackReference b)
            {
                return a.ConstraintHandle == b.ConstraintHandle;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int Hash(ref FallbackReference item)
            {
                return item.ConstraintHandle;
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void Allocate<TBodyReferenceGetter>(int handle, ref int constraintBodyHandles, int bodyCount, Bodies bodies,
           int typeId, ref ConstraintBatch batch, BufferPool pool, ref ConstraintReference reference, TBodyReferenceGetter bodyReferenceGetter, int minimumReferenceCapacity)
            where TBodyReferenceGetter : struct, IBodyReferenceGetter
        {
            var fallbackPool = pool.SpecializeFor<FallbackReference>();
            var intPool = pool.SpecializeFor<int>();
            var referenceListPool = pool.SpecializeFor<QuickSet<FallbackReference, Buffer<FallbackReference>, Buffer<int>, FallbackReferenceComparer>>();
            var minimumReferencePower = SpanHelper.GetContainingPowerOf2(minimumReferenceCapacity);
            for (int i = 0; i < bodyCount; ++i)
            {
                var bodyReference = bodyReferenceGetter.GetBodyReference(bodies, Unsafe.Add(ref constraintBodyHandles, i));

                bodyConstraintReferences.EnsureCapacity(bodyConstraintReferences.Count + 1, intPool, referenceListPool, intPool);
                var bodyAlreadyListed = bodyConstraintReferences.GetTableIndices(ref bodyReference, out var tableIndex, out var elementIndex);
                ref var constraintReferences = ref bodyConstraintReferences.Values[elementIndex];

                if (!bodyAlreadyListed)
                {
                    //The body is not already contained. Create a list for it.
                    QuickSet<FallbackReference, Buffer<FallbackReference>, Buffer<int>, FallbackReferenceComparer>.Create(fallbackPool, intPool, minimumReferencePower, 2, out constraintReferences);
                }
                var fallbackReference = new FallbackReference
                {
                    ConstraintHandle = handle,
                    TypeIndex = batch.TypeIndexToTypeBatchIndex[typeId],
                    IndexInTypeBatch = reference.IndexInTypeBatch,
                    IndexInConstraint = i
                };
                constraintReferences.Add(ref fallbackReference, fallbackPool, intPool);
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void AllocateForActive(int handle, ref int constraintBodyHandles, int bodyCount, Bodies bodies,
           int typeId, ref ConstraintBatch batch, BufferPool pool, ref ConstraintReference reference, int minimumReferenceCapacity = 8)
        {
            Allocate(handle, ref constraintBodyHandles, bodyCount, bodies, typeId, ref batch, pool, ref reference, new ActiveSetGetter(), minimumReferenceCapacity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void AllocateForInactive(int handle, ref int constraintBodyHandles, int bodyCount, Bodies bodies,
          int typeId, ref ConstraintBatch batch, BufferPool pool, ref ConstraintReference reference, int minimumReferenceCapacity = 8)
        {
            Allocate(handle, ref constraintBodyHandles, bodyCount, bodies, typeId, ref batch, pool, ref reference, new InactiveSetGetter(), minimumReferenceCapacity);
        }

        public static void AllocateResults(Solver solver, BufferPool pool, ref ConstraintBatch batch, out Buffer<FallbackTypeBatchResults> results)
        {
            pool.Take(batch.TypeBatches.Count, out results);
            for (int i = 0; i < batch.TypeBatches.Count; ++i)
            {
                ref var typeBatch = ref batch.TypeBatches[i];
                var bodiesPerConstraint = solver.TypeProcessors[typeBatch.TypeId].BodiesPerConstraint;
                ref var typeBatchResults = ref results[i];
                pool.Take(bodiesPerConstraint, out typeBatchResults.BodyVelocities);
                for (int j = 0; j < bodiesPerConstraint; ++j)
                {
                    pool.Take(typeBatch.BundleCount, out typeBatchResults.GetVelocitiesForBody(j));
                }
            }
        }

        public static void DisposeResults(Solver solver, BufferPool pool, ref ConstraintBatch batch, ref Buffer<FallbackTypeBatchResults> results)
        {
            for (int i = 0; i < batch.TypeBatches.Count; ++i)
            {
                var bodiesPerConstraint = solver.TypeProcessors[batch.TypeBatches[i].TypeId].BodiesPerConstraint;
                ref var typeBatchResults = ref results[i];
                for (int j = 0; j < bodiesPerConstraint; ++j)
                {
                    pool.ReturnUnsafely(typeBatchResults.GetVelocitiesForBody(j).Id);
                }
                pool.ReturnUnsafely(typeBatchResults.BodyVelocities.Id);
            }
            pool.Return(ref results);
        }        
    }
}
