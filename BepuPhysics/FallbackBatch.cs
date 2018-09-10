using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics
{
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
            public int TypeBatchIndex;
            public int IndexInTypeBatch;
            public int IndexInConstraint;
        }

        /// <summary>
        /// Gets the number of bodies in the fallback batch.
        /// </summary>
        public int BodyCount { get { return bodyConstraintReferences.Count; } }

        //Every body in the fallback batch must track what constraints are associated with it. These tables must be maintained as constraints are added and removed.
        //Note that this dictionary contains active set body *indices* while active, but body *handles* when associated with an inactive set.
        //This is consistent with the body references stored by active/inactive constraints.
        //Note that this is a dictionary of *sets*. This is because fallback batches are expected to be used in pathological cases where there are many constraints associated with
        //a single body. There are likely to be too many constraints for list-based containment/removal to be faster than the set implementation.
        QuickDictionary<int, QuickSet<FallbackReference, Buffer<FallbackReference>, Buffer<int>, FallbackReferenceComparer>,
            Buffer<int>, Buffer<QuickSet<FallbackReference, Buffer<FallbackReference>, Buffer<int>, FallbackReferenceComparer>>, Buffer<int>, PrimitiveComparer<int>> bodyConstraintReferences;
        //(but is this really ENOUGH generics?)

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
           int typeId, ref ConstraintBatch batch, BufferPool pool, ref ConstraintReference reference, TBodyReferenceGetter bodyReferenceGetter, int minimumBodyCapacity, int minimumReferenceCapacity)
            where TBodyReferenceGetter : struct, IBodyReferenceGetter
        {
            var fallbackPool = pool.SpecializeFor<FallbackReference>();
            var intPool = pool.SpecializeFor<int>();
            var referenceListPool = pool.SpecializeFor<QuickSet<FallbackReference, Buffer<FallbackReference>, Buffer<int>, FallbackReferenceComparer>>();
            var minimumReferencePower = SpanHelper.GetContainingPowerOf2(minimumReferenceCapacity);
            if (bodyConstraintReferences.Keys.Allocated)
            {
                //This is conservative since there's no guarantee that we'll actually need to resize at all if these bodies are already present, but that's fine. 
                bodyConstraintReferences.EnsureCapacity(bodyConstraintReferences.Count + bodyCount, intPool, referenceListPool, intPool);
            }
            else
            {
                //bleuaghg
                QuickDictionary<int, QuickSet<FallbackReference, Buffer<FallbackReference>, Buffer<int>, FallbackReferenceComparer>,
                    Buffer<int>, Buffer<QuickSet<FallbackReference, Buffer<FallbackReference>, Buffer<int>, FallbackReferenceComparer>>, Buffer<int>, PrimitiveComparer<int>>.Create(
                    intPool, referenceListPool, intPool, SpanHelper.GetContainingPowerOf2(minimumBodyCapacity), 2, out bodyConstraintReferences);
            }
            for (int i = 0; i < bodyCount; ++i)
            {
                var bodyReference = bodyReferenceGetter.GetBodyReference(bodies, Unsafe.Add(ref constraintBodyHandles, i));

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
                    TypeBatchIndex = batch.TypeIndexToTypeBatchIndex[typeId],
                    IndexInTypeBatch = reference.IndexInTypeBatch,
                    IndexInConstraint = i
                };
                constraintReferences.Add(ref fallbackReference, fallbackPool, intPool);
            }
        }

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


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal unsafe void AllocateForActive(int handle, ref int constraintBodyHandles, int bodyCount, Bodies bodies,
           int typeId, ref ConstraintBatch batch, BufferPool pool, ref ConstraintReference reference, int minimumBodyCapacity = 8, int minimumReferenceCapacity = 8)
        {
            Allocate(handle, ref constraintBodyHandles, bodyCount, bodies, typeId, ref batch, pool, ref reference, new ActiveSetGetter(), minimumBodyCapacity, minimumReferenceCapacity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void AllocateForInactive(int handle, ref int constraintBodyHandles, int bodyCount, Bodies bodies,
          int typeId, ref ConstraintBatch batch, BufferPool pool, ref ConstraintReference reference, int minimumBodyCapacity = 8, int minimumReferenceCapacity = 8)
        {
            Allocate(handle, ref constraintBodyHandles, bodyCount, bodies, typeId, ref batch, pool, ref reference, new InactiveSetGetter(), minimumBodyCapacity, minimumReferenceCapacity);
        }

        internal unsafe void Remove(Solver solver, BufferPool pool, ref ConstraintBatch batch, int constraintHandle, int typeId, int indexInTypeBatch)
        {
            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var bundleIndex, out var innerIndex);
            var typeProcessor = solver.TypeProcessors[typeId];
            var bodyCount = typeProcessor.BodiesPerConstraint;
            var bodyIndices = stackalloc int[bodyCount];
            var enumerator = new ReferenceCollector(bodyIndices);
            typeProcessor.EnumerateConnectedBodyIndices(ref batch.TypeBatches[batch.TypeIndexToTypeBatchIndex[typeId]], indexInTypeBatch, ref enumerator);
            for (int i = 0; i < bodyCount; ++i)
            {
                var bodyPresent = bodyConstraintReferences.GetTableIndices(ref bodyIndices[i], out var tableIndex, out var bodyReferencesIndex);
                Debug.Assert(bodyPresent, "If we've been asked to remove a constraint associated with a body, that body must be in this batch.");
                ref var constraintReferences = ref bodyConstraintReferences.Values[bodyReferencesIndex];
                //TODO: Should really just be using a dictionary here.
                var dummy = new FallbackReference { ConstraintHandle = constraintHandle };
                var removed = constraintReferences.FastRemove(ref dummy);
                Debug.Assert(removed, "If a constraint removal was requested, it must exist within the referenced body's constraint set.");
                if (constraintReferences.Count == 0)
                {
                    //If there are no more constraints associated with this body, get rid of the body list.
                    constraintReferences.Dispose(pool.SpecializeFor<FallbackReference>(), pool.SpecializeFor<int>());
                    bodyConstraintReferences.FastRemove(tableIndex, bodyReferencesIndex);
                }
            }
            if (bodyConstraintReferences.Count == 0)
            {
                //No constraints remain in the fallback batch. Drop the dictionary.
                bodyConstraintReferences.Dispose(pool.SpecializeFor<int>(), pool.SpecializeFor<QuickSet<FallbackReference, Buffer<FallbackReference>, Buffer<int>, FallbackReferenceComparer>>(), pool.SpecializeFor<int>());
            }
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetJacobiScaleForBodies(ref Vector<int> references, int count, out Vector<float> jacobiScale)
        {
            ref var start = ref Unsafe.As<Vector<int>, int>(ref references);
            Vector<int> counts;
            ref var countsStart = ref Unsafe.As<Vector<int>, int>(ref counts);
            for (int i = 0; i < count; ++i)
            {
                var index = bodyConstraintReferences.IndexOf(ref Unsafe.Add(ref start, i));
                Debug.Assert(index >= 0, "If a prestep is looking up constraint counts associated with a body, it better be in the jacobi batch!");
                Unsafe.Add(ref countsStart, i) = bodyConstraintReferences.Values[index].Count;
            }
            jacobiScale = Vector.ConvertToSingle(counts);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetJacobiScaleForBodies(ref TwoBodyReferences references, int count, out Vector<float> jacobiScaleA, out Vector<float> jacobiScaleB)
        {
            ref var startA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var startB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            Vector<int> countsA, countsB;
            ref var countsAStart = ref Unsafe.As<Vector<int>, int>(ref countsA);
            ref var countsBStart = ref Unsafe.As<Vector<int>, int>(ref countsB);
            for (int i = 0; i < count; ++i)
            {
                var indexA = bodyConstraintReferences.IndexOf(ref Unsafe.Add(ref startA, i));
                var indexB = bodyConstraintReferences.IndexOf(ref Unsafe.Add(ref startB, i));
                Debug.Assert(indexA >= 0 && indexB >= 0, "If a prestep is looking up constraint counts associated with a body, it better be in the jacobi batch!");
                Unsafe.Add(ref countsAStart, i) = bodyConstraintReferences.Values[indexA].Count;
                Unsafe.Add(ref countsBStart, i) = bodyConstraintReferences.Values[indexB].Count;
            }
            jacobiScaleA = Vector.ConvertToSingle(countsA);
            jacobiScaleB = Vector.ConvertToSingle(countsB);
        }

        public void ScatterVelocities(Bodies bodies, ref Buffer<FallbackTypeBatchResults> velocities, int start, int exclusiveEnd)
        {
            for (int i = start; i < exclusiveEnd; ++i)
            {
                //Velocity scattering is only ever executed on the active set, so the body reference is always an index.
                var bodyIndex = bodyConstraintReferences.Keys[i];
                BodyVelocity bodyVelocity = default;
                ref var constraintReferences = ref bodyConstraintReferences.Values[i];
                for (int j = 0; j < constraintReferences.Count; ++j)
                {
                    //TODO: This can't be optimally vectorized due to the inherent gathers involved, but you may be able to do much better in terms of wasted instructions
                    //using platform intrinsics (like many other places). The benefit of true gather instructions here is more than some other places since it's likely all in L3 cache
                    //(if it's a shared L3, anyway).
                    ref var reference = ref constraintReferences[j];
                    ref var typeBatchVelocities = ref velocities[reference.TypeBatchIndex];
                    BundleIndexing.GetBundleIndices(reference.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
                    ref var bundle = ref typeBatchVelocities.BodyVelocities[reference.IndexInConstraint][bundleIndex];
                    ref var offsetBundle = ref GatherScatter.GetOffsetInstance(ref bundle, innerIndex);
                    Vector3Wide.ReadFirst(offsetBundle.Linear, out var linear);
                    Vector3Wide.ReadFirst(offsetBundle.Angular, out var angular);
                    bodyVelocity.Linear += linear;
                    bodyVelocity.Angular += angular;
                }
                //This simply averages all velocity results from the iteration for the body. This is equivalent to PGS/SI in terms of convergence because it is mathematically equivalent
                //to having a linear/angular 'weld' constraint between N separate bodies that happen to all be in the same spot, except each of them has 1/N as much mass as the original.
                //In other words, each jacobi batch constraint computed:
                //newVelocity = oldVelocity + impulse * (1 / (inertia / N)) = oldVelocity + impulse * N / inertia
                //All constraints together give a sum:
                //summedVelocity = (oldVelocity + impulse0 * N / inertia) + (oldVelocity + impulse1 * N / inertia) + (oldVelocity + impulse2 * N / inertia) + ... 
                //averageVelocity = summedVelocity / N = (oldVelocity + impulse0 * N / inertia) / N + (oldVelocity + impulse0 * N / inertia) / N + ...
                //averageVelocity = (oldVelocity / N + impulse0 / inertia) + (oldVelocity / N + impulse0 / inertia) + ...
                //averageVelocity = (oldVelocity / N + oldVelocity / N + ...) + impulse0 / inertia + impulse1 / inertia + ...
                //averageVelocity = oldVelocity + (impulse0 + impulse1 + ... ) / inertia
                //Which is exactly what we want.
                var inverseCount = 1f / constraintReferences.Count;
                bodyVelocity.Linear *= inverseCount;
                bodyVelocity.Angular *= inverseCount;
                bodies.ActiveSet.Velocities[bodyIndex] = bodyVelocity;
            }
        }

        internal void UpdateForBodyMemoryMove(int originalBodyIndex, int newBodyLocation)
        {
            Debug.Assert(!bodyConstraintReferences.ContainsKey(newBodyLocation), "If a body is being moved, as opposed to swapped, then the target index should not be present.");
            bodyConstraintReferences.GetTableIndices(ref originalBodyIndex, out var tableIndex, out var elementIndex);
            var references = bodyConstraintReferences.Values[elementIndex];
            bodyConstraintReferences.FastRemove(tableIndex, elementIndex);
            bodyConstraintReferences.AddUnsafely(ref newBodyLocation, ref references);
        }

        internal void UpdateForBodyMemorySwap(int a, int b)
        {
            var indexA = bodyConstraintReferences.IndexOf(a);
            var indexB = bodyConstraintReferences.IndexOf(b);
            Debug.Assert(indexA >= 0 && indexB >= 0, "A swap requires that both indices are already present.");
            Helpers.Swap(ref bodyConstraintReferences.Values[indexA], ref bodyConstraintReferences.Values[indexB]);
        }

        public void Compact(BufferPool pool)
        {
            if (bodyConstraintReferences.Keys.Allocated)
            {
                var intPool = pool.SpecializeFor<int>();
                var fallbackPool = pool.SpecializeFor<FallbackReference>();
                bodyConstraintReferences.Compact(intPool, pool.SpecializeFor<QuickSet<FallbackReference, Buffer<FallbackReference>, Buffer<int>, FallbackReferenceComparer>>(), intPool);
                for (int i = 0; i < bodyConstraintReferences.Count; ++i)
                {
                    bodyConstraintReferences.Values[i].Compact(fallbackPool, intPool);
                }
            }
        }


        public void Dispose(BufferPool pool)
        {
            if (bodyConstraintReferences.Keys.Allocated)
            {
                var fallbackPool = pool.SpecializeFor<FallbackReference>();
                var intPool = pool.SpecializeFor<int>();
                for (int i = 0; i < bodyConstraintReferences.Count; ++i)
                {
                    bodyConstraintReferences.Values[i].Dispose(fallbackPool, intPool);
                }
                bodyConstraintReferences.Dispose(intPool, pool.SpecializeFor<QuickSet<FallbackReference, Buffer<FallbackReference>, Buffer<int>, FallbackReferenceComparer>>(), intPool);
            }
        }

    }
}
