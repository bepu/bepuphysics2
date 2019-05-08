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
            public int IndexInConstraint;
            public override string ToString()
            {
                return $"{ConstraintHandle}, {IndexInConstraint}";
            }
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
        internal QuickDictionary<int, QuickSet<FallbackReference, FallbackReferenceComparer>, PrimitiveComparer<int>> bodyConstraintReferences;

        internal struct FallbackReferenceComparer : IEqualityComparerRef<FallbackReference>
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
        unsafe void Allocate<TBodyReferenceGetter>(int constraintHandle, ref int constraintBodyHandles, int bodyCount, Bodies bodies,
           int typeId, BufferPool pool, TBodyReferenceGetter bodyReferenceGetter, int minimumBodyCapacity, int minimumReferenceCapacity)
            where TBodyReferenceGetter : struct, IBodyReferenceGetter
        {
            EnsureCapacity(Math.Max(bodyConstraintReferences.Count + bodyCount, minimumBodyCapacity), pool);
            for (int i = 0; i < bodyCount; ++i)
            {
                var bodyReference = bodyReferenceGetter.GetBodyReference(bodies, Unsafe.Add(ref constraintBodyHandles, i));

                var bodyAlreadyListed = bodyConstraintReferences.GetTableIndices(ref bodyReference, out var tableIndex, out var elementIndex);
                //If an entry for this body does not yet exist, we'll create one.
                if (!bodyAlreadyListed)
                    elementIndex = bodyConstraintReferences.Count;
                ref var constraintReferences = ref bodyConstraintReferences.Values[elementIndex];

                if (!bodyAlreadyListed)
                {
                    //The body is not already contained. Create a list for it.
                    constraintReferences = new QuickSet<FallbackReference, FallbackReferenceComparer>(minimumReferenceCapacity, pool);
                    bodyConstraintReferences.Keys[elementIndex] = bodyReference;
                    bodyConstraintReferences.Table[tableIndex] = elementIndex + 1;
                    ++bodyConstraintReferences.Count;
                }
                var fallbackReference = new FallbackReference { ConstraintHandle = constraintHandle, IndexInConstraint = i };
                constraintReferences.AddRef(ref fallbackReference, pool);
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
                return bodyHandle;
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal unsafe void AllocateForActive(int handle, ref int constraintBodyHandles, int bodyCount, Bodies bodies,
           int typeId, BufferPool pool, int minimumBodyCapacity = 8, int minimumReferenceCapacity = 8)
        {
            Allocate(handle, ref constraintBodyHandles, bodyCount, bodies, typeId, pool, new ActiveSetGetter(), minimumBodyCapacity, minimumReferenceCapacity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void AllocateForInactive(int handle, ref int constraintBodyHandles, int bodyCount, Bodies bodies,
          int typeId, BufferPool pool, int minimumBodyCapacity = 8, int minimumReferenceCapacity = 8)
        {
            Allocate(handle, ref constraintBodyHandles, bodyCount, bodies, typeId, pool, new InactiveSetGetter(), minimumBodyCapacity, minimumReferenceCapacity);
        }


        internal unsafe void Remove(int bodyReference, int constraintHandle, ref QuickList<int> allocationIdsToFree)
        {
            var bodyPresent = bodyConstraintReferences.GetTableIndices(ref bodyReference, out var tableIndex, out var bodyReferencesIndex);
            Debug.Assert(bodyPresent, "If we've been asked to remove a constraint associated with a body, that body must be in this batch.");
            ref var constraintReferences = ref bodyConstraintReferences.Values[bodyReferencesIndex];
            //TODO: Should really just be using a dictionary here.
            var dummy = new FallbackReference { ConstraintHandle = constraintHandle };
            var removed = constraintReferences.FastRemoveRef(ref dummy);
            Debug.Assert(removed, "If a constraint removal was requested, it must exist within the referenced body's constraint set.");
            if (constraintReferences.Count == 0)
            {
                //If there are no more constraints associated with this body, get rid of the body list.
                allocationIdsToFree.AllocateUnsafely() = constraintReferences.Span.Id;
                allocationIdsToFree.AllocateUnsafely() = constraintReferences.Table.Id;
                constraintReferences = default;
                bodyConstraintReferences.FastRemove(tableIndex, bodyReferencesIndex);
                if (bodyConstraintReferences.Count == 0)
                {
                    //No constraints remain in the fallback batch. Drop the dictionary.
                    allocationIdsToFree.AllocateUnsafely() = bodyConstraintReferences.Keys.Id;
                    allocationIdsToFree.AllocateUnsafely() = bodyConstraintReferences.Values.Id;
                    allocationIdsToFree.AllocateUnsafely() = bodyConstraintReferences.Table.Id;
                    bodyConstraintReferences = default;
                }
            }
        }


        internal unsafe void Remove(Solver solver, BufferPool bufferPool, ref ConstraintBatch batch, int constraintHandle, int typeId, int indexInTypeBatch)
        {
            var typeProcessor = solver.TypeProcessors[typeId];
            var bodyCount = typeProcessor.BodiesPerConstraint;
            var bodyIndices = stackalloc int[bodyCount];
            var enumerator = new ReferenceCollector(bodyIndices);
            solver.EnumerateConnectedBodies(constraintHandle, ref enumerator);
            var maximumAllocationIdsToFree = 3 + bodyCount * 2;
            var allocationIdsToRemoveMemory = stackalloc int[maximumAllocationIdsToFree];
            var initialSpan = new Buffer<int>(allocationIdsToRemoveMemory, maximumAllocationIdsToFree);
            var allocationIdsToFree = new QuickList<int>(initialSpan);
            typeProcessor.EnumerateConnectedBodyIndices(ref batch.TypeBatches[batch.TypeIndexToTypeBatchIndex[typeId]], indexInTypeBatch, ref enumerator);
            for (int i = 0; i < bodyCount; ++i)
            {
                Remove(bodyIndices[i], constraintHandle, ref allocationIdsToFree);
            }
            for (int i = 0; i < allocationIdsToFree.Count; ++i)
            {
                bufferPool.ReturnUnsafely(allocationIdsToFree[i]);
            }
        }

        internal unsafe void TryRemove(int bodyReference, ref QuickList<int> allocationIdsToFree)
        {
            if (bodyConstraintReferences.Keys.Allocated && bodyConstraintReferences.GetTableIndices(ref bodyReference, out var tableIndex, out var bodyReferencesIndex))
            {
                ref var constraintReferences = ref bodyConstraintReferences.Values[bodyReferencesIndex];
                //If there are no more constraints associated with this body, get rid of the body list.
                allocationIdsToFree.AllocateUnsafely() = constraintReferences.Span.Id;
                allocationIdsToFree.AllocateUnsafely() = constraintReferences.Table.Id;
                bodyConstraintReferences.FastRemove(tableIndex, bodyReferencesIndex);
                if (bodyConstraintReferences.Count == 0)
                {
                    //No constraints remain in the fallback batch. Drop the dictionary.
                    allocationIdsToFree.AllocateUnsafely() = bodyConstraintReferences.Keys.Id;
                    allocationIdsToFree.AllocateUnsafely() = bodyConstraintReferences.Values.Id;
                    allocationIdsToFree.AllocateUnsafely() = bodyConstraintReferences.Table.Id;
                    bodyConstraintReferences = default;
                }
            }
        }

        public static void AllocateResults(Solver solver, BufferPool pool, ref ConstraintBatch batch, out Buffer<FallbackTypeBatchResults> results)
        {
            pool.TakeAtLeast(batch.TypeBatches.Count, out results);
            for (int i = 0; i < batch.TypeBatches.Count; ++i)
            {
                ref var typeBatch = ref batch.TypeBatches[i];
                var bodiesPerConstraint = solver.TypeProcessors[typeBatch.TypeId].BodiesPerConstraint;
                ref var typeBatchResults = ref results[i];
                pool.TakeAtLeast(bodiesPerConstraint, out typeBatchResults.BodyVelocities);
                for (int j = 0; j < bodiesPerConstraint; ++j)
                {
                    pool.TakeAtLeast(typeBatch.BundleCount, out typeBatchResults.GetVelocitiesForBody(j));
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
                var index = bodyConstraintReferences.IndexOfRef(ref Unsafe.Add(ref start, i));
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
                var indexA = bodyConstraintReferences.IndexOfRef(ref Unsafe.Add(ref startA, i));
                var indexB = bodyConstraintReferences.IndexOfRef(ref Unsafe.Add(ref startB, i));
                Debug.Assert(indexA >= 0 && indexB >= 0, "If a prestep is looking up constraint counts associated with a body, it better be in the jacobi batch!");
                Unsafe.Add(ref countsAStart, i) = bodyConstraintReferences.Values[indexA].Count;
                Unsafe.Add(ref countsBStart, i) = bodyConstraintReferences.Values[indexB].Count;
            }
            jacobiScaleA = Vector.ConvertToSingle(countsA);
            jacobiScaleB = Vector.ConvertToSingle(countsB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetJacobiScaleForBodies(ref ThreeBodyReferences references, int count,
            out Vector<float> jacobiScaleA, out Vector<float> jacobiScaleB, out Vector<float> jacobiScaleC)
        {
            ref var startA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var startB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            ref var startC = ref Unsafe.As<Vector<int>, int>(ref references.IndexC);
            Vector<int> countsA, countsB, countsC;
            ref var countsAStart = ref Unsafe.As<Vector<int>, int>(ref countsA);
            ref var countsBStart = ref Unsafe.As<Vector<int>, int>(ref countsB);
            ref var countsCStart = ref Unsafe.As<Vector<int>, int>(ref countsC);
            for (int i = 0; i < count; ++i)
            {
                var indexA = bodyConstraintReferences.IndexOfRef(ref Unsafe.Add(ref startA, i));
                var indexB = bodyConstraintReferences.IndexOfRef(ref Unsafe.Add(ref startB, i));
                var indexC = bodyConstraintReferences.IndexOfRef(ref Unsafe.Add(ref startC, i));
                Debug.Assert(indexA >= 0 && indexB >= 0, "If a prestep is looking up constraint counts associated with a body, it better be in the jacobi batch!");
                Unsafe.Add(ref countsAStart, i) = bodyConstraintReferences.Values[indexA].Count;
                Unsafe.Add(ref countsBStart, i) = bodyConstraintReferences.Values[indexB].Count;
                Unsafe.Add(ref countsCStart, i) = bodyConstraintReferences.Values[indexC].Count;
            }
            jacobiScaleA = Vector.ConvertToSingle(countsA);
            jacobiScaleB = Vector.ConvertToSingle(countsB);
            jacobiScaleC = Vector.ConvertToSingle(countsC);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetJacobiScaleForBodies(ref FourBodyReferences references, int count,
            out Vector<float> jacobiScaleA, out Vector<float> jacobiScaleB, out Vector<float> jacobiScaleC, out Vector<float> jacobiScaleD)
        {
            ref var startA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var startB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            ref var startC = ref Unsafe.As<Vector<int>, int>(ref references.IndexC);
            ref var startD = ref Unsafe.As<Vector<int>, int>(ref references.IndexD);
            Vector<int> countsA, countsB, countsC, countsD;
            ref var countsAStart = ref Unsafe.As<Vector<int>, int>(ref countsA);
            ref var countsBStart = ref Unsafe.As<Vector<int>, int>(ref countsB);
            ref var countsCStart = ref Unsafe.As<Vector<int>, int>(ref countsC);
            ref var countsDStart = ref Unsafe.As<Vector<int>, int>(ref countsD);
            for (int i = 0; i < count; ++i)
            {
                var indexA = bodyConstraintReferences.IndexOfRef(ref Unsafe.Add(ref startA, i));
                var indexB = bodyConstraintReferences.IndexOfRef(ref Unsafe.Add(ref startB, i));
                var indexC = bodyConstraintReferences.IndexOfRef(ref Unsafe.Add(ref startC, i));
                var indexD = bodyConstraintReferences.IndexOfRef(ref Unsafe.Add(ref startD, i));
                Debug.Assert(indexA >= 0 && indexB >= 0, "If a prestep is looking up constraint counts associated with a body, it better be in the jacobi batch!");
                Unsafe.Add(ref countsAStart, i) = bodyConstraintReferences.Values[indexA].Count;
                Unsafe.Add(ref countsBStart, i) = bodyConstraintReferences.Values[indexB].Count;
                Unsafe.Add(ref countsCStart, i) = bodyConstraintReferences.Values[indexC].Count;
                Unsafe.Add(ref countsDStart, i) = bodyConstraintReferences.Values[indexD].Count;
            }
            jacobiScaleA = Vector.ConvertToSingle(countsA);
            jacobiScaleB = Vector.ConvertToSingle(countsB);
            jacobiScaleC = Vector.ConvertToSingle(countsC);
            jacobiScaleD = Vector.ConvertToSingle(countsD);
        }

        [Conditional("DEBUG")]
        unsafe static void ValidateBodyConstraintReference(Solver solver, int setIndex, int bodyReference, int constraintHandle, int expectedIndexInConstraint)
        {
            ref var constraintLocation = ref solver.HandleToConstraint[constraintHandle];
            Debug.Assert(constraintLocation.SetIndex == setIndex);
            Debug.Assert(constraintLocation.BatchIndex == solver.FallbackBatchThreshold, "Should only be working on constraints which are members of the active fallback batch.");
            var debugReferences = stackalloc int[solver.TypeProcessors[constraintLocation.TypeId].BodiesPerConstraint];
            var debugBodyReferenceCollector = new ReferenceCollector(debugReferences);
            solver.EnumerateConnectedBodies(constraintHandle, ref debugBodyReferenceCollector);
            Debug.Assert(debugReferences[expectedIndexInConstraint] == bodyReference, "The constraint's true body references must agree with the fallback batch.");
        }
        [Conditional("DEBUG")]
        public static unsafe void ValidateSetReferences(Solver solver, int setIndex)
        {
            ref var set = ref solver.Sets[setIndex];
            Debug.Assert(set.Allocated);
            if (set.Batches.Count > solver.FallbackBatchThreshold)
            {
                Debug.Assert(set.Fallback.bodyConstraintReferences.Keys.Allocated);
                ref var bodyConstraintReferences = ref set.Fallback.bodyConstraintReferences;
                for (int i = 0; i < bodyConstraintReferences.Count; ++i)
                {
                    //This is a handle on inactive sets, and an index for active sets.
                    var bodyReference = bodyConstraintReferences.Keys[i];
                    ref var references = ref bodyConstraintReferences.Values[i];
                    Debug.Assert(references.Count > 0, "If there exists a body reference set, it should be populated.");
                    for (int j = 0; j < references.Count; ++j)
                    {
                        ref var reference = ref references.Span[j];
                        ValidateBodyConstraintReference(solver, setIndex, bodyReference, reference.ConstraintHandle, reference.IndexInConstraint);
                    }
                }
                ref var batch = ref set.Batches[solver.FallbackBatchThreshold];
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                    var bodiesPerConstraint = solver.TypeProcessors[typeBatch.TypeId].BodiesPerConstraint;
                    var connectedBodies = stackalloc int[bodiesPerConstraint];
                    for (int constraintIndex = 0; constraintIndex < typeBatch.ConstraintCount; ++constraintIndex)
                    {
                        var constraintHandle = typeBatch.IndexToHandle[constraintIndex];
                        var collector = new ReferenceCollector(connectedBodies);
                        solver.EnumerateConnectedBodies(constraintHandle, ref collector);
                        for (int i = 0; i < bodiesPerConstraint; ++i)
                        {
                            var localBodyIndex = bodyConstraintReferences.IndexOf(connectedBodies[i]);
                            Debug.Assert(localBodyIndex >= 0, "Any body referenced by a constraint in the fallback batch should exist within the fallback batch's body listing.");
                            ref var references = ref bodyConstraintReferences.Values[localBodyIndex];
                            var constraintIndexInBodySet = references.IndexOf(new FallbackReference { ConstraintHandle = constraintHandle });
                            Debug.Assert(constraintIndexInBodySet >= 0, "Any constraint in the fallback batch should be in all connected bodies' constraint handle listings.");
                            ref var reference = ref references[constraintIndexInBodySet];
                            Debug.Assert(reference.ConstraintHandle == constraintHandle && reference.IndexInConstraint == i);
                        }
                    }
                }
            }
        }
        [Conditional("DEBUG")]
        public static unsafe void ValidateReferences(Solver solver)
        {
            for (int i = 0; i < solver.Sets.Length; ++i)
            {
                if (solver.Sets[i].Allocated)
                    ValidateSetReferences(solver, i);
            }
        }

        public void ScatterVelocities(Bodies bodies, Solver solver, ref Buffer<FallbackTypeBatchResults> velocities, int start, int exclusiveEnd)
        {
            ref var fallbackBatch = ref solver.ActiveSet.Batches[solver.FallbackBatchThreshold];
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
                    ref var constraintLocation = ref solver.HandleToConstraint[reference.ConstraintHandle];
                    //ValidateReferences(solver, bodyIndex, reference.ConstraintHandle, reference.IndexInConstraint);
                    var typeBatchIndex = fallbackBatch.TypeIndexToTypeBatchIndex[constraintLocation.TypeId];
                    ref var typeBatchVelocities = ref velocities[typeBatchIndex];
                    BundleIndexing.GetBundleIndices(constraintLocation.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
                    ref var bundle = ref typeBatchVelocities.BodyVelocities[reference.IndexInConstraint][bundleIndex];
                    ref var offsetBundle = ref GatherScatter.GetOffsetInstance(ref bundle, innerIndex);
                    Vector3Wide.ReadFirst(offsetBundle.Linear, out var linear);
                    Vector3Wide.ReadFirst(offsetBundle.Angular, out var angular);
                    //bodyVelocity.Linear.Validate();
                    //bodyVelocity.Angular.Validate();
                    bodyVelocity.Linear += linear;
                    bodyVelocity.Angular += angular;
                    //bodyVelocity.Linear.Validate();
                    //bodyVelocity.Angular.Validate();
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
            Debug.Assert(bodyConstraintReferences.Keys.Allocated && !bodyConstraintReferences.ContainsKey(newBodyLocation), "If a body is being moved, as opposed to swapped, then the target index should not be present.");
            bodyConstraintReferences.GetTableIndices(ref originalBodyIndex, out var tableIndex, out var elementIndex);
            var references = bodyConstraintReferences.Values[elementIndex];
            bodyConstraintReferences.FastRemove(tableIndex, elementIndex);
            bodyConstraintReferences.AddUnsafelyRef(ref newBodyLocation, references);
        }

        internal void UpdateForBodyMemorySwap(int a, int b)
        {
            var indexA = bodyConstraintReferences.IndexOf(a);
            var indexB = bodyConstraintReferences.IndexOf(b);
            Debug.Assert(indexA >= 0 && indexB >= 0, "A swap requires that both indices are already present.");
            Helpers.Swap(ref bodyConstraintReferences.Values[indexA], ref bodyConstraintReferences.Values[indexB]);
        }

        internal static void CreateFrom(ref FallbackBatch sourceBatch, BufferPool pool, out FallbackBatch targetBatch)
        {
            //Copy over non-buffer state. This copies buffer references pointlessly, but that doesn't matter.
            targetBatch.bodyConstraintReferences = sourceBatch.bodyConstraintReferences;
            pool.TakeAtLeast(sourceBatch.bodyConstraintReferences.Count, out targetBatch.bodyConstraintReferences.Keys);
            pool.TakeAtLeast(targetBatch.bodyConstraintReferences.Keys.Length, out targetBatch.bodyConstraintReferences.Values);
            pool.TakeAtLeast(sourceBatch.bodyConstraintReferences.TableMask + 1, out targetBatch.bodyConstraintReferences.Table);
            sourceBatch.bodyConstraintReferences.Keys.CopyTo(0, ref targetBatch.bodyConstraintReferences.Keys, 0, sourceBatch.bodyConstraintReferences.Count);
            sourceBatch.bodyConstraintReferences.Values.CopyTo(0, ref targetBatch.bodyConstraintReferences.Values, 0, sourceBatch.bodyConstraintReferences.Count);
            sourceBatch.bodyConstraintReferences.Table.CopyTo(0, ref targetBatch.bodyConstraintReferences.Table, 0, sourceBatch.bodyConstraintReferences.TableMask + 1);

            for (int i = 0; i < sourceBatch.bodyConstraintReferences.Count; ++i)
            {
                ref var source = ref sourceBatch.bodyConstraintReferences.Values[i];
                ref var target = ref targetBatch.bodyConstraintReferences.Values[i];
                target = source;
                pool.TakeAtLeast(source.Count, out target.Span);
                pool.TakeAtLeast(source.TableMask + 1, out target.Table);
                source.Span.CopyTo(0, ref target.Span, 0, source.Count);
                source.Table.CopyTo(0, ref target.Table, 0, source.TableMask + 1);
            }
        }

        internal void EnsureCapacity(int bodyCapacity, BufferPool pool)
        {
            if (bodyConstraintReferences.Keys.Allocated)
            {
                //This is conservative since there's no guarantee that we'll actually need to resize at all if these bodies are already present, but that's fine. 
                bodyConstraintReferences.EnsureCapacity(bodyCapacity, pool);
            }
            else
            {
                bodyConstraintReferences = new QuickDictionary<int, QuickSet<FallbackReference, FallbackReferenceComparer>, PrimitiveComparer<int>>(bodyCapacity, pool);
            }

        }

        public void Compact(BufferPool pool)
        {
            if (bodyConstraintReferences.Keys.Allocated)
            {
                bodyConstraintReferences.Compact(pool);
                for (int i = 0; i < bodyConstraintReferences.Count; ++i)
                {
                    bodyConstraintReferences.Values[i].Compact(pool);
                }
            }
        }


        public void Dispose(BufferPool pool)
        {
            if (bodyConstraintReferences.Keys.Allocated)
            {
                for (int i = 0; i < bodyConstraintReferences.Count; ++i)
                {
                    bodyConstraintReferences.Values[i].Dispose(pool);
                }
                bodyConstraintReferences.Dispose(pool);
            }
        }

    }
}
