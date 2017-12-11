using System;
using System.Collections.Generic;
using System.Text;
using BepuUtilities;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using System.Runtime.CompilerServices;
using BepuUtilities.Memory;
using System.Runtime.InteropServices;
using System.Diagnostics;
using System.Threading;
using BepuUtilities.Collections;
using BepuPhysics.Constraints.Contact;

namespace BepuPhysics.CollisionDetection
{
    /// <summary>
    /// When notified of a new constraint, immediately adds it to the solver.
    /// </summary>
    public partial class NarrowPhase<TCallbacks>
    {
        internal struct PendingConstraintAddCache
        {
            BufferPool pool;
            struct PendingConstraint<TBodyHandles, TDescription, TContactImpulses> where TDescription : IConstraintDescription<TDescription>
            {
                //Note the memory ordering. Body handles come first; deterministic flushes rely the memory layout to sort pending constraints.
                public TBodyHandles BodyHandles;
                public TDescription ConstraintDescription;
                public TContactImpulses Impulses;
                public CollidablePair Pair;
                public PairCacheIndex ConstraintCacheIndex;
            }

            internal Buffer<UntypedList> pendingConstraintsByType;
            internal Buffer<Buffer<ushort>> speculativeBatchIndices;
            int minimumConstraintCountPerCache;

            public PendingConstraintAddCache(BufferPool pool, int minimumConstraintCountPerCache = 128)
            {
                this.pool = pool;
                pool.SpecializeFor<UntypedList>().Take(PairCache.CollisionConstraintTypeCount, out pendingConstraintsByType);
                //Have to clear the memory before use to avoid trash data sticking around.
                pendingConstraintsByType.Clear(0, PairCache.CollisionConstraintTypeCount);
                this.minimumConstraintCountPerCache = minimumConstraintCountPerCache;
                speculativeBatchIndices = new Buffer<Buffer<ushort>>();
            }

            public unsafe void AddConstraint<TBodyHandles, TDescription, TContactImpulses>(int manifoldConstraintType,
                ref CollidablePair pair, PairCacheIndex constraintCacheIndex, TBodyHandles bodyHandles, ref TDescription constraintDescription, ref TContactImpulses impulses)
                where TDescription : IConstraintDescription<TDescription>
            {
                ref var cache = ref pendingConstraintsByType[manifoldConstraintType];
                var byteIndex = cache.Allocate<PendingConstraint<TBodyHandles, TDescription, TContactImpulses>>(minimumConstraintCountPerCache, pool);
                ref var pendingAdd = ref Unsafe.AsRef<PendingConstraint<TBodyHandles, TDescription, TContactImpulses>>(cache.Buffer.Memory + byteIndex);
                pendingAdd.BodyHandles = bodyHandles;
                pendingAdd.ConstraintDescription = constraintDescription;
                pendingAdd.Impulses = impulses;
                pendingAdd.Pair = pair;
                pendingAdd.ConstraintCacheIndex = constraintCacheIndex;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static unsafe void SequentialAddToSimulation<TBodyHandles, TDescription, TContactImpulses>(ref UntypedList list, int narrowPhaseConstraintTypeId, Simulation simulation, PairCache pairCache)
                where TDescription : IConstraintDescription<TDescription>
            {
                if (list.Buffer.Allocated)
                {
                    ref var start = ref Unsafe.As<byte, PendingConstraint<TBodyHandles, TDescription, TContactImpulses>>(ref *list.Buffer.Memory);
                    Debug.Assert(list.Buffer.Length > Unsafe.SizeOf<PendingConstraint<TBodyHandles, TDescription, TContactImpulses>>() * list.Count);
                    Debug.Assert(list.ByteCount == Unsafe.SizeOf<PendingConstraint<TBodyHandles, TDescription, TContactImpulses>>() * list.Count);
                    for (int i = 0; i < list.Count; ++i)
                    {
                        ref var add = ref Unsafe.Add(ref start, i);
                        var handle = simulation.Add(ref Unsafe.As<TBodyHandles, int>(ref add.BodyHandles), typeof(TBodyHandles) == typeof(TwoBodyHandles) ? 2 : 1, ref add.ConstraintDescription);
                        pairCache.CompleteConstraintAdd(simulation.Solver, ref add.Impulses, add.ConstraintCacheIndex, handle, ref add.Pair);
                    }
                }
            }

            /// <summary>
            /// Flushes pending constraints into the simulation without any form of synchronization. Adds occur in the order of manifold generation.
            /// If the contact manifold generation is deterministic, then the result of this add will be deterministic.
            /// </summary>
            internal void FlushSequentially(Simulation simulation, ref PairCache pairCache)
            {
                //This is going to be pretty horrible!
                //There is no type information beyond the index of the cache.
                //In other words, we basically have to do a switch statement (or equivalently manually unrolled loop) to gather objects of the proper type from it. 

                //This follows the same convention as the GatherOldImpulses and ScatterNewImpulses of the PairCache.
                //Constraints cover 16 possible cases:
                //1-4 contacts: 0x3
                //convex vs nonconvex: 0x4
                //1 body versus 2 body: 0x8
                //TODO: Very likely that we'll expand the nonconvex manifold maximum to 8 contacts, so this will need to be adjusted later.
                SequentialAddToSimulation<int, Contact1OneBody, ContactImpulses1>(ref pendingConstraintsByType[0 + 0 + 0], 0 + 0 + 0, simulation, pairCache);
                SequentialAddToSimulation<TwoBodyHandles, Contact1, ContactImpulses1>(ref pendingConstraintsByType[8 + 0 + 0], 8 + 0 + 0, simulation, pairCache);
                SequentialAddToSimulation<TwoBodyHandles, Contact4, ContactImpulses4>(ref pendingConstraintsByType[8 + 0 + 3], 8 + 0 + 3, simulation, pairCache);
            }


            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void AddToSimulationSpeculative<TBodyHandles, TDescription, TContactImpulses>(
                ref PendingConstraint<TBodyHandles, TDescription, TContactImpulses> constraint, int batchIndex, Simulation simulation, PairCache pairCache)
                where TDescription : IConstraintDescription<TDescription>
            {
                //This function takes full responsibility for what a Simulation.Add would do, plus the need to complete the constraint add in the pair cache.
                //1) Allocate in solver batch and type batch.
                //2) Apply the constraint description to the allocated slot in the type batch.
                //3) Add the constraint to the two body lists.
                //4) Notify the pair cache of the addition.
                //This is all done together for the sake of simplicity. You could push the constraint application and the pair cache notification to a different stage- they are 
                //fully parallel and independent of other work (so long as there is no danger of resizing the type batches!).
                //We don't currently put this in multithreaded jobs because: 
                //1) PairCache.CompleteConstraintAdd can't be executed in parallel with a task that may resize type batches. Adding constraints can do that.
                //2) Description application may be corrupted by type batch resizes too.
                //3) Body constraint list adds make use of the main buffer pool, which is also used by adding constraints in the event of type batch resizes and such.
                //4) Delaying the constraint add requires caching the handle to memory, which is kinda goofy since that's half the work already.
                //5) Description application is only about 5-10% of the cost of this function.
                //If any of these assumptions change, or if we find ourselves really hurting in threaded scaling because of these decisions, we can revisit the design.

                //Further, the reason why this function breaks out the individual responsibilities of the Simulation.Add is just because we did a prepass where best guesses for batch slots
                //were computed. In order to make use of those best guesses, we basically created a custom overload. (In fact, if you end up needing this elsewhere, you should probably make it
                //a custom overload instead!)

                int constraintHandle;
                ConstraintReference reference;
                ref var handles = ref Unsafe.As<TBodyHandles, int>(ref constraint.BodyHandles);
                while (!simulation.Solver.TryAllocateInBatch(
                    default(TDescription).ConstraintTypeId, batchIndex,
                    ref Unsafe.As<TBodyHandles, int>(ref constraint.BodyHandles),
                    typeof(TBodyHandles) == typeof(TwoBodyHandles) ? 2 : 1,
                    out constraintHandle, out reference))
                {
                    //If a batch index failed, just try the next one. This is guaranteed to eventually work.
                    ++batchIndex;
                }
                simulation.Solver.ApplyDescription(ref reference, ref constraint.ConstraintDescription);
                ref var aLocation = ref simulation.Bodies.HandleToLocation[handles];
                Debug.Assert(aLocation.SetIndex == 0, "By the time we flush new constraints into the solver, all associated islands should have been activated.");
                simulation.Bodies.AddConstraint(aLocation.Index, constraintHandle, 0);
                if (typeof(TBodyHandles) == typeof(TwoBodyHandles))
                {
                    ref var bLocation = ref simulation.Bodies.HandleToLocation[Unsafe.Add(ref handles, 1)];
                    Debug.Assert(bLocation.SetIndex == 0, "By the time we flush new constraints into the solver, all associated islands should have been activated.");
                    simulation.Bodies.AddConstraint(bLocation.Index, constraintHandle, 1);
                }
                pairCache.CompleteConstraintAdd(simulation.Solver, ref constraint.Impulses, constraint.ConstraintCacheIndex, constraintHandle, ref constraint.Pair);
            }



            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            unsafe void SequentialAddToSimulationSpeculative<TBodyHandles, TDescription, TContactImpulses>(ref UntypedList list, int narrowPhaseConstraintTypeId, Simulation simulation, ref PairCache pairCache)
                where TDescription : IConstraintDescription<TDescription>
            {
                if (list.Buffer.Allocated)
                {
                    ref var start = ref Unsafe.As<byte, PendingConstraint<TBodyHandles, TDescription, TContactImpulses>>(ref *list.Buffer.Memory);
                    Debug.Assert(list.Buffer.Length > Unsafe.SizeOf<PendingConstraint<TBodyHandles, TDescription, TContactImpulses>>() * list.Count);
                    Debug.Assert(list.ByteCount == Unsafe.SizeOf<PendingConstraint<TBodyHandles, TDescription, TContactImpulses>>() * list.Count);
                    ref var speculativeBatchIndicesForType = ref speculativeBatchIndices[narrowPhaseConstraintTypeId];
                    for (int i = 0; i < list.Count; ++i)
                    {
                        AddToSimulationSpeculative(ref Unsafe.Add(ref start, i), (int)speculativeBatchIndicesForType[i], simulation, pairCache);
                    }
                }
            }

            /// <summary>
            /// Flushes pending constraints into the simulation without any form of synchronization. Adds occur in the order of manifold generation.
            /// If the contact manifold generation is deterministic, then the result of this add will be deterministic.
            /// </summary>
            internal void FlushWithSpeculativeBatches(Simulation simulation, ref PairCache pairCache)
            {
                //This is going to be pretty horrible!
                //There is no type information beyond the index of the cache.
                //In other words, we basically have to do a switch statement (or equivalently manually unrolled loop) to gather objects of the proper type from it. 

                //This follows the same convention as the GatherOldImpulses and ScatterNewImpulses of the PairCache.
                //Constraints cover 16 possible cases:
                //1-4 contacts: 0x3
                //convex vs nonconvex: 0x4
                //1 body versus 2 body: 0x8
                //TODO: Very likely that we'll expand the nonconvex manifold maximum to 8 contacts, so this will need to be adjusted later.
                SequentialAddToSimulationSpeculative<int, Contact1OneBody, ContactImpulses1>(ref pendingConstraintsByType[0 + 0 + 0], 0 + 0 + 0, simulation, ref pairCache);
                SequentialAddToSimulationSpeculative<TwoBodyHandles, Contact1, ContactImpulses1>(ref pendingConstraintsByType[8 + 0 + 0], 8 + 0 + 0, simulation, ref pairCache);
                SequentialAddToSimulationSpeculative<TwoBodyHandles, Contact4, ContactImpulses4>(ref pendingConstraintsByType[8 + 0 + 3], 8 + 0 + 3, simulation, ref pairCache);
            }


            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static unsafe void DeterministicAdd<TBodyHandles, TDescription, TContactImpulses>(
                int typeIndex, ref SortConstraintTarget target, OverlapWorker[] overlapWorkers, Simulation simulation, ref PairCache pairCache)
                where TDescription : IConstraintDescription<TDescription>
            {
                ref var cache = ref overlapWorkers[target.WorkerIndex].PendingConstraints;
                ref var constraint = ref Unsafe.As<byte, PendingConstraint<TBodyHandles, TDescription, TContactImpulses>>(
                   ref *(cache.pendingConstraintsByType[typeIndex].Buffer.Memory + target.ByteIndexInCache));
                //TODO: check the cost of this idiv. It occurs on every single constraint add, so it might actually show up...
                //The question is whether it's slower to include another bit of metadata in the target for the non-byte index.
                //If you're willing to bitpack, we could use 10-12 bits for worker index and 20-22 bits for index. 4096 workers and a million new constraints is likely sufficient...
                var index = target.ByteIndexInCache / Unsafe.SizeOf<PendingConstraint<TBodyHandles, TDescription, TContactImpulses>>();
                AddToSimulationSpeculative(ref constraint, cache.speculativeBatchIndices[typeIndex][index], simulation, pairCache);
            }

            internal unsafe static void DeterministicallyAddType(
                int typeIndex, OverlapWorker[] overlapWorkers, ref QuickList<SortConstraintTarget, Buffer<SortConstraintTarget>> constraintsOfType,
                Simulation simulation, ref PairCache pairCache)
            {
                //By hoisting the switch statement outside of the loop, a bunch of pointless branching is avoided and we have have type information with a one time test.

                //This follows the same convention as the GatherOldImpulses and ScatterNewImpulses of the PairCache.
                //Constraints cover 16 possible cases:
                //1-4 contacts: 0x3
                //convex vs nonconvex: 0x4
                //1 body versus 2 body: 0x8
                //TODO: Very likely that we'll expand the nonconvex manifold maximum to 8 contacts, so this will need to be adjusted later.
                switch (typeIndex)
                {
                    //One body
                    //Convex
                    case 0:
                        for (int i = 0; i < constraintsOfType.Count; ++i)
                        {
                            DeterministicAdd<int, Contact1OneBody, ContactImpulses1>(typeIndex, ref constraintsOfType[i], overlapWorkers, simulation, ref pairCache);
                        }
                        break;
                    case 1:
                        break;
                    case 2:
                        break;
                    case 3:
                        break;
                    //Nonconvex
                    case 4 + 0:
                        break;
                    case 4 + 1:
                        break;
                    case 4 + 2:
                        break;
                    case 4 + 3:
                        break;
                    //Two body
                    //Convex
                    case 8 + 0:
                        for (int i = 0; i < constraintsOfType.Count; ++i)
                        {
                            DeterministicAdd<TwoBodyHandles, Contact1, ContactImpulses1>(typeIndex, ref constraintsOfType[i], overlapWorkers, simulation, ref pairCache);
                        }
                        break;
                    case 8 + 1:
                        break;
                    case 8 + 2:
                        break;
                    case 8 + 3:
                        for (int i = 0; i < constraintsOfType.Count; ++i)
                        {
                            DeterministicAdd<TwoBodyHandles, Contact4, ContactImpulses4>(typeIndex, ref constraintsOfType[i], overlapWorkers, simulation, ref pairCache);
                        }
                        break;
                    //Nonconvex
                    case 8 + 4 + 0:
                        break;
                    case 8 + 4 + 1:
                        break;
                    case 8 + 4 + 2:
                        break;
                    case 8 + 4 + 3:
                        break;
                }
            }


            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            internal unsafe void SpeculativeConstraintBatchSearch(Solver solver, int typeIndex, int start, int end)
            {
                ref var list = ref pendingConstraintsByType[typeIndex];
                Debug.Assert(list.Buffer.Allocated, "The target region should be allocated, or else the job scheduler is broken.");
                Debug.Assert(list.Count > 0);
                int byteIndex = start * list.ElementSizeInBytes;
                //This follows the same convention as the GatherOldImpulses and ScatterNewImpulses of the PairCache.
                //Constraints cover 16 possible cases:
                //1-4 contacts: 0x3
                //convex vs nonconvex: 0x4
                //1 body versus 2 body: 0x8
                //TODO: Very likely that we'll expand the nonconvex manifold maximum to 8 contacts, so this will need to be adjusted later.
                int bodyCount = (typeIndex >> 3) + 1;
                ref var speculativeBatchIndicesForType = ref speculativeBatchIndices[typeIndex];
                for (int i = start; i < end; ++i)
                {
                    speculativeBatchIndicesForType[i] = (ushort)solver.FindCandidateBatch(0, ref *(int*)(list.Buffer.Memory + byteIndex), bodyCount);
                    byteIndex += list.ElementSizeInBytes;
                }
            }


            //Note that disposal is separated from the flushes. This is required- accessing buffer pools is a potential race condition. Performing these returns in parallel with the 
            //freshness checker would cause bugs.
            public void Dispose()
            {
                for (int i = 0; i < PairCache.CollisionConstraintTypeCount; ++i)
                {
                    if (pendingConstraintsByType[i].Buffer.Allocated)
                        pool.Return(ref pendingConstraintsByType[i].Buffer);
                }
                pool.SpecializeFor<UntypedList>().Return(ref pendingConstraintsByType);
            }

            internal void AllocateForSpeculativeSearch()
            {
                pool.SpecializeFor<Buffer<ushort>>().Take(PairCache.CollisionConstraintTypeCount, out speculativeBatchIndices);
                speculativeBatchIndices.Clear(0, PairCache.CollisionConstraintTypeCount);
                var indexPool = pool.SpecializeFor<ushort>();
                for (int i = 0; i < PairCache.CollisionConstraintTypeCount; ++i)
                {
                    ref var typeList = ref pendingConstraintsByType[i];
                    if (typeList.Buffer.Allocated)
                    {
                        Debug.Assert(typeList.Count > 0);
                        indexPool.Take(typeList.Count, out speculativeBatchIndices[i]);
                    }
                }
            }

            internal void DisposeSpeculativeSearch()
            {
                var indexPool = pool.SpecializeFor<ushort>();
                for (int i = 0; i < PairCache.CollisionConstraintTypeCount; ++i)
                {
                    ref var indices = ref speculativeBatchIndices[i];
                    if (indices.Allocated)
                    {
                        indexPool.Return(ref indices);
                    }
                }
                pool.SpecializeFor<Buffer<ushort>>().Return(ref speculativeBatchIndices);
            }
            internal int CountConstraints()
            {
                int count = 0;
                for (int i = 0; i < PairCache.CollisionConstraintTypeCount; ++i)
                {
                    count += pendingConstraintsByType[i].Count;
                }
                return count;
            }

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void AddConstraint<TBodyHandles, TDescription, TContactImpulses>(int workerIndex, int manifoldConstraintType, ref CollidablePair pair,
            PairCacheIndex constraintCacheIndex, ref TContactImpulses impulses, TBodyHandles bodyHandles, ref TDescription constraintDescription)
            where TDescription : IConstraintDescription<TDescription>
        {
            overlapWorkers[workerIndex].PendingConstraints.AddConstraint(manifoldConstraintType, ref pair, constraintCacheIndex, bodyHandles, ref constraintDescription, ref impulses);
        }


    }
}
