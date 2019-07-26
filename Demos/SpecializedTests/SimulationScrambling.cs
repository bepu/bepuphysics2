using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;

namespace Demos.SpecializedTests
{
    public static class SimulationScrambling
    {
        public static void ScrambleBodies(Simulation simulation) 
        {
            //Having every single body in order is pretty unrealistic. In a real application, churn and general lack of care will result in 
            //scrambled body versus constraint memory access patterns. That's a big increase in cache misses.
            //Scrambling the body array simulates this.
            //Given a sufficiently large added overhead, it would benefit the engine to include runtime cache optimization.
            //That is, move the memory location of bodies (and constraints, within type batches) to maximize the number of accesses to already-cached bodies.

            Random random = new Random(5);
            for (int i = simulation.Bodies.ActiveSet.Count - 1; i >= 1; --i)
            {
                //This helper function handles the updates that have to be performed across all body-sensitive systems.
                BodyLayoutOptimizer.SwapBodyLocation(simulation.Bodies, simulation.Solver, i, random.Next(i));
            }

        }

        public static void ScrambleConstraints(Solver solver)
        {
            Random random = new Random(5);
            ref var activeSet = ref solver.ActiveSet;
            for (int i = 0; i < activeSet.Batches.Count; ++i)
            {
                for (int j = 0; j < activeSet.Batches[i].TypeBatches.Count; ++j)
                {
                    ref var typeBatch = ref activeSet.Batches[i].TypeBatches[j];
                    solver.TypeProcessors[typeBatch.TypeId].Scramble(ref typeBatch, random, ref solver.HandleToConstraint);
                }
            }
        }
        public static void ScrambleBodyConstraintLists(Simulation simulation)
        {
            Random random = new Random(5);
            //Body lists are isolated enough that we don't have to worry about a bunch of internal bookkeeping. Just pull the list and mess with it.
            //Note that we cannot change the order of bodies within constraints! That would change behavior.
            for (int bodyIndex = 0; bodyIndex < simulation.Bodies.ActiveSet.Count; ++bodyIndex)
            {
                ref var list = ref simulation.Bodies.ActiveSet.Constraints[bodyIndex];
                for (int i = 0; i < list.Count - 1; ++i)
                {
                    ref var currentSlot = ref list[i];
                    ref var otherSlot = ref list[random.Next(i + 1, list.Count)];
                    var currentTemp = currentSlot;
                    currentSlot = otherSlot;
                    otherSlot = currentTemp;
                }
            }
        }


        struct CachedConstraint<T> where T : IConstraintDescription<T>
        {
            public T Description;
            public int BodyA;
            public int BodyB;
        }

        struct BodyEnumerator : IForEach<int>
        {
            public Bodies Bodies;
            public int[] HandlesToIdentity;
            public int IdentityA;
            public int IdentityB;
            public int IndexInConstraint;

            public BodyEnumerator(Bodies bodies, int[] handleToEntryIndex)
            {
                Bodies = bodies;
                HandlesToIdentity = handleToEntryIndex;
                IdentityA = IdentityB = IndexInConstraint = 0;

            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int connectedBodyIndex)
            {
                var entryIndex = HandlesToIdentity[Bodies.ActiveSet.IndexToHandle[connectedBodyIndex]];
                if (IndexInConstraint == 0)
                    IdentityA = entryIndex;
                else
                    IdentityB = entryIndex;
                ++IndexInConstraint;
            }
        }


        static void RemoveConstraint(Simulation simulation, int constraintHandle, int[] constraintHandlesToIdentity, int[] constraintHandles, List<int> removedConstraints)
        {
            var constraintIdentity = constraintHandlesToIdentity[constraintHandle];
            constraintHandlesToIdentity[constraintHandle] = -1;
            constraintHandles[constraintIdentity] = -1;
            simulation.Solver.Remove(constraintHandle);
            removedConstraints.Add(constraintIdentity);
        }

        struct ConstraintBodyValidationEnumerator : IForEach<int>
        {
            public Simulation Simulation;
            public int ConstraintHandle;
            public void LoopBody(int bodyIndex)
            {
                //The body in this constraint should both:
                //1) have a handle associated with it, and 
                //2) the constraint graph list for the body should include the constraint handle.
                Debug.Assert(Simulation.Bodies.ActiveSet.IndexToHandle[bodyIndex] >= 0);
                Debug.Assert(Simulation.Bodies.ActiveSet.BodyIsConstrainedBy(bodyIndex, ConstraintHandle));
            }
        }

        [Conditional("DEBUG")]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void WriteLine(string message)
        {
            //Debug.WriteLine(message);
        }

        [Conditional("DEBUG")]
        static void Validate(Simulation simulation, List<int> removedConstraints, List<int> removedBodies, int originalBodyCount, int originalConstraintCount)
        {
            ref var activeSet = ref simulation.Solver.ActiveSet;
            for (int batchIndex = 0; batchIndex < activeSet.Batches.Count; ++batchIndex)
            {
                ref var batch = ref activeSet.Batches[batchIndex];
                if (batchIndex == activeSet.Batches.Count - 1)
                {
                    Debug.Assert(batch.TypeBatches.Count > 0, "While a lower indexed batch may have zero elements (especially while batch compression isn't active), " +
                        "there should never be an empty batch at the end of the list.");
                }
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                    var typeProcessor = simulation.Solver.TypeProcessors[typeBatch.TypeId];
                    Debug.Assert(typeBatch.ConstraintCount > 0, "If a type batch exists, there should be constraints in it.");
                    for (int indexInTypeBatch = 0; indexInTypeBatch < typeBatch.ConstraintCount; ++indexInTypeBatch)
                    {
                        var constraintHandle = typeBatch.IndexToHandle[indexInTypeBatch];
                        var constraintLocation = simulation.Solver.HandleToConstraint[constraintHandle];
                        Debug.Assert(
                            constraintLocation.IndexInTypeBatch == indexInTypeBatch &&
                            batch.TypeIndexToTypeBatchIndex[constraintLocation.TypeId] == typeBatchIndex &&
                            constraintLocation.BatchIndex == batchIndex, "The constraint location stored by the solver should agree with the actual type batch entries.");
                        ConstraintBodyValidationEnumerator enumerator;
                        enumerator.ConstraintHandle = constraintHandle;
                        enumerator.Simulation = simulation;
                        typeProcessor.EnumerateConnectedBodyIndices(ref typeBatch, indexInTypeBatch, ref enumerator);
                    }
                }
            }
            var constraintCount = 0;
            foreach (var batch in activeSet.Batches)
            {
                foreach (var typeBatch in batch.TypeBatches)
                {
                    constraintCount += typeBatch.ConstraintCount;
                }
            }

            Debug.Assert(removedConstraints.Count + constraintCount == originalConstraintCount, "Must not have lost (or gained) any constraints!");
            Debug.Assert(removedBodies.Count + simulation.Bodies.ActiveSet.Count == originalBodyCount, "Must not have lost (or gained) any bodies!");

        }
        static void FastRemoveAt<T>(List<T> list, int index)
        {
            var lastIndex = list.Count - 1;
            if (lastIndex != index)
            {
                list[index] = list[lastIndex];
            }
            list.RemoveAt(lastIndex);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static void ChurnAddBody(Simulation simulation, BodyDescription[] bodyDescriptions, int[] bodyHandles, int[] bodyHandlesToIdentity,
            int originalConstraintCount, List<int> removedConstraints, List<int> removedBodies, Random random)
        {
            //Add a body.
            var toAddIndex = random.Next(removedBodies.Count);
            var toAdd = removedBodies[toAddIndex];
            FastRemoveAt(removedBodies, toAddIndex);
            var bodyHandle = simulation.Bodies.Add(bodyDescriptions[toAdd]);
            bodyHandlesToIdentity[bodyHandle] = toAdd;
            bodyHandles[toAdd] = bodyHandle;
            WriteLine($"Added body, handle: {bodyHandle}");
            Validate(simulation, removedConstraints, removedBodies, bodyHandles.Length, originalConstraintCount);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static void ChurnRemoveBody<T>(Simulation simulation, int[] bodyHandles, int[] bodyHandlesToIdentity, int[] constraintHandles,
            int[] constraintHandlesToIdentity, CachedConstraint<T>[] constraintDescriptions,
            List<int> removedConstraints, List<int> removedBodies, Random random) where T : IConstraintDescription<T>
        {
            //Remove a body.
            var removedBodyIndex = random.Next(simulation.Bodies.ActiveSet.Count);
            //All constraints associated with the body have to be removed first.
            ref var constraintList = ref simulation.Bodies.ActiveSet.Constraints[removedBodyIndex];
            for (int i = constraintList.Count - 1; i >= 0; --i)
            {
                WriteLine($"Removing constraint (handle: {constraintList[i].ConnectingConstraintHandle}) for a body removal.");
                RemoveConstraint(simulation, constraintList[i].ConnectingConstraintHandle, constraintHandlesToIdentity, constraintHandles, removedConstraints);
            }
#if DEBUG
            Debug.Assert(constraintList.Count == 0, "After we removed all the constraints, the constraint list should be empty! (It's a ref to the actual slot!)");
#endif
            var handle = simulation.Bodies.ActiveSet.IndexToHandle[removedBodyIndex];
            simulation.Bodies.Remove(handle);
            bodyHandles[bodyHandlesToIdentity[handle]] = -1;
            removedBodies.Add(bodyHandlesToIdentity[handle]);
            bodyHandlesToIdentity[handle] = -1;
            WriteLine($"Removed body, former handle: {handle}");
            Validate(simulation, removedConstraints, removedBodies, bodyHandles.Length, constraintHandles.Length);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static void ChurnAddConstraint<T>(Simulation simulation, int[] bodyHandles, int[] constraintHandles, int[] constraintHandlesToIdentity,
            CachedConstraint<T>[] constraintDescriptions, List<int> removedConstraints, List<int> removedBodies, Random random) where T : ITwoBodyConstraintDescription<T>
        {
            //Add a constraint.
            int attemptCount = 0;
            do
            {
                //There's no guarantee that the bodies involved with the removed constraint are actually in the simulation.
                //Rather than doing anything clever, just retry a few times.
                var constraintIdentityIndex = random.Next(removedConstraints.Count);
                var constraintIdentity = removedConstraints[constraintIdentityIndex];
                ref var constraint = ref constraintDescriptions[constraintIdentity];
                int handleA, handleB;
                if ((handleA = bodyHandles[constraint.BodyA]) >= 0 && (handleB = bodyHandles[constraint.BodyB]) >= 0)
                {
                    //The constraint is addable.
                    var constraintHandle = simulation.Solver.Add(handleA, handleB, ref constraint.Description);
                    constraintHandles[constraintIdentity] = constraintHandle;
                    constraintHandlesToIdentity[constraintHandle] = constraintIdentity;
                    WriteLine($"Added constraint, handle: {constraintHandle}");
                    FastRemoveAt(removedConstraints, constraintIdentityIndex);
                    break;
                }
            } while (++attemptCount < 10);
            Validate(simulation, removedConstraints, removedBodies, bodyHandles.Length, constraintHandles.Length);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static void ChurnRemoveConstraint<T>(Simulation simulation, int originalBodyCount,
            int[] constraintHandlesToIdentity, int[] constraintHandles, CachedConstraint<T>[] constraintDescriptions, List<int> removedConstraints, List<int> removedBodies, Random random)
            where T : IConstraintDescription<T>
        {
            //Remove a constraint.
            ref var activeSet = ref simulation.Solver.ActiveSet;
            var batchIndex = random.Next(activeSet.Batches.Count);
            ref var batch = ref activeSet.Batches[batchIndex];
            Debug.Assert(batchIndex < activeSet.Batches.Count - 1 || batch.TypeBatches.Count > 0,
                "While a lower index batch may end up empty due to a lack of active batch compression, " +
                "the last batch should get removed if it becomes empty since there is no danger of pointer invaldiation.");
            if (batch.TypeBatches.Count > 0)
            {
                ref var typeBatch = ref batch.TypeBatches[random.Next(batch.TypeBatches.Count)];
                Debug.Assert(typeBatch.ConstraintCount > 0, "If a type batch exists, it should have constraints in it.");
                var indexInTypeBatch = random.Next(typeBatch.ConstraintCount);
                var constraintHandle = typeBatch.IndexToHandle[indexInTypeBatch];

                RemoveConstraint(simulation, constraintHandle, constraintHandlesToIdentity, constraintHandles, removedConstraints);
                WriteLine($"Removed constraint, former handle: {constraintHandle}");
                Validate(simulation, removedConstraints, removedBodies, originalBodyCount, constraintHandles.Length);
            }
        }

        public static double AddRemoveChurn<T>(Simulation simulation, int iterations, int[] bodyHandles, int[] constraintHandles) where T : ITwoBodyConstraintDescription<T>
        {
            //There are three levels of 'index' for each object in this test:
            //1) The top level 'identity'. Even when a body or constraint gets readded, the slot in the top level array maintains a pointer to the new handle.
            //2) The in-engine handle. Within the engine, it acts as the identity. The engine only cares about tracking identity between calls to add and remove for any given object.
            //3) The index of the object in memory.
            //As we add and remove stuff, we want to still be able to find a particular constraint by its original identity, so we have to do some work to track that.

            //Take a snapshot of the body descriptions.
            var bodyDescriptions = new BodyDescription[bodyHandles.Length];
            var constraintDescriptions = new CachedConstraint<T>[constraintHandles.Length];
            Debug.Assert(simulation.Bodies.ActiveSet.Count == bodyHandles.Length);
            int originalConstraintCount = simulation.Solver.CountConstraints();
            Debug.Assert(constraintHandles.Length == originalConstraintCount);

            //We'll need a mapping from the current handles back to the identity.
            var bodyHandlesToIdentity = new int[simulation.Bodies.HandleToLocation.Length];
            for (int i = 0; i < bodyHandlesToIdentity.Length; ++i)
                bodyHandlesToIdentity[i] = -1;
            var constraintHandlesToIdentity = new int[simulation.Solver.HandleToConstraint.Length];
            for (int i = 0; i < constraintHandlesToIdentity.Length; ++i)
                constraintHandlesToIdentity[i] = -1;

            for (int i = 0; i < bodyHandles.Length; ++i)
            {
                ref var bodyDescription = ref bodyDescriptions[i];
                var handle = bodyHandles[i];
                simulation.Bodies.GetDescription(handle, out bodyDescription);
                bodyHandlesToIdentity[handle] = i;
            }

            for (int i = 0; i < constraintHandles.Length; ++i)
            {
                var constraintHandle = constraintHandles[i];
                constraintHandlesToIdentity[constraintHandle] = i;
                simulation.Solver.GetDescription(constraintHandle, out constraintDescriptions[i].Description);
                simulation.Solver.GetConstraintReference(constraintHandle, out var reference);

                var bodyIdentityEnumerator = new BodyEnumerator(simulation.Bodies, bodyHandlesToIdentity);
                simulation.Solver.TypeProcessors[reference.TypeBatch.TypeId].EnumerateConnectedBodyIndices(ref reference.TypeBatch, reference.IndexInTypeBatch, ref bodyIdentityEnumerator);
                constraintDescriptions[i].BodyA = bodyIdentityEnumerator.IdentityA;
                constraintDescriptions[i].BodyB = bodyIdentityEnumerator.IdentityB;
            }



            //Any time a body is removed, the handle in the associated body entry must be updated to -1.
            //All constraints refer to bodies by their out-of-engine identity so that everything stays robust in the face of adds and removes.
            var removedConstraints = new List<int>();
            var removedBodies = new List<int>();
            var random = new Random(5);

            Validate(simulation, removedConstraints, removedBodies, bodyHandles.Length, originalConstraintCount);

            var constraintActionProbability = originalConstraintCount > 0 ? 1 - (double)simulation.Bodies.ActiveSet.Count / originalConstraintCount : 0;

            var timer = Stopwatch.StartNew();
            for (int iterationIndex = 0; iterationIndex < iterations; ++iterationIndex)
            {
                if (random.NextDouble() < constraintActionProbability)
                {
                    //Constraint action.
                    var constraintRemovalProbability = (originalConstraintCount - removedConstraints.Count) / (double)originalConstraintCount;
                    if (random.NextDouble() < constraintRemovalProbability)
                    {
                        ChurnRemoveConstraint(simulation, bodyHandles.Length, constraintHandlesToIdentity, constraintHandles, constraintDescriptions, removedConstraints, removedBodies, random);
                    }
                    else if (removedConstraints.Count > 0)
                    {
                        ChurnAddConstraint(simulation, bodyHandles, constraintHandles, constraintHandlesToIdentity, constraintDescriptions, removedConstraints, removedBodies, random);
                    }
                }
                else
                {
                    //Body action.
                    var bodyRemovalProbability = (bodyHandles.Length - removedBodies.Count) / (double)bodyHandles.Length;
                    if (random.NextDouble() < bodyRemovalProbability)
                    {
                        ChurnRemoveBody(simulation, bodyHandles, bodyHandlesToIdentity, constraintHandles, constraintHandlesToIdentity, constraintDescriptions, removedConstraints, removedBodies, random);
                    }
                    else if (removedBodies.Count > 0)
                    {
                        ChurnAddBody(simulation, bodyDescriptions, bodyHandles, bodyHandlesToIdentity, originalConstraintCount, removedConstraints, removedBodies, random);
                    }
                }
            }
            timer.Stop();

            //Go ahead and add everything back so the outer test can proceed unaffected. Theoretically.
            while (removedBodies.Count > 0)
            {
                ChurnAddBody(simulation, bodyDescriptions, bodyHandles, bodyHandlesToIdentity, originalConstraintCount, removedConstraints, removedBodies, random);
            }
            while (removedConstraints.Count > 0)
            {
                ChurnAddConstraint(simulation, bodyHandles, constraintHandles, constraintHandlesToIdentity, constraintDescriptions, removedConstraints, removedBodies, random);
            }

            for (int i = 0; i < constraintHandles.Length; ++i)
            {
                simulation.Solver.GetDescription(constraintHandles[i], out T description);
                Debug.Assert(description.Equals(constraintDescriptions[i].Description), "Moving constraints around should not affect their descriptions.");
            }

            var newConstraintCount = simulation.Solver.CountConstraints();
            Debug.Assert(newConstraintCount == originalConstraintCount, "Best have the same number of constraints if we actually added them all back!");
            Debug.Assert(bodyHandles.Length == simulation.Bodies.ActiveSet.Count, "And bodies, too!");

            return timer.Elapsed.TotalSeconds;
        }


    }
}
