using BepuUtilities.Memory;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using BepuPhysics.Collidables;
using BepuUtilities.Collections;
using System;
using BepuUtilities;

namespace BepuPhysics
{
    //You could bitpack these two into 4 bytes, but the value of that is pretty darn questionable.
    public struct BodyConstraintReference
    {
        public ConstraintHandle ConnectingConstraintHandle;
        public int BodyIndexInConstraint;
    }

    /// <summary>
    /// Stores a group of bodies- either the set of active bodies, or the bodies involved in an inactive simulation island.
    /// </summary>
    public struct BodySet
    {
        //Note that all body information is stored in AOS format.
        //While the pose integrator would technically benefit from (AO)SOA, it would only help in a magical infinite bandwidth scenario.
        //In practice, the pose integrator's actual AOSOA-benefitting chunk can't even scale to 2 threads, even with only 4-wide SIMD.
        //On top of that, the narrow phase and solver both need to access the body's information in a noncontiguous way. While the layout optimizer stages can help here to a degree,
        //the simple fact is that the scattered loads will likely waste a lot of cache line space- the majority, even, for wider SIMD bundles.
        //(Consider: noncontiguously sampling velocities.Linear.X on an AVX512 AOSOA layout would load a 64 byte cache line and use only 4 bytes of it!)

        //Plus, no one wants to deal with AOSOA layouts when writing game logic. Realistically, body data will be the most frequently accessed property in the engine, 
        //and not having to do a transpose to pull it into AOS is much less painful.

        /// <summary>
        /// Remaps a body index to its handle.
        /// </summary>
        public Buffer<BodyHandle> IndexToHandle;

        public Buffer<MotionState> MotionStates;
        public Buffer<BodyInertia> LocalInertias;

        /// <summary>
        /// The collidables owned by each body in the set. Speculative margins, continuity settings, and shape indices can be changed directly.
        /// Shape indices cannot transition between pointing at a shape and pointing at nothing or vice versa without notifying the broad phase of the collidable addition or removal.
        /// </summary>
        public Buffer<Collidable> Collidables;
        /// <summary>
        /// Activity states of bodies in the set.
        /// </summary>
        public Buffer<BodyActivity> Activity;
        /// <summary>
        /// List of constraints and constraint related data associated with each body in the set.
        /// </summary>
        public Buffer<BodyConstraints> Constraints;

        public int Count;
        /// <summary>
        /// Gets whether this instance is backed by allocated memory.
        /// </summary>
        public bool Allocated { get { return IndexToHandle.Allocated; } }

        public BodySet(int initialCapacity, BufferPool pool) : this()
        {
            InternalResize(initialCapacity, pool);
        }

        internal int Add(in BodyDescription bodyDescription, BodyHandle handle, int minimumConstraintCapacity, BufferPool pool)
        {
            var index = Count;
            if (index == IndexToHandle.Length)
            {
                InternalResize(IndexToHandle.Length * 2, pool);
            }
            ++Count;
            IndexToHandle[index] = handle;
            //Collidable's broad phase index is left unset. The Bodies collection is responsible for attaching that data.
            Constraints[index].References = new QuickList<BodyConstraintReference>(minimumConstraintCapacity, pool);

            ApplyDescriptionByIndex(index, bodyDescription);
            return index;
        }

        internal bool RemoveAt(int bodyIndex, out BodyHandle handle, out int movedBodyIndex, out BodyHandle movedBodyHandle)
        {
            handle = IndexToHandle[bodyIndex];
            //Move the last body into the removed slot.
            --Count;
            bool bodyMoved = bodyIndex < Count;
            ref var constraintsForRemovedSlot = ref Constraints[bodyIndex];
            if (bodyMoved)
            {
                movedBodyIndex = Count;
                //Copy the memory state of the last element down.
                MotionStates[bodyIndex] = MotionStates[movedBodyIndex];
                LocalInertias[bodyIndex] = LocalInertias[movedBodyIndex];
                Activity[bodyIndex] = Activity[movedBodyIndex];
                Collidables[bodyIndex] = Collidables[movedBodyIndex];
                //Note that the constraint list is NOT disposed before being overwritten.
                //The two callers for this function are 'true' removal, and sleeping. 
                //During true removal, the caller is responsible for removing all constraints and disposing the list.
                //In sleeping, the reference to the list is simply copied into the sleeping set.
                Constraints[bodyIndex] = Constraints[movedBodyIndex];
                //Point the body handles at the new location.
                movedBodyHandle = IndexToHandle[movedBodyIndex];
                IndexToHandle[bodyIndex] = movedBodyHandle;
            }
            else
            {
                movedBodyIndex = -1;
                movedBodyHandle = new BodyHandle(-1);
            }
            return bodyMoved;
        }

        internal void ApplyDescriptionByIndex(int index, in BodyDescription description)
        {
            Debug.Assert(!MathChecker.IsInvalid(description.Pose.Position.LengthSquared()), $"Invalid body position: {description.Pose.Position}");
            Debug.Assert(Math.Abs(1 - description.Pose.Orientation.LengthSquared()) < 1e-3f, $"Body orientation not unit length: {description.Pose.Orientation}");
            Debug.Assert(!MathChecker.IsInvalid(description.Velocity.Linear.LengthSquared()), $"Invalid body linear velocity: {description.Velocity.Linear}");
            Debug.Assert(!MathChecker.IsInvalid(description.Velocity.Angular.LengthSquared()), $"Invalid body angular velocity: {description.Velocity.Angular}");
            Debug.Assert(!MathChecker.IsInvalid(
                description.LocalInertia.InverseInertiaTensor.XX * description.LocalInertia.InverseInertiaTensor.XX +
                description.LocalInertia.InverseInertiaTensor.YX * description.LocalInertia.InverseInertiaTensor.YX +
                description.LocalInertia.InverseInertiaTensor.YY * description.LocalInertia.InverseInertiaTensor.YY +
                description.LocalInertia.InverseInertiaTensor.ZX * description.LocalInertia.InverseInertiaTensor.ZX +
                description.LocalInertia.InverseInertiaTensor.ZY * description.LocalInertia.InverseInertiaTensor.ZY +
                description.LocalInertia.InverseInertiaTensor.ZZ * description.LocalInertia.InverseInertiaTensor.ZZ), $"Invalid body inverse inertia tensor: {description.LocalInertia.InverseInertiaTensor}");
            Debug.Assert(!MathChecker.IsInvalid(description.LocalInertia.InverseMass) && description.LocalInertia.InverseMass >= 0, $"Invalid body inverse mass: {description.LocalInertia.InverseMass}");

            ref var state = ref MotionStates[index];
            state.Pose = description.Pose;
            state.Velocity = description.Velocity;
            //TODO: We're just trying this right now; note redundancy that we need to deal with.
            state.PackedLocalInertia = new PackedInertia(description.LocalInertia);
            LocalInertias[index] = description.LocalInertia;
            ref var collidable = ref Collidables[index];
            collidable.Continuity = description.Collidable.Continuity;
            collidable.SpeculativeMargin = description.Collidable.SpeculativeMargin;
            //Note that we change the shape here. If the collidable transitions from shapeless->shapeful or shapeful->shapeless, the broad phase has to be notified 
            //so that it can create/remove an entry. That's why this function isn't public.
            collidable.Shape = description.Collidable.Shape;
            ref var activity = ref Activity[index];
            activity.SleepThreshold = description.Activity.SleepThreshold;
            activity.MinimumTimestepsUnderThreshold = description.Activity.MinimumTimestepCountUnderThreshold;
            activity.TimestepsUnderThresholdCount = 0;
            activity.SleepCandidate = false;
        }

        public void GetDescription(int index, out BodyDescription description)
        {
            ref var state = ref MotionStates[index];
            description.Pose = state.Pose;
            description.Velocity = state.Velocity;
            description.LocalInertia = LocalInertias[index];
            ref var collidable = ref Collidables[index];
            description.Collidable.Continuity = collidable.Continuity;
            description.Collidable.Shape = collidable.Shape;
            description.Collidable.SpeculativeMargin = collidable.SpeculativeMargin;
            ref var activity = ref Activity[index];
            description.Activity.SleepThreshold = activity.SleepThreshold;
            description.Activity.MinimumTimestepCountUnderThreshold = activity.MinimumTimestepsUnderThreshold;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void AddConstraint(Solver solver, UnconstrainedBodies unconstrainedBodies, int bodyIndex, ConstraintHandle constraintHandle, int bodyIndexInConstraint, BufferPool pool)
        {
            BodyConstraintReference constraint;
            constraint.ConnectingConstraintHandle = constraintHandle;
            constraint.BodyIndexInConstraint = bodyIndexInConstraint;
            ref var constraints = ref Constraints[bodyIndex];
            Debug.Assert(constraints.References.Span.Allocated, "Any time a body is created, a list should be built to support it.");
            if (constraints.References.Span.Length == constraints.References.Count)
                constraints.References.Resize(constraints.References.Span.Length * 2, pool);
            constraints.References.AllocateUnsafely() = constraint;

            ref var constraintLocation = ref solver.HandleToConstraint[constraintHandle.Value];
            Debug.Assert(constraintLocation.SetIndex == 0, "Constraints should only be added to active bodies.");
            var batchIndex = constraintLocation.BatchIndex;
            if (constraints.References.Count == 1)
            {
                //The body is transitioning from unconstrained to constrained. The constraint will now be responsible for its integration.
                Debug.Assert(constraints.UnconstrainedIndex >= 0 && constraints.UnconstrainedIndex < unconstrainedBodies.Count);
                Debug.Assert(unconstrainedBodies.BodyIndices[constraints.UnconstrainedIndex] == bodyIndex);
                if (unconstrainedBodies.RemoveAt(constraints.UnconstrainedIndex, out var movedUnconstrainedBodyIndex))
                {
                    Constraints[movedUnconstrainedBodyIndex].UnconstrainedIndex = constraints.UnconstrainedIndex;
                }
                constraints.MinimumBatch = batchIndex;
                constraints.MaximumBatch = batchIndex;
                constraints.MinimumConstraint = constraintHandle;
                constraints.MaximumConstraint = constraintHandle;
                constraints.MinimumIndexInConstraint = bodyIndexInConstraint;
                constraints.MaximumIndexInConstraint = bodyIndexInConstraint;
                solver.AddEarlyIntegrationResponsibilityToConstraint(constraintHandle, bodyIndexInConstraint, bodyIndex);
                solver.AddLateIntegrationResponsibilityToConstraint(constraintHandle, bodyIndexInConstraint, bodyIndex);
            }
            else
            {
                var minimum = constraints.MinimumBatch;
                var maximum = constraints.MaximumBatch;
                if (batchIndex < minimum)
                {
                    solver.RemoveEarlyIntegrationResponsibilityFromConstraint(constraints.MinimumConstraint, constraints.MinimumIndexInConstraint, bodyIndex);
                    constraints.MinimumBatch = batchIndex;
                    constraints.MinimumConstraint = constraintHandle;
                    constraints.MinimumIndexInConstraint = bodyIndexInConstraint;
                    solver.AddEarlyIntegrationResponsibilityToConstraint(constraintHandle, bodyIndexInConstraint, bodyIndex);

                }
                else if (batchIndex > maximum)
                {
                    solver.RemoveLateIntegrationResponsibilityFromConstraint(constraints.MaximumConstraint, constraints.MaximumIndexInConstraint, bodyIndex);
                    constraints.MaximumBatch = batchIndex;
                    constraints.MaximumConstraint = constraintHandle;
                    constraints.MaximumIndexInConstraint = bodyIndexInConstraint;
                    solver.AddLateIntegrationResponsibilityToConstraint(constraintHandle, bodyIndexInConstraint, bodyIndex);
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void RemoveConstraintReference(Solver solver, UnconstrainedBodies unconstrainedBodies, int bodyIndex, ConstraintHandle constraintHandle, int minimumConstraintCapacityPerBody, BufferPool pool)
        {
            //This uses a linear search. That's fine; bodies will rarely have more than a handful of constraints associated with them.
            //Attempting to use something like a hash set for fast removes would just introduce more constant overhead and slow it down on average.
            ref var constraints = ref Constraints[bodyIndex];
            Debug.Assert(solver.HandleToConstraint[constraintHandle.Value].SetIndex == 0, "Removals must only occur on the active set.");
            ref var list = ref constraints.References;
            var isMinimum = constraintHandle == constraints.MinimumConstraint;
            if (isMinimum || constraintHandle == constraints.MaximumConstraint)
            {
                //This constraint used to have integration responsibility for this body.
                if (isMinimum)
                {
                    solver.RemoveEarlyIntegrationResponsibilityFromConstraint(constraints.MinimumConstraint, constraints.MinimumIndexInConstraint, bodyIndex);
                }
                else
                {
                    solver.RemoveLateIntegrationResponsibilityFromConstraint(constraints.MaximumConstraint, constraints.MaximumIndexInConstraint, bodyIndex);
                }
                if (list.Count == 1)
                {
                    //Removing this constraint from the list will leave the body unconstrained, and it should enter the unconstrained integration set.
                    constraints.UnconstrainedIndex = unconstrainedBodies.Add(bodyIndex, pool);
                    list.Count = 0;
                }
                else
                {
                    //We need to know which constraint is now responsible for the body.
                    if (isMinimum)
                    {
                        int newBatchIndex = int.MaxValue;
                        int newIndexInConstraint = -1;
                        ConstraintHandle newConstraintHandle = default;
                        //Find the new minimum batch index, and remove the target constraint.
                        for (int i = list.Count - 1; i >= 0; --i)
                        {
                            ref var reference = ref list[i];
                            if (reference.ConnectingConstraintHandle.Value == constraintHandle.Value)
                            {
                                list.FastRemoveAt(i);
                            }
                            else
                            {
                                var batchIndex = solver.HandleToConstraint[reference.ConnectingConstraintHandle.Value].BatchIndex;
                                if (batchIndex < newBatchIndex)
                                {
                                    newBatchIndex = batchIndex;
                                    newConstraintHandle = reference.ConnectingConstraintHandle;
                                    newIndexInConstraint = reference.BodyIndexInConstraint;
                                }
                            }
                        }
                        constraints.MinimumBatch = newBatchIndex;
                        constraints.MinimumConstraint = newConstraintHandle;
                        constraints.MinimumIndexInConstraint = newIndexInConstraint;
                        Debug.Assert(solver.HandleToConstraint[newConstraintHandle.Value].SetIndex >= 0);
                        solver.AddEarlyIntegrationResponsibilityToConstraint(newConstraintHandle, newIndexInConstraint, bodyIndex);
                    }
                    else
                    {
                        int newBatchIndex = -1;
                        int newIndexInConstraint = -1;
                        ConstraintHandle newConstraintHandle = default;
                        //Find the new maximum batch index, and remove the target constraint.
                        for (int i = list.Count - 1; i >= 0; --i)
                        {
                            ref var reference = ref list[i];
                            if (reference.ConnectingConstraintHandle.Value == constraintHandle.Value)
                            {
                                list.FastRemoveAt(i);
                            }
                            else
                            {
                                var batchIndex = solver.HandleToConstraint[reference.ConnectingConstraintHandle.Value].BatchIndex;
                                if (batchIndex > newBatchIndex)
                                {
                                    newBatchIndex = batchIndex;
                                    newConstraintHandle = reference.ConnectingConstraintHandle;
                                    newIndexInConstraint = reference.BodyIndexInConstraint;
                                }
                            }
                        }
                        constraints.MaximumBatch = newBatchIndex;
                        constraints.MaximumConstraint = newConstraintHandle;
                        constraints.MaximumIndexInConstraint = newIndexInConstraint;
                        Debug.Assert(solver.HandleToConstraint[newConstraintHandle.Value].SetIndex >= 0);
                        solver.AddLateIntegrationResponsibilityToConstraint(newConstraintHandle, newIndexInConstraint, bodyIndex);
                    }
                }

            }
            else
            {
                //This constraint did not have any integration responsibilities; we can remove it with no fanfare.
                for (int i = 0; i < list.Count; ++i)
                {
                    ref var element = ref list[i];
                    if (element.ConnectingConstraintHandle.Value == constraintHandle.Value)
                    {
                        list.FastRemoveAt(i);
                        break;
                    }
                }
            }
            //Note the conservative resizing threshold. If the current capacity is 8, the minimum capacity is 4, and the current count is 4, it COULD resize,
            //but this will not do so. Instead, it will wait for another halving- the current count would need to be 2 before the capacity is allowed to drop to 4.
            //This helps avoid excessive resizing when constraints are churning rapidly.
            var conservativeCount = 2 * list.Count;
            var targetCapacity = conservativeCount > minimumConstraintCapacityPerBody ? conservativeCount : minimumConstraintCapacityPerBody;
            //Don't bother trying to resize if it would end up just being the same power of 2.
            if (list.Span.Length >= 2 * targetCapacity)
            {
                //The list can be trimmed down a bit while still holding all existing constraints and obeying the minimum capacity.
                list.Resize(targetCapacity, pool);
            }
        }

        public bool BodyIsConstrainedBy(int bodyIndex, ConstraintHandle constraintHandle)
        {
            ref var list = ref Constraints[bodyIndex].References;
            for (int i = 0; i < list.Count; ++i)
            {
                if (list[i].ConnectingConstraintHandle.Value == constraintHandle.Value)
                {
                    return true;
                }
            }
            return false;
        }


        /// <summary>
        /// Swaps the memory of two bodies. Indexed by memory slot, not by handle index.
        /// </summary>
        /// <param name="slotA">Memory slot of the first body to swap.</param>
        /// <param name="slotB">Memory slot of the second body to swap.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void Swap(int slotA, int slotB, ref Buffer<BodyMemoryLocation> handleToIndex)
        {
            handleToIndex[IndexToHandle[slotA].Value].Index = slotB;
            handleToIndex[IndexToHandle[slotB].Value].Index = slotA;
            Helpers.Swap(ref IndexToHandle[slotA], ref IndexToHandle[slotB]);
            Helpers.Swap(ref Collidables[slotA], ref Collidables[slotB]);
            Helpers.Swap(ref MotionStates[slotA], ref MotionStates[slotB]);
            Helpers.Swap(ref LocalInertias[slotA], ref LocalInertias[slotB]);
            Helpers.Swap(ref Activity[slotA], ref Activity[slotB]);
            Helpers.Swap(ref Constraints[slotA], ref Constraints[slotB]);
        }

        internal unsafe void InternalResize(int targetBodyCapacity, BufferPool pool)
        {
            Debug.Assert(targetBodyCapacity > 0, "Resize is not meant to be used as Dispose. If you want to return everything to the pool, use Dispose instead.");
            //Note that we base the bundle capacities on post-resize capacity of the IndexToHandle array. This simplifies the conditions on allocation, but increases memory use.
            //You may want to change this in the future if memory use is concerning.
            targetBodyCapacity = BufferPool.GetCapacityForCount<int>(targetBodyCapacity);
            Debug.Assert(MotionStates.Length != BufferPool.GetCapacityForCount<RigidPoses>(targetBodyCapacity), "Should not try to use internal resize of the result won't change the size.");
            pool.ResizeToAtLeast(ref MotionStates, targetBodyCapacity, Count);
            pool.ResizeToAtLeast(ref LocalInertias, targetBodyCapacity, Count);
            pool.ResizeToAtLeast(ref IndexToHandle, targetBodyCapacity, Count);
            pool.ResizeToAtLeast(ref Collidables, targetBodyCapacity, Count);
            pool.ResizeToAtLeast(ref Activity, targetBodyCapacity, Count);
            pool.ResizeToAtLeast(ref Constraints, targetBodyCapacity, Count);
        }

        public unsafe void Clear(BufferPool pool)
        {
            for (int i = 0; i < Count; ++i)
            {
                Constraints[i].References.Dispose(pool);
            }
            Count = 0;
        }

        /// <summary>
        /// Disposes the buffers, but nothing inside of the buffers. Per-body constraint lists stored in the set will not be returned.
        /// </summary>
        /// <param name="pool">Pool to return the set's top level buffers to.</param>
        public void DisposeBuffers(BufferPool pool)
        {
            pool.Return(ref MotionStates);
            pool.Return(ref LocalInertias);
            pool.Return(ref IndexToHandle);
            pool.Return(ref Collidables);
            pool.Return(ref Activity);
            pool.Return(ref Constraints);
        }

        /// <summary>
        /// Disposes the body set's buffers and any resources within them.
        /// </summary>
        /// <param name="pool">Pool to return resources to.</param>
        public void Dispose(BufferPool pool)
        {
            for (int i = 0; i < Count; ++i)
            {
                Constraints[i].References.Dispose(pool);
            }
            DisposeBuffers(pool);
            this = new BodySet();
        }
    }
}
