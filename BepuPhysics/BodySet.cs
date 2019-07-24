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
        public int ConnectingConstraintHandle;
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
        public Buffer<int> IndexToHandle;

        public Buffer<RigidPose> Poses;
        public Buffer<BodyVelocity> Velocities;
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
        /// List of constraints associated with each body in the set.
        /// </summary>
        public Buffer<QuickList<BodyConstraintReference>> Constraints;

        public int Count;
        /// <summary>
        /// Gets whether this instance is backed by allocated memory.
        /// </summary>
        public bool Allocated { get { return IndexToHandle.Allocated; } }

        public BodySet(int initialCapacity, BufferPool pool) : this()
        {
            InternalResize(initialCapacity, pool);
        }

        internal int Add(in BodyDescription bodyDescription, int handle, int minimumConstraintCapacity, BufferPool pool)
        {
            var index = Count;
            if (index == IndexToHandle.Length)
            {
                InternalResize(IndexToHandle.Length * 2, pool);
            }
            ++Count;
            IndexToHandle[index] = handle;
            //Collidable's broad phase index is left unset. The Bodies collection is responsible for attaching that data.
            Constraints[index] = new QuickList<BodyConstraintReference>(minimumConstraintCapacity, pool);
            ApplyDescriptionByIndex(index, bodyDescription);
            return index;
        }

        internal bool RemoveAt(int bodyIndex, BufferPool pool, out int handle, out int movedBodyIndex, out int movedBodyHandle)
        {
            handle = IndexToHandle[bodyIndex];
            //Move the last body into the removed slot.
            --Count;
            bool bodyMoved = bodyIndex < Count;
            if (bodyMoved)
            {
                movedBodyIndex = Count;
                //Copy the memory state of the last element down.
                Poses[bodyIndex] = Poses[movedBodyIndex];
                Velocities[bodyIndex] = Velocities[movedBodyIndex];
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
                movedBodyHandle = -1;
            }
            return bodyMoved;
        }

        internal void ApplyDescriptionByIndex(int index, in BodyDescription description)
        {
            Poses[index] = description.Pose;
            Velocities[index] = description.Velocity;
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
            description.Pose = Poses[index];
            description.Velocity = Velocities[index];
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
        internal void AddConstraint(int bodyIndex, int constraintHandle, int bodyIndexInConstraint, BufferPool pool)
        {
            BodyConstraintReference constraint;
            constraint.ConnectingConstraintHandle = constraintHandle;
            constraint.BodyIndexInConstraint = bodyIndexInConstraint;
            ref var constraints = ref Constraints[bodyIndex];
            Debug.Assert(constraints.Span.Allocated, "Any time a body is created, a list should be built to support it.");
            if (constraints.Span.Length == constraints.Count)
                constraints.Resize(constraints.Span.Length * 2, pool);
            constraints.AllocateUnsafely() = constraint;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void RemoveConstraintReference(int bodyIndex, int constraintHandle, int minimumConstraintCapacityPerBody, BufferPool pool)
        {
            //This uses a linear search. That's fine; bodies will rarely have more than a handful of constraints associated with them.
            //Attempting to use something like a hash set for fast removes would just introduce more constant overhead and slow it down on average.
            ref var list = ref Constraints[bodyIndex];
            for (int i = 0; i < list.Count; ++i)
            {
                ref var element = ref list[i];
                if (element.ConnectingConstraintHandle == constraintHandle)
                {
                    list.FastRemoveAt(i);
                    break;
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

        public bool BodyIsConstrainedBy(int bodyIndex, int constraintHandle)
        {
            ref var list = ref Constraints[bodyIndex];
            for (int i = 0; i < list.Count; ++i)
            {
                if (list[i].ConnectingConstraintHandle == constraintHandle)
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
        internal void Swap(int slotA, int slotB, ref Buffer<BodyLocation> handleToIndex)
        {
            handleToIndex[IndexToHandle[slotA]].Index = slotB;
            handleToIndex[IndexToHandle[slotB]].Index = slotA;
            Helpers.Swap(ref IndexToHandle[slotA], ref IndexToHandle[slotB]);
            Helpers.Swap(ref Collidables[slotA], ref Collidables[slotB]);
            Helpers.Swap(ref Poses[slotA], ref Poses[slotB]);
            Helpers.Swap(ref Velocities[slotA], ref Velocities[slotB]);
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
            Debug.Assert(Poses.Length != BufferPool.GetCapacityForCount<RigidPoses>(targetBodyCapacity), "Should not try to use internal resize of the result won't change the size.");
            pool.ResizeToAtLeast(ref Poses, targetBodyCapacity, Count);
            pool.ResizeToAtLeast(ref Velocities, targetBodyCapacity, Count);
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
                Constraints[i].Dispose(pool);
            }
            Count = 0;
        }

        /// <summary>
        /// Disposes the buffers, but nothing inside of the buffers. Per-body constraint lists stored in the set will not be returned.
        /// </summary>
        /// <param name="pool">Pool to return the set's top level buffers to.</param>
        public void DisposeBuffers(BufferPool pool)
        {
            pool.Return(ref Poses);
            pool.Return(ref Velocities);
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
                Constraints[i].Dispose(pool);
            }
            DisposeBuffers(pool);
            this = new BodySet();
        }
    }
}
