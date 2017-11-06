using BepuUtilities.Memory;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using BepuPhysics.Collidables;

namespace BepuPhysics
{
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
        /// <summary>
        /// The set of collidables owned by each body. Speculative margins, continuity settings, and shape indices can be changed directly.
        /// Shape indices cannot transition between pointing at a shape and pointing at nothing or vice versa without notifying the broad phase of the collidable addition or removal.
        /// </summary>
        public Buffer<Collidable> Collidables;

        public Buffer<RigidPose> Poses;
        public Buffer<BodyVelocity> Velocities;
        public Buffer<BodyInertia> LocalInertias;

        public int Count;
        /// <summary>
        /// Gets whether this instance is backed by allocated memory.
        /// </summary>
        public bool Allocated { get { return IndexToHandle.Allocated; } }

        public BodySet(int initialCapacity, BufferPool pool) : this()
        {
            InternalResize(initialCapacity, pool);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void Swap<T>(ref T a, ref T b)
        {
            var temp = a;
            a = b;
            b = temp;
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
            Swap(ref IndexToHandle[slotA], ref IndexToHandle[slotB]);
            Swap(ref Collidables[slotA], ref Collidables[slotB]);
            Swap(ref Poses[slotA], ref Poses[slotB]);
            Swap(ref Velocities[slotA], ref Velocities[slotB]);
            Swap(ref LocalInertias[slotA], ref LocalInertias[slotB]);
        }

        internal unsafe void InternalResize(int targetBodyCapacity, BufferPool pool)
        {
            Debug.Assert(targetBodyCapacity > 0, "Resize is not meant to be used as Dispose. If you want to return everything to the pool, use Dispose instead.");
            //Note that we base the bundle capacities on post-resize capacity of the IndexToHandle array. This simplifies the conditions on allocation, but increases memory use.
            //You may want to change this in the future if memory use is concerning.
            targetBodyCapacity = BufferPool<int>.GetLowestContainingElementCount(targetBodyCapacity);
            Debug.Assert(Poses.Length != BufferPool<RigidPoses>.GetLowestContainingElementCount(targetBodyCapacity), "Should not try to use internal resize of the result won't change the size.");
            pool.SpecializeFor<RigidPose>().Resize(ref Poses, targetBodyCapacity, Count);
            pool.SpecializeFor<BodyVelocity>().Resize(ref Velocities, targetBodyCapacity, Count);
            pool.SpecializeFor<BodyInertia>().Resize(ref LocalInertias, targetBodyCapacity, Count);
            pool.SpecializeFor<int>().Resize(ref IndexToHandle, targetBodyCapacity, Count);
            pool.SpecializeFor<Collidable>().Resize(ref Collidables, targetBodyCapacity, Count);
            //TODO: You should probably examine whether these protective initializations are still needed.
            //Initialize all the indices beyond the copied region to -1.
            Unsafe.InitBlockUnaligned(((int*)IndexToHandle.Memory) + Count, 0xFF, (uint)(sizeof(int) * (IndexToHandle.Length - Count)));
            //Collidables beyond the body count should all point to nothing, which corresponds to zero.
            Collidables.Clear(Count, Collidables.Length - Count);
        }

        public unsafe void Clear()
        {
            Count = 0;
            //TODO: Should confirm that these inits are still needed. They are for Handle->Location, but this is the opposite direction.
            Unsafe.InitBlockUnaligned(IndexToHandle.Memory, 0xFF, (uint)(sizeof(int) * IndexToHandle.Length));
        }


        public void Dispose(BufferPool pool)
        {
            pool.SpecializeFor<RigidPose>().Return(ref Poses);
            pool.SpecializeFor<BodyVelocity>().Return(ref Velocities);
            pool.SpecializeFor<BodyInertia>().Return(ref LocalInertias);
            pool.SpecializeFor<int>().Return(ref IndexToHandle);
            pool.SpecializeFor<Collidable>().Return(ref Collidables);
        }

        internal int Add(ref BodyDescription bodyDescription, int handle)
        {
            var index = Count++;
            IndexToHandle[index] = handle;
            ref var collidable = ref Collidables[index];
            collidable.Shape = bodyDescription.Collidable.Shape;
            collidable.Continuity = bodyDescription.Collidable.Continuity;
            collidable.SpeculativeMargin = bodyDescription.Collidable.SpeculativeMargin;
            //Collidable's broad phase index is left unset. The simulation is responsible for attaching that data.

            Poses[index] = bodyDescription.Pose;
            Velocities[index] = bodyDescription.Velocity;
            LocalInertias[index] = bodyDescription.LocalInertia;
            return index;
        }

        internal bool RemoveAt(int bodyIndex, out int handle, out int movedBodyIndex, out int movedBodyHandle)
        {
            handle = IndexToHandle[bodyIndex];
            //Move the last body into the removed slot.
            //This does introduce disorder- there may be value in a second overload that preserves order, but it would require large copies.
            //In the event that so many adds and removals are performed at once that they destroy contiguity, it may be better to just
            //explicitly sort after the fact rather than attempt to retain contiguity incrementally. Handle it as a batch, in other words.
            --Count;
            bool bodyMoved = bodyIndex < Count;
            if (bodyMoved)
            {
                movedBodyIndex = Count;
                //Copy the memory state of the last element down.
                Poses[bodyIndex] = Poses[movedBodyIndex];
                Velocities[bodyIndex] = Velocities[movedBodyIndex];
                LocalInertias[bodyIndex] = LocalInertias[movedBodyIndex];
                //Note that if you ever treat the world inertias as 'always updated', it would need to be copied here.
                Collidables[bodyIndex] = Collidables[movedBodyIndex];
                //Point the body handles at the new location.
                movedBodyHandle = IndexToHandle[movedBodyIndex];
                IndexToHandle[bodyIndex] = movedBodyHandle;

            }
            else
            {
                movedBodyIndex = -1;
                movedBodyHandle = -1;
            }
            //We rely on the collidable references being nonexistent beyond the body count.
            //TODO: is this still true? Are these inits required?
            Collidables[Count] = new Collidable();
            //The indices should also be set to all -1's beyond the body count.
            IndexToHandle[Count] = -1;
            return bodyMoved;
        }
    }
}
