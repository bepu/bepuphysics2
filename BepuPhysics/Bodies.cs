using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics.Constraints;
using BepuPhysics.Collidables;
using BepuUtilities.Collections;

namespace BepuPhysics
{
    public struct BodyLocation
    {
        /// <summary>
        /// Index of the set owning the body reference. If the island index is 0, the body is active.
        /// </summary>
        public int SetIndex;
        /// <summary>
        /// Index of the body within its owning set. If the body is active (and so the Island index is -1), this is an index into the Bodies data arrays. 
        /// If it is nonnegative, it is an index into the inactive island 
        /// </summary>
        public int Index;
    }

    /// <summary>
    /// Stores a group of bodies- either the set of active bodies, or the bodies involve in an inactive simulation island.
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
    }

    /// <summary>
    /// Collection of all allocated bodies.
    /// </summary>
    public class Bodies
    {

        /// <summary>
        /// Remaps a body handle to the actual array index of the body.
        /// The backing array index may change in response to cache optimization.
        /// </summary>
        public Buffer<BodyLocation> HandleToIndex;
        public IdPool<Buffer<int>> HandlePool;
        /// <summary>
        /// The set of existing bodies. The slot at index 0 contains all active bodies. Later slots, if allocated, contain the bodies associated with inactive islands.
        /// Note that this buffer does not necessarily contain contiguous elements. When a set is removed, a gap remains.
        /// </summary>
        public Buffer<BodySet> Sets;
        /// <summary>
        /// Gets a reference to the active set, stored in the index 0 of the Sets buffer.
        /// </summary>
        /// <returns>Reference to the active body set.</returns>
        public unsafe ref BodySet ActiveSet { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return ref Unsafe.As<byte, BodySet>(ref *Sets.Memory); } }

        /// <summary>
        /// The world transformed inertias of active bodies as of the last update. Note that this is not automatically updated for direct orientation changes or for body memory moves.
        /// It is only updated once during the frame. It should be treated as ephemeral information.
        /// </summary>
        public Buffer<BodyInertia> Inertias;
        protected internal BufferPool pool;

        public unsafe Bodies(BufferPool pool, int initialCapacity, int initialSetCapacity)
        {
            this.pool = pool;
            InternalResize(Math.Max(1, initialCapacity));

            IdPool<Buffer<int>>.Create(pool.SpecializeFor<int>(), initialCapacity, out HandlePool);
            pool.SpecializeFor<BodySet>().Take(initialSetCapacity, out var Sets);
        }

        unsafe void InternalResize(int targetBodyCapacity)
        {
            Debug.Assert(targetBodyCapacity > 0, "Resize is not meant to be used as Dispose. If you want to return everything to the pool, use Dispose instead.");
            //Note that we base the bundle capacities on post-resize capacity of the HandleToIndex array. This simplifies the conditions on allocation, but increases memory use.
            //You may want to change this in the future if memory use is concerning.
            targetBodyCapacity = BufferPool<int>.GetLowestContainingElementCount(targetBodyCapacity);
            Debug.Assert(targetBodyCapacity >= HandlePool.HighestPossiblyClaimedId + 1, "A resize should never attempt to shrink allocations below the size necessary to hold extant handles.");
            Debug.Assert(Poses.Length != BufferPool<RigidPoses>.GetLowestContainingElementCount(targetBodyCapacity), "Should not try to use internal resize of the result won't change the size.");
            pool.SpecializeFor<RigidPose>().Resize(ref Poses, targetBodyCapacity, Count);
            pool.SpecializeFor<BodyVelocity>().Resize(ref Velocities, targetBodyCapacity, Count);
            pool.SpecializeFor<BodyInertia>().Resize(ref LocalInertias, targetBodyCapacity, Count);
            pool.SpecializeFor<BodyInertia>().Resize(ref Inertias, targetBodyCapacity, Count);
            pool.SpecializeFor<int>().Resize(ref IndexToHandle, targetBodyCapacity, Count);
            pool.SpecializeFor<BodyLocation>().Resize(ref HandleToIndex, targetBodyCapacity, Count);
            pool.SpecializeFor<Collidable>().Resize(ref Collidables, targetBodyCapacity, Count);
            //Initialize all the indices beyond the copied region to -1.
            Unsafe.InitBlockUnaligned(((int*)HandleToIndex.Memory) + Count, 0xFF, (uint)(sizeof(BodyLocation) * (HandleToIndex.Length - Count)));
            Unsafe.InitBlockUnaligned(((int*)IndexToHandle.Memory) + Count, 0xFF, (uint)(sizeof(int) * (IndexToHandle.Length - Count)));
            //Collidables beyond the body count should all point to nothing, which corresponds to zero.
            Collidables.Clear(Count, Collidables.Length - Count);
            //Note that we do NOT modify the idpool's internal queue size here. We lazily handle that during adds, and during explicit calls to EnsureCapacity, Compact, and Resize.
            //The idpool's internal queue will often be nowhere near as large as the actual body size except in corner cases, so in the usual case, being lazy saves a little space.
            //If the user wants to guarantee zero resizes, EnsureCapacity provides them the option to do so.
        }

        public unsafe int Add(ref BodyDescription bodyDescription)
        {
            if (Count == HandleToIndex.Length)
            {
                Debug.Assert(HandleToIndex.Allocated, "The backing memory of the bodies set should be initialized before use.");
                //Out of room; need to resize.
                var newSize = HandleToIndex.Length << 1;
                InternalResize(newSize);
            }
            Debug.Assert(Math.Abs(bodyDescription.Pose.Orientation.Length() - 1) < 1e-6f, "Orientation should be initialized to a unit length quaternion.");
            var handle = HandlePool.Take();
            var index = Count++;
            HandleToIndex[handle] = new BodyLocation { Island = -1, Index = index };
            IndexToHandle[index] = handle;
            ref var collidable = ref Collidables[index];
            collidable.Shape = bodyDescription.Collidable.Shape;
            collidable.Continuity = bodyDescription.Collidable.Continuity;
            collidable.SpeculativeMargin = bodyDescription.Collidable.SpeculativeMargin;
            //Collidable's broad phase index is left unset. The simulation is responsible for attaching that data.

            Poses[index] = bodyDescription.Pose;
            Velocities[index] = bodyDescription.Velocity;
            LocalInertias[index] = bodyDescription.LocalInertia;
            //TODO: Should the world inertias be updated on add? That would suggest a convention of also updating world inertias on any orientation change, which might not be wise given the API.
            return handle;
        }

        /// <summary>
        /// Removes a body from the set by index and returns whether a move occurred. If another body took its place, the move is output.
        /// </summary>
        /// <param name="bodyIndex">Index of the body to remove.</param>
        /// <param name="movedBodyOriginalIndex">Original index of the body that was moved into the removed body's slot. -1 if no body had to be moved.</param>
        /// <returns>True if a body was moved, false otherwise.</returns>
        public bool RemoveAt(int bodyIndex, out int movedBodyOriginalIndex)
        {
            Debug.Assert(bodyIndex >= 0 && bodyIndex < Count);
            var handle = IndexToHandle[bodyIndex];
            //Move the last body into the removed slot.
            //This does introduce disorder- there may be value in a second overload that preserves order, but it would require large copies.
            //In the event that so many adds and removals are performed at once that they destroy contiguity, it may be better to just
            //explicitly sort after the fact rather than attempt to retain contiguity incrementally. Handle it as a batch, in other words.
            --Count;
            bool bodyMoved = bodyIndex < Count;
            if (bodyMoved)
            {
                movedBodyOriginalIndex = Count;
                //Copy the memory state of the last element down.
                Poses[bodyIndex] = Poses[movedBodyOriginalIndex];
                Velocities[bodyIndex] = Velocities[movedBodyOriginalIndex];
                LocalInertias[bodyIndex] = LocalInertias[movedBodyOriginalIndex];
                //Note that if you ever treat the world inertias as 'always updated', it would need to be copied here.
                Collidables[bodyIndex] = Collidables[movedBodyOriginalIndex];
                //Point the body handles at the new location.
                var lastHandle = IndexToHandle[movedBodyOriginalIndex];
                Debug.Assert(HandleToIndex[lastHandle].Island == -1 && HandleToIndex[lastHandle].Index == movedBodyOriginalIndex);
                HandleToIndex[lastHandle].Index = bodyIndex;
                IndexToHandle[bodyIndex] = lastHandle;

            }
            else
            {
                movedBodyOriginalIndex = -1;
            }
            //We rely on the collidable references being nonexistent beyond the body count.
            Collidables[Count] = new Collidable();
            //The indices should also be set to all -1's beyond the body count.
            IndexToHandle[Count] = -1;
            HandlePool.Return(handle, pool.SpecializeFor<int>());
            HandleToIndex[handle].Index = -1;
            return bodyMoved;
        }

        [Conditional("DEBUG")]
        internal void ValidateHandle(int handle)
        {
            Debug.Assert(handle >= 0, "Handles must be nonnegative.");
            Debug.Assert(handle <= HandlePool.HighestPossiblyClaimedId && HandlePool.HighestPossiblyClaimedId < HandleToIndex.Length,
                "Existing handles must fit within the body handle->index mapping.");
            ref var location = ref HandleToIndex[handle];
            Debug.Assert(location.Island >= -1, "Island indices must be -1 or higher.");
            if (location.Island == -1)
            {
                //This is an active body.
                Debug.Assert(IndexToHandle[location.Index] == handle, "The index->handle and handle->index mappings must agree.");
            }
            else
            {
                //This is an inactive body.
                Debug.Assert(location.Island >= 0 && location.Island < Deactivator.Islands.Length && Deactivator.Islands[location.Island].Allocated, "Island index must be valid.");
                Deactivator.Islands[location.Island]
            }
            Debug.Assert(handle >= HandleToIndex.Length || HandleToIndex[handle].Index < 0 || HandleToIndex[handle].Island >= 0 || IndexToHandle[HandleToIndex[handle].Index] == handle,
                "If a handle exists, both directions should match.");
        }
        [Conditional("DEBUG")]
        public void ValidateExistingHandle(int handle)
        {
            Debug.Assert(handle >= 0, "Handles must be nonnegative.");
            Debug.Assert(handle < HandleToIndex.Length && HandleToIndex[handle] >= 0 && IndexToHandle[HandleToIndex[handle]] == handle,
                "This body handle doesn't seem to exist, or the mappings are out of sync. If a handle exists, both directions should match.");
        }

        /// <summary>
        /// Removes a body from the set and returns whether a move occurred. If another body took its place, the move is output.
        /// </summary>
        /// <param name="handle">Handle of the body to remove.</param>
        /// <param name="removedIndex">Former index of the body that was removed.</param>
        /// <param name="movedBodyOriginalIndex">Original index of the body that was moved into the removed body's slot. -1 if no body had to be moved.</param>
        /// <returns>True if a body was moved, false otherwise.</returns>
        public bool Remove(int handle, out int removedIndex, out int movedBodyOriginalIndex)
        {
            ValidateExistingHandle(handle);
            removedIndex = HandleToIndex[handle];
            return RemoveAt(removedIndex, out movedBodyOriginalIndex);
        }

        public void GetDescription(int handle, out BodyDescription description)
        {
            ValidateExistingHandle(handle);
            var index = HandleToIndex[handle];
            BundleIndexing.GetBundleIndices(index, out var bundleIndex, out var innerIndex);
            description.Pose = Poses[index];
            description.Velocity = Velocities[index];
            description.LocalInertia = LocalInertias[index];
            ref var collidable = ref Collidables[index];
            description.Collidable.Continuity = collidable.Continuity;
            description.Collidable.Shape = collidable.Shape;
            description.Collidable.SpeculativeMargin = collidable.SpeculativeMargin;
        }
        internal void SetDescriptionByIndex(int index, ref BodyDescription description)
        {
            BundleIndexing.GetBundleIndices(index, out var bundleIndex, out var innerIndex);
            Poses[index] = description.Pose;
            Velocities[index] = description.Velocity;
            LocalInertias[index] = description.LocalInertia;
            ref var collidable = ref Collidables[index];
            collidable.Continuity = description.Collidable.Continuity;
            collidable.SpeculativeMargin = description.Collidable.SpeculativeMargin;
            //Note that we change the shape here. If the collidable transitions from shapeless->shapeful or shapeful->shapeless, the broad phase has to be notified 
            //so that it can create/remove an entry. That's why this function isn't public.
            collidable.Shape = description.Collidable.Shape;
        }



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void GatherInertiaForBody(ref BodyInertia source, ref float targetInertiaBase, int targetLaneIndex)
        {
            ref var targetSlot = ref Unsafe.Add(ref targetInertiaBase, targetLaneIndex);
            targetSlot = source.InverseInertiaTensor.M11;
            Unsafe.Add(ref targetSlot, Vector<float>.Count) = source.InverseInertiaTensor.M21;
            Unsafe.Add(ref targetSlot, 2 * Vector<float>.Count) = source.InverseInertiaTensor.M22;
            Unsafe.Add(ref targetSlot, 3 * Vector<float>.Count) = source.InverseInertiaTensor.M31;
            Unsafe.Add(ref targetSlot, 4 * Vector<float>.Count) = source.InverseInertiaTensor.M32;
            Unsafe.Add(ref targetSlot, 5 * Vector<float>.Count) = source.InverseInertiaTensor.M33;
            Unsafe.Add(ref targetSlot, 6 * Vector<float>.Count) = source.InverseMass;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void GatherPoseForBody(ref RigidPose source, ref float targetPositionBase, ref float targetOrientationBase, int targetLaneIndex)
        {
            ref var targetPositionSlot = ref Unsafe.Add(ref targetPositionBase, targetLaneIndex);
            ref var targetOrientationSlot = ref Unsafe.Add(ref targetOrientationBase, targetLaneIndex);
            targetPositionSlot = source.Position.X;
            Unsafe.Add(ref targetPositionSlot, Vector<float>.Count) = source.Position.Y;
            Unsafe.Add(ref targetPositionSlot, 2 * Vector<float>.Count) = source.Position.Z;
            targetOrientationSlot = source.Orientation.X;
            Unsafe.Add(ref targetOrientationSlot, Vector<float>.Count) = source.Orientation.Y;
            Unsafe.Add(ref targetOrientationSlot, 2 * Vector<float>.Count) = source.Orientation.Z;
            Unsafe.Add(ref targetOrientationSlot, 3 * Vector<float>.Count) = source.Orientation.W;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void GatherVelocityForBody(ref BodyVelocity source, ref float targetLinearVelocityBase, ref float targetAngularVelocityBase, int targetLaneIndex)
        {
            ref var targetLinearSlot = ref Unsafe.Add(ref targetLinearVelocityBase, targetLaneIndex);
            ref var targetAngularSlot = ref Unsafe.Add(ref targetAngularVelocityBase, targetLaneIndex);
            targetLinearSlot = source.Linear.X;
            Unsafe.Add(ref targetLinearSlot, Vector<float>.Count) = source.Linear.Y;
            Unsafe.Add(ref targetLinearSlot, 2 * Vector<float>.Count) = source.Linear.Z;
            targetAngularSlot = source.Angular.X;
            Unsafe.Add(ref targetAngularSlot, Vector<float>.Count) = source.Angular.Y;
            Unsafe.Add(ref targetAngularSlot, 2 * Vector<float>.Count) = source.Angular.Z;
        }

        //TODO: In future versions, we will likely store the body position in different forms to allow for extremely large worlds.
        //That will be an opt-in feature. The default implementation will use the FP32 representation, but the user could choose to swap it out for a int64 based representation.
        //This affects other systems- AABB calculation, pose integration, solving, and in extreme (64 bit) cases, the broadphase.
        //We want to insulate other systems from direct knowledge about the implementation of positions when possible.
        //These functions support the solver's needs while hiding absolute positions.
        //In order to support other absolute positions, we'll need alternate implementations of this and other functions.
        //But for the most part, we don't want to pay the overhead of an abstract invocation within the inner loop of the solver. 
        //Given the current limits of C# and the compiler, the best option seems to be a interface implementing struct that provides this functionality.
        //The users would be type specialized by the compiler, avoiding virtual invocation. 
        [MethodImpl(MethodImplOptions.AggressiveInlining)] //Note that this doesn't do anything at the moment- the stackalloc hack blocks inlining.
        internal void GatherInertiaAndPose(ref TwoBodyReferences references, int count,
            out Vector3Wide localPositionB, out QuaternionWide orientationA, out QuaternionWide orientationB,
            out BodyInertias inertiaA, out BodyInertias inertiaB)
        {
            unsafe
            {
                //Note that we're storing data into vectors by reinterpreting the underlying memory.
                //Unfortunately, there's a compiler bug: https://github.com/dotnet/coreclr/issues/11804
                //Without intervention, a whole lotta stuff goes sideways when the data stored to registers is loaded from memory, 
                //resulting in uninitialized data flying around. Best case, constraints using this diverged massively as the localPositionB was whoknowswhat,
                //worst case the entire process gets torn down by access violations.
                //The inclusion of this hack convinces the JIT to store the gathered values into stack memory rather than aggressively enregistering it.
                //(Why? I didn't bother checking too closely. Maybe the JIT's more aggressive about looking for type punning in the presence of certain kinds of unsafe code.)
                //TODO: remove this once the fix is in. It blocks inlining.
                var hiPleaseDontRemoveThisWithoutTestingOrEverythingMightExplode = stackalloc byte[0];
            }
            ref var targetInertiaBaseA = ref Unsafe.As<Vector<float>, float>(ref inertiaA.InverseInertiaTensor.M11);
            ref var targetInertiaBaseB = ref Unsafe.As<Vector<float>, float>(ref inertiaB.InverseInertiaTensor.M11);
            Vector3Wide positionA, positionB;
            ref var targetPositionBaseA = ref Unsafe.As<Vector<float>, float>(ref positionA.X);
            ref var targetPositionBaseB = ref Unsafe.As<Vector<float>, float>(ref positionB.X);
            ref var targetOrientationBaseA = ref Unsafe.As<Vector<float>, float>(ref orientationA.X);
            ref var targetOrientationBaseB = ref Unsafe.As<Vector<float>, float>(ref orientationB.X);

            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);

            ref var poses = ref ActiveSet.Poses;
            for (int i = 0; i < count; ++i)
            {
                ref var indexA = ref Unsafe.Add(ref baseIndexA, i);
                GatherInertiaForBody(ref Inertias[indexA], ref targetInertiaBaseA, i);
                GatherPoseForBody(ref poses[indexA], ref targetPositionBaseA, ref targetOrientationBaseA, i);
                var indexB = Unsafe.Add(ref indexA, Vector<float>.Count);
                GatherInertiaForBody(ref Inertias[indexB], ref targetInertiaBaseB, i);
                GatherPoseForBody(ref poses[indexB], ref targetPositionBaseB, ref targetOrientationBaseB, i);
            }
            Vector3Wide.Subtract(ref positionB, ref positionA, out localPositionB);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void GatherInertia(ref TwoBodyReferences references, int count,
            out BodyInertias inertiaA, out BodyInertias inertiaB)
        {
            ref var targetInertiaBaseA = ref Unsafe.As<Vector<float>, float>(ref inertiaA.InverseInertiaTensor.M11);
            ref var targetInertiaBaseB = ref Unsafe.As<Vector<float>, float>(ref inertiaB.InverseInertiaTensor.M11);

            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);

            for (int i = 0; i < count; ++i)
            {
                ref var indexA = ref Unsafe.Add(ref baseIndexA, i);
                GatherInertiaForBody(ref Inertias[indexA], ref targetInertiaBaseA, i);
                var indexB = Unsafe.Add(ref indexA, Vector<float>.Count);
                GatherInertiaForBody(ref Inertias[indexB], ref targetInertiaBaseB, i);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void GatherInertia(ref Vector<int> references, int count,
            out BodyInertias inertiaA)
        {
            ref var targetInertiaBaseA = ref Unsafe.As<Vector<float>, float>(ref inertiaA.InverseInertiaTensor.M11);

            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references);

            for (int i = 0; i < count; ++i)
            {
                GatherInertiaForBody(ref Inertias[Unsafe.Add(ref baseIndexA, i)], ref targetInertiaBaseA, i);
            }
        }

        //This looks a little different because it's used by AABB calculation, not constraint pairs.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void GatherDataForBounds(ref int start, int count, out RigidPoses poses, out BodyVelocities velocities, out Vector<int> shapeIndices, out Vector<float> maximumExpansion)
        {
            Debug.Assert(count <= Vector<float>.Count);
            ref var targetPositionBase = ref Unsafe.As<Vector<float>, float>(ref poses.Position.X);
            ref var targetOrientationBase = ref Unsafe.As<Vector<float>, float>(ref poses.Orientation.X);
            ref var targetLinearBase = ref Unsafe.As<Vector<float>, float>(ref velocities.LinearVelocity.X);
            ref var targetAngularBase = ref Unsafe.As<Vector<float>, float>(ref velocities.AngularVelocity.X);
            ref var targetShapeBase = ref Unsafe.As<Vector<int>, int>(ref shapeIndices);
            ref var targetExpansionBase = ref Unsafe.As<Vector<float>, float>(ref maximumExpansion);
            ref var activeSet = ref ActiveSet;
            for (int i = 0; i < count; ++i)
            {
                var index = Unsafe.Add(ref start, i);
                GatherPoseForBody(ref activeSet.Poses[index], ref targetPositionBase, ref targetOrientationBase, i);
                GatherVelocityForBody(ref activeSet.Velocities[index], ref targetLinearBase, ref targetAngularBase, i);
                ref var collidable = ref activeSet.Collidables[index];
                Unsafe.Add(ref targetShapeBase, i) = collidable.Shape.Index;
                //Not entirely pleased with the fact that this pulls in some logic from bounds calculation.
                Unsafe.Add(ref targetExpansionBase, i) = collidable.Continuity.AllowExpansionBeyondSpeculativeMargin ? float.MaxValue : collidable.SpeculativeMargin;
            }
        }


        /// <summary>
        /// Clears all bodies from all sets without releasing any memory that wouldn't be released by a sequence of regular removals.
        /// </summary>
        public unsafe void Clear()
        {
            ActiveSet.Clear();
            //While the top level pool represents active bodies and will persist (as it would after a series of removals),
            //subsequent sets represent inactive bodies. When they are not present, the backing memory is released.
            for (int i = 1; i < Sets.Length; ++i)
            {
                ref var set = ref Sets[i];
                if (set.Allocated)
                    set.Dispose(pool);
            }
            Unsafe.InitBlockUnaligned(HandleToIndex.Memory, 0xFF, (uint)(sizeof(BodyLocation) * HandleToIndex.Length));
            HandlePool.Clear();
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void ResizeHandles(int newCapacity)
        {
            newCapacity = BufferPool<BodyLocation>.GetLowestContainingElementCount(newCapacity);
            if (newCapacity != HandleToIndex.Length)
            {
                var highestPotentialHandle = HandlePool.HighestPossiblyClaimedId + 1;
                pool.SpecializeFor<BodyLocation>().Resize(ref HandleToIndex, newCapacity, highestPotentialHandle);
                if (HandleToIndex.Length > highestPotentialHandle)
                {
                    Unsafe.InitBlockUnaligned(
                      ((BodyLocation*)HandleToIndex.Memory) + highestPotentialHandle, 0xFF,
                      (uint)(sizeof(BodyLocation) * (HandleToIndex.Length - highestPotentialHandle)));
                }
            }
        }
        /// <summary>
        /// Resizes the allocated spans for active body data. Note that this is conservative; it will never orphan existing objects.
        /// </summary>
        /// <param name="capacity">Target body data capacity.</param>
        public void Resize(int capacity)
        {
            var targetBodyCapacity = BufferPool<int>.GetLowestContainingElementCount(Math.Max(capacity, ActiveSet.Count));
            if (ActiveSet.IndexToHandle.Length != targetBodyCapacity)
            {
                ActiveSet.InternalResize(targetBodyCapacity, pool);
            }
            var targetHandleCapacity = BufferPool<int>.GetLowestContainingElementCount(Math.Max(capacity, HandlePool.HighestPossiblyClaimedId + 1));
            if (HandleToIndex.Length != targetHandleCapacity)
            {
                ResizeHandles(targetHandleCapacity);
            }
        }

        /// <summary>
        /// Increases the size of active body buffers if needed to hold the target capacity.
        /// </summary>
        /// <param name="capacity">Target data capacity.</param>
        public void EnsureCapacity(int capacity)
        {
            if (ActiveSet.IndexToHandle.Length < capacity)
            {
                ActiveSet.InternalResize(capacity, pool);
            }
            if (HandleToIndex.Length < capacity)
            {
                ResizeHandles(capacity);
            }
        }


        /// <summary>
        /// Returns all body resources to the pool used to create them.
        /// </summary>
        /// <remarks>The object can be reused if it is reinitialized by using EnsureCapacity or Resize.</remarks>
        public void Dispose()
        {
            for (int i = 0; i < Sets.Length; ++i)
            {
                ref var set = ref Sets[i];
                if (set.Allocated)
                {
                    set.Dispose(pool);
                }
            }
            pool.SpecializeFor<BodySet>().Return(ref Sets);
            pool.SpecializeFor<BodyInertia>().Return(ref Inertias);
            pool.SpecializeFor<BodyLocation>().Return(ref HandleToIndex);
            HandlePool.Dispose(pool.SpecializeFor<int>());
        }

    }
}
