using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics
{
    //You could bitpack these two into 4 bytes, but the value of that is pretty darn questionable.
    public struct BodyConstraintReference
    {
        public int ConnectingConstraintHandle;
        public int BodyIndexInConstraint;
    }

    public class ConstraintGraph
    {
        int constraintCountPerBodyEstimate;


        //We need a per-body list of constraints because any movement of a body requires updating the connected constraints.
        //Since we have that, we'll also use it to find adjacent bodies. Finding adjacent bodies is required for two purposes:
        //1) Deactivating an island of low energy bodies.
        //2) Optimizing the memory layout of bodies so that connected bodies tend to be adjacent or at least nearby.

        //Conceptually, we could do something simple like a List<List<int>> for a per-body list of constraint handles.
        //An obvious optimization to that is to pool the internal lists to avoid GC overhead, but there is another factor to consider beyond just garbage generation-
        //heap complexity.

        //The goal here is to reduce the cost of a GC. While the engine should never produce garbage itself during regular execution, the application using it might.
        //The cost of a GC is partially related to the number of references it has to track down. An array of reference types requires examining every one of those references.
        //So, if you have 16384 bodies with lists stored in List<int>[] representation, the GC has at least 32768 references it needs to consider (the list, and its internal array)!
        //In contrast, if you pack the representation into a single int[] with the lists suballocated from it, the GC doesn't need to scan it- ints aren't reference types.

        //QuickLists based on a pointer-backed Buffer<int> contain no references at all, so QuickList<int, Buffer<int>>[] only costs a single reference- for the top level array.
        //The rest of it is just a bunch of value types.
        Buffer<QuickList<BodyConstraintReference, Buffer<BodyConstraintReference>>> constraintLists;
        BufferPool<BodyConstraintReference> bufferPool;
        Solver solver;



        /// <summary>
        /// Constructs a constraint connectivity graph.
        /// </summary>
        /// <param name="solver">Solver associated with the constraints in this graph.</param>
        /// <param name="rawPool">Pool from which per-body lists are allocated.</param>
        /// <param name="initialBodyCountEstimate">Initial estimate for the number of bodies which will exist in the graph.
        /// If the number is exceeded, an internal buffer will resize and the old buffer will return to the pool.</param>
        /// <param name="initialConstraintCountPerBodyEstimate">Initial estimate for the number of constraints that will exist for each body.
        /// If the number is exceeded, the body list will be resized, but the old list will be returned to the pool for reuse.</param>
        public ConstraintGraph(Solver solver, BufferPool rawPool, int initialBodyCountEstimate, int initialConstraintCountPerBodyEstimate)
        {
            this.solver = solver;
            constraintCountPerBodyEstimate = initialConstraintCountPerBodyEstimate;
            var capacityInBytes = initialBodyCountEstimate * initialConstraintCountPerBodyEstimate * sizeof(int);
            bufferPool = rawPool.SpecializeFor<BodyConstraintReference>();

            rawPool.SpecializeFor<QuickList<BodyConstraintReference, Buffer<BodyConstraintReference>>>().Take(initialBodyCountEstimate, out constraintLists);
        }

        //Note that constraints only contain direct references to the memory locations of bodies, not to their handles.
        //While we could get the handles from the memory locations, it is cheaper/simpler just to deal with the memory locations directly.
        //To this end, the constraint lists are ordered according to the body memory order, not the handle order.
        //Every time a body moves due to cache optimization or removal, these lists must be updated.

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SwapBodies(int oldLocation, int newLocation)
        {
            var temp = constraintLists[oldLocation];
            constraintLists[oldLocation] = constraintLists[newLocation];
            constraintLists[newLocation] = temp;
        }

        /// <summary>
        /// Adds a list for a body to the given location.
        /// </summary>
        /// <param name="bodyIndex">Location to allocate a list.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddBodyList(int bodyIndex)
        {
            //Note that we trust the user to provide valid locations. The graph shouldn't do any of its own positioning- it is slaved to the body memory layout.
            //This isn't a system that external users will be using under any normal circumstance, so trust should be okay.
            Debug.Assert(constraintLists.Length >= bodyIndex, "We should only ever add body lists one by one, and they should always be appended. Indices beyond the count should not appear.");
            if (bodyIndex == constraintLists.Length)
            {
                //Not enough room for this body! Resize required.
                bufferPool.Raw.SpecializeFor<QuickList<BodyConstraintReference, Buffer<BodyConstraintReference>>>().Resize(ref constraintLists, 1 << SpanHelper.GetContainingPowerOf2(bodyIndex + 1), constraintLists.Length);
            }
            QuickList<BodyConstraintReference, Buffer<BodyConstraintReference>>.Create(bufferPool, constraintCountPerBodyEstimate, out constraintLists[bodyIndex]);
        }

        /// <summary>
        /// Frees the list associated with a body at the given location and returns true if the freed list was empty.
        /// </summary>
        /// <param name="bodyIndex">Location to remove.</param>
        /// <param name="replacementIndex">If nonnegative, the index to pull a replacement list from.</param>
        /// <returns>True if the freed list was empty, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool RemoveBodyList(int bodyIndex, int replacementIndex)
        {
            ref var list = ref constraintLists[bodyIndex];
            var empty = list.Count == 0;
            list.Dispose(bufferPool);

            if (replacementIndex >= 0)
            {
                constraintLists[bodyIndex] = constraintLists[replacementIndex];
            }
            return empty;
        }

        /// <summary>
        /// Adds a constraint to the specified body.
        /// </summary>
        /// <param name="bodyIndex">Index of the body to add the constraint handle to.</param>
        /// <param name="constraintHandle">Constraint handle to add to the body's list.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddConstraint(int bodyIndex, int constraintHandle, int bodyIndexInConstraint)
        {
            BodyConstraintReference constraint;
            constraint.ConnectingConstraintHandle = constraintHandle;
            constraint.BodyIndexInConstraint = bodyIndexInConstraint;
            constraintLists[bodyIndex].Add(ref constraint, bufferPool);
        }

        struct RemovalPredicate : IPredicate<BodyConstraintReference>
        {
            public int ConstraintHandleToRemove;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool Matches(ref BodyConstraintReference item)
            {
                return item.ConnectingConstraintHandle == ConstraintHandleToRemove;
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RemoveConstraint(int bodyIndex, int constraintHandle)
        {
            //This uses a linear search. That's fine; bodies will rarely have more than a handful of constraints associated with them.
            //Attempting to use something like a hash set for fast removes would just introduce more constant overhead and slow it down on average.
            ref var list = ref constraintLists[bodyIndex];
            RemovalPredicate predicate;
            predicate.ConstraintHandleToRemove = constraintHandle;
            list.FastRemove(ref predicate);
            if (list.Count <= list.Span.Length / 2 && list.Count >= constraintCountPerBodyEstimate)
            {
                //The list has shrunk quite a bit, and it's above the maximum size. Might as well try to trim a little.
                bufferPool.Resize(ref list.Span, list.Count, list.Count);
            }
        }

        //TODO: It's likely that we'll eventually have something very similar to all of this per body list stuff for collision detection pairs. We'll worry about
        //de-duping that code later.


        //sooooooo idiomatic

        struct ConstraintBodiesEnumerator<TInnerEnumerator> : IForEach<int> where TInnerEnumerator : IForEach<int>
        {
            public TInnerEnumerator InnerEnumerator;
            public int SourceBodyIndex;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int connectedBodyIndex)
            {
                if (SourceBodyIndex != connectedBodyIndex)
                {
                    //Note that this may report the same body multiple times if it is connected multiple times! That's fine and potentially useful; let the user deal with it.
                    InnerEnumerator.LoopBody(connectedBodyIndex);
                }
            }

        }

        /// <summary>
        /// Enumerates all the bodies connected to a given body.
        /// Bodies which are connected by more than one constraint will be reported multiple times.
        /// </summary>
        /// <typeparam name="TEnumerator">Type of the enumerator to execute on each connected body.</typeparam>
        /// <param name="bodyIndex">Index of the body to enumerate the connections of. This body will not appear in the set of enumerated bodies, even if it is connected to itself somehow.</param>
        /// <param name="enumerator">Enumerator instance to run on each connected body.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void EnumerateConnectedBodies<TEnumerator>(int bodyIndex, ref TEnumerator enumerator) where TEnumerator : IForEach<int>
        {
            ref var list = ref constraintLists[bodyIndex];
            ConstraintBodiesEnumerator<TEnumerator> constraintBodiesEnumerator;
            constraintBodiesEnumerator.InnerEnumerator = enumerator;
            constraintBodiesEnumerator.SourceBodyIndex = bodyIndex;

            //Note reverse iteration. This is useful when performing O(1) removals where the last element is put into the position of the removed element.
            //Non-reversed iteration would result in skipped elements if the loop body removed anything. This relies on convention; any remover should be aware of this order.
            for (int i = list.Count - 1; i >= 0; --i)
            {
                solver.EnumerateConnectedBodyIndices(list[i].ConnectingConstraintHandle, ref constraintBodiesEnumerator);
            }
            //Note that we have to assume the enumerator contains state mutated by the internal loop bodies.
            //If it's a value type, those mutations won't be reflected in the original reference. 
            //Copy them back in.
            enumerator = constraintBodiesEnumerator.InnerEnumerator;
        }

        //This could return a readonly span. It is public, after all. If it becomes an issue we can deal with it.
        /// <summary>
        /// Gets the list of constraints associated with the given body.
        /// </summary>
        /// <param name="bodyIndex">Body to look up the constraint list for.</param>
        /// <returns>List of constraints associated with the body.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref QuickList<BodyConstraintReference, Buffer<BodyConstraintReference>> GetConstraintList(int bodyIndex)
        {
            return ref constraintLists[bodyIndex];
        }

        /// <summary>
        /// Checks whether a body is referenced by the given constraint handle.
        /// </summary>
        /// <param name="bodyIndex">Body to check for a constraint.</param>
        /// <param name="constraintHandle">Constraint handle to look for in the body's constraint list.</param>
        /// <returns>True if the body is connected to the constraint, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool BodyIsConstrainedBy(int bodyIndex, int constraintHandle)
        {
            //This is a special case that bypasses the need for a predicate.
            //It's unclear how valuable this actually is- this shouldn't be a particularly common operation. We actually only added it for debugging purposes.
            ref var list = ref constraintLists[bodyIndex];
            for (int i = 0; i < list.Count; ++i)
            {
                if (list[i].ConnectingConstraintHandle == constraintHandle)
                    return true;
            }
            return false;
        }

        /// <summary>
        /// Returns all per-body lists to the buffer pool, but leaves the backing buffer in place. 
        /// </summary>
        /// <remarks>This does not track the state necessary to protect against redundant clears. It is assumed that the graph contains as many bodies as are in the given body set.</remarks>
        internal void Clear(Bodies bodies)
        {
            for (int i = 0; i < bodies.Count; ++i)
            {
                Debug.Assert(constraintLists[i].Span.Length != 0, "Clears should only be performed if the graph was not already cleared.");
                constraintLists[i].Dispose(bufferPool);
#if DEBUG
                constraintLists[i] = new QuickList<BodyConstraintReference, Buffer<BodyConstraintReference>>();
#endif
            }
        }
        /// <summary>
        /// Returns the graph's backing buffer to the pool. Does not make any changes to the constraint lists held within it.
        /// </summary>
        /// <remarks><para>This does not track the state necessary to protect against redundant clears. It is assumed that the graph contains as many bodies as are in the given body set.</para>
        /// <para>The graph can be reused after disposal, though it is a good idea to use EnsureCapacity or Resize to avoid a bunch of unnecessary resizes.</para></remarks>
        internal void Dispose()
        {
            Debug.Assert(!constraintLists[0].Span.Allocated, "The graph should be cleared before it is disposed.");
            Debug.Assert(constraintLists.Length > 0, "The graph should not be redundantly disposed.");
            bufferPool.Raw.SpecializeFor<QuickList<BodyConstraintReference, Buffer<BodyConstraintReference>>>().Return(ref constraintLists);
            constraintLists = new Buffer<QuickList<BodyConstraintReference, Buffer<BodyConstraintReference>>>();
        }



        internal void EnsureCapacity(Bodies bodies, int bodyCount, int constraintsPerBody)
        {
            if (constraintLists.Length < bodyCount)
                bufferPool.Raw.SpecializeFor<QuickList<BodyConstraintReference, Buffer<BodyConstraintReference>>>().Resize(ref constraintLists, bodyCount, bodies.Count);
            if (constraintsPerBody > constraintCountPerBodyEstimate)
            {
                constraintCountPerBodyEstimate = constraintsPerBody;
                for (int i = 0; i < bodies.Count; ++i)
                {
                    ref var list = ref constraintLists[i];
                    if (list.Span.Length < constraintsPerBody)
                        bufferPool.Resize(ref list.Span, constraintsPerBody, list.Count);
                }
            }
        }

        internal void Compact(Bodies bodies, int bodyCount, int constraintsPerBody)
        {
            var targetBodyCapacity = BufferPool<QuickList<BodyConstraintReference, Buffer<BodyConstraintReference>>>.GetLowestContainingElementCount(Math.Max(bodies.Count, bodyCount));
            if (constraintLists.Length > targetBodyCapacity)
                bufferPool.Raw.SpecializeFor<QuickList<BodyConstraintReference, Buffer<BodyConstraintReference>>>().Resize(ref constraintLists, targetBodyCapacity, bodies.Count);
            constraintsPerBody = Math.Max(1, constraintsPerBody);
            if (constraintsPerBody < constraintCountPerBodyEstimate)
            {
                //Note that we still compact per body lists, even though the constraint removal trim is aggressive. The user may have changed the 
                //minimum size per body list. Performance is assumed to not really matter here- resizing stuff is not a fast operation.
                this.constraintCountPerBodyEstimate = constraintsPerBody;
                for (int i = 0; i < bodies.Count; ++i)
                {
                    ref var list = ref constraintLists[i];
                    //Don't resize below the current constraint count.
                    var targetListCapacity = BufferPool<BodyConstraintReference>.GetLowestContainingElementCount(constraintsPerBody < list.Count ? list.Count : constraintsPerBody);
                    if (targetListCapacity < list.Span.Length)
                        bufferPool.Resize(ref list.Span, targetListCapacity, list.Count);
                }
            }
        }
        internal void Resize(Bodies bodies, int bodyCount, int constraintsPerBody)
        {
            var targetBodyCapacity = BufferPool<QuickList<BodyConstraintReference, Buffer<BodyConstraintReference>>>.GetLowestContainingElementCount(Math.Max(bodies.Count, bodyCount));
            if (constraintLists.Length != targetBodyCapacity)
                bufferPool.Raw.SpecializeFor<QuickList<BodyConstraintReference, Buffer<BodyConstraintReference>>>().Resize(ref constraintLists, targetBodyCapacity, bodies.Count);
            constraintsPerBody = Math.Max(1, constraintsPerBody);
            if (constraintsPerBody < constraintCountPerBodyEstimate)
            {
                //Note that we still compact per body lists, even though the constraint removal trim is aggressive. The user may have changed the 
                //minimum size per body list. Performance is assumed to not really matter here- resizing stuff is not a fast operation.
                this.constraintCountPerBodyEstimate = constraintsPerBody;
                for (int i = 0; i < bodies.Count; ++i)
                {
                    ref var list = ref constraintLists[i];
                    //Don't resize below the current constraint count.
                    var targetListCapacity = BufferPool<BodyConstraintReference>.GetLowestContainingElementCount(constraintsPerBody < list.Count ? list.Count : constraintsPerBody);
                    if (targetListCapacity != list.Span.Length)
                        bufferPool.Resize(ref list.Span, targetListCapacity, list.Count);
                }
            }
        }

    }
}
