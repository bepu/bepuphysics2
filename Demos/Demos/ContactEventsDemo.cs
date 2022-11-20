using System;
using System.Numerics;
using BepuUtilities;
using BepuPhysics;
using DemoContentLoader;
using DemoRenderer;
using BepuPhysics.Collidables;
using DemoUtilities;
using DemoRenderer.UI;
using BepuPhysics.CollisionDetection;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System.Runtime.CompilerServices;
using BepuPhysics.Constraints;
using System.Diagnostics;
using System.Threading;
using System.Runtime.InteropServices;

namespace Demos.Demos
{
    //Bepuphysics v2 doesn't have any concept of 'events'. It has callbacks that report the current status of contact manifolds.
    //Events can be built around those callbacks. This demo shows one way of doing that.

    //It's worth noting a few things about this event handler approach:
    //1) It's a little nonidiomatic, since there are no actual C# 'events' being used. An interface is invoked instead. This preserves the generic manifold type.
    //You could shuffle things around a bit and invoke delegates for a more event-ish workflow, or even function pointers if you wanted to jump through some hoops.
    //2) All event handlers (except OnPairEnded) potentially execute from the multithreaded context of the simulation execution, so you have to be careful about what the event handlers do.
    //A "deferred" event model could be built on top of this.
    //3) Event handling isn't free; try to avoid attaching events to everything if you can avoid it, and try to make the event handlers as cheap as possible.
    //4) This provides no insight into the constraints associated with these contacts. For example, if you want to spawn particles in response to heavy collisions or play sounds
    //with volume dependent on the impact force, you will need to go pull impulse data from the solver. Further, those sorts of use cases often only apply to nearby objects,
    //so a listener based model isn't ideal. Instead, querying nearby active objects and examining their contact constraints would be much more direct.
    //5) Contacts can be created speculatively. A contact existing does not guarantee a nonnegative penetration depth!
    //Note that only manifolds with contacts with nonnegative depths are reported as 'touching' below.
    //6) On the flipside of #6, an impact is not guaranteed to always have a nonnegative penetration depth at some point!
    //Solving the constraints associated with speculative contacts could end up bouncing an object off a surface between two collision detection runs that both show negative depth.
    //7) There are other ways of pulling contact data. For example, check out the SolverContactEnumerationDemo which pulls data from the solver constraints created from contacts.
    //Pulling accumulated impulses from the solver is one way to never miss an actual collision; if the contact constraints applied any force, then there must have been an impact.
    //8) Pairs involving two sleeping bodies will not generate events. That means OnTouching and OnPairUpdated will not fire for sleeping pairs.

    /// <summary>
    /// Implements handlers for various collision events.
    /// </summary>
    public interface IContactEventHandler
    {
        /// <summary>
        /// Fires when a contact is added.
        /// </summary>
        /// <typeparam name="TManifold">Type of the contact manifold detected.</typeparam>
        /// <param name="eventSource">Collidable that the event was attached to.</param>
        /// <param name="pair">Collidable pair triggering the event.</param>
        /// <param name="contactManifold">Set of remaining contacts in the collision.</param>
        /// <param name="contactOffset">Offset from the pair's local origin to the new contact.</param>
        /// <param name="contactNormal">Normal of the new contact.</param>
        /// <param name="depth">Depth of the new contact.</param>
        /// <param name="featureId">Feature id of the new contact.</param>
        /// <param name="contactIndex">Index of the new contact in the contact manifold.</param>
        /// <param name="workerIndex">Index of the worker thread that fired this event.</param>
        void OnContactAdded<TManifold>(CollidableReference eventSource, CollidablePair pair, ref TManifold contactManifold,
            Vector3 contactOffset, Vector3 contactNormal, float depth, int featureId, int contactIndex, int workerIndex) where TManifold : unmanaged, IContactManifold<TManifold>
        {
        }

        /// <summary>
        /// Fires when a contact is removed.
        /// </summary>
        /// <typeparam name="TManifold">Type of the contact manifold detected.</typeparam>
        /// <param name="eventSource">Collidable that the event was attached to.</param>
        /// <param name="pair">Collidable pair triggering the event.</param>
        /// <param name="contactManifold">Set of remaining contacts in the collision.</param>
        /// <param name="removedFeatureId">Feature id of the contact that was removed and is no longer present in the contact manifold.</param>
        /// <param name="workerIndex">Index of the worker thread that fired this event.</param>
        void OnContactRemoved<TManifold>(CollidableReference eventSource, CollidablePair pair, ref TManifold contactManifold, int removedFeatureId, int workerIndex) where TManifold : unmanaged, IContactManifold<TManifold>
        {
        }

        /// <summary>
        /// Fires the first time a pair is observed to be touching. Touching means that there are contacts with nonnegative depths in the manifold.
        /// </summary>
        /// <typeparam name="TManifold">Type of the contact manifold detected.</typeparam>
        /// <param name="eventSource">Collidable that the event was attached to.</param>
        /// <param name="pair">Collidable pair triggering the event.</param>
        /// <param name="contactManifold">Set of remaining contacts in the collision.</param>
        /// <param name="workerIndex">Index of the worker thread that fired this event.</param>
        void OnStartedTouching<TManifold>(CollidableReference eventSource, CollidablePair pair, ref TManifold contactManifold, int workerIndex) where TManifold : unmanaged, IContactManifold<TManifold>
        {
        }

        /// <summary>
        /// Fires whenever a pair is observed to be touching. Touching means that there are contacts with nonnegative depths in the manifold. Will not fire for sleeping pairs.
        /// </summary>
        /// <typeparam name="TManifold">Type of the contact manifold detected.</typeparam>
        /// <param name="eventSource">Collidable that the event was attached to.</param>
        /// <param name="pair">Collidable pair triggering the event.</param>
        /// <param name="contactManifold">Set of remaining contacts in the collision.</param>
        /// <param name="workerIndex">Index of the worker thread that fired this event.</param>
        void OnTouching<TManifold>(CollidableReference eventSource, CollidablePair pair, ref TManifold contactManifold, int workerIndex) where TManifold : unmanaged, IContactManifold<TManifold>
        {
        }


        /// <summary>
        /// Fires when a pair stops touching. Touching means that there are contacts with nonnegative depths in the manifold.
        /// </summary>
        /// <typeparam name="TManifold">Type of the contact manifold detected.</typeparam>
        /// <param name="eventSource">Collidable that the event was attached to.</param>
        /// <param name="pair">Collidable pair triggering the event.</param>
        /// <param name="contactManifold">Set of remaining contacts in the collision.</param>
        /// <param name="workerIndex">Index of the worker thread that fired this event.</param>
        void OnStoppedTouching<TManifold>(CollidableReference eventSource, CollidablePair pair, ref TManifold contactManifold, int workerIndex) where TManifold : unmanaged, IContactManifold<TManifold>
        {
        }


        /// <summary>
        /// Fires when a pair is observed for the first time.
        /// </summary>
        /// <typeparam name="TManifold">Type of the contact manifold detected.</typeparam>
        /// <param name="eventSource">Collidable that the event was attached to.</param>
        /// <param name="pair">Collidable pair triggering the event.</param>
        /// <param name="contactManifold">Set of remaining contacts in the collision.</param>
        /// <param name="workerIndex">Index of the worker thread that fired this event.</param>
        void OnPairCreated<TManifold>(CollidableReference eventSource, CollidablePair pair, ref TManifold contactManifold, int workerIndex) where TManifold : unmanaged, IContactManifold<TManifold>
        {
        }

        /// <summary>
        /// Fires whenever a pair is updated. Will not fire for sleeping pairs.
        /// </summary>
        /// <typeparam name="TManifold">Type of the contact manifold detected.</typeparam>
        /// <param name="eventSource">Collidable that the event was attached to.</param>
        /// <param name="pair">Collidable pair triggering the event.</param>
        /// <param name="contactManifold">Set of remaining contacts in the collision.</param>
        /// <param name="workerIndex">Index of the worker thread that fired this event.</param>
        void OnPairUpdated<TManifold>(CollidableReference eventSource, CollidablePair pair, ref TManifold contactManifold, int workerIndex) where TManifold : unmanaged, IContactManifold<TManifold>
        {
        }

        /// <summary>
        /// Fires when a pair ends.
        /// </summary>
        /// <typeparam name="TManifold">Type of the contact manifold detected.</typeparam>
        /// <param name="eventSource">Collidable that the event was attached to.</param>
        /// <param name="pair">Collidable pair triggering the event.</param>
        void OnPairEnded(CollidableReference eventSource, CollidablePair pair)
        {
        }
    }

    /// <summary>
    /// Watches a set of bodies and statics for contact changes and reports events.
    /// </summary>
    public unsafe class ContactEvents : IDisposable
    {
        //To know what events to emit, we have to track the previous state of a collision. We don't need to keep around old positions/offets/normals/depths, so it's quite a bit lighter.
        [StructLayout(LayoutKind.Sequential)]
        struct PreviousCollision
        {
            public CollidableReference Collidable;
            public bool Fresh;
            public bool WasTouching;
            public int ContactCount;
            //FeatureIds are identifiers encoding what features on the involved shapes contributed to the contact. We store up to 4 feature ids, one for each potential contact.
            //A "feature" is things like a face, vertex, or edge. There is no single interpretation for what a feature is- the mapping is defined on a per collision pair level.
            //In this demo, we only care to check whether a given contact in the current frame maps onto a contact from a previous frame.
            //We can use this to only emit 'contact added' events when a new contact with an unrecognized id is reported.
            public int FeatureId0;
            public int FeatureId1;
            public int FeatureId2;
            public int FeatureId3;
        }

        Simulation simulation;
        IThreadDispatcher threadDispatcher;
        WorkerBufferPools threadPools;
        BufferPool pool;

        //We'll use a handle->index mapping in a CollidableProperty to point at our contiguously stored listeners (in the later listeners array).
        //Note that there's also IndexSets for the statics and bodies; those will be checked first before accessing the listenerIndices.
        //The CollidableProperty is quite barebones- it doesn't try to stop all invalid accesses, and the backing memory isn't guaranteed to be zero initialized.
        //IndexSets are tightly bitpacked and are cheap to access, so they're an easy way to check if a collidable can trigger an event before doing any further processing.
        CollidableProperty<int> listenerIndices;
        IndexSet staticListenerFlags;
        IndexSet bodyListenerFlags;
        int listenerCount;

        //For the purpose of this demo, we'll use some regular ol' interfaces rather than using the struct-implementing-interface for specialization.
        //This array will be GC tracked as a result, but that should be mostly fine. If you've got hundreds of thousands of event handlers, you may want to consider alternatives.
        struct Listener
        {
            public CollidableReference Source;
            public IContactEventHandler Handler;
            public QuickList<PreviousCollision> PreviousCollisions;
        }
        Listener[] listeners;

        //The callbacks are invoked from a multithreaded context, and we don't know how many pairs will exist. 
        //Rather than attempting to synchronize all accesses, every worker thread spits out the results into a worker-local list to be processed later by the main thread flush.
        struct PendingWorkerAdd
        {
            public int ListenerIndex;
            public PreviousCollision Collision;
        }
        QuickList<PendingWorkerAdd>[] pendingWorkerAdds;

        /// <summary>
        /// Creates a new contact events stream.
        /// </summary>
        /// <param name="threadDispatcher">Thread dispatcher to pull per-thread buffer pools from, if any.</param>
        /// <param name="pool">Buffer pool used to manage resources internally. If null, the simulation's pool will be used.</param>
        /// <param name="initialListenerCapacity">Number of listeners to allocate space for initially.</param>
        public ContactEvents(IThreadDispatcher threadDispatcher = null, BufferPool pool = null, int initialListenerCapacity = 64)
        {
            this.threadDispatcher = threadDispatcher;
            this.pool = pool;
            listeners = new Listener[initialListenerCapacity];
        }

        IUnmanagedMemoryPool GetPoolForWorker(int workerIndex)
        {
            return threadDispatcher == null ? pool : threadPools[workerIndex];
        }

        /// <summary>
        /// Initializes the contact events system with a simulation.
        /// </summary>
        /// <param name="simulation">Simulation to use with the contact events demo.</param>
        /// <remarks>The constructor and initialization are split because of how this class is expected to be used. 
        /// It will be passed into a simulation's constructor as a part of its contact callbacks, so there is no simulation available at the time of construction.</remarks>
        public void Initialize(Simulation simulation)
        {
            this.simulation = simulation;
            if (pool == null)
                pool = simulation.BufferPool;
            threadPools = threadDispatcher != null ? new WorkerBufferPools(pool, threadDispatcher.ThreadCount) : null;
            simulation.Timestepper.BeforeCollisionDetection += SetFreshnessForCurrentActivityStatus;
            listenerIndices = new CollidableProperty<int>(simulation, pool);
            pendingWorkerAdds = new QuickList<PendingWorkerAdd>[threadDispatcher == null ? 1 : threadDispatcher.ThreadCount];
        }

        /// <summary>
        /// Begins listening for events related to the given collidable.
        /// </summary>
        /// <param name="collidable">Collidable to monitor for events.</param>
        /// <param name="handler">Handlers to use for the collidable.</param>
        public void Register(CollidableReference collidable, IContactEventHandler handler)
        {
            Debug.Assert(!IsListener(collidable), "Should only try to register listeners that weren't previously registered");
            if (collidable.Mobility == CollidableMobility.Static)
                staticListenerFlags.Add(collidable.RawHandleValue, pool);
            else
                bodyListenerFlags.Add(collidable.RawHandleValue, pool);
            if (listenerCount > listeners.Length)
            {
                Array.Resize(ref listeners, listeners.Length * 2);
            }
            //Note that allocations for the previous collision list are deferred until they actually exist.
            listeners[listenerCount] = new Listener { Handler = handler, Source = collidable };
            listenerIndices[collidable] = listenerCount;
            ++listenerCount;
        }

        /// <summary>
        /// Begins listening for events related to the given body.
        /// </summary>
        /// <param name="body">Body to monitor for events.</param>
        /// <param name="handler">Handlers to use for the body.</param>
        public void Register(BodyHandle body, IContactEventHandler handler)
        {
            Register(simulation.Bodies[body].CollidableReference, handler);
        }

        /// <summary>
        /// Begins listening for events related to the given static.
        /// </summary>
        /// <param name="staticHandle">Static to monitor for events.</param>
        /// <param name="handler">Handlers to use for the static.</param>
        public void Register(StaticHandle staticHandle, IContactEventHandler handler)
        {
            Register(new CollidableReference(staticHandle), handler);
        }

        /// <summary>
        /// Stops listening for events related to the given collidable.
        /// </summary>
        /// <param name="collidable">Collidable to stop listening for.</param>
        public void Unregister(CollidableReference collidable)
        {
            Debug.Assert(IsListener(collidable), "Should only try to unregister listeners that actually exist.");
            if (collidable.Mobility == CollidableMobility.Static)
            {
                staticListenerFlags.Remove(collidable.RawHandleValue);
            }
            else
            {
                bodyListenerFlags.Remove(collidable.RawHandleValue);
            }
            var index = listenerIndices[collidable];
            --listenerCount;
            ref var removedSlot = ref listeners[index];
            if (removedSlot.PreviousCollisions.Span.Allocated)
                removedSlot.PreviousCollisions.Dispose(pool);
            ref var lastSlot = ref listeners[listenerCount];
            if (index < listenerCount)
            {
                listenerIndices[lastSlot.Source] = index;
                removedSlot = lastSlot;
            }
            lastSlot = default;
        }

        /// <summary>
        /// Stops listening for events related to the given body.
        /// </summary>
        /// <param name="body">Body to stop listening for.</param>
        public void Unregister(BodyHandle body)
        {
            Unregister(simulation.Bodies[body].CollidableReference);
        }

        /// <summary>
        /// Stops listening for events related to the given static.
        /// </summary>
        /// <param name="staticHandle">Static to stop listening for.</param>
        public void Unregister(StaticHandle staticHandle)
        {
            Unregister(new CollidableReference(staticHandle));
        }

        /// <summary>
        /// Checks if a collidable is registered as a listener.
        /// </summary>
        /// <param name="collidable">Collidable to check.</param>
        /// <returns>True if the collidable has been registered as a listener, false otherwise.</returns>
        public bool IsListener(CollidableReference collidable)
        {
            if (collidable.Mobility == CollidableMobility.Static)
            {
                return staticListenerFlags.Contains(collidable.RawHandleValue);
            }
            else
            {
                return bodyListenerFlags.Contains(collidable.RawHandleValue);
            }
        }

        /// <summary>
        /// Callback attached to the simulation's ITimestepper which executes just prior to collision detection to take a snapshot of activity states to determine which pairs we should expect updates in.
        /// </summary>
        void SetFreshnessForCurrentActivityStatus(float dt, IThreadDispatcher threadDispatcher)
        {
            //Every single pair tracked by the contact events has a 'freshness' flag. If the final flush sees a pair that is stale, it'll remove it
            //and any necessary events to represent the end of that pair are reported.
            //HandleManifoldForCollidable sets 'Fresh' to true for any processed pair, but pairs between sleeping or static bodies will not show up in HandleManifoldForCollidable since they're not active.
            //We don't want Flush to report that sleeping pairs have stopped colliding, so we pre-initialize any such sleeping/static pair as 'fresh'.

            //This could be multithreaded reasonably easily if there are a ton of listeners or collisions, but that would be a pretty high bar.
            //For simplicity, the demo will keep it single threaded.
            var bodyHandleToLocation = simulation.Bodies.HandleToLocation;
            for (int listenerIndex = 0; listenerIndex < listenerCount; ++listenerIndex)
            {
                ref var listener = ref listeners[listenerIndex];
                var source = listener.Source;
                //If it's a body, and it's in the active set (index 0), then every pair associated with the listener should expect updates.
                var sourceExpectsUpdates = source.Mobility != CollidableMobility.Static && bodyHandleToLocation[source.BodyHandle.Value].SetIndex == 0;
                if (sourceExpectsUpdates)
                {
                    var previousCollisions = listeners[listenerIndex].PreviousCollisions;
                    for (int j = 0; j < previousCollisions.Count; ++j)
                    {
                        //Pair updates will set the 'freshness' to true when they happen, so that they won't be considered 'stale' in the flush and removed.
                        previousCollisions[j].Fresh = false;
                    }
                }
                else
                {
                    //The listener is either static or sleeping. We should only expect updates if the other collidable is awake.
                    var previousCollisions = listeners[listenerIndex].PreviousCollisions;
                    for (int j = 0; j < previousCollisions.Count; ++j)
                    {
                        ref var previousCollision = ref previousCollisions[j];
                        previousCollision.Fresh = previousCollision.Collidable.Mobility == CollidableMobility.Static || bodyHandleToLocation[previousCollision.Collidable.BodyHandle.Value].SetIndex > 0;
                    }
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void UpdatePreviousCollision<TManifold>(ref PreviousCollision collision, ref TManifold manifold, bool isTouching) where TManifold : unmanaged, IContactManifold<TManifold>
        {
            Debug.Assert(manifold.Count <= 4, "This demo was built on the assumption that nonconvex manifolds will have a maximum of four contacts, but that might have changed.");
            //If the above assert gets hit because of a change to nonconvex manifold capacities, the packed feature id representation this uses will need to be updated.
            //I very much doubt the nonconvex manifold will ever use more than 8 contacts, so addressing this wouldn't require much of a change.
            for (int j = 0; j < manifold.Count; ++j)
            {
                Unsafe.Add(ref collision.FeatureId0, j) = manifold.GetFeatureId(j);
            }
            collision.ContactCount = manifold.Count;
            collision.Fresh = true;
            collision.WasTouching = isTouching;
        }

        void HandleManifoldForCollidable<TManifold>(int workerIndex, CollidableReference source, CollidableReference other, CollidablePair pair, ref TManifold manifold) where TManifold : unmanaged, IContactManifold<TManifold>
        {
            //The "source" refers to the object that an event handler was (potentially) attached to, so we look for listeners registered for it.
            //(This function is called for both orders of the pair, so we'll catch listeners for either.)
            if (IsListener(source))
            {
                var listenerIndex = listenerIndices[source];
                //This collidable is registered. Is the opposing collidable present?
                ref var listener = ref listeners[listenerIndex];

                int previousCollisionIndex = -1;
                bool isTouching = false;
                for (int i = 0; i < listener.PreviousCollisions.Count; ++i)
                {
                    ref var collision = ref listener.PreviousCollisions[i];
                    //Since the 'Packed' field contains both the handle type (dynamic, kinematic, or static) and the handle index packed into a single bitfield, an equal value guarantees we are dealing with the same collidable.
                    if (collision.Collidable.Packed == other.Packed)
                    {
                        previousCollisionIndex = i;
                        //This manifold is associated with an existing collision.
                        //For every contact in the old collsion still present (by feature id), set a flag in this bitmask so we can know when a contact is removed.
                        int previousContactsStillExist = 0;
                        for (int contactIndex = 0; contactIndex < manifold.Count; ++contactIndex)
                        {
                            //We can check if each contact was already present in the previous frame by looking at contact feature ids. See the 'PreviousCollision' type for a little more info on FeatureIds.
                            var featureId = manifold.GetFeatureId(contactIndex);
                            var featureIdWasInPreviousCollision = false;
                            for (int previousContactIndex = 0; previousContactIndex < collision.ContactCount; ++previousContactIndex)
                            {
                                if (featureId == Unsafe.Add(ref collision.FeatureId0, previousContactIndex))
                                {
                                    featureIdWasInPreviousCollision = true;
                                    previousContactsStillExist |= 1 << previousContactIndex;
                                    break;
                                }
                            }
                            if (!featureIdWasInPreviousCollision)
                            {
                                manifold.GetContact(contactIndex, out var offset, out var normal, out var depth, out _);
                                listener.Handler.OnContactAdded(source, pair, ref manifold, offset, normal, depth, featureId, contactIndex, workerIndex);
                            }
                            if (manifold.GetDepth(ref manifold, contactIndex) >= 0)
                                isTouching = true;
                        }
                        if (previousContactsStillExist != (1 << collision.ContactCount) - 1)
                        {
                            //At least one contact that used to exist no longer does.
                            for (int previousContactIndex = 0; previousContactIndex < collision.ContactCount; ++previousContactIndex)
                            {
                                if ((previousContactsStillExist & (1 << previousContactIndex)) == 0)
                                {
                                    listener.Handler.OnContactRemoved(source, pair, ref manifold, Unsafe.Add(ref collision.FeatureId0, previousContactIndex), workerIndex);
                                }
                            }
                        }
                        if (!collision.WasTouching && isTouching)
                        {
                            listener.Handler.OnStartedTouching(source, pair, ref manifold, workerIndex);
                        }
                        else if (collision.WasTouching && !isTouching)
                        {
                            listener.Handler.OnStoppedTouching(source, pair, ref manifold, workerIndex);
                        }
                        if (isTouching)
                        {
                            listener.Handler.OnTouching(source, pair, ref manifold, workerIndex);
                        }
                        UpdatePreviousCollision(ref collision, ref manifold, isTouching);
                        break;
                    }
                }
                if (previousCollisionIndex < 0)
                {
                    //There was no collision previously.
                    ref var addsforWorker = ref pendingWorkerAdds[workerIndex];
                    //EnsureCapacity will create the list if it doesn't already exist.
                    addsforWorker.EnsureCapacity(Math.Max(addsforWorker.Count + 1, 64), GetPoolForWorker(workerIndex));
                    ref var pendingAdd = ref addsforWorker.AllocateUnsafely();
                    pendingAdd.ListenerIndex = listenerIndex;
                    pendingAdd.Collision.Collidable = other;
                    listener.Handler.OnPairCreated(source, pair, ref manifold, workerIndex);
                    //Dispatch events for all contacts in this new manifold.
                    for (int i = 0; i < manifold.Count; ++i)
                    {
                        manifold.GetContact(i, out var offset, out var normal, out var depth, out var featureId);
                        listener.Handler.OnContactAdded(source, pair, ref manifold, offset, normal, depth, featureId, i, workerIndex);
                        if (depth >= 0)
                            isTouching = true;
                    }
                    if (isTouching)
                    {
                        listener.Handler.OnStartedTouching(source, pair, ref manifold, workerIndex);
                        listener.Handler.OnTouching(source, pair, ref manifold, workerIndex);
                    }
                    UpdatePreviousCollision(ref pendingAdd.Collision, ref manifold, isTouching);
                }
                listener.Handler.OnPairUpdated(source, pair, ref manifold, workerIndex);

            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void HandleManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold) where TManifold : unmanaged, IContactManifold<TManifold>
        {
            HandleManifoldForCollidable(workerIndex, pair.A, pair.B, pair, ref manifold);
            HandleManifoldForCollidable(workerIndex, pair.B, pair.A, pair, ref manifold);
        }

        //For final events fired by the flush that still expect a manifold, we'll provide a special empty type.
        struct EmptyManifold : IContactManifold<EmptyManifold>
        {
            public int Count => 0;
            public bool Convex => true;
            //This type never has any contacts, so there's no need for any property grabbers.
            public void GetContact(int contactIndex, out Vector3 offset, out Vector3 normal, out float depth, out int featureId) { throw new NotImplementedException(); }
            public ref float GetDepth(ref EmptyManifold manifold, int contactIndex) { throw new NotImplementedException(); }
            public int GetFeatureId(int contactIndex) { throw new NotImplementedException(); }
            public ref int GetFeatureId(ref EmptyManifold manifold, int contactIndex) { throw new NotImplementedException(); }
            public ref Vector3 GetNormal(ref EmptyManifold manifold, int contactIndex) { throw new NotImplementedException(); }
            public ref Vector3 GetOffset(ref EmptyManifold manifold, int contactIndex) { throw new NotImplementedException(); }
        }

        public void Flush()
        {
            //For simplicity, this is completely sequential. Note that it's technically possible to extract more parallelism, but the complexity cost is high and you would need
            //very large numbers of events being processed to make it worth it.

            //Remove any stale collisions. Stale collisions are those which should have received a new manifold update but did not because the manifold is no longer active.
            for (int i = 0; i < listenerCount; ++i)
            {
                ref var listener = ref listeners[i];
                //Note reverse order. We remove during iteration.
                for (int j = listener.PreviousCollisions.Count - 1; j >= 0; --j)
                {
                    ref var collision = ref listener.PreviousCollisions[j];
                    if (!collision.Fresh)
                    {
                        //Sort the references to be consistent with the direct narrow phase results.
                        CollidablePair pair;
                        NarrowPhase.SortCollidableReferencesForPair(listener.Source, collision.Collidable, out _, out _, out pair.A, out pair.B);
                        if (collision.ContactCount > 0)
                        {
                            var emptyManifold = new EmptyManifold();
                            for (int previousContactCount = 0; previousContactCount < collision.ContactCount; ++previousContactCount)
                            {
                                listener.Handler.OnContactRemoved(listener.Source, pair, ref emptyManifold, Unsafe.Add(ref collision.FeatureId0, previousContactCount), 0);
                            }
                            if (collision.WasTouching)
                                listener.Handler.OnStoppedTouching(listener.Source, pair, ref emptyManifold, 0);
                        }
                        listener.Handler.OnPairEnded(collision.Collidable, pair);
                        //This collision was not updated since the last flush despite being active. It should be removed.
                        listener.PreviousCollisions.FastRemoveAt(j);
                        if (listener.PreviousCollisions.Count == 0)
                        {
                            listener.PreviousCollisions.Dispose(pool);
                            listener.PreviousCollisions = default;
                        }
                    }
                    else
                    {
                        collision.Fresh = false;
                    }
                }
            }

            for (int i = 0; i < pendingWorkerAdds.Length; ++i)
            {
                ref var pendingAdds = ref pendingWorkerAdds[i];
                for (int j = 0; j < pendingAdds.Count; ++j)
                {
                    ref var add = ref pendingAdds[j];
                    ref var collisions = ref listeners[add.ListenerIndex].PreviousCollisions;
                    //Ensure capacity will initialize the slot if necessary.
                    collisions.EnsureCapacity(Math.Max(8, collisions.Count + 1), pool);
                    collisions.AllocateUnsafely() = pendingAdds[j].Collision;
                }
                if (pendingAdds.Span.Allocated)
                    pendingAdds.Dispose(GetPoolForWorker(i));
                //We rely on zeroing out the count for lazy initialization.
                pendingAdds = default;
            }
            threadPools?.Clear();
        }

        public void Dispose()
        {
            if (bodyListenerFlags.Flags.Allocated)
                bodyListenerFlags.Dispose(pool);
            if (staticListenerFlags.Flags.Allocated)
                staticListenerFlags.Dispose(pool);
            listenerIndices.Dispose();
            simulation.Timestepper.BeforeCollisionDetection -= SetFreshnessForCurrentActivityStatus;
            threadPools?.Dispose();
            for (int i = 0; i < pendingWorkerAdds.Length; ++i)
            {
                Debug.Assert(!pendingWorkerAdds[i].Span.Allocated, "The pending worker adds should have been disposed by the previous flush.");
            }
        }
    }

    //The narrow phase needs a way to tell our contact events system about changes to contacts, so they'll need to be a part of the INarrowPhaseCallbacks.
    public unsafe struct ContactEventCallbacks : INarrowPhaseCallbacks
    {
        ContactEvents events;

        public ContactEventCallbacks(ContactEvents events)
        {
            this.events = events;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
        {
            pairMaterial.FrictionCoefficient = 1f;
            pairMaterial.MaximumRecoveryVelocity = 2f;
            pairMaterial.SpringSettings = new SpringSettings(30, 1);
            events.HandleManifold(workerIndex, pair, ref manifold);
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
        {
            return true;
        }

        public void Initialize(Simulation simulation)
        {
            events.Initialize(simulation);
        }

        public void Dispose()
        {
        }

    }

    /// <summary>
    /// Shows how to use the contact event handler above to respond to new collisions.
    /// </summary>
    public class ContactEventsDemo : Demo
    {
        struct ContactResponseParticle
        {
            public Vector3 Position;
            public float Age;
            public Vector3 Normal;
        }

        class EventHandler : IContactEventHandler
        {
            public Simulation Simulation;
            public BufferPool Pool;
            public QuickList<ContactResponseParticle> Particles;

            public EventHandler(Simulation simulation, BufferPool pool)
            {
                Simulation = simulation;
                Particles = new QuickList<ContactResponseParticle>(128, pool);
                Pool = pool;
            }

            public void OnContactAdded<TManifold>(CollidableReference eventSource, CollidablePair pair, ref TManifold contactManifold,
                Vector3 contactOffset, Vector3 contactNormal, float depth, int featureId, int contactIndex, int workerIndex) where TManifold : unmanaged, IContactManifold<TManifold>
            {
                //Simply ignore any particles beyond the allocated space.
                var index = Interlocked.Increment(ref Particles.Count) - 1;
                if (index < Particles.Span.Length)
                {
                    ref var particle = ref Particles[index];

                    //Contact data is calibrated according to the order of the pair, so using A's position is important.
                    particle.Position = contactOffset + (pair.A.Mobility == CollidableMobility.Static ?
                        new StaticReference(pair.A.StaticHandle, Simulation.Statics).Pose.Position :
                        new BodyReference(pair.A.BodyHandle, Simulation.Bodies).Pose.Position);
                    particle.Age = 0;
                    particle.Normal = contactNormal;
                }
            }

            public void Dispose()
            {
                //In the demo we won't actually call this, since it's going to persist until the demo dies. At that point, the buffer pool will be dropped and all its allocations will be cleaned up anyway.
                Particles.Dispose(Pool);
            }

        }

        ContactEvents events;
        EventHandler eventHandler;


        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 8, -20);
            camera.Yaw = MathHelper.Pi;

            events = new ContactEvents(ThreadDispatcher, BufferPool);
            Simulation = Simulation.Create(BufferPool, new ContactEventCallbacks(events), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));
            eventHandler = new EventHandler(Simulation, BufferPool);

            var listenedBody1 = Simulation.Bodies.Add(BodyDescription.CreateConvexDynamic(new Vector3(0, 5, 0), 1, Simulation.Shapes, new Box(1, 2, 3)));
            events.Register(Simulation.Bodies[listenedBody1].CollidableReference, eventHandler);

            var listenedBody2 = Simulation.Bodies.Add(BodyDescription.CreateConvexDynamic(new Vector3(0.5f, 10, 0), 1, Simulation.Shapes, new Capsule(0.25f, 0.7f)));
            events.Register(Simulation.Bodies[listenedBody2].CollidableReference, eventHandler);


            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0), Simulation.Shapes.Add(new Box(30, 1, 30))));
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 3, 15), Simulation.Shapes.Add(new Box(30, 5, 1))));
        }

        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            base.Update(window, camera, input, dt);
            //Can't forget to flush the events!
            //base.Update includes a call to the Simulation.Timestep, which will have called the event handlers if necessary.
            //Any newly observed collisions need to be pushed into the main storage, and any dead collisions need to be cleaned out.
            events.Flush();

            //Age and scoot the particles we created for new contacts for the animation.
            ref var particles = ref eventHandler.Particles;
            //The count was incremented across multiple threads; it may have gone beyond the buffer size. Ignore the extra.
            if (particles.Count > particles.Span.Length)
                particles.Count = particles.Span.Length;
            for (int i = particles.Count - 1; i >= 0; --i)
            {
                ref var particle = ref particles[i];
                particle.Age += dt;
                if (particle.Age > 0.7325f)
                {
                    particles.FastRemoveAt(i);
                }
                else
                {
                    particle.Position += particle.Normal * (2 * dt);
                }

            }
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            ref var particles = ref eventHandler.Particles;
            for (int i = particles.Count - 1; i >= 0; --i)
            {
                ref var particle = ref particles[i];
                var radius = particle.Age * (particle.Age * (0.135f - 2.7f * particle.Age) + 1.35f);
                var pose = new RigidPose(particle.Position);
                renderer.Shapes.AddShape(new Sphere(radius), Simulation.Shapes, pose, new Vector3(0, 1, 0));
            }

            var resolution = renderer.Surface.Resolution;
            renderer.TextBatcher.Write(text.Clear().Append("The library does not have a built-in concept of contact events like 'contact added' and 'contact removed'."), new Vector2(16, resolution.Y - 80), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("The INarrowPhaseCallbacks interface exposes callbacks that can be used to create such events."), new Vector2(16, resolution.Y - 64), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("This demo shows how events could be implemented. Green particles are spawned on contact add."), new Vector2(16, resolution.Y - 48), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("The full list of events supported in the demo's source:"), new Vector2(16, resolution.Y - 32), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("OnContactAdded, OnContactRemoved, OnStartedTouching, OnTouching, OnStoppedTouching, OnPairCreated, OnPairUpdated, and OnPairEnded."), new Vector2(16, resolution.Y - 16), 16, Vector3.One, font);

            base.Render(renderer, camera, input, text, font);
        }
    }
}
