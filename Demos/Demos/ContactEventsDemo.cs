using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
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

namespace Demos.Demos
{
    //Bepuphysics v2 doesn't have any concept of 'events'. It has callbacks that report the current status of contact manifolds.
    //Events can be built around those callbacks. This demo shows one way of doing that.

    //It's worth noting a few things about this event handler approach:
    //1) Every contact event is passed through the save event handler logic. It doesn't support hooking up unique logic to individual collidables.
    //That can be worked around by rewriting it to be a little more idiomatic, wrapping the handler in another system which uses delegates, or waiting for C# to support unmanaged delegates:
    //https://github.com/dotnet/csharplang/blob/master/proposals/static-delegates.md
    //2) This only implements 'on contact added' events. It wouldn't be very difficult to add OnContactRemoved or OnTouching events.
    //3) All event handlers execute from the multithreaded context of the simulation execution, so you have to be careful about what the event handlers do.
    //A "deferred" event model could be built on top of this.
    //4) This model doesn't expose the contact data for direct modification; it operates strictly as a notification.
    //5) This does quite a bit of work and slows down the narrow phase. 
    //6) This provides no insight into the constraints associated with these contacts. For example, if you want to spawn particles in response to heavy collisions or play sounds
    //with volume dependent on the impact force, you will need to go pull impulse data from the solver. Further, those sorts of use cases often only apply to nearby objects,
    //so a listener based model isn't ideal. Instead, querying nearby active objects and examining their contact constraints would be much more direct.
    //7) Contacts can be created speculatively. A contact existing does not guarantee a nonnegative penetration depth! We make no attempt to hide this fact in the demos.
    //You could modify this to only consider nonnegative depth contacts as existing for the purposes of events, but it gets more complicated.
    //8) There are other ways of pulling contact data. For example, check out how the contact line extractor works in the DemoRenderer. 
    //It pulls data directly from the solver data and could be extended to pull other information like impulses.

    public interface IContactEventHandler
    {
        void OnContactAdded<TManifold>(CollidableReference eventSource, CollidablePair pair, ref TManifold contactManifold,
            in Vector3 contactOffset, in Vector3 contactNormal, float depth, int featureId, int contactIndex, int workerIndex) where TManifold : IContactManifold;
    }

    public unsafe class ContactEvents<TEventHandler> : IDisposable where TEventHandler : IContactEventHandler
    {
        struct PreviousCollisionData
        {
            public CollidableReference Collidable;
            public bool Fresh;
            public int ContactCount;
            public int FeatureId0;
            public int FeatureId1;
            public int FeatureId2;
            public int FeatureId3;
        }

        Bodies bodies;
        public TEventHandler EventHandler;
        BufferPool pool;
        IThreadDispatcher threadDispatcher;


        QuickDictionary<CollidableReference, QuickList<PreviousCollisionData>, CollidableReferenceComparer> listeners;

        //Since the narrow phase works on multiple threads, we can't modify the collision data during execution.
        //The pending changes are stored in per-worker collections and flushed afterwards.
        struct PendingNewEntry
        {
            public int ListenerIndex;
            public PreviousCollisionData Collision;
        }

        QuickList<PendingNewEntry>[] pendingWorkerAdds;

        public ContactEvents(TEventHandler eventHandler, BufferPool pool, IThreadDispatcher threadDispatcher, int initialListenerCapacity = 32)
        {
            this.EventHandler = eventHandler;
            this.pool = pool;
            this.threadDispatcher = threadDispatcher;
            pendingWorkerAdds = new QuickList<PendingNewEntry>[threadDispatcher == null ? 1 : threadDispatcher.ThreadCount];
            listeners = new QuickDictionary<CollidableReference, QuickList<PreviousCollisionData>, CollidableReferenceComparer>(initialListenerCapacity, pool);
        }

        public void Initialize(Bodies bodies)
        {
            this.bodies = bodies;
        }

        /// <summary>
        /// Begins listening for events related to the given collidable.
        /// </summary>
        /// <param name="collidable">Collidable to monitor for events.</param>
        public void RegisterListener(CollidableReference collidable)
        {
            listeners.Add(collidable, default, pool);
        }

        /// <summary>
        /// Stops listening for events related to the given collidable.
        /// </summary>
        /// <param name="collidable">Collidable to stop listening for.</param>
        public void UnregisterListener(CollidableReference collidable)
        {
            var exists = listeners.GetTableIndices(ref collidable, out var tableIndex, out var elementIndex);
            Debug.Assert(exists, "Should only try to unregister listeners that actually exist.");
            listeners.Values[elementIndex].Dispose(pool);
            listeners.FastRemove(tableIndex, elementIndex);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        bool FeatureIdContained(int featureId, ulong previousFeatureIds)
        {
            var contained0 = (((int)previousFeatureIds ^ featureId) & 0xFFFF) == 0;
            var contained1 = (((int)(previousFeatureIds >> 16) ^ featureId) & 0xFFFF) == 0;
            var contained2 = (((int)(previousFeatureIds >> 32) ^ featureId) & 0xFFFF) == 0;
            var contained3 = (((int)(previousFeatureIds >> 48) ^ featureId) & 0xFFFF) == 0;
            return contained0 | contained1 | contained2 | contained3;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void UpdatePreviousCollision<TManifold>(ref PreviousCollisionData collision, ref TManifold manifold) where TManifold : IContactManifold
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
        }

        void HandleManifoldForCollidable<TManifold>(int workerIndex, CollidableReference source, CollidableReference other, CollidablePair pair, ref TManifold manifold) where TManifold : IContactManifold
        {
            if (listeners.GetTableIndices(ref source, out var tableIndex, out var listenerIndex))
            {
                //This collidable is registered. Is the opposing collidable present?
                ref var previousCollisions = ref listeners.Values[listenerIndex];
                int previousCollisionIndex = -1;
                for (int i = 0; i < previousCollisions.Count; ++i)
                {
                    ref var collision = ref previousCollisions[i];
                    if (collision.Collidable.Packed == other.Packed)
                    {
                        previousCollisionIndex = i;
                        //This manifold is associated with an existing collision.
                        for (int contactIndex = 0; contactIndex < manifold.Count; ++contactIndex)
                        {
                            var featureId = manifold.GetFeatureId(contactIndex);
                            var featureIdIsOld = false;
                            for (int previousContactIndex = 0; previousContactIndex < collision.ContactCount; ++previousContactIndex)
                            {
                                if (featureId == Unsafe.Add(ref collision.FeatureId0, previousContactIndex))
                                {
                                    featureIdIsOld = true;
                                    break;
                                }
                            }
                            if (!featureIdIsOld)
                            {
                                manifold.GetContact(contactIndex, out var offset, out var normal, out var depth, out _);
                                EventHandler.OnContactAdded(source, pair, ref manifold, offset, normal, depth, featureId, contactIndex, workerIndex);
                            }
                        }
                        UpdatePreviousCollision(ref collision, ref manifold);
                        break;
                    }
                }
                if (previousCollisionIndex < 0)
                {
                    //There was no collision previously.
                    ref var addsforWorker = ref pendingWorkerAdds[workerIndex];
                    //EnsureCapacity will create the list if it doesn't already exist.
                    addsforWorker.EnsureCapacity(Math.Max(addsforWorker.Count + 1, 64), threadDispatcher != null ? threadDispatcher.GetThreadMemoryPool(workerIndex) : pool);
                    ref var pendingAdd = ref addsforWorker.AllocateUnsafely();
                    pendingAdd.ListenerIndex = listenerIndex;
                    pendingAdd.Collision.Collidable = other;
                    UpdatePreviousCollision(ref pendingAdd.Collision, ref manifold);
                    //Dispatch events for all contacts in this new manifold.
                    for (int i = 0; i < manifold.Count; ++i)
                    {
                        manifold.GetContact(i, out var offset, out var normal, out var depth, out var featureId);
                        EventHandler.OnContactAdded(source, pair, ref manifold, offset, normal, depth, featureId, i, workerIndex);
                    }
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void HandleManifold(int workerIndex, CollidablePair pair, ConvexContactManifold* manifold)
        {
            HandleManifoldForCollidable(workerIndex, pair.A, pair.B, pair, ref *manifold);
            HandleManifoldForCollidable(workerIndex, pair.B, pair.A, pair, ref *manifold);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void HandleManifold(int workerIndex, CollidablePair pair, NonconvexContactManifold* manifold)
        {
            HandleManifoldForCollidable(workerIndex, pair.A, pair.B, pair, ref *manifold);
            HandleManifoldForCollidable(workerIndex, pair.B, pair.A, pair, ref *manifold);
        }

        public void Flush()
        {
            //For simplicity, this is completely sequential. Note that it's technically possible to extract more parallelism, but the complexity cost is high and you would need
            //very large numbers of events being processed to make it worth it.

            //Remove any stale collisions. Stale collisions are those which should have received a new manifold update but did not because the manifold is no longer active.
            for (int i = 0; i < listeners.Count; ++i)
            {
                var collidable = listeners.Keys[i];
                //Pairs involved with inactive bodies do not need to be checked for freshness. If we did, it would result in inactive manifolds being considered a removal, and 
                //more contact added events would fire when the bodies woke up.
                if (collidable.Mobility != CollidableMobility.Static && bodies.HandleToLocation[collidable.Handle].SetIndex > 0)
                    continue;
                ref var collisions = ref listeners.Values[i];
                //Note reverse order. We remove during iteration.
                for (int j = collisions.Count - 1; j >= 0; --j)
                {
                    ref var collision = ref collisions[j];
                    //Again, any pair involving inactive bodies does not need to be examined.
                    if (collision.Collidable.Mobility != CollidableMobility.Static && bodies.HandleToLocation[collision.Collidable.Handle].SetIndex > 0)
                        continue;
                    if (!collision.Fresh)
                    {
                        //This collision was not updated since the last flush despite being active. It should be removed.
                        collisions.FastRemoveAt(j);
                        if (collisions.Count == 0)
                        {
                            collisions.Dispose(pool);
                            collisions = default;
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
                    ref var collisions = ref listeners.Values[add.ListenerIndex];
                    //Ensure capacity will initialize the slot if necessary.
                    collisions.EnsureCapacity(Math.Max(8, collisions.Count + 1), pool);
                    collisions.AllocateUnsafely() = pendingAdds[j].Collision;
                }
                if (pendingAdds.Span.Allocated)
                    pendingAdds.Dispose(threadDispatcher == null ? pool : threadDispatcher.GetThreadMemoryPool(i));
                //We rely on zeroing out the count for lazy initialization.
                pendingAdds = default;
            }
        }

        public void Dispose()
        {
            listeners.Dispose(pool);
            for (int i = 0; i < pendingWorkerAdds.Length; ++i)
            {
                Debug.Assert(!pendingWorkerAdds[i].Span.Allocated, "The pending worker adds should have been disposed by the previous flush.");
            }
        }
    }

    public unsafe struct ContactEventCallbacks<TEventHandler> : INarrowPhaseCallbacks where TEventHandler : IContactEventHandler
    {
        ContactEvents<TEventHandler> events;

        public ContactEventCallbacks(ContactEvents<TEventHandler> events)
        {
            this.events = events;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void ConfigureMaterial(out PairMaterialProperties pairMaterial)
        {
            pairMaterial.FrictionCoefficient = 1f;
            pairMaterial.MaximumRecoveryVelocity = 2f;
            pairMaterial.SpringSettings = new SpringSettings(30, 1);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, NonconvexContactManifold* manifold, out PairMaterialProperties pairMaterial)
        {
            ConfigureMaterial(out pairMaterial);
            events.HandleManifold(workerIndex, pair, manifold);
            return true;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, ConvexContactManifold* manifold, out PairMaterialProperties pairMaterial)
        {
            ConfigureMaterial(out pairMaterial);
            events.HandleManifold(workerIndex, pair, manifold);
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ConvexContactManifold* manifold)
        {
            return true;
        }

        public void Initialize(Simulation simulation)
        {
            events.Initialize(simulation.Bodies);
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

        struct EventHandler : IContactEventHandler
        {
            public Simulation Simulation;
            public QuickList<ContactResponseParticle> Particles;

            public void OnContactAdded<TManifold>(CollidableReference eventSource, CollidablePair pair, ref TManifold contactManifold,
                in Vector3 contactOffset, in Vector3 contactNormal, float depth, int featureId, int contactIndex, int workerIndex) where TManifold : IContactManifold
            {
                //var other = pair.A.Packed == eventSource.Packed ? pair.B : pair.A;
                //Console.WriteLine($"Added contact: ({eventSource}, {other}): {featureId}");
                //Simply ignore any particles beyond the allocated space.
                var index = Interlocked.Increment(ref Particles.Count) - 1;
                if (index < Particles.Span.Length)
                {
                    ref var particle = ref Particles[index];

                    //Contact data is calibrated according to the order of the pair, so using A's position is important.
                    particle.Position = contactOffset + (pair.A.Mobility == CollidableMobility.Static ?
                        new StaticReference(pair.A.Handle, Simulation.Statics).Pose.Position :
                        new BodyReference(pair.A.Handle, Simulation.Bodies).Pose.Position);
                    particle.Age = 0;
                    particle.Normal = contactNormal;
                }
            }
        }

        ContactEvents<EventHandler> events;

        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 8, -20);
            camera.Yaw = MathHelper.Pi;

            events = new ContactEvents<EventHandler>(new EventHandler(), BufferPool, ThreadDispatcher);
            Simulation = Simulation.Create(BufferPool, new ContactEventCallbacks<EventHandler>(events), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));
            events.EventHandler.Particles = new QuickList<ContactResponseParticle>(128, BufferPool);
            events.EventHandler.Simulation = Simulation;

            var listenedBody1 = Simulation.Bodies.Add(BodyDescription.CreateConvexDynamic(new Vector3(0, 5, 0), 1, Simulation.Shapes, new Box(1, 2, 3)));
            events.RegisterListener(new CollidableReference(CollidableMobility.Dynamic, listenedBody1));

            var listenedBody2 = Simulation.Bodies.Add(BodyDescription.CreateConvexDynamic(new Vector3(0.5f, 10, 0), 1, Simulation.Shapes, new Capsule(0.25f, 0.7f)));
            events.RegisterListener(new CollidableReference(CollidableMobility.Dynamic, listenedBody2));
            
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(30, 1, 30)), 0.04f)));
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 3, 15), new CollidableDescription(Simulation.Shapes.Add(new Box(30, 5, 1)), 0.04f)));

        }

        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            base.Update(window, camera, input, dt);
            events.Flush();

            ref var particles = ref events.EventHandler.Particles;
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
            ref var particles = ref events.EventHandler.Particles;
            for (int i = particles.Count - 1; i >= 0; --i)
            {
                ref var particle = ref particles[i];
                var radius = particle.Age * (particle.Age * (0.135f - 2.7f * particle.Age) + 1.35f);
                var pose = new RigidPose(particle.Position);
                renderer.Shapes.AddShape(new Sphere(radius), Simulation.Shapes, ref pose, new Vector3(0, 1, 0));
            }
            base.Render(renderer, camera, input, text, font);
        }
    }
}
