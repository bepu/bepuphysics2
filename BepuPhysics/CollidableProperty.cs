using BepuPhysics.Collidables;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics
{
    //TODO: It would be nice to have an index-aligned version of this. Quite a bit more complicated to implement since indices move around a lot and involve complex multithreaded bookkeeping.
    /// <summary>
    /// Convenience collection that stores extra properties about bodies and statics, indexed by the body or static handle.
    /// </summary>
    /// <typeparam name="T">Type of the data to store.</typeparam>
    /// <remarks>This is built for use cases relying on random access like the narrow phase. For maximum performance with sequential access, an index-aligned structure would be better.</remarks>
    public class CollidableProperty<T> : IDisposable where T : unmanaged
    {
        //Bodies and statics each have 'handle spaces', like namespaces. A body and static can have the same integer valued handle.
        //So, we need to have two different buffers for data.
        Simulation simulation;
        BufferPool pool;
        Buffer<T> bodyData;
        Buffer<T> staticData;

        //The split constructor/initialize looks a bit odd, but this is very frequently used for narrow phase callbacks. The instance needs to exist before the simulation does.

        /// <summary>
        /// Constructs a new collection to store handle-aligned body properties. Assumes the Initialize function will be called later to provide the Bodies collection.
        /// </summary>
        /// <param name="pool">Pool from which to pull internal resources. If null, uses the later Initialize-provided Bodies pool.</param>
        public CollidableProperty(BufferPool pool = null)
        {
            this.pool = pool;
        }

        /// <summary>
        /// Constructs a new collection to store handle-aligned body and static properties.
        /// </summary>
        /// <param name="simulation">Simulation to track.</param>
        /// <param name="pool">Pool from which to pull internal resources. If null, uses the Simulation pool.</param>
        public CollidableProperty(Simulation simulation, BufferPool pool = null)
        {
            this.simulation = simulation;
            this.pool = pool == null ? simulation.BufferPool : pool;
            this.pool.TakeAtLeast(simulation.Bodies.HandleToLocation.Length, out bodyData);
            this.pool.TakeAtLeast(simulation.Statics.HandleToIndex.Length, out staticData);
        }

        /// <summary>
        /// Initializes the property collection if the Bodies/Statics-less constructor was used.
        /// </summary>
        /// <param name="bodies">Bodies collection to track.</param>
        /// <param name="statics">Statics collection to track.</param>
        public void Initialize(Simulation simulation)
        {
            if (this.simulation != null)
                throw new InvalidOperationException("Initialize should only be used on a collection which was constructed without defining the Simulation.");
            this.simulation = simulation;
            if (pool == null)
                pool = simulation.Bodies.Pool;
            pool.TakeAtLeast(simulation.Bodies.HandleToLocation.Length, out bodyData);
            pool.TakeAtLeast(simulation.Statics.HandleToIndex.Length, out staticData);
        }

        /// <summary>
        /// Gets a reference to the properties associated with a body's handle.
        /// </summary>
        /// <param name="bodyHandle">Body handle to retrieve the properties for.</param>
        /// <returns>Reference to properties associated with a body handle.</returns>
        public ref T this[BodyHandle bodyHandle]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                Debug.Assert(simulation.Bodies.BodyExists(bodyHandle));
                return ref bodyData[bodyHandle.Value];
            }
        }
        /// <summary>
        /// Gets a reference to the properties associated with a static's handle.
        /// </summary>
        /// <param name="staticHandle">Static handle to retrieve the properties for.</param>
        /// <returns>Reference to properties associated with a static handle.</returns>
        public ref T this[StaticHandle staticHandle]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                Debug.Assert(simulation.Statics.StaticExists(staticHandle));
                return ref staticData[staticHandle.Value];
            }
        }
        /// <summary>
        /// Gets a reference to the properties associated with a collidable.
        /// </summary>
        /// <param name="staticHandle">Collidable to retrieve the properties for.</param>
        /// <returns>Reference to properties associated with a collidable.</returns>
        public ref T this[CollidableReference collidable]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                if(collidable.Mobility == CollidableMobility.Static)
                    return ref this[collidable.StaticHandle];
                return ref this[collidable.BodyHandle];
            }
        }

        /// <summary>
        /// Ensures there is space for a given body handle and returns a reference to the used memory.
        /// </summary>
        /// <param name="bodyHandle">Body handle to allocate for.</param>
        /// <returns>Reference to the data for the given body.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref T Allocate(BodyHandle bodyHandle)
        {
            Debug.Assert(simulation.Bodies.BodyExists(bodyHandle), "The body handle should have been allocated in the bodies set before any attempt to create a property for it.");
            if (bodyHandle.Value >= bodyData.Length)
            {
                var targetCount = BufferPool.GetCapacityForCount<T>(simulation.Bodies.HandlePool.HighestPossiblyClaimedId + 1);
                Debug.Assert(targetCount > bodyData.Length, "Given what just happened, we must require a resize.");
                pool.ResizeToAtLeast(ref bodyData, targetCount, Math.Min(simulation.Bodies.HandlePool.HighestPossiblyClaimedId + 1, bodyData.Length));
            }
            return ref bodyData[bodyHandle.Value];
        }

        /// <summary>
        /// Ensures there is space for a given static handle and returns a reference to the used memory.
        /// </summary>
        /// <param name="handle">Static handle to allocate for.</param>
        /// <returns>Reference to the data for the given static.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref T Allocate(StaticHandle handle)
        {
            Debug.Assert(simulation.Statics.StaticExists(handle), "The static handle should have been allocated in the statics set before any attempt to create a property for it.");
            if (handle.Value >= staticData.Length)
            {
                var targetCount = BufferPool.GetCapacityForCount<T>(simulation.Statics.HandlePool.HighestPossiblyClaimedId + 1);
                Debug.Assert(targetCount > staticData.Length, "Given what just happened, we must require a resize.");
                pool.ResizeToAtLeast(ref staticData, targetCount, Math.Min(simulation.Statics.HandlePool.HighestPossiblyClaimedId + 1, staticData.Length));
            }
            return ref staticData[handle.Value];
        }

        /// <summary>
        /// Ensures there is space for a given collidable reference and returns a reference to the used memory.
        /// </summary>
        /// <param name="handle">Collidable reference to allocate for.</param>
        /// <returns>Reference to the data for the given collidable.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref T Allocate(CollidableReference collidableReference)
        {
            if (collidableReference.Mobility == CollidableMobility.Static)
            {
                return ref Allocate(collidableReference.StaticHandle);
            }
            return ref Allocate(collidableReference.BodyHandle);
        }

        /// <summary>
        /// Ensures that the internal structures have at least the given capacity for bodies.
        /// </summary>
        /// <param name="capacity">Capacity to ensure.</param>
        public void EnsureBodyCapacity(int capacity)
        {
            var targetCount = BufferPool.GetCapacityForCount<T>(Math.Max(simulation.Bodies.HandlePool.HighestPossiblyClaimedId + 1, capacity));
            if (targetCount > bodyData.Length)
            {
                pool.ResizeToAtLeast(ref bodyData, targetCount, Math.Min(bodyData.Length, simulation.Bodies.HandlePool.HighestPossiblyClaimedId + 1));
            }
        }
        /// <summary>
        /// Ensures that the internal structures have at least the given capacity for statics.
        /// </summary>
        /// <param name="capacity">Capacity to ensure.</param>
        public void EnsureStaticCapacity(int capacity)
        {
            var targetCount = BufferPool.GetCapacityForCount<T>(Math.Max(simulation.Statics.HandlePool.HighestPossiblyClaimedId + 1, capacity));
            if (targetCount > staticData.Length)
            {
                pool.ResizeToAtLeast(ref staticData, targetCount, Math.Min(staticData.Length, simulation.Statics.HandlePool.HighestPossiblyClaimedId + 1));
            }
        }

        /// <summary>
        /// Compacts the memory used by the collection for bodies to a safe minimum based on the Bodies collection.
        /// </summary>
        public void CompactBodies()
        {
            var targetCount = BufferPool.GetCapacityForCount<T>(simulation.Bodies.HandlePool.HighestPossiblyClaimedId + 1);
            if (targetCount < bodyData.Length)
            {
                pool.ResizeToAtLeast(ref bodyData, targetCount, simulation.Bodies.HandlePool.HighestPossiblyClaimedId + 1);
            }
        }
        /// <summary>
        /// Compacts the memory used by the collection for statics to a safe minimum based on the Statics collection.
        /// </summary>
        public void CompactStatics()
        {
            var targetCount = BufferPool.GetCapacityForCount<T>(simulation.Statics.HandlePool.HighestPossiblyClaimedId + 1);
            if (targetCount < staticData.Length)
            {
                pool.ResizeToAtLeast(ref staticData, targetCount, simulation.Statics.HandlePool.HighestPossiblyClaimedId + 1);
            }
        }

        /// <summary>
        /// Returns all held resources.
        /// </summary>
        public void Dispose()
        {
            pool.Return(ref bodyData);
            pool.Return(ref staticData);
        }
    }
}
