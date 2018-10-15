using BepuUtilities.Memory;
using DemoRenderer;
using DemoUtilities;
using BepuPhysics;
using System;
using System.Collections.Generic;
using System.Text;
using DemoRenderer.UI;
using DemoContentLoader;

namespace Demos
{
    public abstract class Demo : IDisposable
    {
        /// <summary>
        /// Gets the simulation created by the demo's Initialize call.
        /// </summary>
        public Simulation Simulation { get; protected set; }

        //Note that the buffer pool used by the simulation is not considered to be *owned* by the simulation. The simulation merely uses the pool.
        //Disposing the simulation will not dispose or clear the buffer pool.
        /// <summary>
        /// Gets the buffer pool used by the demo's simulation.
        /// </summary>
        public BufferPool BufferPool { get; private set; }

        /// <summary>
        /// Gets the thread dispatcher available for use by the simulation.
        /// </summary>
        public SimpleThreadDispatcher ThreadDispatcher { get; private set; }

        protected Demo()
        {
            BufferPool = new BufferPool();
            ThreadDispatcher = new SimpleThreadDispatcher(Environment.ProcessorCount);
        }

        public abstract void Initialize(ContentArchive content, Camera camera);

        public virtual void Update(Input input, float dt)
        {
            //TODO: While for the sake of the demos, using one update per render is probably the easiest/best choice,
            //we can't assume that every monitor has a 60hz refresh rate. One simple option here is to just measure the primary display's refresh rate ahead of time
            //and use that as the simulation timestep duration. Different displays would affect the simulation, but it wouldn't be too bad, and it would be locally consistent.
            Simulation.Timestep(1 / 60f, ThreadDispatcher);
        }                           

        public virtual void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
        }

        protected virtual void OnDispose()
        {

        }

        bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                OnDispose();
                Simulation.Dispose();
                BufferPool.Clear();
                ThreadDispatcher.Dispose();
            }
        }

#if DEBUG
        ~Demo()
        {
            Helpers.CheckForUndisposed(disposed, this);
        }
#endif
    }
}
