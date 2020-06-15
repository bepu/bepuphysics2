using BepuUtilities.Memory;
using DemoRenderer;
using DemoUtilities;
using BepuPhysics;
using System;
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
            //Generally, shoving as many threads as possible into the simulation won't produce the best results on systems with multiple logical cores per physical core.
            //Environment.ProcessorCount reports logical core count only, so we'll use a simple heuristic here- it'll leave one or two logical cores idle.
            //For the common Intel quad core with hyperthreading, this'll use six logical cores and leave two logical cores free to be used for other stuff.
            //This is by no means perfect. To maximize performance, you'll need to profile your simulation and target hardware.
            //Note that issues can be magnified on older operating systems like Windows 7 if all logical cores are given work.

            //Generally, the more memory bandwidth you have relative to CPU compute throughput, and the more collision detection heavy the simulation is relative to solving,
            //the more benefit you get out of SMT/hyperthreading. 
            //For example, if you're using the 64 core quad memory channel AMD 3990x on a scene composed of thousands of ragdolls, 
            //there won't be enough memory bandwidth to even feed half the physical cores. Using all 128 logical cores would just add overhead.

            //It may be worth using something like hwloc to extract extra information to reason about.
            var targetThreadCount = Math.Max(1, Environment.ProcessorCount > 4 ? Environment.ProcessorCount - 2 : Environment.ProcessorCount - 1);
            ThreadDispatcher = new SimpleThreadDispatcher(targetThreadCount);
        }

        public virtual void LoadGraphicalContent(ContentArchive content, RenderSurface surface)
        {
        }

        public abstract void Initialize(ContentArchive content, Camera camera);


        public virtual void Update(Window window, Camera camera, Input input, float dt)
        {
            //In the demos, we use one time step per frame. We don't bother modifying the physics time step duration for different monitors so different refresh rates
            //change the rate of simulation. This doesn't actually change the result of the simulation, though, and the simplicity is a good fit for the demos.
            //In the context of a 'real' application, you could instead use a time accumulator to take time steps of fixed length as needed, or
            //fully decouple simulation and rendering rates across different threads.
            //(In either case, you'd also want to interpolate or extrapolate simulation results during rendering for smoothness.)
            //Note that taking steps of variable length can reduce stability. Gradual or one-off changes can work reasonably well.
            Simulation.Timestep(1 / 60f, ThreadDispatcher);

            ////Here's an example of how it would look to use more frequent updates, but still with a fixed amount of time simulated per update call:
            //const float timeToSimulate = 1 / 60f;
            //const int timestepsPerUpdate = 2;
            //const float timePerTimestep = timeToSimulate / timestepsPerUpdate;
            //for (int i = 0; i < timestepsPerUpdate; ++i)
            //{
            //    Simulation.Timestep(timePerTimestep, ThreadDispatcher);
            //}

            ////And here's an example of how to use an accumulator to take a number of timesteps of fixed length in response to variable update dt:
            //timeAccumulator += dt;
            //var targetTimestepDuration = 1 / 120f;
            //while (timeAccumulator >= targetTimestepDuration)
            //{
            //    Simulation.Timestep(targetTimestepDuration, ThreadDispatcher);
            //    timeAccumulator -= targetTimestepDuration;
            //}
            ////If you wanted to smooth out the positions of rendered objects to avoid the 'jitter' that an unpredictable number of time steps per update would cause,
            ////you can just interpolate the previous and current states using a weight based on the time remaining in the accumulator:
            //var interpolationWeight = timeAccumulator / targetTimestepDuration;
        }
        //If you're using the accumulator-based timestep approach above, you'll need this field.
        //float timeAccumulator;

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
            DemoRenderer.Helpers.CheckForUndisposed(disposed, this);
        }
#endif
    }
}
