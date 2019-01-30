using BepuPhysics;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Text;

namespace Demos
{
    public struct TimelineStats
    {
        public double Total;
        public double Average;
        public double Min;
        public double Max;
        public double StdDev;
    }
    
    /// <summary>
    /// Stores the time it took to complete stages of the physics simulation in a ring buffer. Once the ring buffer is full, the oldest results will be removed.
    /// </summary>
    public class SimulationTimeSamples
    {
        public TimingsRingBuffer Simulation;
        public TimingsRingBuffer PoseIntegrator;
        public TimingsRingBuffer Sleeper;
        public TimingsRingBuffer BroadPhaseUpdate;
        public TimingsRingBuffer CollisionTesting;
        public TimingsRingBuffer NarrowPhaseFlush;
        public TimingsRingBuffer Solver;
        public TimingsRingBuffer BodyOptimizer;
        public TimingsRingBuffer ConstraintOptimizer;
        public TimingsRingBuffer BatchCompressor;

        public SimulationTimeSamples(int frameCapacity, BufferPool pool)
        {
            Simulation = new TimingsRingBuffer(frameCapacity, pool);
            PoseIntegrator = new TimingsRingBuffer(frameCapacity, pool);
            Sleeper = new TimingsRingBuffer(frameCapacity, pool);
            BroadPhaseUpdate = new TimingsRingBuffer(frameCapacity, pool);
            CollisionTesting = new TimingsRingBuffer(frameCapacity, pool);
            NarrowPhaseFlush = new TimingsRingBuffer(frameCapacity, pool);
            Solver = new TimingsRingBuffer(frameCapacity, pool);
            BodyOptimizer = new TimingsRingBuffer(frameCapacity, pool);
            ConstraintOptimizer = new TimingsRingBuffer(frameCapacity, pool);
            BatchCompressor = new TimingsRingBuffer(frameCapacity, pool);
        }

        public void RecordFrame(Simulation simulation)
        {
            //This requires the simulation to be compiled with profiling enabled.
            Simulation.Add(simulation.Profiler[simulation]);
            PoseIntegrator.Add(simulation.Profiler[simulation.PoseIntegrator]);
            Sleeper.Add(simulation.Profiler[simulation.Sleeper]);
            BroadPhaseUpdate.Add(simulation.Profiler[simulation.BroadPhase]);
            CollisionTesting.Add(simulation.Profiler[simulation.BroadPhaseOverlapFinder]);
            NarrowPhaseFlush.Add(simulation.Profiler[simulation.NarrowPhase]);
            Solver.Add(simulation.Profiler[simulation.Solver]);
            BodyOptimizer.Add(simulation.Profiler[simulation.BodyLayoutOptimizer]);
            ConstraintOptimizer.Add(simulation.Profiler[simulation.ConstraintLayoutOptimizer]);
            BatchCompressor.Add(simulation.Profiler[simulation.SolverBatchCompressor]);
        }

        public void Dispose()
        {
            Simulation.Dispose();
            PoseIntegrator.Dispose();
            Sleeper.Dispose();
            BroadPhaseUpdate.Dispose();
            CollisionTesting.Dispose();
            NarrowPhaseFlush.Dispose();
            Solver.Dispose();
            BodyOptimizer.Dispose();
            ConstraintOptimizer.Dispose();
            BatchCompressor.Dispose();
        }
    }
}
