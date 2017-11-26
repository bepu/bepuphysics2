using BepuPhysics;
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
        public TimingsRingBuffer Deactivation;
        public TimingsRingBuffer BroadPhaseUpdate;
        public TimingsRingBuffer CollisionTesting;
        public TimingsRingBuffer NarrowPhaseFlush;
        public TimingsRingBuffer Solver;
        public TimingsRingBuffer BodyOptimizer;
        public TimingsRingBuffer ConstraintOptimizer;
        public TimingsRingBuffer BatchCompressor;

        public SimulationTimeSamples(int frameCapacity)
        {
            Simulation = new TimingsRingBuffer(frameCapacity);
            PoseIntegrator = new TimingsRingBuffer(frameCapacity);
            Deactivation = new TimingsRingBuffer(frameCapacity);
            BroadPhaseUpdate = new TimingsRingBuffer(frameCapacity);
            CollisionTesting = new TimingsRingBuffer(frameCapacity);
            NarrowPhaseFlush = new TimingsRingBuffer(frameCapacity);
            Solver = new TimingsRingBuffer(frameCapacity);
            BodyOptimizer = new TimingsRingBuffer(frameCapacity);
            ConstraintOptimizer = new TimingsRingBuffer(frameCapacity);
            BatchCompressor = new TimingsRingBuffer(frameCapacity);
        }

        public void RecordFrame(Simulation simulation)
        {
            //This requires the simulation to be compiled with profiling enabled.
            Simulation.Add(simulation.Timings[simulation]);
            PoseIntegrator.Add(simulation.Timings[simulation.PoseIntegrator]);
            Deactivation.Add(simulation.Timings[simulation.Deactivator]);
            BroadPhaseUpdate.Add(simulation.Timings[simulation.BroadPhase]);
            CollisionTesting.Add(simulation.Timings[simulation.BroadPhaseOverlapFinder]);
            NarrowPhaseFlush.Add(simulation.Timings[simulation.NarrowPhase]);
            Solver.Add(simulation.Timings[simulation.Solver]);
            BodyOptimizer.Add(simulation.Timings[simulation.BodyLayoutOptimizer]);
            ConstraintOptimizer.Add(simulation.Timings[simulation.ConstraintLayoutOptimizer]);
            BatchCompressor.Add(simulation.Timings[simulation.SolverBatchCompressor]);
        }
    }
}
