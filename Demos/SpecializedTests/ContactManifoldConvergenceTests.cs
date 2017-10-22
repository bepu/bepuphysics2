using BepuUtilities;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Constraints;
using BepuPhysics.Constraints.Contact;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;
using static BepuPhysics.Solver;
using Quaternion = BepuUtilities.Quaternion;

namespace Demos.SpecializedTests
{
    static class ContactManifoldConvergenceTests
    {
        public static void Test()
        {
            //const int bodyCount = 8;
            //SimulationSetup.BuildStackOfBodiesOnGround(bodyCount, false, true, out var bodies, out var solver, out var graph, out var bodyHandles, out var constraintHandles);
            const int width = 17;
            const int height = 18;
            const int length = 17;
            SimulationSetup.BuildLattice(
                new RegularGridWithKinematicBaseBuilder(new Vector3(1), new Vector3()),
                new ContactManifoldConstraintBuilder(),
                width, height, length, out var simulation, out var bodyHandles, out var constraintHandles);

            //SimulationSetup.ScrambleBodies(simulation);
            //SimulationSetup.ScrambleConstraints(simulation.Solver);
            //SimulationSetup.ScrambleBodyConstraintLists(simulation);
            //SimulationSetup.AddRemoveChurn(simulation, 100000, bodyHandles, constraintHandles);

            var threadDispatcher = new SimpleThreadDispatcher(8);
            //var threadDispatcher = new NotQuiteAThreadDispatcher(1);

            //double compressionTimeAccumulator = 0;
            //const int iterations = 100;
            //const int internalCompressionIterations = 100;
            //simulation.SolverBatchCompressor.Compress(simulation.BufferPool, threadDispatcher); //prejit
            //for (int i = 0; i < iterations; ++i)
            //{
            //    //SimulationSetup.AddRemoveChurn(simulation, 100, bodyHandles, constraintHandles);
            //    GC.Collect(3, GCCollectionMode.Forced, true);
            //    var start = Stopwatch.GetTimestamp();
            //    for (int j = 0; j < internalCompressionIterations; ++j)
            //    {
            //        simulation.SolverBatchCompressor.Compress(simulation.BufferPool, threadDispatcher);
            //    }
            //    compressionTimeAccumulator += (Stopwatch.GetTimestamp() - start) / (double)Stopwatch.Frequency;
            //}
            //Console.WriteLine($"Time per compression: {1e6 * compressionTimeAccumulator / (iterations * internalCompressionIterations)} us");
            //GC.Collect(3, GCCollectionMode.Forced, true);

            ////Attempt cache optimization.
            //int bodyOptimizationIterations = bodyHandles.Length * 1;
            //simulation.BodyLayoutOptimizer.OptimizationFraction = 0.004f;
            //simulation.BodyLayoutOptimizer.IncrementalOptimize(simulation.BufferPool, threadDispatcher);//prejit
            var timer = Stopwatch.StartNew();
            //for (int i = 0; i < bodyOptimizationIterations; ++i)
            //{
            //    simulation.BodyLayoutOptimizer.IncrementalOptimize(simulation.BufferPool, threadDispatcher);
            //}
            //timer.Stop();
            //var optimizationTime = timer.Elapsed.TotalSeconds;
            //Console.WriteLine($"Finished {bodyOptimizationIterations} body optimizations, time (ms): {optimizationTime * 1e3}, per iteration (us): {optimizationTime * 1e6 / bodyOptimizationIterations}");

            ////Note that constraint optimization should be performed after body optimization, since body optimization moves the bodies- and so affects the optimal constraint position.
            //simulation.ConstraintLayoutOptimizer.OptimizationFraction = 0.044f;
            //int constraintOptimizationIterations = 8192;

            //simulation.ConstraintLayoutOptimizer.Update(simulation.BufferPool, threadDispatcher);//prejit
            //timer.Restart();
            //for (int i = 0; i < constraintOptimizationIterations; ++i)
            //{
            //    simulation.ConstraintLayoutOptimizer.Update(simulation.BufferPool, threadDispatcher);

            //}
            //timer.Stop();
            //Console.WriteLine($"Finished constraint optimizations, time (ms): {timer.Elapsed.TotalMilliseconds}" +
            //    $", per iteration (us): {timer.Elapsed.TotalSeconds * 1e6 / constraintOptimizationIterations}");
            const float inverseDt = 60f;
            const float dt = 1 / inverseDt;
            const int iterationCount = 32;
            const int frameCount = 256;
            simulation.Solver.IterationCount = iterationCount;

            //If we don't initialize the inertias in a per-frame update, we must do so explicitly.
            simulation.Bodies.LocalInertias.CopyTo(0, ref simulation.Bodies.Inertias, 0, simulation.Bodies.LocalInertias.Length);

            //Technically we're not doing any position integration or collision detection yet, so these frames are pretty meaningless.
            timer.Reset();

            //var threadPool = new NotQuiteAThreadPool();
            Console.WriteLine($"Using {threadDispatcher.ThreadCount} workers.");
            double solveTime = 0;
            for (int frameIndex = 0; frameIndex < frameCount; ++frameIndex)
            {
                var energyBefore = simulation.Bodies.GetBodyEnergyHeuristic();
                //Update the penetration depths associated with the constraints.
                //This simulates actual position integration and repeated contact detection, allowing the constraints to properly spring.
                for (int i = 0; i < constraintHandles.Length; ++i)
                {
                    simulation.Solver.GetConstraintReference(constraintHandles[i], out var constraint);
                    var typeBatch = constraint.TypeBatch as Contact4TypeBatch;

                    BundleIndexing.GetBundleIndices(constraint.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
                    ref var bodyReferences = ref typeBatch.BodyReferences[bundleIndex];
                    var indexA = GatherScatter.Get(ref bodyReferences.IndexA, innerIndex);
                    var indexB = GatherScatter.Get(ref bodyReferences.IndexB, innerIndex);

                    var velocityA = simulation.Bodies.Velocities[indexA].Linear;
                    var velocityB = simulation.Bodies.Velocities[indexB].Linear;
                    var relativeVelocity = velocityA - velocityB;
                    Vector3 normal;
                    unsafe { var mmhmm = &normal; }
                    GatherScatter.GetLane(ref typeBatch.PrestepData[bundleIndex].Normal.X, innerIndex, ref normal.X, 3);
                    var penetrationChange = -dt * Vector3.Dot(relativeVelocity, normal);
                    ref var penetrationDepth = ref GatherScatter.Get(ref typeBatch.PrestepData[bundleIndex].PenetrationDepth0, innerIndex);
                    penetrationDepth += penetrationChange;
                    GatherScatter.Get(ref typeBatch.PrestepData[bundleIndex].PenetrationDepth1, innerIndex) += penetrationChange;
                    GatherScatter.Get(ref typeBatch.PrestepData[bundleIndex].PenetrationDepth2, innerIndex) += penetrationChange;
                    GatherScatter.Get(ref typeBatch.PrestepData[bundleIndex].PenetrationDepth3, innerIndex) += penetrationChange;

                    if (i == 0)
                        //if (penetrationDepth > 0.2)
                        Console.WriteLine($"manifold[{i}] penetration: {penetrationDepth}, va: {velocityA}, vb: {velocityB}");

                }

                //Apply some gravity so we can simulate sorta-kinda stacking.
                var bodyBundleCount = simulation.Bodies.Count >> BundleIndexing.VectorShift;
                var impulse = -10 * dt;
                for (int i = 0; i < bodyBundleCount; ++i)
                {
                    //(We're using an impulse rather than direct velocity change just because we're being lazy about the kinematics.)
                    simulation.Bodies.Velocities[i].Linear.Y += simulation.Bodies.LocalInertias[i].InverseMass * impulse;
                }
                timer.Start();
                simulation.Solver.Update(dt);
                //simulation.Solver.MultithreadedUpdate(threadDispatcher, simulation.BufferPool, dt, inverseDt);
                timer.Stop();
                var energyAfter = simulation.Bodies.GetBodyEnergyHeuristic();
                Console.WriteLine($"Body energy {frameIndex}: {energyAfter}, delta: {energyAfter - energyBefore}");
            }

            Console.WriteLine($"Time (ms): {(1e3 * timer.Elapsed.TotalSeconds)}");
            Console.WriteLine($"Solve time (ms): {1e3 * solveTime}");


            threadDispatcher.Dispose();
            simulation.BufferPool.Clear();

        }



    }
}

