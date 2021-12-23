# What's substepping?
Substepping integrates body velocities and positions and solves constraints more than once per call to `Simulation.Timestep`. For some simulations with complex constraint configurations, high stiffness, or high mass ratios, substepping is the fastest way to find a stable solution.

You can configure a simulation to use substepping by passing a `SolveDescription` to `Simulation.Create` that has more than one substep. For example, to create a simulation that uses 8 substeps and 1 velocity iteration per substep:
```cs
var simulation = Simulation.Create(BufferPool, new NarrowPhaseCallbacks(), new PoseIntegratorCallbacks(), new SolveDescription(velocityIterationCount: 1, substepCount: 8));
```

# How substepping fits into a timestep
Each call to `Simulation.Timestep(dt, ...)` simulates one frame with duration equal to `dt`. In the [`DefaultTimestepper`](../BepuPhysics/DefaultTimestepper.cs) (which, as the name implies, is the `ITimestepper` implementation used if no other is specified) executes a frame like so:
```cs
simulation.Sleep();
simulation.PredictBoundingBoxes(dt, threadDispatcher);
simulation.CollisionDetection(dt, threadDispatcher);
simulation.Solve(dt, threadDispatcher);
simulation.IncrementallyOptimizeDataStructures(threadDispatcher);
```
There's only one execution of collision each stage per call to `Timestep`, each responsible for covering the specified `dt`.

When configured to use more than one substep, `Simulation.Solve` will integrate bodies and solve constraints as if `Simulation.Timestep` was called `Simulation.Solver.SubstepCount` times, each time with a duration equal to `dt / Simulation.Solver.SubstepCount`.

The difference between using substepping and explicitly calling `Timestep` more frequently is that none of the other stages run during substeps. For example, contact constraints are incrementally updated in an approximate way, but full collision detection is not run. This allows substeps to be much faster than full timesteps.

# Velocity iteration scheduling
While the simplest approach is to use the same number of velocity iterations for all substeps, they are allowed to vary. You can provide a `VelocityIterationScheduler` callback in the `SolveDescription` to define how many velocity iterations each substep should take. There's also a helper that takes a span of integers defining the velocity iteration counts to use for each substep.
```cs
var simulation = Simulation.Create(BufferPool, new NarrowPhaseCallbacks(), new PoseIntegratorCallbacks(), new SolveDescription(new[] {2, 1, 1}));
```
The above snippet would use 3 substeps with 2 velocity iterations on the first substep, then one velocity iteration on the second and third substeps.

# Callbacks

The solver exposes events that fire at the beginning and end of each substep: `SubstepStarted` and `SubstepEnded`. These events are called from worker thread 0 in the solver's thread dispatch; the dispatch does not end in between substeps to keep overhead low. 

(Note that attempting to dispatch multithreaded work from the same `IThreadDispatcher` instance that dispatched the solver's workers requires that the `IThreadDispatcher` implementation is reentrant. The demos `SimpleThreadDispatcher` is not.)

