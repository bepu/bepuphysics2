# Substepping

Each call to `Simulation.Timestep(dt, ...)` simulates one frame with duration equal to `dt`. In the [`DefaultTimestepper`](../BepuPhysics/DefaultTimestepper.cs) (which, as the name implies, is the `ITimestepper` implementation used if no other is specified) executes a frame like so:

```cs
simulation.Sleep();
simulation.PredictBoundingBoxes(dt, threadDispatcher);
simulation.CollisionDetection(dt, threadDispatcher);
simulation.Solve(dt, threadDispatcher);
simulation.IncrementallyOptimizeDataStructures(threadDispatcher);
```
There's only one execution of collision each stage per call to `Timestep`, each responsible for covering the specified `dt`.

The solver, however, can be configured to take multiple timesteps internally for every call to `Simulation.Timestep`. These are called substeps. When creating a simulation:
