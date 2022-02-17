# 2.4

At a high level, 2.4 was a solver revamp. Data layout and access patterns were significantly changed and dispatch logic was reworked. Substepping is massively cheaper now, and even simulations without any need of substepping will still benefit significantly. Whole frame speedups in excess of 2x were not uncommon in benchmarks. Scenes that were especially difficult for the solver in 2.3 were observed to be over 3.5x faster in 2.4.

## API Breaking Changes

1. The library now depends on .NET 6.
1. `Simulation.Create` no longer requires an `ITimestepper`, though one can still be provided. `PositionFirstTimestepper` and `PositionLastTimestepper` no longer exist; the only built-in `ITimestepper` implementation is the `DefaultTimestepper`. This is a result of 2.4 moving entirely to an embedded substepping solver.
1. `Simulation.Create` now takes a `SolveDescription`. It can be used to configure the number of substeps, velocity iterations, and synchronized batches. There exist helper implicit casts; for example, passing an integer will simply use the value as the number of substeps, with the velocity iteration count set to 1 and `FallbackBatchThreshold` set to the default. `Solver.IterationCount` property renamed to `Solver.VelocityIterationCount`. See the [substepping documentation](Substepping.md) for more information.
1. `IPoseIntegratorCallbacks.IntegrateVelocity` now exposes multiple lanes of bodies in SIMD vectors, rather than a single body at a time. It also exposes two new properties, `IntegrateVelocityForKinematics` and `AllowSubstepsForUnconstrainedBodies`. If `IntegrateVelocityForKinematics` is false, then `IntegrateVelocity` will not include any kinematic bodies in active lanes. This is convenient when applying gravity; if only dynamics are ever invoked, then there's no need to check kinematicity prior to applying gravity. If `AllowSubstepsForUnconstrainedBodies` is true, then bodies with no constraints will have their velocities and poses integrated for every substep; if false, they will only be integrated a single time for the full frame duration. All constrained bodies are substepped if the solver is substepped. The vectorized nature of this callback is probably going to be annoying (and/or confusing) for a lot of people; sorry about that. Maintaining a scalar callback added too much overhead. You can build one on top, though!
1. When constructing a collidable description, the overload that takes a speculative margin now means the *maximum* speculative margin. 2.4 uses adaptive speculative margins that can shrink or expand according to velocity. You will usually see the same behavior, just cheaper. If you want to match the old behavior as much as possible, set both the minimum and maximum bounds to the same value. Leaving speculative margins bounds at [0, float.MaxValue] is a good default now. Note that there are some new implicit casts relevant to `BodyDescription` creation that can make things shorter, including just passing a shape index directly which defaults to passive continuity with [0, float.MaxValue] adaptive speculative margin. See [continuous collision detection documentation](ContinuousCollisionDetection.md) for more information.
1. Statics no longer have any configurable speculative margin settings and do not take a `CollidableDescription` in their constructor. They still have a `Continuity` field if a static should a static need continuous collision detection to be enabled. All information related to a static is now in one spot, stored in the `Statics.StaticsBuffer`.
1. `ContinuousDetectionSettings` renamed to `ContinuousDetection`.
1. `INarrowPhaseCallbacks.AllowContactGeneration` now exposes `ref float speculativeMargin`. Most use cases can safely ignore this completely, but if you find yourself wanting fine grained control over the speculative margin, that's now exposed.
1. `IConvexShape.ComputeInertia` now returns instead of using an out parameter. Similar changes applied to things like `Mesh.ComputeClosedInertia` and `Mesh.ComputeOpenInertia`.
1. Some callbacks that previously had `struct` generic constraints now require `unmanaged`, like `INarrowPhaseCallbacks.ConfigureContactManifold`.
1. `BodyOptimizer`/`ConstraintOptimizer` stages no longer exist, and their profiler entries have been removed.
1. `Solver.ApplyDescription` no longer requires a ref parameter.
1. `BodyInertias` and `BodyVelocities` renamed to `BodyInertiaWide` and `BodyVelocityWide` to match naming convention of other wide types.
1. `IThreadDispatcher` now takes an additional `maximumWorkerCount` parameter. Specifying a maximum worker count lower than the `IThreadDispatcher.ThreadCount` may allow the implementation to do less work. In practice, the `BepuUtilities.ThreadDispatcher` uses this to significantly reduce dispatch overhead for low job count use cases.
1. All body state used by the solver now bundled together into `BodySet.SolverStates`. Includes pose, velocity, and inertia.
1. Constraint type batches no longer have a 'projection' buffer; anything loaded from it is now recalculated on the fly.

## Other Changes

1. Added `CenterDistanceLimit`! Useful for clothy stuff.

2. Added `AngularAxisGearMotor`; transforms angular motion with a multiplier, somewhat like a gear ratio.

3. `RigidPose`, `BodyVelocity`, `CollidableDescription`, and `BodyActivityDescription` all now have helper implicit casts to optionally make configuration a little less syntax-noisy.

4. You can now get a `BodyReference` or `StaticReference` from their respective collections by using `Simulation.Bodies[BodyHandle]` and `Simulation.Statics[StaticHandle]`.

5. The presence of a kinematic body in a constraint batch will no longer block another constraint attached to that kinematic body from being added to the constraint batch. The velocity of kinematics in constraint batches are treated as read-only. This will improve performance on simulations where a kinematic body has a ton of constraints associated with it.

6. Broad phase dispatches combined. The long-waiting broad phase revamp is still waiting; this just reduces dispatch related overhead slightly.

7. Code paths dependent on trigonometric approximations have had their accuracy improved by a few orders of magnitude. `AngularHinge`, `TwistServo`, `QuaternionWide.GetAxisAngleFromQuaternion`, and orientation integration are all improved. The improvement in orientation integration in particular helps avoid contact drift and helps integration with angular momentum conservation. 

8. Constraints in the fallback batch now have sequentialized execution, rather than using a jacobi solver. This improves simulation quality in pathological cases where a single dynamic body is associated with tons of constraints, but that situation is still best avoided. Anything that ends up in the fallback batch will cost more by virtue of being executed sequentially. More information and potential future improvements here: [Fallback batch improvements · Issue #162 · bepu/bepuphysics2 (github.com)](https://github.com/bepu/bepuphysics2/issues/162).

9. Contact constraints now solve friction last. In simulations that don't let the solver reach an equilibrium solution, you might notice that an unstable stack of bodies exhibits more tangential jitter and less penetration jitter than it used to. In a simulation that allows enough time to solve the constraints, there should be no visible difference. (This change was motivated by the fact that penetration has a corrective feedback loop via depth, while friction is open ended. Giving friction the final word slightly reduces drift.)

10. `VolumeConstraint` had a warmstarting jacobian bug; it's fixed. Should be higher quality (and stiffer, if configured to be stiff).

11. `Hinge` and `SwivelHinge` never made use of accumulated impulses, oops. Should be higher quality (and stiffer, if configured to be stiff).

12. `SwivelHinge` no longer has a unguarded NaNsplode codepath.

13. Fixed a bug in `AngularMotor` that used the wrong inertia. 

14. Guarded against a sphere-cylinder division by zero.

15. Fixed a capsule-cylinder determinism bug.

16. Fixed a triangle-cylinder determinism bug.

17. Fixed capsule-cylinder contact generation bug.

18. Fixed cylinder-cylinder contact generation bug.

19. Box-box now has a bundle early out.

20. All triangle-involving collision pairs now consistent in triangle degeneracy testing.

21. All triangle-involving collision pairs now handle collisions with normals pointing nearly perpendicular to the triangle plane much more gracefully.

22. Fixed a bunch of other tiny weird triangle collision bugs too.

23. `MeshReduction` revamped a bit. It should catch more boundary bumps, and it no longer tries to do a quadratically catastrophic operation when the number of triangles being considered is large. At a certain (extreme) point, it now simply doesn't bother with boundary smoothing at all.

    
