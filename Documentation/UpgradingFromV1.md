# Upgrading from v1

v2 is a complete rewrite and is about as different from v1 as any unrelated physics engine. Reading the [Getting Started documentation](GettingStarted.md) to get an overview is recommended. The following maps concepts and features in v1 to their equivalents in v2.

Some of this might look way more complicated, and it might seem like features were removed for no reason, but trust me it's great! For some definitions of great! The API just avoids hiding performance-related things and punts responsibility for application-specific convenience features to the user. Stuff that looks the same as the v1 version can be created pretty easily, it's just not in the core library.

| Concept | v1| v2 |
| --- | --- | --- |
| Top level type containing all execution stages and all existing objects | `Space` | `Simulation` | 
| Mobile dynamic and kinematic objects | `Entity` | Body, but there is no `Body` type- see `Simulation.Bodies` to allocate, access, and delete bodies. Creating a body using `Simulation.Bodies.Add` returns a handle that uniquely identifies the body for the duration of its existence, and `Simulation.Bodies.HandleToLocation` finds the current memory location of a body. `BodyReference` can be used to handle lookups for you.|
| Mobile object properties | `Entity` properties, like `Position` and `LinearVelocity` | Create a `BodyReference` from the body handle, then access properties like `Pose` and `Velocity`. Can also manually perform the lookup into the `Simulation.Bodies` sets and their raw property buffers. |
| Collision events | `entity.CollisionInformation.Events` | No out of the box events; [`ContactEventsDemo`](../Demos/Demos/ContactEventsDemo.cs) shows how to use narrow phase callbacks to create events. |
| Enumerating existing collisions | `entity.CollisionInformation.Pairs` | Collision data is not explicitly cached anywhere. Narrow phase callbacks can be used to collect collision information. Collision-created contact constraints (and all other connected constraints) can be enumerated using the Constraints body property. See the [`SolverContactEnumerationDemo`](../Demos/Demos/SolverContactEnumerationDemo.cs) for an example of enumerating contact constraints. |
| Collision filtering | `e.CollisionInformation.CollisionRules` and `CollisionRules` static functions | `INarrowPhaseCallbacks` has `AllowContactGeneration` and `ConfigureContactManifold` which return a boolean that controls whether narrow phase testing and constraint generation should proceed. See [`RagdollDemo`](../Demos/Demos/RagdollDemo.cs) for an example of collision filtering. |
| Custom gravity | `entity.Gravity` | `IPoseIntegratorCallbacks` can be used to implement any form of gravity or other per-body velocity influence. See [PlanetDemo](../Demos/Demos/PlanetDemo.cs) for an example. | 
| Object velocity damping | `entity.LinearDamping` and `entity.AngularDamping` | `IPoseIntegratorCallbacks` again- damping is just a velocity influence. See [DemoCallbacks](../Demos/DemoCallbacks.cs) for an example. |
| Scene-wide ray casts | `Space.RayCast` or `s.BroadPhase.QueryAccelerator.RayCast` for AABB-only testing | `Simulation.RayCast` or `Simulation.BroadPhase.RayCast` for AABB-only testing |
| Scene-wide sweep tests | `Space.ConvexCast` | `Simulation.Sweep`, which supports angular motion in the sweep as well, or `Simulation.BroadPhase.Sweep` for AABB-only testing |
| Bounding box queries | `s.BroadPhase.QueryAccelerator.GetEntries` | `Simulation.BroadPhase.GetOverlaps` |
| Collidable composed of a bunch of triangles | `StaticMesh` or `InstancedMesh` | `Mesh` |
| Triangulated heightmap collidable | `Terrain` | There is no dedicated heightmap type at the moment, so just `Mesh` |
| Sphere collision shape | `SphereShape` | `Sphere` |
| Capsule collision shape | `CapsuleShape` | `Capsule` |
| Box collision shape | `BoxShape` | `Box` |
| Triangle collision shape | `TriangleShape` | You can probably guess, `Triangle` |
| Cylinder collision shape | `CylinderShape` | `Cylinder` | 
| Convex hull collision shape | `ConvexHullShape` | `ConvexHull` |
| Cone collision shape | `ConeShape` | N/A- consider a `ConvexHull` approximation |
| Weird combination shape in minkowski space that no one ever used | `MinkowskiSumShape` | N/A- I'd say "consider a `ConvexHull` approximation," but I'm pretty sure no one will ever want to do this |
| Combination shape that acts like a convex hull around arbitrary subshapes | `WrappedShape` | N/A- consider a `ConvexHull` approximation. Not sure if anyone ever used this one either. | 
| Affinely transformable wrapper around any other convex shape | `TransformableShape` | N/A- consider a `ConvexHull` approximation |
| Multiple convex shapes bundled into one shape supporting concavity | `CompoundShape` | `Compound` for shapes with only a few pieces; `BigCompound` for ones that have enough children to benefit from an acceleration structure |
| Static-only optimization for a bunch of other convex shapes | `StaticGroup` | Doesn't exist because it's not necessary; just use `Simulation.Statics.Add` to toss them directly into the simulation. The broad phase can handle it just fine. |
| Should you use really detailed concave triangle soup meshes for mobile objects? | No | Still no! v2 might be way faster than v1, but that doesn't mean you should just throw all those spare cycles right into the garbage can! |
| Constraints | [Oof there's a lot of them](https://github.com/bepu/bepuphysics1/tree/master/BEPUphysics/Constraints) | [Here's another bunch](../BepuPhysics/Constraints); main differences are the addition of a couple of simultaneously solved combos like `Hinge` and `SwivelHinge` which will be more stable than their old `SolverGroup` v1 versions, a couple of new cloth/deformable-helpful constraints, and the removal of questionable constraints like the `EllipseSwingLimit`. |
| Constraint springiness (for those with position goals) | `constraint.SpringSettings.Stiffness` and `constraint.SpringSettings.Damping` | `SpringSettings` set in the constraint description, which contains `Frequency` and `DampingRatio` properties. `Frequency` is the undamped frequency of oscillation of the constraint, and `DampingRatio` is the ratio of the damping to critical damping, 0 being undamped, 1 being critically damped, and higher values being overdamped. Try to avoid using `Frequency` values higher than around half the update rate. That is, if you're updating at 60hz, 30 is a generally stable upper bound for `Frequency`. Higher frequencies in complex constraint configurations may require faster update rates or using the `SubsteppingTimestepper`. Note that, given a mass, frequency, and damping ratio, an equivalent stiffness constant and damping constant can be computed (and vice versa). I should probably add a helper for that at some point. |
| Buoyancy zone | `FluidVolume` | N/A, may show up later in a less bad form |
| Mesh-based collision detector with containment testing | `DetectorVolume` | No support for out of the box containment events, but a `Mesh` and narrow phase callbacks can perform a pretty similar job. |