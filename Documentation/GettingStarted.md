# Getting Started

The best place to start is the demos in the `Demos.sln` solution. Jump into one in the [`Demos/Demos/`](../Demos/Demos/) folder and start playing around.

The [`Demo`](../Demos/Demo.cs) type is a very simple helper class that contains a few properties and functions used across all demos. It creates a `BufferPool` and `ThreadDispatcher` and provides a basic `Update` implementation that just calls `Simulation.Timestep` once with a fixed duration. A couple of alternative timestep examples are also provided in the comments.

Each `Demo` implementation creates its own `Simulation`. Some `Demo` types have extra input or rendering logic; input and rendering are fully separate from the actual physics library.

The [`SimpleSelfContainedDemo`](../Demos/Demos/SimpleSelfContainedDemo.cs) shows how to set up a simple simulation without relying on any of the Demos-provided infrastructure.

## Building a Simulation

`Simulation.Create` builds a `Simulation` instance, but has some unusual callback parameters- they take the form of interface-implementing structs.

The `TNarrowPhaseCallbacks` parameter handles collision-related callbacks, giving the user a way to control collision filtering, contact manifolds, materials, and a path to implementing things like collision events.

`TPoseIntegratorCallbacks` provides control over per-body velocity integration behavior. It's typically used for implementing gravity or damping. 

Examples: 
1. [`DemoCallbacks`](../Demos/DemoCallbacks.cs) shows the frequently used callbacks, including damping and unidirectional gravity and a basic passthrough `INarrowPhaseCallbacks`.
2. [`PlanetDemo`](../Demos/Demos/PlanetDemo.cs) shows how to create a planet-like gravity source.
3. [`RagdollDemo`](../Demos/Demos/RagdollDemo.cs) contains a slightly more advanced `INarrowPhaseCallbacks` collision filtering approach based on bitfields. Used to stop connected pieces of the ragdolls from colliding.
4. [`ContactEventsDemo`](../Demos/Demos/ContactEventsDemo.cs) shows one possible way to collect contact data from the `INarrowPhaseCallbacks` and use it to generate contact events.

Note that `TNarrowPhaseCallbacks` and `TPoseIntegratorCallbacks` are required to be structs and are provided with generic parameters rather than directly as interfaces. This is because the compiler has enough knowledge (and is forced) to avoid virtual dispatch and, when appropriate, inline the callbacks. Given that many of the callbacks are called with extremely high frequency, this can add up.

A similar pattern is used in many places across the engine. You can think about these as compile time delegates or closures, just with nastier syntax.

`SolveDescription` describes how the simulation should schedule updates. You can set the number of velocity iterations and substeps that occur in each simulation timestep. For simulations with difficult constraint configurations, using more substeps can help stabilize the simulation far more cheaply than increasing velocity iterations can. For advanced use cases, you can schedule the number of velocity iterations for each substep individually.

There's also a fallback batch threshold, which you can safely leave at the default value almost always- it's the number of synchronized constraint batches that the simulation will create before falling back to a special case solve when individual bodies have excessive numbers of constraints connected to them. 

`Simulation.Create` also has a couple of optional parameters: initial allocation sizes to pull from the resource pool (to avoid unnecessary resizing later), and the `ITimestepper` which defines the order of stage execution within the engine.

The engine includes only one `ITimeStepper` type out of the box, which gets used if no other `ITimestepper` is provided: the [`DefaultTimestepper`](../BepuPhysics/DefaultTimestepper.cs). It checks candidates for sleeping, computes bounding boxes, performs collision detection, solves constraints (which includes any necessary body velocity/pose integration), then does some incremental optimization work on internal data structures. There are callbacks between stages that can be hooked into. A custom `ITimestepper` could be provided that changes what stages execute or their order.

## Timestepping/updating

`Simulation.Timestep` pushes the simulation forward by the given amount of time. If a `IThreadDispatcher` is provided, it is used to multithread the simulation.

The `Simulation` does not do anything fancy internally with temporal accumulation or anything else. Each `Timestep` simulates exactly the amount of time specified.

Examples of other timestepping strategies, like accumulating time and simulating as many timesteps of a fixed duration as necessary, are shown in [`Demo.Update`](../Demos/Demo.cs#L40).

It is recommended that the time step duration provided to `Simulation.Timestep` is always the same. Wildly varying timesteps can introduce instability. One-off changes or gradual changes can work well enough, but try to avoid jumping around constantly. 

For varying timestep durations, `Solver.ScaleAccumulatedImpulses` can be used to scale the cached constraint solution by the ratio of the previous duration to the current duration. It's not free and it won't guarantee stability, but it can help.

## Bodies

In v2, 'body' refers to a basic collidable mobile object that can be connected to constraints.

To create a body, pass a `BodyDescription` to `Simulation.Bodies.Add`. It returns a handle that uniquely identifies the body for as long as it belongs to the simulation. The `BodyDescription` specifies the initial pose, velocity, inertia, collidable, and activity state.

The `BodyDescription` type offers static factory functions for convenience.

Creating a body with zero inverse mass and inverse inertia will create a kinematic body, a body that will not change its velocity in response to any force. (`BodyDescription.CreateKinematic` only differs from `BodyDescription.CreateDynamic` in that it provides a zeroed out BodyInertia.) A dynamic body can be given a zeroed inverse inertia tensor to lock rotation only. Individual rows can also be zeroed out to lock rotation around the related local body axes.

The `CollidableDescription` takes a reference to a shape allocated in the `Simulation.Shapes` set. Shapes are allocated independently from bodies. Multiple bodies can refer to the same allocated shape. Collidables are also allowed to refer to no shape at all which can be useful for creating some constraint systems.

Note that there is no internal list of `BodyDescription` instances, nor a single "Body" type anywhere. Instead, body properties are split across different buffers depending on internal memory access patterns. Further, there are multiple `BodySet` instances, each with their own set of buffers. These buffers are set up for efficient internal access, with the first `BodySet` storing all active bodies and the later sets containing inactive body data. The `BodyDescription` is decomposed into these separate pieces upon being added.

To access an existing body's data, the body's current memory location must be looked up using the handle returned by the `Add` call. The `BodyReference` convenience type can make this a little easier by hiding the lookup process. You can get a `BodyReference` by indexing: `Simulation.Bodies[BodyHandle]`.

Bodies may move around in memory during execution or when other bodies are added, removed, awoken, or slept. Holding onto the raw memory location through one of these changes may result in the pointer pointing to undefined data. The `BodyReference` type performs lookups on demand, and remains valid so long as the wrapped `BodyHandle` does.

## Statics

Statics are similar to bodies but don't have velocity, inertia, or activity states. They're just immobile collidable shapes, ideal for level geometry.

Statics are computationally cheap ([and will get even cheaper](https://github.com/bepu/bepuphysics2/issues/7)). Feel free to have thousands of them.

To create a static object, pass a `StaticDescription` to `Simulation.Statics.Add`.

## Constraints

Constraints can be used to control the relative motion of bodies. There are a [whole bunch of them](../BepuPhysics/Constraints). Like bodies and statics, constraints are created by building a description and passing it to `Simulation.Solver.Add`.

Some examples of constraints in the demos include:
1. [`RagdollDemo`](../Demos/Demos/RagdollDemo.cs) is a... demo of ragdolls. 
2. [`CarDemo`](../Demos/Demos/CarDemo.cs) shows how to build a simple constraint based car (and bad AI drivers).
3. [`RopeStabilityDemo`](../Demos/Demos/RopeStabilityDemo.cs) shows some constraint configuration failure modes and how to fix them.
4. [`NewtDemo`](../Demos/Demos/NewtDemo.cs) shows how to make a squishy newt.
5. [`ClothDemo`](../Demos/Demos/ClothDemo.cs) shows how to make sheets of cloth with different properties.

Existing raw constraint data is more difficult to access than body data. There is a similar handle->memory location lookup, but the data itself is stored a few layers deep in array-of-structures-of-arrays format for performance. Pulling data out of this representation is not very convenient, so the `Solver` has `ApplyDescription` and `GetDescription` for accessing constraint data. Custom descriptions can be created to access only subsets of a constraint's full data.

## Queries

The engine supports scene-wide queries through `Simulation.RayCast` and `Simulation.Sweep`. Sweeps support both linear and angular motion for convex shapes. Both functions make use of hit handlers- `IRayHitHandler` and `ISweepHitHandler`. The handlers can filter out objects and respond to found impacts.

The [`Grabber`](../Demos/Grabber.cs), [`RayCastingDemo`](../Demos/Demos/RayCastingDemo.cs) and [`SweepDemo`](../Demos/Demos/SweepDemo.cs) have some examples of these queries, plus some other more advanced cases.

The broad phase acceleration structures can be [directly queried](../BepuPhysics/CollisionDetection/BroadPhase_Queries.cs) by ray casts, bounding boxes, and swept bounding boxes. 

## Memory

Almost all of the internally used memory is pulled from a custom allocator working with pinned memory. Almost all stages have some form of memory management API for preallocating, compacting, or disposing resources.

The `Simulation` has a few top-level functions to help orchestrate all the different stage allocations, including `Clear`, `EnsureCapacity`, `Resize`, and `Dispose`. Most use cases get by with the default initial allocation sizes and then Disposing upon completion.

Notably, the `Simulation` does not have its own allocator. It relies on `BufferPool` instances provided by the user. All of the resource management APIs on the `Simulation` and its stages operate on these pools. In other words, if you're okay with just clearing the `BufferPool` instances that a simulation relies on and letting the GC pick up the pieces, you don't technically have to `Clear` or `Dispose` the `Simulation` itself. (Of course, that `Simulation` instance should not be used after it has had the rug pulled out from under it.)

Failure to return resources to a `BufferPool` or to clear an unused `BufferPool` can cause memory leaks.

## Asserts and debugging

Many invalid inputs and error states are left unchecked in release mode. Most error checking is performed using `Debug.Assert`, requiring compilation with the `DEBUG` symbol. Including `CHECKMATH` (as the `Debug` configuration does by default) is also helpful for tracking down NaN-explosions sometimes.

The `LEAKDEBUG` symbol can be added to help track down `BufferPool` related memory leaks. It is extremely expensive, but adds deep diagnostics for every allocation.

## A note on design intent

As the above suggests, the engine uses a lot of idioms which are historically uncommon in C#. There are value types and refs everywhere, SIMD out the wazoo, object data is split and packed into performance-optimized storage formats, APIs directly expose the underlying data, language features are abused for metaprogramming, and the engine generally lets you break stuff.

In summary, it's a very low level API. The intent is to maximize performance, then expose as much as possible to let application-specific convenient abstractions be built on top.

All of this puts a heavier burden on users. They must be familiar with value type semantics, new performance minded language features, pointers, and all sorts of other unusual-for-C# stuff. If you've got questions, feel free to post them in the [discussions](https://github.com/bepu/bepuphysics2/discussions).