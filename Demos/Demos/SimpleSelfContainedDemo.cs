﻿using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Demos.Demos
{
    /// <summary>
    /// Shows a completely isolated usage of the engine without using any of the other demo physics-related types.
    /// </summary>
    public static class SimpleSelfContainedDemo
    {
        //The simulation has a variety of extension points that must be defined. 
        //The demos tend to reuse a few types like the DemoNarrowPhaseCallbacks, but this demo will provide its own (super simple) versions.
        //If you're wondering why the callbacks are interface implementing structs rather than classes or events, it's because 
        //the compiler can specialize the implementation using the compile time type information. That avoids dispatch overhead associated
        //with delegates or virtual dispatch and allows inlining, which is valuable for extremely high frequency logic like contact callbacks.
        struct NarrowPhaseCallbacks : INarrowPhaseCallbacks
        {
            /// <summary>
            /// Performs any required initialization logic after the Simulation instance has been constructed.
            /// </summary>
            /// <param name="simulation">Simulation that owns these callbacks.</param>
            public void Initialize(Simulation simulation)
            {
                //Often, the callbacks type is created before the simulation instance is fully constructed, so the simulation will call this function when it's ready.
                //Any logic which depends on the simulation existing can be put here.
            }

            /// <summary>
            /// Chooses whether to allow contact generation to proceed for two overlapping collidables.
            /// </summary>
            /// <param name="workerIndex">Index of the worker that identified the overlap.</param>
            /// <param name="a">Reference to the first collidable in the pair.</param>
            /// <param name="b">Reference to the second collidable in the pair.</param>
            /// <param name="speculativeMargin">Reference to the speculative margin used by the pair.
            /// The value was already initialized by the narrowphase by examining the speculative margins of the involved collidables, but it can be modified.</param>
            /// <returns>True if collision detection should proceed, false otherwise.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
            {
                //Before creating a narrow phase pair, the broad phase asks this callback whether to bother with a given pair of objects.
                //This can be used to implement arbitrary forms of collision filtering. See the RagdollDemo or NewtDemo for examples.
                //Here, we'll make sure at least one of the two bodies is dynamic.
                //The engine won't generate static-static pairs, but it will generate kinematic-kinematic pairs.
                //That's useful if you're trying to make some sort of sensor/trigger object, but since kinematic-kinematic pairs
                //can't generate constraints (both bodies have infinite inertia), simple simulations can just ignore such pairs.

                //This function also exposes the speculative margin. It can be validly written to, but that is a very rare use case.
                //Most of the time, you can ignore this function's speculativeMargin parameter entirely.
                return a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic;
            }

            /// <summary>
            /// Chooses whether to allow contact generation to proceed for the children of two overlapping collidables in a compound-including pair.
            /// </summary>
            /// <param name="workerIndex">Index of the worker thread processing this pair.</param>
            /// <param name="pair">Parent pair of the two child collidables.</param>
            /// <param name="childIndexA">Index of the child of collidable A in the pair. If collidable A is not compound, then this is always 0.</param>
            /// <param name="childIndexB">Index of the child of collidable B in the pair. If collidable B is not compound, then this is always 0.</param>
            /// <returns>True if collision detection should proceed, false otherwise.</returns>
            /// <remarks>This is called for each sub-overlap in a collidable pair involving compound collidables. If neither collidable in a pair is compound, this will not be called.
            /// For compound-including pairs, if the earlier call to AllowContactGeneration returns false for owning pair, this will not be called. Note that it is possible
            /// for this function to be called twice for the same subpair if the pair has continuous collision detection enabled; 
            /// the CCD sweep test that runs before the contact generation test also asks before performing child pair tests.</remarks>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
            {
                //This is similar to the top level broad phase callback above. It's called by the narrow phase before generating subpairs between children in parent shapes. 
                //This only gets called in pairs that involve at least one shape type that can contain multiple children, like a Compound.
                return true;
            }

            /// <summary>
            /// Provides a notification that a manifold has been created for a pair. Offers an opportunity to change the manifold's details. 
            /// </summary>
            /// <param name="workerIndex">Index of the worker thread that created this manifold.</param>
            /// <param name="pair">Pair of collidables that the manifold was detected between.</param>
            /// <param name="manifold">Set of contacts detected between the collidables.</param>
            /// <param name="pairMaterial">Material properties of the manifold.</param>
            /// <returns>True if a constraint should be created for the manifold, false otherwise.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
            {
                //The IContactManifold parameter includes functions for accessing contact data regardless of what the underlying type of the manifold is.
                //If you want to have direct access to the underlying type, you can use the manifold.Convex property and a cast like Unsafe.As<TManifold, ConvexContactManifold or NonconvexContactManifold>(ref manifold).

                //The engine does not define any per-body material properties. Instead, all material lookup and blending operations are handled by the callbacks.
                //For the purposes of this demo, we'll use the same settings for all pairs.
                //(Note that there's no 'bounciness' or 'coefficient of restitution' property!
                //Bounciness is handled through the contact spring settings instead. Setting See here for more details: https://github.com/bepu/bepuphysics2/issues/3 and check out the BouncinessDemo for some options.)
                pairMaterial.FrictionCoefficient = 1f;
                pairMaterial.MaximumRecoveryVelocity = 2f;
                pairMaterial.SpringSettings = new SpringSettings(30, 1);
                //For the purposes of the demo, contact constraints are always generated.
                return true;
            }

            /// <summary>
            /// Provides a notification that a manifold has been created between the children of two collidables in a compound-including pair.
            /// Offers an opportunity to change the manifold's details. 
            /// </summary>
            /// <param name="workerIndex">Index of the worker thread that created this manifold.</param>
            /// <param name="pair">Pair of collidables that the manifold was detected between.</param>
            /// <param name="childIndexA">Index of the child of collidable A in the pair. If collidable A is not compound, then this is always 0.</param>
            /// <param name="childIndexB">Index of the child of collidable B in the pair. If collidable B is not compound, then this is always 0.</param>
            /// <param name="manifold">Set of contacts detected between the collidables.</param>
            /// <returns>True if this manifold should be considered for constraint generation, false otherwise.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
            {
                return true;
            }

            /// <summary>
            /// Releases any resources held by the callbacks. Called by the owning narrow phase when it is being disposed.
            /// </summary>
            public void Dispose()
            {
            }
        }

        //Note that the engine does not require any particular form of gravity- it, like all the contact callbacks, is managed by a callback.
        public struct PoseIntegratorCallbacks : IPoseIntegratorCallbacks
        {
            /// <summary>
            /// Performs any required initialization logic after the Simulation instance has been constructed.
            /// </summary>
            /// <param name="simulation">Simulation that owns these callbacks.</param>
            public void Initialize(Simulation simulation)
            {
                //In this demo, we don't need to initialize anything.
                //If you had a simulation with per body gravity stored in a CollidableProperty<T> or something similar, having the simulation provided in a callback can be helpful.
            }

            /// <summary>
            /// Gets how the pose integrator should handle angular velocity integration.
            /// </summary>
            public readonly AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;

            /// <summary>
            /// Gets whether the integrator should use substepping for unconstrained bodies when using a substepping solver.
            /// If true, unconstrained bodies will be integrated with the same number of substeps as the constrained bodies in the solver.
            /// If false, unconstrained bodies use a single step of length equal to the dt provided to Simulation.Timestep. 
            /// </summary>
            public readonly bool AllowSubstepsForUnconstrainedBodies => false;

            /// <summary>
            /// Gets whether the velocity integration callback should be called for kinematic bodies.
            /// If true, IntegrateVelocity will be called for bundles including kinematic bodies.
            /// If false, kinematic bodies will just continue using whatever velocity they have set.
            /// Most use cases should set this to false.
            /// </summary>
            public readonly bool IntegrateVelocityForKinematics => false;

            public Vector3 Gravity;

            public PoseIntegratorCallbacks(Vector3 gravity) : this()
            {
                Gravity = gravity;
            }

            //Note that velocity integration uses "wide" types. These are array-of-struct-of-arrays types that use SIMD accelerated types underneath.
            //Rather than handling a single body at a time, the callback handles up to Vector<float>.Count bodies simultaneously.
            Vector3Wide gravityWideDt;

            /// <summary>
            /// Callback invoked ahead of dispatches that may call into <see cref="IntegrateVelocity"/>.
            /// It may be called more than once with different values over a frame. For example, when performing bounding box prediction, velocity is integrated with a full frame time step duration.
            /// During substepped solves, integration is split into substepCount steps, each with fullFrameDuration / substepCount duration.
            /// The final integration pass for unconstrained bodies may be either fullFrameDuration or fullFrameDuration / substepCount, depending on the value of AllowSubstepsForUnconstrainedBodies. 
            /// </summary>
            /// <param name="dt">Current integration time step duration.</param>
            /// <remarks>This is typically used for precomputing anything expensive that will be used across velocity integration.</remarks>
            public void PrepareForIntegration(float dt)
            {
                //No reason to recalculate gravity * dt for every body; just cache it ahead of time.
                gravityWideDt = Vector3Wide.Broadcast(Gravity * dt);
            }

            /// <summary>
            /// Callback for a bundle of bodies being integrated.
            /// </summary>
            /// <param name="bodyIndices">Indices of the bodies being integrated in this bundle.</param>
            /// <param name="position">Current body positions.</param>
            /// <param name="orientation">Current body orientations.</param>
            /// <param name="localInertia">Body's current local inertia.</param>
            /// <param name="integrationMask">Mask indicating which lanes are active in the bundle. Active lanes will contain 0xFFFFFFFF, inactive lanes will contain 0.</param>
            /// <param name="workerIndex">Index of the worker thread processing this bundle.</param>
            /// <param name="dt">Durations to integrate the velocity over. Can vary over lanes.</param>
            /// <param name="velocity">Velocity of bodies in the bundle. Any changes to lanes which are not active by the integrationMask will be discarded.</param>
            public void IntegrateVelocity(Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation, BodyInertiaWide localInertia, Vector<int> integrationMask, int workerIndex, Vector<float> dt, ref BodyVelocityWide velocity)
            {
                //This also is a handy spot to implement things like position dependent gravity or per-body damping.
                //We don't have to check for kinematics; IntegrateVelocityForKinematics returns false in this type, so we'll never see them in this callback.
                //Note that these are SIMD operations and "Wide" types. There are Vector<float>.Count lanes of execution being evaluated simultaneously.
                //The types are laid out in array-of-structures-of-arrays (AOSOA) format. That's because this function is frequently called from vectorized contexts within the solver.
                //Transforming to "array of structures" (AOS) format for the callback and then back to AOSOA would involve a lot of overhead, so instead the callback works on the AOSOA representation directly.
                velocity.Linear += gravityWideDt;
            }

        }

        public static void Run()
        {
            //The buffer pool is a source of raw memory blobs for the engine to use.
            var bufferPool = new BufferPool();
            //The following sets up a simulation with the callbacks defined above, and tells it to use 8 velocity iterations per substep and only one substep per solve.
            //It uses the default SubsteppingTimestepper. You could use a custom ITimestepper implementation to customize when stages run relative to each other, or to insert more callbacks.         
            var simulation = Simulation.Create(bufferPool, new NarrowPhaseCallbacks(), new PoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));

            //Drop a ball on a big static box.
            var sphere = new Sphere(1);
            var sphereInertia = sphere.ComputeInertia(1);
            simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(0, 5, 0), sphereInertia, simulation.Shapes.Add(sphere), 0.01f));

            simulation.Statics.Add(new StaticDescription(new Vector3(0, 0, 0), simulation.Shapes.Add(new Box(500, 1, 500))));

            //Any IThreadDispatcher implementation can be used for multithreading. Here, we use the BepuUtilities.ThreadDispatcher implementation.
            var threadDispatcher = new ThreadDispatcher(Environment.ProcessorCount);

            //Now take 100 time steps!
            for (int i = 0; i < 100; ++i)
            {
                //Multithreading is pretty pointless for a simulation of one ball, but passing a IThreadDispatcher instance is all you have to do to enable multithreading.
                //If you don't want to use multithreading, don't pass a IThreadDispatcher.
                
                //Note that each timestep is 0.01 units in duration, so all 100 time steps will last 1 unit of time.
                //(Usually, units of time are defined to be seconds, but the engine has no preconceived notions about units. All it sees are the numbers.)
                simulation.Timestep(0.01f, threadDispatcher);
            }

            //If you intend to reuse the BufferPool, disposing the simulation is a good idea- it returns all the buffers to the pool for reuse.
            //Here, we dispose it, but it's not really required; we immediately thereafter clear the BufferPool of all held memory.
            //Note that failing to dispose buffer pools can result in memory leaks.
            simulation.Dispose();
            threadDispatcher.Dispose();
            bufferPool.Clear();
        }
    }
}
