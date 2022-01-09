using System;
using System.Collections.Generic;
using System.Text;
using BepuUtilities;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;

namespace BepuPhysics.CollisionDetection
{
    /// <summary>
    /// Material properties governing the interaction between colliding bodies. Used by the narrow phase to create constraints of the appropriate configuration.
    /// </summary>
    public struct PairMaterialProperties
    {

        /// <summary>
        /// Coefficient of friction to apply for the constraint. Maximum friction force will be equal to the normal force times the friction coefficient.
        /// </summary>
        public float FrictionCoefficient;
        /// <summary>
        /// Maximum relative velocity along the contact normal at which the collision constraint will recover from penetration. Clamps the velocity goal created from the spring settings.
        /// </summary>
        public float MaximumRecoveryVelocity;
        /// <summary>
        /// Defines the constraint's penetration recovery spring properties.
        /// </summary>
        public SpringSettings SpringSettings;

        /// <summary>
        /// Constructs a pair's material properties.
        /// </summary>
        /// <param name="frictionCoefficient">Coefficient of friction to apply for the constraint. Maximum friction force will be equal to the normal force times the friction coefficient.</param>
        /// <param name="maximumRecoveryVelocity">Maximum relative velocity along the contact normal at which the collision constraint will recover from penetration. Clamps the velocity goal created from the spring settings. </param>
        /// <param name="springSettings">Defines the constraint's penetration recovery spring properties.</param>
        public PairMaterialProperties(float frictionCoefficient, float maximumRecoveryVelocity, SpringSettings springSettings)
        {
            FrictionCoefficient = frictionCoefficient;
            MaximumRecoveryVelocity = maximumRecoveryVelocity;
            SpringSettings = springSettings;
        }
    }

    /// <summary>
    /// Defines handlers for narrow phase events.
    /// </summary>
    public unsafe interface INarrowPhaseCallbacks
    {
        /// <summary>
        /// Performs any required initialization logic after the Simulation instance has been constructed.
        /// </summary>
        /// <param name="simulation">Simulation that owns these callbacks.</param>
        void Initialize(Simulation simulation);

        /// <summary>
        /// Chooses whether to allow contact generation to proceed for two overlapping collidables.
        /// </summary>
        /// <param name="workerIndex">Index of the worker that identified the overlap.</param>
        /// <param name="a">Reference to the first collidable in the pair.</param>
        /// <param name="b">Reference to the second collidable in the pair.</param>
        /// <param name="speculativeMargin">Reference to the speculative margin used by the pair.
        /// The value was already initialized by the narrowphase by examining the speculative margins of the involved collidables, but it can be modified.</param>
        /// <returns>True if collision detection should proceed, false otherwise.</returns>
        bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin);


        /// <summary>
        /// Provides a notification that a manifold has been created for a pair. Offers an opportunity to change the manifold's details. 
        /// </summary>
        /// <param name="workerIndex">Index of the worker thread that created this manifold.</param>
        /// <param name="pair">Pair of collidables that the manifold was detected between.</param>
        /// <param name="manifold">Set of contacts detected between the collidables.</param>
        /// <param name="pairMaterial">Material properties of the manifold.</param>
        /// <returns>True if a constraint should be created for the manifold, false otherwise.</returns>
        bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>;

        //TODO: There is an argument for finer grained material tuning, both per child and per contact. Need an efficient way to do this before we commit-
        //one possibility is a material per convex manifold. For nonconvex manifolds, there would be a material property per contact.
        //That's not an ideal setup- it's an extra 16-20 bytes per contact in the solver, which is pretty painful.

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
        bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB);
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
        bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold);
        
        /// <summary>
        /// Releases any resources held by the callbacks. Called by the owning narrow phase when it is being disposed.
        /// </summary>
        void Dispose();
    }
}
