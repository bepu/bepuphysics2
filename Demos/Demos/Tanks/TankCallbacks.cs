using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Threading;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities.Collections;
using BepuUtilities.Memory;

namespace Demos.Demos.Tanks
{
    /// <summary>
    /// Stores properties about a body in the tank demo.
    /// </summary>
    public struct TankBodyProperties
    {
        /// <summary>
        /// Controls which collidables the body can collide with.
        /// </summary>
        public SubgroupCollisionFilter Filter;
        /// <summary>
        /// Friction coefficient to use for the body.
        /// </summary>
        public float Friction;
        /// <summary>
        /// True if the body is a projectile and should explode on contact.
        /// </summary>
        public bool Projectile;
    }

    /// <summary>
    /// For the tank demo, we want both wheel-body collision filtering and different friction for wheels versus the tank body.
    /// </summary>
    struct TankCallbacks : INarrowPhaseCallbacks
    {
        public BodyProperty<TankBodyProperties> Properties;
        public SpinLock ProjectileLock;
        public QuickList<int> ExplodingProjectiles;
        public void Initialize(Simulation simulation)
        {
            Properties.Initialize(simulation.Bodies);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b)
        {
            //It's impossible for two statics to collide, and pairs are sorted such that bodies always come before statics.
            if (b.Mobility != CollidableMobility.Static)
            {
                return SubgroupCollisionFilter.AllowCollision(Properties[a.Handle].Filter, Properties[b.Handle].Filter);
            }
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
        {
            //This function is called for children of compounds, triangles in meshes, and similar cases, but we don't perform any child-level filtering in the tank demo.
            //The top level filter will always run before this function has a chance to, so we don't have to do anything here.
            return true;
        }

        //The engine hands off a direct pointer to the contact manifold that would be used for constraint generation (if allowed), but a lot of logic is shared between the manifold types.
        //We'll make use of the IContactManifold interface to combine most of the logic.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void HandlePair<TManifold>(CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : struct, IContactManifold<TManifold>
        {
            //Different tank parts have different friction values. Wheels tend to stick more than the body of the tank.
            ref var propertiesA = ref Properties[pair.A.Handle];
            pairMaterial.FrictionCoefficient = propertiesA.Friction;
            if (pair.B.Mobility != CollidableMobility.Static)
            {
                //If two bodies collide, just average the friction. Other options include min(a, b) or a * b.
                ref var propertiesB = ref Properties[pair.B.Handle];
                pairMaterial.FrictionCoefficient = (pairMaterial.FrictionCoefficient + propertiesB.Friction) * 0.5f;
            }
            //These are just some nice standard values. Higher maximum velocities can result in more energy being introduced during deep contact.
            //Finite spring stiffness helps the solver converge to a solution in difficult cases. Try to keep the spring frequency at around half of the timestep frequency or less.
            pairMaterial.MaximumRecoveryVelocity = 2f;
            pairMaterial.SpringSettings = new SpringSettings(30, 1);

            if (propertiesA.Projectile || (pair.B.Mobility != CollidableMobility.Static) && Properties[pair.B.Handle].Projectile)
            {
                for (int i = 0; i < manifold.Count; ++i)
                {
                    //This probably looks a bit odd. You can't return refs to the this instance in structs, and interfaces can't require static functions...
                    //so we use this redundant construction to get a direct reference to a contact's depth with near zero overhead.
                    //There's a more typical out parameter overload for contact properties too. And there's always the option of using the manifold pointers directly.
                    if (manifold.GetDepth(ref manifold, i) >= 0)
                    {
                        //An actual collision was found. 
                        if (propertiesA.Projectile)
                        {
                            bool lockTaken = false;
                            ProjectileLock.Enter(ref lockTaken);
                            //The exploding projectiles list should have been sized ahead of time to hold all projectiles, so no dynamic allocations should be required.
                            //Note that we have to protect against redundant adds- a projectile might hit multiple things in the same frame. Wouldn't want it to explode multiple times.
                            if (!ExplodingProjectiles.Contains(pair.A.Handle))
                                ExplodingProjectiles.AllocateUnsafely() = pair.A.Handle;
                            ProjectileLock.Exit();
                        }
                        if (pair.B.Mobility != CollidableMobility.Static && Properties[pair.B.Handle].Projectile)
                        {
                            //Could technically combine the locks in the case that both bodies are projectiles, but that's not exactly common.
                            bool lockTaken = false;
                            ProjectileLock.Enter(ref lockTaken);
                            if (!ExplodingProjectiles.Contains(pair.B.Handle))
                                ExplodingProjectiles.AllocateUnsafely() = pair.B.Handle;
                            ProjectileLock.Exit();
                        }
                        break;
                    }
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, NonconvexContactManifold* manifold, out PairMaterialProperties pairMaterial)
        {
            HandlePair(pair, ref *manifold, out pairMaterial);
            return true;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, ConvexContactManifold* manifold, out PairMaterialProperties pairMaterial)
        {
            HandlePair(pair, ref *manifold, out pairMaterial);
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ConvexContactManifold* manifold)
        {
            return true;
        }

        public void Dispose()
        {
            Properties.Dispose();
        }
    }
}