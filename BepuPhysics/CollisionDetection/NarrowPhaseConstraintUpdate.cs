using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System.Runtime.CompilerServices;
using BepuPhysics.Constraints;
using System.Diagnostics;
using System.Numerics;
using BepuPhysics.Collidables;
using System;
using BepuPhysics.Constraints.Contact;

namespace BepuPhysics.CollisionDetection
{
    /// <summary>
    /// Associated with a pair of two collidables that each are controlled by bodies.
    /// </summary>
    struct TwoBodyHandles
    {
        public int A;
        public int B;
    }

    /// <summary>
    /// Special type for collision pairs that do not need to store any supplementary information.
    /// </summary>
    struct EmptyCollisionCache : IPairCacheEntry
    {
        public int TypeId => -1;
    }

    public struct ContactImpulses1
    {
        public float Impulse0;
    }
    public struct ContactImpulses2
    {
        public float Impulse0;
        public float Impulse1;
    }
    public struct ContactImpulses3
    {
        public float Impulse0;
        public float Impulse1;
        public float Impulse2;
    }
    public struct ContactImpulses4
    {
        public float Impulse0;
        public float Impulse1;
        public float Impulse2;
        public float Impulse3;
    }

    public partial class NarrowPhase<TCallbacks> where TCallbacks : struct, INarrowPhaseCallbacks
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe void RedistributeImpulses<TContactImpulses>(int oldContactCount, float* oldImpulses, int* oldFeatureIds, ContactManifold* manifold, ref TContactImpulses newImpulsesContainer)
        {
            //Map the new contacts to the old contacts.
            var newFeatureIds = &manifold->FeatureId0;
            var newContactCount = manifold->ContactCount;
            ref var newImpulses = ref Unsafe.As<TContactImpulses, float>(ref newImpulsesContainer);
            for (int i = 0; i < newContactCount; ++i)
            {
                Unsafe.Add(ref newImpulses, i) = 0;
                for (int j = 0; j < oldContactCount; ++j)
                {
                    if (oldFeatureIds[j] == newFeatureIds[i])
                    {
                        Unsafe.Add(ref newImpulses, i) = oldImpulses[j];
                    }
                }
            }
            //TODO: 'Unclaimed' impulse from old unmatched contacts could be redistributed to try to conserve total impulse. Something to fiddle with once we have a test case running.
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void RequestAddConstraint<TDescription, TBodyHandles, TContactImpulses>(int workerIndex, int manifoldConstraintType,
            ref CollidablePair pair, PairCacheIndex constraintCacheIndex, ref TContactImpulses newImpulses,
            ref TDescription description, TBodyHandles bodyHandles) where TDescription : IConstraintDescription<TDescription>
        {
            //Note that this branch is (was?) JIT constant.
            if (typeof(TBodyHandles) != typeof(TwoBodyHandles) && typeof(TBodyHandles) != typeof(int))
            {
                throw new InvalidOperationException("Invalid body handles type; the narrow phase should only use TwoBodyHandles or int.");
            }
            AddConstraint(workerIndex, manifoldConstraintType, ref pair, constraintCacheIndex, ref newImpulses, bodyHandles, ref description);
        }

        unsafe void UpdateConstraint<TBodyHandles, TDescription, TContactImpulses, TCollisionCache, TConstraintCache>(int workerIndex, ref CollidablePair pair,
            ContactManifold* manifold, int manifoldTypeAsConstraintType, ref TCollisionCache collisionCache, ref TDescription description, TBodyHandles bodyHandles)
            where TConstraintCache : IPairCacheEntry
            where TCollisionCache : IPairCacheEntry
            where TDescription : IConstraintDescription<TDescription>
        {
            var newConstraintCache = default(TConstraintCache); //TODO: no need for this init; if blittable generics exist later, we can fix it easily
            PairCache.FillNewConstraintCache(&manifold->FeatureId0, ref newConstraintCache);

            var index = PairCache.IndexOf(ref pair);
            if (index >= 0)
            {
                //The previous frame had a constraint for this pair.
                ref var pointers = ref PairCache.GetPointers(index);
                Debug.Assert(pointers.ConstraintCache.Exists, "If a pair was persisted in the narrow phase, there should be a constraint associated with it.");

                var constraintCacheIndex = pointers.ConstraintCache;
                var constraintCachePointer = PairCache.GetOldConstraintCachePointer(index);
                var constraintHandle = *(int*)constraintCachePointer;
                Solver.GetConstraintReference(constraintHandle, out var constraintReference);
                //TODO: Check codegen; this if statement should JIT to a single path.
                //We specialize the 1-contact case since the 'redistribution' is automatic.
                var newImpulses = default(TContactImpulses);
                if (typeof(TConstraintCache) == typeof(ConstraintCache1))
                {
                    PairCache.GatherOldImpulses(ref constraintReference, (float*)Unsafe.AsPointer(ref newImpulses));
                }
                else
                {
                    var oldContactCount = PairCache.GetContactCount(constraintCacheIndex.Type);
                    var oldImpulses = stackalloc float[oldContactCount];
                    PairCache.GatherOldImpulses(ref constraintReference, oldImpulses);
                    //The first slot in the constraint cache is the constraint handle; the following slots are feature ids.
                    RedistributeImpulses(oldContactCount, oldImpulses, (int*)constraintCachePointer + 1, manifold, ref newImpulses);
                }

                if (manifoldTypeAsConstraintType == constraintReference.TypeBatch.TypeId)
                {
                    //Since the old constraint is the same type, we aren't going to remove the old constraint and add a new one. That means no deferred process is going
                    //to update the constraint cache's constraint handle. The good news is that we already have a valid constraint handle from the pre-existing constraint.
                    //It's exactly the same type, so we can just overwrite its properties without worry.
                    //Note that we rely on the constraint handle being stored in the first 4 bytes of the constraint cache.
                    *(int*)Unsafe.AsPointer(ref newConstraintCache) = constraintHandle;
                    PairCache.Update(workerIndex, index, ref pointers, ref collisionCache, ref newConstraintCache);
                    //There exists a constraint and it has the same type as the manifold. Directly apply the new description and impulses.
                    Solver.ApplyDescription(ref constraintReference, ref description);
                    PairCache.ScatterNewImpulses(ref constraintReference, ref newImpulses);
                }
                else
                {
                    //There exists a constraint, but it's a different type. This is more complex:
                    //1) The new manifold's constraint must be added, but upon the adder's return the solver is not guaranteed to contain the constraint- it may be deferred.
                    //(This allows a custom adder to implement deferred approaches. For example, determinism requires a consistent order of solver constraint addition, and a post-sort 
                    //can be used to guarantee that consistent order. We can also defer smaller batches for the sake of limiting sync overheads. 4-16 adds within a single lock
                    //means a 4-16x reduction in lock-related overhead, assuming no contests.)
                    //2) The old constraint must be removed.
                    PairCache.Update(workerIndex, index, ref pointers, ref collisionCache, ref newConstraintCache);
                    RequestAddConstraint(workerIndex, manifoldTypeAsConstraintType, ref pair, constraintCacheIndex, ref newImpulses, ref description, bodyHandles);
                    ConstraintRemover.EnqueueRemoval(workerIndex, constraintHandle);
                }
            }
            else
            {
                //No preexisting constraint; add a fresh constraint and pair cache entry.
                //The pair cache entry has to be created first so that the adder has a place to put the result of the constraint add.
                var constraintCacheIndex = PairCache.Add(workerIndex, ref pair, ref collisionCache, ref newConstraintCache);
                var newImpulses = default(TContactImpulses);
                //TODO: It would be nice to avoid the impulse scatter for fully new constraints; it's going to be all zeroes regardless. Worth investigating later.
                RequestAddConstraint(workerIndex, manifoldTypeAsConstraintType, ref pair, constraintCacheIndex, ref newImpulses, ref description, bodyHandles);
                //This is a new connection in the constraint graph, so we must check to see if any involved body is inactive.
                //Note that this is only possible when both colliders are bodies. If only one collider is a body, then it must be active otherwise this pair would never have been tested.
                if (typeof(TBodyHandles) == typeof(TwoBodyHandles))
                {
                    ref var twoBodyHandles = ref Unsafe.As<TBodyHandles, TwoBodyHandles>(ref bodyHandles);
                    ref var locationA = ref Bodies.HandleToLocation[twoBodyHandles.A];
                    ref var locationB = ref Bodies.HandleToLocation[twoBodyHandles.B];
                    //Only one of the two can be inactive.
                    if (locationA.SetIndex != locationB.SetIndex)
                    {
                        ref var overlapWorker = ref overlapWorkers[workerIndex];
                        overlapWorker.PendingSetActivations.Add(locationA.SetIndex > 0 ? locationA.SetIndex : locationB.SetIndex, overlapWorker.Batcher.pool.SpecializeFor<int>());
                    }
                }
            }
        }

        unsafe void UpdateConstraintForManifold<TCollisionCache, TBodyHandles>(int workerIndex, ref CollidablePair pair, ContactManifold* manifold, ref TCollisionCache collisionCache,
            ref PairMaterialProperties material, TBodyHandles bodyHandles)
            where TCollisionCache : IPairCacheEntry
        {
            //Note that this function has two responsibilities:
            //1) Create the description of the constraint that should represent the new manifold.
            //2) Add that constraint (or update an existing constraint) with that description, updating any accumulated impulses as needed.
            //Conceptually, it would be nicer to have a single function for each of these- create a description, and then apply it.
            //However, we cannot return the type knowledge we extract from the constraint cache index. Instead, we make use of the information in-place.

            //TODO: Should check codegen and alternatives here.
            //TODO: Descriptions will be changing to not have redundant B offsets, this will have to change to match.
            Debug.Assert(manifold->ContactCount > 0);
            //Constraint types only use 3 bits, since their contact count can never be zero.
            //1-4 contacts: 0x3
            //nonconvex: 0x4
            //1 body versus 2 body: 0x8
            //TODO: Very likely that we'll expand the nonconvex manifold maximum to 8 contacts, so this will need to be adjusted later.
            var manifoldTypeAsConstraintType = ((manifold->PackedConvexityAndContactCount >> 1) & 4) | ((manifold->PackedConvexityAndContactCount & 7) - 1);
            if (typeof(TBodyHandles) == typeof(TwoBodyHandles))
                manifoldTypeAsConstraintType |= 0x8;
            switch (manifoldTypeAsConstraintType)
            {
                //One body
                //Convex
                case 0:
                    {
                        Contact1OneBody description;
                        description.Contact0.OffsetA = manifold->Offset0;
                        description.Contact0.PenetrationDepth = manifold->Depth0;
                        description.FrictionCoefficient = material.FrictionCoefficient;
                        description.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
                        description.SpringSettings = material.SpringSettings;
                        description.Normal = manifold->ConvexNormal;

                        //TODO: Check init hack.
                        UpdateConstraint<TBodyHandles, Contact1OneBody, ContactImpulses1, TCollisionCache, ConstraintCache1>(
                            workerIndex, ref pair, manifold, manifoldTypeAsConstraintType, ref collisionCache, ref *&description, bodyHandles);
                    }
                    break;
                case 1:
                    break;
                case 2:
                    break;
                case 3:
                    break;
                //Nonconvex
                case 4 + 0:
                    break;
                case 4 + 1:
                    break;
                case 4 + 2:
                    break;
                case 4 + 3:
                    break;
                //Two body
                //Convex
                case 8 + 0:
                    {
                        Contact1 description;
                        description.Contact0.OffsetA = manifold->Offset0;
                        description.Contact0.PenetrationDepth = manifold->Depth0;
                        description.OffsetB = manifold->OffsetB;
                        description.FrictionCoefficient = material.FrictionCoefficient;
                        description.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
                        description.SpringSettings = material.SpringSettings;
                        description.Normal = manifold->ConvexNormal;

                        //TODO: Check init hack.
                        UpdateConstraint<TBodyHandles, Contact1, ContactImpulses1, TCollisionCache, ConstraintCache1>(
                            workerIndex, ref pair, manifold, manifoldTypeAsConstraintType, ref collisionCache, ref *&description, bodyHandles);
                    }
                    break;
                case 8 + 1:
                    break;
                case 8 + 2:
                    break;
                case 8 + 3:
                    {
                        Contact4 description;
                        var descriptionContacts = &description.Contact0;
                        var offsets = &manifold->Offset0;
                        var depths = &manifold->Depth0;
                        for (int i = 0; i < 4; ++i)
                        {
                            ref var descriptionContact = ref descriptionContacts[i];
                            descriptionContact.OffsetA = offsets[i];
                            descriptionContact.PenetrationDepth = depths[i];
                        }
                        description.OffsetB = manifold->OffsetB;
                        description.FrictionCoefficient = material.FrictionCoefficient;
                        description.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
                        description.SpringSettings = material.SpringSettings;
                        description.Normal = manifold->ConvexNormal;

                        //TODO: Check init hack.
                        UpdateConstraint<TBodyHandles, Contact4, ContactImpulses4, TCollisionCache, ConstraintCache4>(
                            workerIndex, ref pair, manifold, manifoldTypeAsConstraintType, ref collisionCache, ref *&description, bodyHandles);
                    }
                    break;
                //Nonconvex
                case 8 + 4 + 0:
                    break;
                case 8 + 4 + 1:
                    break;
                case 8 + 4 + 2:
                    break;
                case 8 + 4 + 3:
                    break;

            }

        }

        public unsafe void UpdateConstraintsForPair<TCollisionCache>(int workerIndex, ref CollidablePair pair, ContactManifold* manifold, ref TCollisionCache collisionCache) where TCollisionCache : IPairCacheEntry
        {
            //Note that we do not check for the pair being between two statics before reporting it. The assumption is that, if the initial broadphase pair filter allowed such a pair
            //to reach this point, the user probably wants to receive some information about the resulting contact manifold.
            //That said, such a pair cannot generate constraints no matter what- constraints must involve at least one body, always.
            var aMobility = pair.A.Mobility;
            var bMobility = pair.B.Mobility;
            Debug.Assert(aMobility != CollidableMobility.Static, "The broad phase should not generate static-static pairs ever, and any static collidable should be in slot B.");
            if (Callbacks.ConfigureContactManifold(workerIndex, pair, manifold, out var pairMaterial) &&
                //Note that, even if the callback says 'yeah sure create a constraint for those', it never makes sense to generate constraints between two nondynamics.
                //It would just result in a bunch of NaNs when computing the effective mass.
                (aMobility == CollidableMobility.Dynamic || bMobility == CollidableMobility.Dynamic))
            {
                if (manifold->ContactCount > 0)
                {
                    if (bMobility != CollidableMobility.Static)
                    {
                        //Two bodies.
                        Debug.Assert(pair.A.Mobility != CollidableMobility.Static && pair.B.Mobility != CollidableMobility.Static);
                        var bodyHandles = new TwoBodyHandles { A = pair.A.Handle, B = pair.B.Handle };
                        UpdateConstraintForManifold(workerIndex, ref pair, manifold, ref collisionCache, ref pairMaterial, bodyHandles);
                    }
                    else
                    {
                        //One of the two collidables is static.
                        Debug.Assert(pair.A.Mobility != CollidableMobility.Static && pair.B.Mobility == CollidableMobility.Static);
                        UpdateConstraintForManifold(workerIndex, ref pair, manifold, ref collisionCache, ref pairMaterial, pair.A.Handle);
                    }
                }
                //In the event that there are no contacts in the new manifold, the pair is left in a stale state. It will be removed by the stale removal post process. 
            }
        }

    }
}