using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System.Runtime.CompilerServices;
using BepuPhysics.Constraints;
using System.Diagnostics;
using System.Numerics;
using BepuPhysics.Collidables;
using System;
using BepuPhysics.Constraints.Contact;
using BepuUtilities;

namespace BepuPhysics.CollisionDetection
{
    /// <summary>
    /// Associated with a pair of two collidables that each are controlled by bodies.
    /// </summary>
    public struct TwoBodyHandles
    {
        public int A;
        public int B;
    }

    /// <summary>
    /// Special type for collision pairs that do not need to store any supplementary information.
    /// </summary>
    struct EmptyCollisionCache : IPairCacheEntry
    {
        public int CacheTypeId => -1;
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
    public struct ContactImpulses5
    {
        public float Impulse0;
        public float Impulse1;
        public float Impulse2;
        public float Impulse3;
        public float Impulse4;
    }
    public struct ContactImpulses6
    {
        public float Impulse0;
        public float Impulse1;
        public float Impulse2;
        public float Impulse3;
        public float Impulse4;
        public float Impulse5;
    }
    public struct ContactImpulses7
    {
        public float Impulse0;
        public float Impulse1;
        public float Impulse2;
        public float Impulse3;
        public float Impulse4;
        public float Impulse5;
        public float Impulse6;
    }
    public struct ContactImpulses8
    {
        public float Impulse0;
        public float Impulse1;
        public float Impulse2;
        public float Impulse3;
        public float Impulse4;
        public float Impulse5;
        public float Impulse6;
        public float Impulse7;
    }

    public partial class NarrowPhase<TCallbacks> where TCallbacks : struct, INarrowPhaseCallbacks
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe void RedistributeImpulses<TContactImpulses>(
            int oldContactCount, int* oldFeatureIds, float* oldImpulses,
            int newContactCount, ref int newFeatureIds, ref TContactImpulses newImpulsesContainer)
        {
            //Map the new contacts to the old contacts.
            ref var newImpulses = ref Unsafe.As<TContactImpulses, float>(ref newImpulsesContainer);
            //Note that the pointer casts below are not actually GC holes:
            //contact manifolds passed down here from the collision batcher and friends are all stored either on the stack or in pinned buffers.
            int unmatchedCount = 0;
            for (int i = 0; i < newContactCount; ++i)
            {
                ref var newImpulse = ref Unsafe.Add(ref newImpulses, i);
                //Accumulated impulses cannot be negative; we use a negative value as a 'unmatched' flag.
                newImpulse = -1;
                for (int j = 0; j < oldContactCount; ++j)
                {
                    if (oldFeatureIds[j] == Unsafe.Add(ref newFeatureIds, i))
                    {
                        newImpulse = oldImpulses[j];
                        //Eliminate the old impulse so that it will not be distributed to the unmatched contacts.
                        oldImpulses[j] = 0;
                        break;
                    }
                }
                if (newImpulse < 0)
                {
                    ++unmatchedCount;
                }
            }
            //Distribute any missing impulse evenly over the remaining unmatched contacts.
            if (unmatchedCount > 0)
            {
                float unmatchedImpulse = 0;
                for (int i = 0; i < oldContactCount; ++i)
                {
                    unmatchedImpulse += oldImpulses[i];
                }
                var impulsePerUnmatched = unmatchedImpulse / unmatchedCount;
                for (int i = 0; i < newContactCount; ++i)
                {
                    ref var newImpulse = ref Unsafe.Add(ref newImpulses, i);
                    //If we flagged the impulse as unmatched in the first loop, we can fill it here.
                    if (newImpulse < 0)
                    {
                        //newImpulse = 0;
                        newImpulse = impulsePerUnmatched;
                    }

                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void RequestAddConstraint<TDescription, TBodyHandles, TContactImpulses>(int workerIndex, int manifoldConstraintType,
            ref CollidablePair pair, PairCacheIndex constraintCacheIndex, ref TContactImpulses newImpulses,
            ref TDescription description, TBodyHandles bodyHandles) where TBodyHandles : unmanaged where TDescription : unmanaged, IConstraintDescription<TDescription>
        {
            //Note that this branch is (was?) JIT constant.
            if (typeof(TBodyHandles) != typeof(TwoBodyHandles) && typeof(TBodyHandles) != typeof(int))
            {
                throw new InvalidOperationException("Invalid body handles type; the narrow phase should only use TwoBodyHandles or int.");
            }
            AddConstraint(workerIndex, manifoldConstraintType, ref pair, constraintCacheIndex, ref newImpulses, bodyHandles, ref description);
        }

        public unsafe void UpdateConstraint<TBodyHandles, TDescription, TContactImpulses, TCollisionCache, TConstraintCache>(int workerIndex, ref CollidablePair pair,
            int manifoldTypeAsConstraintType, ref TConstraintCache newConstraintCache, ref TCollisionCache collisionCache,
            ref TDescription description, TBodyHandles bodyHandles)
            where TBodyHandles : unmanaged
            where TConstraintCache : unmanaged, IPairCacheEntry
            where TCollisionCache : unmanaged, IPairCacheEntry
            where TDescription : unmanaged, IConstraintDescription<TDescription>
            where TContactImpulses : unmanaged
        {
            var index = PairCache.IndexOf(ref pair);
            if (index >= 0)
            {
                //The previous frame had a constraint for this pair.
                ref var pointers = ref PairCache.GetPointers(index);
                Debug.Assert(pointers.ConstraintCache.Exists, "If a pair was persisted in the narrow phase, there should be a constraint associated with it.");

                var constraintCacheIndex = pointers.ConstraintCache;
                var oldConstraintCachePointer = PairCache.GetOldConstraintCachePointer(index);
                var constraintHandle = *(ConstraintHandle*)oldConstraintCachePointer;
                var constraintReference = Solver.GetConstraintReference(constraintHandle);
                Debug.Assert(
                    constraintReference.typeBatchPointer != null &&
                    constraintReference.IndexInTypeBatch >= 0 &&
                    constraintReference.IndexInTypeBatch < constraintReference.TypeBatch.ConstraintCount,
                    "Handle-retrieved constraint reference must point to a constraint of expected type, or else something is corrupted.");
                var newImpulses = default(TContactImpulses);
                var accessor = contactConstraintAccessors[constraintReference.TypeBatch.TypeId];
                var oldImpulses = stackalloc float[accessor.ContactCount];
                accessor.GatherOldImpulses(ref constraintReference, oldImpulses);
#if DEBUG
                for (int i = 0; i < accessor.ContactCount; ++i)
                {
                    Debug.Assert(oldImpulses[i] >= 0, "Penetration limit impulses must be nonnegative. Otherwise, something's wrong. Busted gather? Broken constraint?");
                }
#endif
                //The first slot in the constraint cache is the constraint handle; the following slots are feature ids.
                RedistributeImpulses(
                    accessor.ContactCount, (int*)oldConstraintCachePointer + 1, oldImpulses,
                    newConstraintCache.CacheTypeId + 1, ref Unsafe.Add(ref Unsafe.As<TConstraintCache, int>(ref newConstraintCache), 1), ref newImpulses);

                if (manifoldTypeAsConstraintType == constraintReference.TypeBatch.TypeId)
                {
                    //Since the old constraint is the same type, we aren't going to remove the old constraint and add a new one. That means no deferred process is going
                    //to update the constraint cache's constraint handle. The good news is that we already have a valid constraint handle from the pre-existing constraint.
                    //It's exactly the same type, so we can just overwrite its properties without worry.
                    //Note that we rely on the constraint handle being stored in the first 4 bytes of the constraint cache.
                    Unsafe.As<TConstraintCache, ConstraintHandle>(ref newConstraintCache) = constraintHandle;
                    PairCache.Update(workerIndex, index, ref pointers, ref collisionCache, ref newConstraintCache);
                    //There exists a constraint and it has the same type as the manifold. Directly apply the new description and impulses.
                    Solver.ApplyDescriptionWithoutWaking(constraintReference, description);
                    accessor.ScatterNewImpulses(ref constraintReference, ref newImpulses);
                }
                else
                {
                    //There exists a constraint, but it's a different type. This is more complex:
                    //1) The new manifold's constraint must be added, but upon the adder's return the solver does not yet contain the constraint. They are deferred.
                    //2) The old constraint must be removed.
                    PairCache.Update(workerIndex, index, ref pointers, ref collisionCache, ref newConstraintCache);
                    RequestAddConstraint(workerIndex, manifoldTypeAsConstraintType, ref pair, pointers.ConstraintCache, ref newImpulses, ref description, bodyHandles);
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
                        overlapWorker.PendingSetAwakenings.Add(locationA.SetIndex > 0 ? locationA.SetIndex : locationB.SetIndex, overlapWorker.Batcher.Pool);
                    }
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int ExtractContactConstraintBodyCount(int contactConstraintTypeId)
        {
            Debug.Assert(contactConstraintTypeId >= 0 && contactConstraintTypeId < PairCache.CollisionConstraintTypeCount);
            //[0, 3] and [8, 14] are one body. Otherwise, two.
            //Could probably be a little more clever than this if it matters.
            return contactConstraintTypeId <= 3 || (contactConstraintTypeId >= 8 && contactConstraintTypeId <= 14) ? 1 : 2;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static int GetConvexConstraintTypeId<TBodyHandles>(int contactCount)
        {
            //Convex constraints:
            //1-4 contacts: 0x3
            //2 body (unset is 1 body): 0x4
            //So:
            //Convex one body constraints, contact count 1 through 4: [0, 3]
            //Convex two body constraints, contact count 1 through 4: [4, 7]
            int manifoldTypeAsConstraintType;
            Debug.Assert(contactCount > 0);
            manifoldTypeAsConstraintType = (contactCount - 1);
            if (typeof(TBodyHandles) == typeof(TwoBodyHandles))
                manifoldTypeAsConstraintType |= 0x4;
            return manifoldTypeAsConstraintType;
        }

        //TODO: If you end up changing the NarrowPhasePendingConstraintAdds and PairCache hardcoded type handling, you should change this too. This is getting silly.
        unsafe void UpdateConstraintForManifold<TContactManifold, TCollisionCache, TBodyHandles>(
            int workerIndex, ref CollidablePair pair, ref TContactManifold manifold, ref TCollisionCache collisionCache, ref PairMaterialProperties material, TBodyHandles bodyHandles)
            where TCollisionCache : unmanaged, IPairCacheEntry
        {
            //Note that this function has two responsibilities:
            //1) Create the description of the constraint that should represent the new manifold.
            //2) Add that constraint (or update an existing constraint) with that description, updating any accumulated impulses as needed.
            int manifoldTypeAsConstraintType;
            if (typeof(TContactManifold) == typeof(ConvexContactManifold))
            {
                ref var convexManifold = ref Unsafe.As<TContactManifold, ConvexContactManifold>(ref manifold);
                manifoldTypeAsConstraintType = GetConvexConstraintTypeId<TBodyHandles>(convexManifold.Count);
            }
            else
            {
                Debug.Assert(typeof(TContactManifold) == typeof(NonconvexContactManifold));
                ref var nonconvexManifold = ref Unsafe.As<TContactManifold, NonconvexContactManifold>(ref manifold);
                Debug.Assert(nonconvexManifold.Count > 0);
                if (nonconvexManifold.Count == 1)
                {
                    //No 'nonconvex' one contact constraints.
                    manifoldTypeAsConstraintType = GetConvexConstraintTypeId<TBodyHandles>(1);
                }
                else
                {
                    //Nonconvex constraints:
                    //To skip over convex constraints, start at 8
                    //Nonconvex constraints range from 2 to 8 contacts (1 contact constraints are considered convex)
                    //One body constraints have lower ids
                    //So:
                    //Nonconvex one body constraints, contact count 2 through 8: [8, 14]
                    //Nonconvex two body constraints, contact count 2 through 8: [15, 21]
                    manifoldTypeAsConstraintType = 8 + (nonconvexManifold.Count - 2);
                    if (typeof(TBodyHandles) == typeof(TwoBodyHandles))
                        manifoldTypeAsConstraintType += 7;
                }
            }
            contactConstraintAccessors[manifoldTypeAsConstraintType].UpdateConstraintForManifold(this, manifoldTypeAsConstraintType, workerIndex, ref pair, ref manifold, ref collisionCache, ref material, bodyHandles);
        }

        public unsafe void UpdateConstraintsForPair<TContactManifold, TCollisionCache>(int workerIndex, ref CollidablePair pair, ref TContactManifold manifold, ref TCollisionCache collisionCache) 
            where TCollisionCache : unmanaged, IPairCacheEntry
            where TContactManifold : unmanaged, IContactManifold<TContactManifold>
        {
            //Note that we do not check for the pair being between two statics before reporting it. The assumption is that, if the initial broadphase pair filter allowed such a pair
            //to reach this point, the user probably wants to receive some information about the resulting contact manifold.
            //That said, such a pair cannot generate constraints no matter what- constraints must involve at least one body, always.
            var aMobility = pair.A.Mobility;
            var bMobility = pair.B.Mobility;
            Debug.Assert(aMobility != CollidableMobility.Static, "The broad phase should not generate static-static pairs ever, and any static collidable should be in slot B.");
            bool allowConstraint;
            allowConstraint = Callbacks.ConfigureContactManifold(workerIndex, pair, ref manifold, out var pairMaterial) && manifold.Count > 0;
            if (allowConstraint &&
                //Note that, even if the callback says 'yeah sure create a constraint for those', it never makes sense to generate constraints between two nondynamics.
                //It would just result in a bunch of NaNs when computing the effective mass.
                (aMobility == CollidableMobility.Dynamic || bMobility == CollidableMobility.Dynamic))
            {
                if (bMobility != CollidableMobility.Static)
                {
                    //Two bodies.
                    Debug.Assert(pair.A.Mobility != CollidableMobility.Static && pair.B.Mobility != CollidableMobility.Static);
                    var bodyHandles = new TwoBodyHandles { A = pair.A.BodyHandle.Value, B = pair.B.BodyHandle.Value };
                    UpdateConstraintForManifold(workerIndex, ref pair, ref manifold, ref collisionCache, ref pairMaterial, bodyHandles);
                }
                else
                {
                    //One of the two collidables is static.
                    Debug.Assert(pair.A.Mobility != CollidableMobility.Static && pair.B.Mobility == CollidableMobility.Static);
                    UpdateConstraintForManifold(workerIndex, ref pair, ref manifold, ref collisionCache, ref pairMaterial, pair.A.BodyHandle.Value);
                }
                //In the event that there are no contacts in the new manifold, the pair is left in a stale state. It will be removed by the stale removal post process. 
            }
        }

    }
}