using BepuPhysics.Constraints;
using BepuPhysics.Constraints.Contact;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection
{
    /// <summary>
    /// Provides indirection for reading from and updating constraints in the narrow phase.
    /// </summary>
    /// <remarks>This, like many other similar constructions in the engine, could conceptually be replaced by static function pointers and a few supplementary data fields.
    /// We probably will do exactly that at some point.</remarks>
    public abstract class ContactConstraintAccessor
    {
        public int ConstraintTypeId { get; protected set; }

        protected int AccumulatedImpulseBundleStrideInBytes;
        protected int ContactCount;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void GatherOldImpulses(ref ConstraintReference constraintReference, float* oldImpulses)
        {
            //Note that we do not modify the friction accumulated impulses. This is just for simplicity- the impact of accumulated impulses on friction *should* be relatively
            //hard to notice compared to penetration impulses. TODO: We should, however, test this assumption.
            BundleIndexing.GetBundleIndices(constraintReference.IndexInTypeBatch, out var bundleIndex, out var inner);
            ref var buffer = ref constraintReference.TypeBatch.AccumulatedImpulses;
            //Note that we assume that the tangent friction impulses always come first. This should be safe for now, but it is important to keep in mind for later.
            ref var baseImpulse = ref Unsafe.As<byte, Vector<float>>(ref buffer[AccumulatedImpulseBundleStrideInBytes * bundleIndex + Unsafe.SizeOf<Vector2Wide>()]);
            GatherScatter.GetLane(ref baseImpulse, inner, ref *oldImpulses, ContactCount);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ScatterNewImpulses<TContactImpulses>(ref ConstraintReference constraintReference, ref TContactImpulses contactImpulses)
        {
            //Note that we do not modify the friction accumulated impulses. This is just for simplicity- the impact of accumulated impulses on friction *should* be relatively
            //hard to notice compared to penetration impulses. TODO: We should, however, test this assumption.
            BundleIndexing.GetBundleIndices(constraintReference.IndexInTypeBatch, out var bundleIndex, out var inner);
            ref var buffer = ref constraintReference.TypeBatch.AccumulatedImpulses;
            //Note that we assume that the tangent friction impulses always come first. This should be safe for now, but it is important to keep in mind for later.
            ref var baseImpulse = ref Unsafe.As<byte, Vector<float>>(ref buffer[AccumulatedImpulseBundleStrideInBytes * bundleIndex + Unsafe.SizeOf<Vector2Wide>()]);
            GatherScatter.SetLane(ref baseImpulse, inner, ref Unsafe.As<TContactImpulses, float>(ref contactImpulses), ContactCount);
        }

        public abstract void DeterministicallyAdd<TCallbacks>(
            int typeIndex, NarrowPhase<TCallbacks>.OverlapWorker[] overlapWorkers,
            ref QuickList<NarrowPhase<TCallbacks>.SortConstraintTarget, Buffer<NarrowPhase<TCallbacks>.SortConstraintTarget>> constraintsOfType,
            Simulation simulation, PairCache pairCache) where TCallbacks : struct, INarrowPhaseCallbacks;

        public abstract void FlushWithSpeculativeBatches<TCallbacks>(ref UntypedList list, int narrowPhaseConstraintTypeId,
            ref Buffer<Buffer<ushort>> speculativeBatchIndices, Simulation simulation, PairCache pairCache)
            where TCallbacks : struct, INarrowPhaseCallbacks;

        public abstract void FlushSequentially<TCallbacks>(ref UntypedList list, int narrowPhaseConstraintTypeId, Simulation simulation, PairCache pairCache)
            where TCallbacks : struct, INarrowPhaseCallbacks;

        public abstract unsafe void UpdateConstraintForManifold<TCollisionCache, TCallBodyHandles, TCallbacks>(
            NarrowPhase<TCallbacks> narrowPhase, int manifoldTypeAsConstraintType, int workerIndex,
            ref CollidablePair pair, ContactManifold* manifold, ref TCollisionCache collisionCache, ref PairMaterialProperties material, TCallBodyHandles bodyHandles)
            where TCallbacks : struct, INarrowPhaseCallbacks
            where TCollisionCache : IPairCacheEntry;

    }

    //Note that the vast majority of the 'work' done by these accessor implementations is just type definitions used to call back into some other functions that need that type knowledge.
    public abstract class ContactConstraintAccessor<TConstraintDescription, TBodyHandles, TAccumulatedImpulses, TContactImpulses, TConstraintCache> : ContactConstraintAccessor
        where TConstraintDescription : IConstraintDescription<TConstraintDescription>
        where TConstraintCache : IPairCacheEntry
    {
        protected ContactConstraintAccessor()
        {
            Debug.Assert(
                typeof(TContactImpulses) == typeof(ContactImpulses1) ||
                typeof(TContactImpulses) == typeof(ContactImpulses2) ||
                typeof(TContactImpulses) == typeof(ContactImpulses3) ||
                typeof(TContactImpulses) == typeof(ContactImpulses4));
            ContactCount = Unsafe.SizeOf<TContactImpulses>() / Unsafe.SizeOf<float>();
            Debug.Assert((ContactCount + 3) * Unsafe.SizeOf<Vector<float>>() == Unsafe.SizeOf<TAccumulatedImpulses>(),
                "The layout of accumulated impulses seems to have changed; the assumptions of contact accessors are probably no longer valid.");
            AccumulatedImpulseBundleStrideInBytes = Unsafe.SizeOf<TAccumulatedImpulses>();
            ConstraintTypeId = default(TConstraintDescription).ConstraintTypeId;
        }
        public override void DeterministicallyAdd<TCallbacks>(int typeIndex, NarrowPhase<TCallbacks>.OverlapWorker[] overlapWorkers,
            ref QuickList<NarrowPhase<TCallbacks>.SortConstraintTarget, Buffer<NarrowPhase<TCallbacks>.SortConstraintTarget>> constraintsOfType,
            Simulation simulation, PairCache pairCache)
        {
            for (int i = 0; i < constraintsOfType.Count; ++i)
            {
                NarrowPhase<TCallbacks>.PendingConstraintAddCache.DeterministicAdd<TBodyHandles, TConstraintDescription, TContactImpulses>(
                    typeIndex, ref constraintsOfType[i], overlapWorkers, simulation, ref pairCache);
            }
        }
        public override void FlushSequentially<TCallbacks>(ref UntypedList list, int narrowPhaseConstraintTypeId, Simulation simulation, PairCache pairCache)
        {
            NarrowPhase<TCallbacks>.PendingConstraintAddCache.SequentialAddToSimulation<TBodyHandles, TConstraintDescription, TContactImpulses>(
                ref list, narrowPhaseConstraintTypeId, simulation, pairCache);
        }

        public override void FlushWithSpeculativeBatches<TCallbacks>(ref UntypedList list, int narrowPhaseConstraintTypeId, ref Buffer<Buffer<ushort>> speculativeBatchIndices, Simulation simulation, PairCache pairCache)
        {
            NarrowPhase<TCallbacks>.PendingConstraintAddCache.SequentialAddToSimulationSpeculative<TBodyHandles, TConstraintDescription, TContactImpulses>(
                ref list, narrowPhaseConstraintTypeId, ref speculativeBatchIndices, simulation, pairCache);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected static unsafe void UpdateConstraint<TCallbacks, TCollisionCache, TCallBodyHandles>(
            NarrowPhase<TCallbacks> narrowPhase, int manifoldTypeAsConstraintType, int workerIndex,
            ref CollidablePair pair, ContactManifold* manifold, ref TCollisionCache collisionCache, ref TConstraintDescription description, TCallBodyHandles bodyHandles)
            where TCallbacks : struct, INarrowPhaseCallbacks where TCollisionCache : IPairCacheEntry
        {
            //Note that we let the user pass in a body handles type to a generic function, rather than requiring that the top level abstract class define the type.
            //That allows a type inconsistency, but it's easy to catch.
            Debug.Assert(typeof(TCallBodyHandles) == typeof(TBodyHandles), "Don't call an update with inconsistent body handle types.");
            narrowPhase.UpdateConstraint<TBodyHandles, TConstraintDescription, TContactImpulses, TCollisionCache, TConstraintCache>(
                workerIndex, ref pair, manifold, manifoldTypeAsConstraintType, ref collisionCache, ref description, Unsafe.As<TCallBodyHandles, TBodyHandles>(ref bodyHandles));
        }
    }

    //TODO: The following has a fairly goofy amount of redundancy. It's not as bad as the original version, but should you find yourself needing to make any significant changes
    //to the layout, that would probably be a good time to bite the bullet and unify these. At least at the level of convex/nonconvex x onebody/twobody.
    public class Contact1OneBodyAccessor : ContactConstraintAccessor<Contact1OneBody, int, Contact1AccumulatedImpulses, ContactImpulses1, ConstraintCache1>
    {
        public override unsafe void UpdateConstraintForManifold<TCollisionCache, TCallBodyHandles, TCallbacks>(
            NarrowPhase<TCallbacks> narrowPhase, int manifoldTypeAsConstraintType, int workerIndex,
            ref CollidablePair pair, ContactManifold* manifold, ref TCollisionCache collisionCache, ref PairMaterialProperties material, TCallBodyHandles bodyHandles)
        {
            Contact1OneBody description;
            description.Contact0.OffsetA = manifold->Offset0;
            description.Contact0.PenetrationDepth = manifold->Depth0;
            description.FrictionCoefficient = material.FrictionCoefficient;
            description.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
            description.SpringSettings = material.SpringSettings;
            description.Normal = manifold->ConvexNormal;

            UpdateConstraint(narrowPhase, manifoldTypeAsConstraintType, workerIndex, ref pair, manifold, ref collisionCache, ref description, bodyHandles);
        }
    }
    public class Contact2OneBodyAccessor : ContactConstraintAccessor<Contact2OneBody, int, Contact2AccumulatedImpulses, ContactImpulses2, ConstraintCache2>
    {
        public override unsafe void UpdateConstraintForManifold<TCollisionCache, TCallBodyHandles, TCallbacks>(
            NarrowPhase<TCallbacks> narrowPhase, int manifoldTypeAsConstraintType, int workerIndex,
            ref CollidablePair pair, ContactManifold* manifold, ref TCollisionCache collisionCache, ref PairMaterialProperties material, TCallBodyHandles bodyHandles)
        {
            Contact2OneBody description;
            description.Contact0.OffsetA = manifold->Offset0;
            description.Contact0.PenetrationDepth = manifold->Depth0;
            description.Contact1.OffsetA = manifold->Offset1;
            description.Contact1.PenetrationDepth = manifold->Depth1;
            description.FrictionCoefficient = material.FrictionCoefficient;
            description.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
            description.SpringSettings = material.SpringSettings;
            description.Normal = manifold->ConvexNormal;

            UpdateConstraint(narrowPhase, manifoldTypeAsConstraintType, workerIndex, ref pair, manifold, ref collisionCache, ref description, bodyHandles);
        }
    }
    public class Contact3OneBodyAccessor : ContactConstraintAccessor<Contact3OneBody, int, Contact3AccumulatedImpulses, ContactImpulses3, ConstraintCache3>
    {
        public override unsafe void UpdateConstraintForManifold<TCollisionCache, TCallBodyHandles, TCallbacks>(
            NarrowPhase<TCallbacks> narrowPhase, int manifoldTypeAsConstraintType, int workerIndex,
            ref CollidablePair pair, ContactManifold* manifold, ref TCollisionCache collisionCache, ref PairMaterialProperties material, TCallBodyHandles bodyHandles)
        {
            Contact3OneBody description;
            description.Contact0.OffsetA = manifold->Offset0;
            description.Contact0.PenetrationDepth = manifold->Depth0;
            description.Contact1.OffsetA = manifold->Offset1;
            description.Contact1.PenetrationDepth = manifold->Depth1;
            description.Contact2.OffsetA = manifold->Offset2;
            description.Contact2.PenetrationDepth = manifold->Depth2;
            description.FrictionCoefficient = material.FrictionCoefficient;
            description.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
            description.SpringSettings = material.SpringSettings;
            description.Normal = manifold->ConvexNormal;

            UpdateConstraint(narrowPhase, manifoldTypeAsConstraintType, workerIndex, ref pair, manifold, ref collisionCache, ref description, bodyHandles);
        }
    }
    public class Contact4OneBodyAccessor : ContactConstraintAccessor<Contact4OneBody, int, Contact4AccumulatedImpulses, ContactImpulses4, ConstraintCache4>
    {
        public override unsafe void UpdateConstraintForManifold<TCollisionCache, TCallBodyHandles, TCallbacks>(
            NarrowPhase<TCallbacks> narrowPhase, int manifoldTypeAsConstraintType, int workerIndex,
            ref CollidablePair pair, ContactManifold* manifold, ref TCollisionCache collisionCache, ref PairMaterialProperties material, TCallBodyHandles bodyHandles)
        {
            Contact4OneBody description;
            description.Contact0.OffsetA = manifold->Offset0;
            description.Contact0.PenetrationDepth = manifold->Depth0;
            description.Contact1.OffsetA = manifold->Offset1;
            description.Contact1.PenetrationDepth = manifold->Depth1;
            description.Contact2.OffsetA = manifold->Offset2;
            description.Contact2.PenetrationDepth = manifold->Depth2;
            description.Contact3.OffsetA = manifold->Offset3;
            description.Contact3.PenetrationDepth = manifold->Depth3;
            description.FrictionCoefficient = material.FrictionCoefficient;
            description.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
            description.SpringSettings = material.SpringSettings;
            description.Normal = manifold->ConvexNormal;

            UpdateConstraint(narrowPhase, manifoldTypeAsConstraintType, workerIndex, ref pair, manifold, ref collisionCache, ref description, bodyHandles);
        }
    }
    public class Contact1Accessor : ContactConstraintAccessor<Contact1, TwoBodyHandles, Contact1AccumulatedImpulses, ContactImpulses1, ConstraintCache1>
    {
        public override unsafe void UpdateConstraintForManifold<TCollisionCache, TCallBodyHandles, TCallbacks>(
            NarrowPhase<TCallbacks> narrowPhase, int manifoldTypeAsConstraintType, int workerIndex,
            ref CollidablePair pair, ContactManifold* manifold, ref TCollisionCache collisionCache, ref PairMaterialProperties material, TCallBodyHandles bodyHandles)
        {
            Contact1 description;
            description.Contact0.OffsetA = manifold->Offset0;
            description.Contact0.PenetrationDepth = manifold->Depth0;
            description.OffsetB = manifold->OffsetB;
            description.FrictionCoefficient = material.FrictionCoefficient;
            description.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
            description.SpringSettings = material.SpringSettings;
            description.Normal = manifold->ConvexNormal;

            UpdateConstraint(narrowPhase, manifoldTypeAsConstraintType, workerIndex, ref pair, manifold, ref collisionCache, ref description, bodyHandles);
        }
    }
    public class Contact2Accessor : ContactConstraintAccessor<Contact2, TwoBodyHandles, Contact2AccumulatedImpulses, ContactImpulses2, ConstraintCache2>
    {
        public override unsafe void UpdateConstraintForManifold<TCollisionCache, TCallBodyHandles, TCallbacks>(
            NarrowPhase<TCallbacks> narrowPhase, int manifoldTypeAsConstraintType, int workerIndex,
            ref CollidablePair pair, ContactManifold* manifold, ref TCollisionCache collisionCache, ref PairMaterialProperties material, TCallBodyHandles bodyHandles)
        {
            Contact2 description;
            description.Contact0.OffsetA = manifold->Offset0;
            description.Contact0.PenetrationDepth = manifold->Depth0;
            description.Contact1.OffsetA = manifold->Offset1;
            description.Contact1.PenetrationDepth = manifold->Depth1;
            description.OffsetB = manifold->OffsetB;
            description.FrictionCoefficient = material.FrictionCoefficient;
            description.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
            description.SpringSettings = material.SpringSettings;
            description.Normal = manifold->ConvexNormal;

            UpdateConstraint(narrowPhase, manifoldTypeAsConstraintType, workerIndex, ref pair, manifold, ref collisionCache, ref description, bodyHandles);
        }
    }
    public class Contact3Accessor : ContactConstraintAccessor<Contact3, TwoBodyHandles, Contact3AccumulatedImpulses, ContactImpulses3, ConstraintCache3>
    {
        public override unsafe void UpdateConstraintForManifold<TCollisionCache, TCallBodyHandles, TCallbacks>(
            NarrowPhase<TCallbacks> narrowPhase, int manifoldTypeAsConstraintType, int workerIndex,
            ref CollidablePair pair, ContactManifold* manifold, ref TCollisionCache collisionCache, ref PairMaterialProperties material, TCallBodyHandles bodyHandles)
        {
            Contact3 description;
            description.Contact0.OffsetA = manifold->Offset0;
            description.Contact0.PenetrationDepth = manifold->Depth0;
            description.Contact1.OffsetA = manifold->Offset1;
            description.Contact1.PenetrationDepth = manifold->Depth1;
            description.Contact2.OffsetA = manifold->Offset2;
            description.Contact2.PenetrationDepth = manifold->Depth2;
            description.OffsetB = manifold->OffsetB;
            description.FrictionCoefficient = material.FrictionCoefficient;
            description.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
            description.SpringSettings = material.SpringSettings;
            description.Normal = manifold->ConvexNormal;

            UpdateConstraint(narrowPhase, manifoldTypeAsConstraintType, workerIndex, ref pair, manifold, ref collisionCache, ref description, bodyHandles);
        }
    }
    public class Contact4Accessor : ContactConstraintAccessor<Contact4, TwoBodyHandles, Contact4AccumulatedImpulses, ContactImpulses4, ConstraintCache4>
    {
        public override unsafe void UpdateConstraintForManifold<TCollisionCache, TCallBodyHandles, TCallbacks>(
            NarrowPhase<TCallbacks> narrowPhase, int manifoldTypeAsConstraintType, int workerIndex,
            ref CollidablePair pair, ContactManifold* manifold, ref TCollisionCache collisionCache, ref PairMaterialProperties material, TCallBodyHandles bodyHandles)
        {
            Contact4 description;
            description.Contact0.OffsetA = manifold->Offset0;
            description.Contact0.PenetrationDepth = manifold->Depth0;
            description.Contact1.OffsetA = manifold->Offset1;
            description.Contact1.PenetrationDepth = manifold->Depth1;
            description.Contact2.OffsetA = manifold->Offset2;
            description.Contact2.PenetrationDepth = manifold->Depth2;
            description.Contact3.OffsetA = manifold->Offset3;
            description.Contact3.PenetrationDepth = manifold->Depth3;
            description.OffsetB = manifold->OffsetB;
            description.FrictionCoefficient = material.FrictionCoefficient;
            description.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
            description.SpringSettings = material.SpringSettings;
            description.Normal = manifold->ConvexNormal;

            UpdateConstraint(narrowPhase, manifoldTypeAsConstraintType, workerIndex, ref pair, manifold, ref collisionCache, ref description, bodyHandles);
        }
    }

}
