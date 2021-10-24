using BepuPhysics.Constraints;
using BepuPhysics.Constraints.Contact;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
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
        public int ContactCount { get; protected set; }
        protected bool Convex;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void GatherOldImpulses(ref ConstraintReference constraintReference, float* oldImpulses)
        {
            BundleIndexing.GetBundleIndices(constraintReference.IndexInTypeBatch, out var bundleIndex, out var inner);
            ref var buffer = ref constraintReference.TypeBatch.AccumulatedImpulses;
            if (Convex)
            {
                //Note that we do not modify the friction accumulated impulses. This is just for simplicity- the impact of accumulated impulses on friction *should* be relatively
                //hard to notice compared to penetration impulses. TODO: We should, however, test this assumption.
                //Note that we assume that the tangent friction impulses always come first. This should be safe for now, but it is important to keep in mind for later.
                ref var start = ref GatherScatter.GetOffsetInstance(ref Unsafe.As<byte, Vector<float>>(ref buffer[AccumulatedImpulseBundleStrideInBytes * bundleIndex + Unsafe.SizeOf<Vector2Wide>()]), inner);
                for (int i = 0; i < ContactCount; ++i)
                {
                    oldImpulses[i] = Unsafe.Add(ref start, i)[0];
                }
            }
            else
            {
                ref var start = ref GatherScatter.GetOffsetInstance(ref Unsafe.As<byte, NonconvexAccumulatedImpulses>(ref buffer[AccumulatedImpulseBundleStrideInBytes * bundleIndex]), inner);
                for (int i = 0; i < ContactCount; ++i)
                {
                    oldImpulses[i] = Unsafe.Add(ref start, i).Penetration[0];
                }
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ScatterNewImpulses<TContactImpulses>(ref ConstraintReference constraintReference, ref TContactImpulses contactImpulses)
        {
            //Note that we do not modify the friction accumulated impulses. This is just for simplicity- the impact of accumulated impulses on friction *should* be relatively
            //hard to notice compared to penetration impulses. TODO: We should, however, test this assumption.
            BundleIndexing.GetBundleIndices(constraintReference.IndexInTypeBatch, out var bundleIndex, out var inner);
            ref var buffer = ref constraintReference.TypeBatch.AccumulatedImpulses;
            Debug.Assert(constraintReference.TypeBatch.TypeId == ConstraintTypeId);
            if (Convex)
            {
                //Note that we assume that the tangent friction impulses always come first. This should be safe for now, but it is important to keep in mind for later.
                ref var sourceStart = ref Unsafe.As<TContactImpulses, float>(ref contactImpulses);
                ref var targetStart = ref GatherScatter.GetOffsetInstance(ref Unsafe.As<byte, Vector<float>>(ref buffer[AccumulatedImpulseBundleStrideInBytes * bundleIndex + Unsafe.SizeOf<Vector2Wide>()]), inner);
                for (int i = 0; i < ContactCount; ++i)
                {
                    GatherScatter.GetFirst(ref Unsafe.Add(ref targetStart, i)) = Unsafe.Add(ref sourceStart, i);
                }
            }
            else
            {
                ref var sourceStart = ref Unsafe.As<TContactImpulses, float>(ref contactImpulses);
                ref var targetStart = ref GatherScatter.GetOffsetInstance(ref Unsafe.As<byte, NonconvexAccumulatedImpulses>(ref buffer[AccumulatedImpulseBundleStrideInBytes * bundleIndex]), inner);
                for (int i = 0; i < ContactCount; ++i)
                {
                    GatherScatter.GetFirst(ref Unsafe.Add(ref targetStart, i).Penetration) = Unsafe.Add(ref sourceStart, i);
                }
            }
        }

        public abstract void DeterministicallyAdd<TCallbacks>(
            int typeIndex, NarrowPhase<TCallbacks>.OverlapWorker[] overlapWorkers,
            ref QuickList<NarrowPhase<TCallbacks>.SortConstraintTarget> constraintsOfType,
            Simulation simulation, PairCache pairCache) where TCallbacks : struct, INarrowPhaseCallbacks;

        public abstract void FlushWithSpeculativeBatches<TCallbacks>(ref UntypedList list, int narrowPhaseConstraintTypeId,
            ref Buffer<Buffer<ushort>> speculativeBatchIndices, Simulation simulation, PairCache pairCache)
            where TCallbacks : struct, INarrowPhaseCallbacks;

        public abstract void FlushSequentially<TCallbacks>(ref UntypedList list, int narrowPhaseConstraintTypeId, Simulation simulation, PairCache pairCache)
            where TCallbacks : struct, INarrowPhaseCallbacks;

        public abstract unsafe void UpdateConstraintForManifold<TContactManifold, TCollisionCache, TCallBodyHandles, TCallbacks>(
            NarrowPhase<TCallbacks> narrowPhase, int manifoldTypeAsConstraintType, int workerIndex,
            ref CollidablePair pair, ref TContactManifold manifoldPointer, ref TCollisionCache collisionCache, ref PairMaterialProperties material, TCallBodyHandles bodyHandles)
            where TCallbacks : struct, INarrowPhaseCallbacks
            where TCollisionCache : unmanaged, IPairCacheEntry;

        /// <summary>
        /// Extracts references to data from a contact constraint of the accessor's type.
        /// </summary>
        /// <typeparam name="TExtractor">Type of the extractor to handle the extracted references.</typeparam>
        /// <param name="constraintHandle">Handle of the contact constraint to extract.</param>
        /// <param name="solver">Solver in which the constraint lives.</param>
        /// <param name="extractor">Extractor to handle the extracted references.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ExtractContactData<TExtractor>(ConstraintHandle constraintHandle, Solver solver, ref TExtractor extractor) where TExtractor : struct, ISolverContactDataExtractor
        {
            ExtractContactData(solver.HandleToConstraint[constraintHandle.Value], solver, ref extractor);
        }
        /// <summary>
        /// Extracts references to data from a contact constraint of the accessor's type.
        /// </summary>
        /// <typeparam name="TExtractor">Type of the extractor to handle the extracted references.</typeparam>
        /// <param name="constraintLocation">Location of the constraint in the solver.</param>
        /// <param name="solver">Solver in which the constraint lives.</param>
        /// <param name="extractor">Extractor to handle the extracted references.</param>
        public abstract void ExtractContactData<TExtractor>(in ConstraintLocation constraintLocation, Solver solver, ref TExtractor extractor) where TExtractor : struct, ISolverContactDataExtractor;

        /// <summary>
        /// Extracts references to data from a contact constraint of the accessor's type.
        /// </summary>
        /// <typeparam name="TExtractor">Type of the extractor to handle the extracted references.</typeparam>
        /// <param name="constraintHandle">Handle of the contact constraint to extract.</param>
        /// <param name="solver">Solver in which the constraint lives.</param>
        /// <param name="extractor">Extractor to handle the extracted references.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ExtractContactPrestepAndImpulses<TExtractor>(ConstraintHandle constraintHandle, Solver solver, ref TExtractor extractor) where TExtractor : struct, ISolverContactPrestepAndImpulsesExtractor
        {
            ExtractContactPrestepAndImpulses(solver.HandleToConstraint[constraintHandle.Value], solver, ref extractor);
        }
        /// <summary>
        /// Extracts references to data from a contact constraint of the accessor's type.
        /// </summary>
        /// <typeparam name="TExtractor">Type of the extractor to handle the extracted references.</typeparam>
        /// <param name="constraintLocation">Location of the constraint in the solver.</param>
        /// <param name="solver">Solver in which the constraint lives.</param>
        /// <param name="extractor">Extractor to handle the extracted references.</param>
        public abstract void ExtractContactPrestepAndImpulses<TExtractor>(in ConstraintLocation constraintLocation, Solver solver, ref TExtractor extractor) where TExtractor : struct, ISolverContactPrestepAndImpulsesExtractor;


    }

    //Note that the vast majority of the 'work' done by these accessor implementations is just type definitions used to call back into some other functions that need that type knowledge.
    public abstract class ContactConstraintAccessor<TConstraintDescription, TBodyHandles, TPrestepData, TAccumulatedImpulses, TContactImpulses, TConstraintCache> : ContactConstraintAccessor
        where TBodyHandles : unmanaged
        where TConstraintDescription : unmanaged, IConstraintDescription<TConstraintDescription>
        where TContactImpulses : unmanaged
        where TConstraintCache : unmanaged, IPairCacheEntry
        where TPrestepData : unmanaged
    {
        protected ContactConstraintAccessor()
        {
            Debug.Assert(
                typeof(TContactImpulses) == typeof(ContactImpulses1) ||
                typeof(TContactImpulses) == typeof(ContactImpulses2) ||
                typeof(TContactImpulses) == typeof(ContactImpulses3) ||
                typeof(TContactImpulses) == typeof(ContactImpulses4) ||
                typeof(TContactImpulses) == typeof(ContactImpulses5) ||
                typeof(TContactImpulses) == typeof(ContactImpulses6) ||
                typeof(TContactImpulses) == typeof(ContactImpulses7) ||
                typeof(TContactImpulses) == typeof(ContactImpulses8));
            ContactCount = Unsafe.SizeOf<TContactImpulses>() / Unsafe.SizeOf<float>();

            Convex =
                typeof(TConstraintDescription) == typeof(Contact1) ||
                typeof(TConstraintDescription) == typeof(Contact2) ||
                typeof(TConstraintDescription) == typeof(Contact3) ||
                typeof(TConstraintDescription) == typeof(Contact4) ||
                typeof(TConstraintDescription) == typeof(Contact1OneBody) ||
                typeof(TConstraintDescription) == typeof(Contact2OneBody) ||
                typeof(TConstraintDescription) == typeof(Contact3OneBody) ||
                typeof(TConstraintDescription) == typeof(Contact4OneBody);
            if (Convex)
            {
                Debug.Assert((ContactCount + 3) * Unsafe.SizeOf<Vector<float>>() == Unsafe.SizeOf<TAccumulatedImpulses>(),
                    "The layout of convex accumulated impulses seems to have changed; the assumptions of impulse gather/scatter are probably no longer valid.");
            }
            else
            {
                Debug.Assert(ContactCount * 3 * Unsafe.SizeOf<Vector<float>>() == Unsafe.SizeOf<TAccumulatedImpulses>(),
                    "The layout of nonconvex accumulated impulses seems to have changed; the assumptions of impulse gather/scatter are probably no longer valid.");
            }
            //Note that this test has to special case count == 1; 1 contact manifolds have no feature ids.
            Debug.Assert(Unsafe.SizeOf<TConstraintCache>() == sizeof(int) * (1 + ContactCount) &&
                default(TConstraintCache).CacheTypeId == ContactCount - 1,
                "The type of the constraint cache should hold as many contacts as the contact impulses requires.");
            AccumulatedImpulseBundleStrideInBytes = Unsafe.SizeOf<TAccumulatedImpulses>();
            ConstraintTypeId = default(TConstraintDescription).ConstraintTypeId;
        }
        public override void DeterministicallyAdd<TCallbacks>(int typeIndex, NarrowPhase<TCallbacks>.OverlapWorker[] overlapWorkers,
            ref QuickList<NarrowPhase<TCallbacks>.SortConstraintTarget> constraintsOfType,
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
            ref CollidablePair pair, ref TConstraintCache constraintCache, ref TCollisionCache collisionCache, ref TConstraintDescription description, TCallBodyHandles bodyHandles)
            where TCallbacks : struct, INarrowPhaseCallbacks where TCollisionCache : unmanaged, IPairCacheEntry
        {
            //Note that we let the user pass in a body handles type to a generic function, rather than requiring that the top level abstract class define the type.
            //That allows a type inconsistency, but it's easy to catch.
            Debug.Assert(typeof(TCallBodyHandles) == typeof(TBodyHandles), "Don't call an update with inconsistent body handle types.");
            narrowPhase.UpdateConstraint<TBodyHandles, TConstraintDescription, TContactImpulses, TCollisionCache, TConstraintCache>(
                workerIndex, ref pair, manifoldTypeAsConstraintType, ref constraintCache, ref collisionCache, ref description, Unsafe.As<TCallBodyHandles, TBodyHandles>(ref bodyHandles));
        }

        protected static void CopyContactData(ref ConvexContactManifold manifold, out TConstraintCache constraintCache, out TConstraintDescription description)
        {
            //TODO: Unnecessary zero inits. Unsafe.SkipInit would help here once available. Could also hack away with pointers.
            constraintCache = default;
            description = default;
            //This should be a compilation time constant provided an inlined constant property.
            var contactCount = constraintCache.CacheTypeId + 1;
            Debug.Assert(contactCount == manifold.Count, "Relying on generic specialization; should be the same value!");
            //Contact data comes first in the constraint description memory layout.
            ref var targetContacts = ref Unsafe.As<TConstraintDescription, ConstraintContactData>(ref description);
            ref var targetFeatureIds = ref Unsafe.Add(ref Unsafe.As<TConstraintCache, int>(ref constraintCache), 1);
            for (int i = 0; i < contactCount; ++i)
            {
                ref var sourceContact = ref Unsafe.Add(ref manifold.Contact0, i);
                ref var targetContact = ref Unsafe.Add(ref targetContacts, i);
                Unsafe.Add(ref targetFeatureIds, i) = sourceContact.FeatureId;
                targetContact.OffsetA = sourceContact.Offset;
                targetContact.PenetrationDepth = sourceContact.Depth;
            }
        }
        protected static void CopyContactData(ref NonconvexContactManifold manifold, ref TConstraintCache constraintCache, ref NonconvexConstraintContactData targetContacts)
        {
            //TODO: Check codegen. This should be a compilation time constant. If it's not, just use the ContactCount that we cached.
            var contactCount = constraintCache.CacheTypeId + 1;
            Debug.Assert(contactCount == manifold.Count, "Relying on generic specialization; should be the same value!");
            ref var targetFeatureIds = ref Unsafe.Add(ref Unsafe.As<TConstraintCache, int>(ref constraintCache), 1);
            for (int i = 0; i < contactCount; ++i)
            {
                ref var sourceContact = ref Unsafe.Add(ref manifold.Contact0, i);
                ref var targetContact = ref Unsafe.Add(ref targetContacts, i);
                Unsafe.Add(ref targetFeatureIds, i) = sourceContact.FeatureId;
                targetContact.OffsetA = sourceContact.Offset;
                targetContact.Normal = sourceContact.Normal;
                targetContact.PenetrationDepth = sourceContact.Depth;
            }
        }
        protected static void CopyContactData(ref NonconvexContactManifold manifold, ref TConstraintCache constraintCache, ref ConstraintContactData targetContacts)
        {
            Debug.Assert(manifold.Count == 1, "Nonconvex manifolds used to create convex constraints must only have one contact.");
            //TODO: Check codegen. This should be a compilation time constant. If it's not, just use the ContactCount that we cached.
            var contactCount = constraintCache.CacheTypeId + 1;
            Debug.Assert(contactCount == manifold.Count, "Relying on generic specialization; should be the same value!");
            Unsafe.Add(ref Unsafe.As<TConstraintCache, int>(ref constraintCache), 1) = manifold.Contact0.FeatureId;
            targetContacts.OffsetA = manifold.Contact0.Offset;
            targetContacts.PenetrationDepth = manifold.Contact0.Depth;
        }
    }


    public class ConvexOneBodyAccessor<TConstraintDescription, TPrestepData, TAccumulatedImpulses, TContactImpulses, TConstraintCache> :
        ContactConstraintAccessor<TConstraintDescription, int, TPrestepData, TAccumulatedImpulses, TContactImpulses, TConstraintCache>
        where TConstraintDescription : unmanaged, IConvexOneBodyContactConstraintDescription<TConstraintDescription>
        where TContactImpulses : unmanaged
        where TConstraintCache : unmanaged, IPairCacheEntry
        where TPrestepData : unmanaged, IConvexContactPrestep<TPrestepData>
        where TAccumulatedImpulses : unmanaged, IConvexContactAccumulatedImpulses<TAccumulatedImpulses>
    {
        public override void UpdateConstraintForManifold<TContactManifold, TCollisionCache, TCallBodyHandles, TCallbacks>(
            NarrowPhase<TCallbacks> narrowPhase, int manifoldTypeAsConstraintType, int workerIndex,
            ref CollidablePair pair, ref TContactManifold manifoldPointer, ref TCollisionCache collisionCache, ref PairMaterialProperties material, TCallBodyHandles bodyHandles)
        {
            Debug.Assert(typeof(TCallBodyHandles) == typeof(int));
            if (typeof(TContactManifold) == typeof(ConvexContactManifold))
            {
                ref var manifold = ref Unsafe.As<TContactManifold, ConvexContactManifold>(ref manifoldPointer);
                CopyContactData(ref manifold, out var constraintCache, out var description);
                description.CopyManifoldWideProperties(ref manifold.Normal, ref material);
                UpdateConstraint(narrowPhase, manifoldTypeAsConstraintType, workerIndex, ref pair, ref constraintCache, ref collisionCache, ref description, bodyHandles);
            }
            else
            {
                Debug.Assert(typeof(TContactManifold) == typeof(NonconvexContactManifold));
                ref var manifold = ref Unsafe.As<TContactManifold, NonconvexContactManifold>(ref manifoldPointer);
                Debug.Assert(manifold.Count == 1, "Nonconvex manifolds should only result in convex constraints when the contact count is 1.");
                Unsafe.SkipInit(out TConstraintCache constraintCache);
                Unsafe.SkipInit(out TConstraintDescription description);
                CopyContactData(ref manifold, ref constraintCache, ref description.GetFirstContact(ref description));
                description.CopyManifoldWideProperties(ref manifold.Contact0.Normal, ref material);
                UpdateConstraint(narrowPhase, manifoldTypeAsConstraintType, workerIndex, ref pair, ref constraintCache, ref collisionCache, ref description, bodyHandles);
            }
        }

        public override void ExtractContactData<TExtractor>(in ConstraintLocation constraintLocation, Solver solver, ref TExtractor extractor)
        {
            Debug.Assert(constraintLocation.TypeId == ConstraintTypeId);
            ref var batch = ref solver.Sets[constraintLocation.SetIndex].Batches[constraintLocation.BatchIndex];
            ref var typeBatch = ref batch.TypeBatches[batch.TypeIndexToTypeBatchIndex[constraintLocation.TypeId]];
            BundleIndexing.GetBundleIndices(constraintLocation.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            //Active constraints store body indices as references; inactive constraints store handles.
            var bodyReference = Buffer<Vector<int>>.Get(ref typeBatch.BodyReferences, bundleIndex)[innerIndex];
            var bodyHandle = constraintLocation.SetIndex == 0 ? solver.bodies.ActiveSet.IndexToHandle[bodyReference] : new BodyHandle(bodyReference);
            ref var prestep = ref GatherScatter.GetOffsetInstance(ref Buffer<TPrestepData>.Get(ref typeBatch.PrestepData, bundleIndex), innerIndex);
            ref var impulses = ref GatherScatter.GetOffsetInstance(ref Buffer<TAccumulatedImpulses>.Get(ref typeBatch.AccumulatedImpulses, bundleIndex), innerIndex);
            extractor.ConvexOneBody(bodyHandle, ref prestep, ref impulses);
        }

        public override void ExtractContactPrestepAndImpulses<TExtractor>(in ConstraintLocation constraintLocation, Solver solver, ref TExtractor extractor)
        {
            Debug.Assert(constraintLocation.TypeId == ConstraintTypeId);
            ref var batch = ref solver.Sets[constraintLocation.SetIndex].Batches[constraintLocation.BatchIndex];
            ref var typeBatch = ref batch.TypeBatches[batch.TypeIndexToTypeBatchIndex[constraintLocation.TypeId]];
            BundleIndexing.GetBundleIndices(constraintLocation.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            ref var prestep = ref GatherScatter.GetOffsetInstance(ref Buffer<TPrestepData>.Get(ref typeBatch.PrestepData, bundleIndex), innerIndex);
            ref var impulses = ref GatherScatter.GetOffsetInstance(ref Buffer<TAccumulatedImpulses>.Get(ref typeBatch.AccumulatedImpulses, bundleIndex), innerIndex);
            extractor.ConvexOneBody(ref prestep, ref impulses);
        }
    }

    public class ConvexTwoBodyAccessor<TConstraintDescription, TPrestepData, TAccumulatedImpulses, TContactImpulses, TConstraintCache> :
        ContactConstraintAccessor<TConstraintDescription, TwoBodyHandles, TPrestepData, TAccumulatedImpulses, TContactImpulses, TConstraintCache>
        where TConstraintDescription : unmanaged, IConvexTwoBodyContactConstraintDescription<TConstraintDescription>
        where TContactImpulses : unmanaged
        where TConstraintCache : unmanaged, IPairCacheEntry
        where TPrestepData : unmanaged, ITwoBodyConvexContactPrestep<TPrestepData>
        where TAccumulatedImpulses : unmanaged, IConvexContactAccumulatedImpulses<TAccumulatedImpulses>
    {
        public override void UpdateConstraintForManifold<TContactManifold, TCollisionCache, TCallBodyHandles, TCallbacks>(
            NarrowPhase<TCallbacks> narrowPhase, int manifoldTypeAsConstraintType, int workerIndex,
            ref CollidablePair pair, ref TContactManifold manifoldPointer, ref TCollisionCache collisionCache, ref PairMaterialProperties material, TCallBodyHandles bodyHandles)
        {
            Debug.Assert(typeof(TCallBodyHandles) == typeof(TwoBodyHandles));
            if (typeof(TContactManifold) == typeof(ConvexContactManifold))
            {
                ref var manifold = ref Unsafe.As<TContactManifold, ConvexContactManifold>(ref manifoldPointer);
                CopyContactData(ref manifold, out var constraintCache, out var description);
                description.CopyManifoldWideProperties(ref manifold.OffsetB, ref manifold.Normal, ref material);
                UpdateConstraint(narrowPhase, manifoldTypeAsConstraintType, workerIndex, ref pair, ref constraintCache, ref collisionCache, ref description, bodyHandles);
            }
            else
            {
                Debug.Assert(typeof(TContactManifold) == typeof(NonconvexContactManifold));
                ref var manifold = ref Unsafe.As<TContactManifold, NonconvexContactManifold>(ref manifoldPointer);
                Debug.Assert(manifold.Count == 1, "Nonconvex manifolds should only result in convex constraints when the contact count is 1.");
                Unsafe.SkipInit(out TConstraintCache constraintCache);
                Unsafe.SkipInit(out TConstraintDescription description);
                CopyContactData(ref manifold, ref constraintCache, ref description.GetFirstContact(ref description));
                description.CopyManifoldWideProperties(ref manifold.OffsetB, ref manifold.Contact0.Normal, ref material);
                UpdateConstraint(narrowPhase, manifoldTypeAsConstraintType, workerIndex, ref pair, ref constraintCache, ref collisionCache, ref description, bodyHandles);
            }
        }
        public override void ExtractContactData<TExtractor>(in ConstraintLocation constraintLocation, Solver solver, ref TExtractor extractor)
        {
            Debug.Assert(constraintLocation.TypeId == ConstraintTypeId);
            ref var batch = ref solver.Sets[constraintLocation.SetIndex].Batches[constraintLocation.BatchIndex];
            ref var typeBatch = ref batch.TypeBatches[batch.TypeIndexToTypeBatchIndex[constraintLocation.TypeId]];
            BundleIndexing.GetBundleIndices(constraintLocation.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            ref var prestep = ref GatherScatter.GetOffsetInstance(ref Buffer<TPrestepData>.Get(ref typeBatch.PrestepData, bundleIndex), innerIndex);
            ref var impulses = ref GatherScatter.GetOffsetInstance(ref Buffer<TAccumulatedImpulses>.Get(ref typeBatch.AccumulatedImpulses, bundleIndex), innerIndex);

            BodyHandle bodyHandleA, bodyHandleB;
            ref var bodyReferences = ref GatherScatter.GetOffsetInstance(ref Buffer<TwoBodyReferences>.Get(ref typeBatch.BodyReferences, bundleIndex), innerIndex);
            //Active constraints store body indices as references; inactive constraints store handles.
            if (constraintLocation.SetIndex == 0)
            {
                bodyHandleA = solver.bodies.ActiveSet.IndexToHandle[bodyReferences.IndexA[0]];
                bodyHandleB = solver.bodies.ActiveSet.IndexToHandle[bodyReferences.IndexB[0]];
            }
            else
            {
                bodyHandleA = new BodyHandle(bodyReferences.IndexA[0]);
                bodyHandleB = new BodyHandle(bodyReferences.IndexB[0]);
            }
            extractor.ConvexTwoBody(bodyHandleA, bodyHandleB, ref prestep, ref impulses);
        }

        public override void ExtractContactPrestepAndImpulses<TExtractor>(in ConstraintLocation constraintLocation, Solver solver, ref TExtractor extractor)
        {
            Debug.Assert(constraintLocation.TypeId == ConstraintTypeId);
            ref var batch = ref solver.Sets[constraintLocation.SetIndex].Batches[constraintLocation.BatchIndex];
            ref var typeBatch = ref batch.TypeBatches[batch.TypeIndexToTypeBatchIndex[constraintLocation.TypeId]];
            BundleIndexing.GetBundleIndices(constraintLocation.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            ref var prestep = ref GatherScatter.GetOffsetInstance(ref Buffer<TPrestepData>.Get(ref typeBatch.PrestepData, bundleIndex), innerIndex);
            ref var impulses = ref GatherScatter.GetOffsetInstance(ref Buffer<TAccumulatedImpulses>.Get(ref typeBatch.AccumulatedImpulses, bundleIndex), innerIndex);
            extractor.ConvexTwoBody(ref prestep, ref impulses);
        }
    }

    public class NonconvexOneBodyAccessor<TConstraintDescription, TPrestepData, TAccumulatedImpulses, TContactImpulses, TConstraintCache> :
        ContactConstraintAccessor<TConstraintDescription, int, TPrestepData, TAccumulatedImpulses, TContactImpulses, TConstraintCache>
        where TConstraintDescription : unmanaged, INonconvexOneBodyContactConstraintDescription<TConstraintDescription>
        where TContactImpulses : unmanaged
        where TConstraintCache : unmanaged, IPairCacheEntry
        where TPrestepData : unmanaged, INonconvexContactPrestep<TPrestepData>
        where TAccumulatedImpulses : unmanaged, INonconvexContactAccumulatedImpulses<TAccumulatedImpulses>
    {
        public override void UpdateConstraintForManifold<TContactManifold, TCollisionCache, TCallBodyHandles, TCallbacks>(
            NarrowPhase<TCallbacks> narrowPhase, int manifoldTypeAsConstraintType, int workerIndex,
            ref CollidablePair pair, ref TContactManifold manifoldPointer, ref TCollisionCache collisionCache, ref PairMaterialProperties material, TCallBodyHandles bodyHandles)
        {
            Debug.Assert(typeof(TCallBodyHandles) == typeof(int));
            ref var manifold = ref Unsafe.As<TContactManifold, NonconvexContactManifold>(ref manifoldPointer);
            Unsafe.SkipInit(out TConstraintCache constraintCache);
            Unsafe.SkipInit(out TConstraintDescription description);
            CopyContactData(ref manifold, ref constraintCache, ref description.GetFirstContact(ref description));
            description.CopyManifoldWideProperties(ref material);
            UpdateConstraint(narrowPhase, manifoldTypeAsConstraintType, workerIndex, ref pair, ref constraintCache, ref collisionCache, ref description, bodyHandles);
        }

        public override void ExtractContactData<TExtractor>(in ConstraintLocation constraintLocation, Solver solver, ref TExtractor extractor)
        {
            Debug.Assert(constraintLocation.TypeId == ConstraintTypeId);
            ref var batch = ref solver.Sets[constraintLocation.SetIndex].Batches[constraintLocation.BatchIndex];
            ref var typeBatch = ref batch.TypeBatches[batch.TypeIndexToTypeBatchIndex[constraintLocation.TypeId]];
            BundleIndexing.GetBundleIndices(constraintLocation.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            //Active constraints store body indices as references; inactive constraints store handles.
            var bodyReference = Buffer<Vector<int>>.Get(ref typeBatch.BodyReferences, bundleIndex)[innerIndex];
            var bodyHandle = constraintLocation.SetIndex == 0 ? solver.bodies.ActiveSet.IndexToHandle[bodyReference] : new BodyHandle(bodyReference);
            ref var prestep = ref GatherScatter.GetOffsetInstance(ref Buffer<TPrestepData>.Get(ref typeBatch.PrestepData, bundleIndex), innerIndex);
            ref var impulses = ref GatherScatter.GetOffsetInstance(ref Buffer<TAccumulatedImpulses>.Get(ref typeBatch.AccumulatedImpulses, bundleIndex), innerIndex);
            extractor.NonconvexOneBody(bodyHandle, ref prestep, ref impulses);
        }

        public override void ExtractContactPrestepAndImpulses<TExtractor>(in ConstraintLocation constraintLocation, Solver solver, ref TExtractor extractor)
        {
            Debug.Assert(constraintLocation.TypeId == ConstraintTypeId);
            ref var batch = ref solver.Sets[constraintLocation.SetIndex].Batches[constraintLocation.BatchIndex];
            ref var typeBatch = ref batch.TypeBatches[batch.TypeIndexToTypeBatchIndex[constraintLocation.TypeId]];
            BundleIndexing.GetBundleIndices(constraintLocation.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            ref var prestep = ref GatherScatter.GetOffsetInstance(ref Buffer<TPrestepData>.Get(ref typeBatch.PrestepData, bundleIndex), innerIndex);
            ref var impulses = ref GatherScatter.GetOffsetInstance(ref Buffer<TAccumulatedImpulses>.Get(ref typeBatch.AccumulatedImpulses, bundleIndex), innerIndex);
            extractor.NonconvexOneBody(ref prestep, ref impulses);
        }
    }

    public class NonconvexTwoBodyAccessor<TConstraintDescription, TPrestepData, TAccumulatedImpulses, TContactImpulses, TConstraintCache> :
        ContactConstraintAccessor<TConstraintDescription, TwoBodyHandles, TPrestepData, TAccumulatedImpulses, TContactImpulses, TConstraintCache>
        where TConstraintDescription : unmanaged, INonconvexTwoBodyContactConstraintDescription<TConstraintDescription>
        where TContactImpulses : unmanaged
        where TConstraintCache : unmanaged, IPairCacheEntry
        where TPrestepData : unmanaged, ITwoBodyNonconvexContactPrestep<TPrestepData>
        where TAccumulatedImpulses : unmanaged, INonconvexContactAccumulatedImpulses<TAccumulatedImpulses>
    {
        public override void UpdateConstraintForManifold<TContactManifold, TCollisionCache, TCallBodyHandles, TCallbacks>(
            NarrowPhase<TCallbacks> narrowPhase, int manifoldTypeAsConstraintType, int workerIndex,
            ref CollidablePair pair, ref TContactManifold manifoldPointer, ref TCollisionCache collisionCache, ref PairMaterialProperties material, TCallBodyHandles bodyHandles)
        {
            Debug.Assert(typeof(TCallBodyHandles) == typeof(TwoBodyHandles));
            ref var manifold = ref Unsafe.As<TContactManifold, NonconvexContactManifold>(ref manifoldPointer);
            Unsafe.SkipInit(out TConstraintCache constraintCache);
            Unsafe.SkipInit(out TConstraintDescription description);
            CopyContactData(ref manifold, ref constraintCache, ref description.GetFirstContact(ref description));
            description.CopyManifoldWideProperties(ref manifold.OffsetB, ref material);
            UpdateConstraint(narrowPhase, manifoldTypeAsConstraintType, workerIndex, ref pair, ref constraintCache, ref collisionCache, ref description, bodyHandles);
        }

        public override void ExtractContactData<TExtractor>(in ConstraintLocation constraintLocation, Solver solver, ref TExtractor extractor)
        {
            Debug.Assert(constraintLocation.TypeId == ConstraintTypeId);
            ref var batch = ref solver.Sets[constraintLocation.SetIndex].Batches[constraintLocation.BatchIndex];
            ref var typeBatch = ref batch.TypeBatches[batch.TypeIndexToTypeBatchIndex[constraintLocation.TypeId]];
            BundleIndexing.GetBundleIndices(constraintLocation.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            ref var prestep = ref GatherScatter.GetOffsetInstance(ref Buffer<TPrestepData>.Get(ref typeBatch.PrestepData, bundleIndex), innerIndex);
            ref var impulses = ref GatherScatter.GetOffsetInstance(ref Buffer<TAccumulatedImpulses>.Get(ref typeBatch.AccumulatedImpulses, bundleIndex), innerIndex);

            BodyHandle bodyHandleA, bodyHandleB;
            ref var bodyReferences = ref GatherScatter.GetOffsetInstance(ref Buffer<TwoBodyReferences>.Get(ref typeBatch.BodyReferences, bundleIndex), innerIndex);
            //Active constraints store body indices as references; inactive constraints store handles.
            if (constraintLocation.SetIndex == 0)
            {
                bodyHandleA = solver.bodies.ActiveSet.IndexToHandle[bodyReferences.IndexA[0]];
                bodyHandleB = solver.bodies.ActiveSet.IndexToHandle[bodyReferences.IndexB[0]];
            }
            else
            {
                bodyHandleA = new BodyHandle(bodyReferences.IndexA[0]);
                bodyHandleB = new BodyHandle(bodyReferences.IndexB[0]);
            }
            extractor.NonconvexTwoBody(bodyHandleA, bodyHandleB, ref prestep, ref impulses);
        }

        public override void ExtractContactPrestepAndImpulses<TExtractor>(in ConstraintLocation constraintLocation, Solver solver, ref TExtractor extractor)
        {
            Debug.Assert(constraintLocation.TypeId == ConstraintTypeId);
            ref var batch = ref solver.Sets[constraintLocation.SetIndex].Batches[constraintLocation.BatchIndex];
            ref var typeBatch = ref batch.TypeBatches[batch.TypeIndexToTypeBatchIndex[constraintLocation.TypeId]];
            BundleIndexing.GetBundleIndices(constraintLocation.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            ref var prestep = ref GatherScatter.GetOffsetInstance(ref Buffer<TPrestepData>.Get(ref typeBatch.PrestepData, bundleIndex), innerIndex);
            ref var impulses = ref GatherScatter.GetOffsetInstance(ref Buffer<TAccumulatedImpulses>.Get(ref typeBatch.AccumulatedImpulses, bundleIndex), innerIndex);
            extractor.NonconvexTwoBody(ref prestep, ref impulses);
        }
    }
}
