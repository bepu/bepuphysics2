using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace BepuPhysics.CollisionDetection
{


    public partial class NarrowPhase<TCallbacks> where TCallbacks : struct, INarrowPhaseCallbacks
    {
        public enum ConstraintGeneratorType
        {
            /// <summary>
            /// Pair which will directly produce constraints.
            /// </summary>
            Discrete = 0,
            /// <summary>
            /// Pair expecting both a discrete and inner sphere manifolds.
            /// </summary>
            Linear = 1,
            /// <summary>
            /// Pair expecting multiple discrete manifolds.
            /// </summary>
            Substep = 2,
            /// <summary>
            /// Pair expecting both inner sphere manifolds and multiple discrete manifolds.
            /// </summary>
            SubstepWithLinear = 3
        }

        public struct CollisionCallbacks : ICollisionCallbacks
        {
            int workerIndex;
            BufferPool pool;
            NarrowPhase<TCallbacks> narrowPhase;


            //TODO: Arguably, this could be handled within the streaming batcher. The main advantage of doing so would be CCD-like queries being available.
            //On the other hand, we may want to simply modularize it. Something like a low level convex batcher, which is used by the more general purpose discrete batcher
            //(which handles compounds, meshes, and boundary smoothing), which is used by the CCD batcher.
            unsafe static void ResolveLinearManifold<TMainManifold>(void* mainManifoldPointer, ConvexContactManifold* linearManifold, NonconvexContactManifold* outManifold)
            {
                //This function is responsible for prioritizing which contacts to include when there are more contacts than slots.
                //Note that the linear manifold is constructed from sphere-convex tests whose depths are guaranteed to be less than the deepest contact
                //of the main manifold by virtue of the contributing spheres being smaller than the containing shape.

                //Further, while it's possible for the sphere-sourced contacts to be deeper than some of the main manifold contacts due to speculative contact generation,
                //there is no strong reason to prefer the sphere source contacts just because their depth happens to be greater- in fact, if we trust the main manifold
                //contact generation logic, they should tend to be more representative of the true manifold.

                //In other words, we simply trust that all contacts in the main manifold are good choices, and the linear manifold's responsibility is simply to fill in gaps.
                //If there are no gaps, then we don't use the linear manifold at all.

                //So first, copy all main manifold contacts.
                Debug.Assert(linearManifold->Count == 2, "Inner sphere derived contacts should only ever contribute one contact per involved body, no more, no less.");
                var outContacts = &outManifold->Contact0;
                if (typeof(TMainManifold) == typeof(ConvexContactManifold))
                {
                    var mainManifold = (ConvexContactManifold*)mainManifoldPointer;
                    outManifold->OffsetB = mainManifold->OffsetB;
                    outManifold->Count = mainManifold->Count;
                    var mainManifoldContacts = &mainManifold->Contact0;
                    for (int i = 0; i < outManifold->Count; ++i)
                    {
                        ref var outContact = ref outContacts[i];
                        ref var mainContact = ref mainManifoldContacts[i];
                        outContact.Offset = mainContact.Offset;
                        outContact.Depth = mainContact.Depth;
                        outContact.Normal = mainManifold->Normal;
                        outContact.FeatureId = mainContact.FeatureId;
                    }
                }
                else
                {
                    Debug.Assert(typeof(TMainManifold) == typeof(NonconvexContactManifold));
                    var mainManifold = (NonconvexContactManifold*)mainManifoldPointer;
                    outManifold->OffsetB = mainManifold->OffsetB;
                    outManifold->Count = mainManifold->Count;
                    var mainManifoldContacts = &mainManifold->Contact0;
                    for (int i = 0; i < outManifold->Count; ++i)
                    {
                        outContacts[i] = mainManifoldContacts[i];
                    }
                }

                //Copy deepest contacts of linear manifold until no space remains.
                //Note that 'space remaining' differs depending on what kind of manifold we're creating.
                //If the source pair is a convex pair, then there is no reason to use more than four contacts.
                //If the source pair is a nonconvex pair, there may be more.

                //TODO: The dataflow between collision batcher and CCD handlers needs some work. This is left unfinished until we figure out those details.

            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            unsafe static void ResolveSubstepManifold(ref SubstepManifolds substeps, ConvexContactManifold* outManifold)
            {
                //Scan the substeps looking for the first substep that contains any contacts.
                //TODO: There are situations involving very high angular velocity where the first contacts are not the best choice. 
                //If this turns out to be a problem in practice, you may want to change the heuristic to prefer *approaching* contacts over merely existing contacts.
                //However, determining whether a contact is approaching requires computing the relative velocity at its position, which isn't free. (Not super expensive, but not free.)
                //For the sake of simplicity, just use 'first contact' for now.

                //TODO: If the first manifold is not full, you could also pull contacts from later substeps. They might help post-collision rotate-through-the-ground type penetration.

                float inverseCount = 1f / substeps.Manifolds.Count;
                for (int i = 0; i < substeps.Manifolds.Count; ++i)
                {
                    ref var manifold = ref substeps.Manifolds[i];
                    var contactCount = manifold.Count;
                    if (contactCount > 0)
                    {
                        *outManifold = manifold;
                        //Once the best substep is selected, transform the contact positions and depths to be relative to the poses at t=0.
                        //Since the contact position offsets are not rotated, all we have to do is add the offset from t=0 to the current time to each contact position
                        //and modify the penetration depths according to that offset along the normal.
                        var offset = substeps.RelativeOffsetChange * (i * inverseCount);
                        var contacts = &outManifold->Contact0;
                        var penetrationOffset = Vector3.Dot(offset, outManifold->Normal);
                        for (int j = 0; j < contactCount; ++j)
                        {
                            ref var contact = ref contacts[j];
                            contact.Offset += offset;
                            contact.Depth += penetrationOffset;
                        }
                        return;
                    }
                }
                //If there are no contacts, then just return an empty manifold.
                *outManifold = substeps.Manifolds[0];
                //TODO: This does not properly handle nonconvexes. Dataflow between collision batcher and CCD needs a rework to make inner sphere CCD clean.
            }

            struct SubstepManifolds
            {
                //TODO: Only handles convexes for now; need to revisit later.
                public QuickList<ConvexContactManifold, Buffer<ConvexContactManifold>> Manifolds;
                public Vector3 RelativeOffsetChange;
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void Initialize(BufferPool pool, int capacity)
                {
                    QuickList<ConvexContactManifold, Buffer<ConvexContactManifold>>.Create(pool.SpecializeFor<ConvexContactManifold>(), capacity, out Manifolds);
                }


            }
            struct DiscretePair
            {
                public CollidablePair Pair;
                public float SpeculativeMargin;

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void Initialize(ref CollidablePair pair, float speculativeMargin)
                {
                    Pair = pair;
                    SpeculativeMargin = speculativeMargin;
                }
            }
            struct LinearPair
            {
                //TODO: Only handles convexes for now; need to revisit later.
                public ConvexContactManifold DiscreteManifold;
                public NonconvexContactManifold LinearManifold;
                public CollidablePair Pair;
                public float SpeculativeMargin;
                public int ManifoldsReported;

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void Initialize(ref CollidablePair pair, float speculativeMargin)
                {
                    Pair = pair;
                    SpeculativeMargin = speculativeMargin;
                    ManifoldsReported = 0;
                    LinearManifold.Count = 2;
                }
            }

            struct SubstepPair
            {
                public SubstepManifolds Manifolds;
                public CollidablePair Pair;
                public float SpeculativeMargin;
                public int ManifoldsReported;

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void Initialize(BufferPool pool, int substepCapacity, ref CollidablePair pair, float speculativeMargin)
                {
                    Manifolds.Initialize(pool, substepCapacity);
                    Pair = pair;
                    SpeculativeMargin = speculativeMargin;
                    ManifoldsReported = 0;
                }
            }

            struct SubstepWithLinearPair
            {
                public SubstepManifolds SubstepManifolds;
                //TODO: Only handles convexes for now; need to revisit later.
                public NonconvexContactManifold LinearManifold;
                public CollidablePair Pair;
                public float SpeculativeMargin;
                public int ManifoldsReported;

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void Initialize(BufferPool pool, int substepCapacity, ref CollidablePair pair, float speculativeMargin)
                {
                    SubstepManifolds.Initialize(pool, substepCapacity);
                    Pair = pair;
                    ManifoldsReported = 0;
                    SpeculativeMargin = speculativeMargin;
                    LinearManifold.Count = 2;
                }
            }


            struct ContinuationCache<T>
            {
                public IdPool<Buffer<int>> Ids;
                public Buffer<T> Caches;

                public ContinuationCache(BufferPool pool)
                {
                    IdPool<Buffer<int>>.Create(pool.SpecializeFor<int>(), 32, out Ids);
                    pool.SpecializeFor<T>().Take(128, out Caches);
                }

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public ref T Allocate(BufferPool pool, out int index)
                {
                    index = Ids.Take();
                    if (Caches.Length <= index)
                    {
                        pool.SpecializeFor<T>().Resize(ref Caches, index + 1, Caches.Length);
                    }
                    return ref Caches[index];
                }

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void Return(int index, BufferPool pool)
                {
                    Ids.Return(index, pool.SpecializeFor<int>());
                }

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void Dispose(BufferPool pool)
                {
                    Ids.Dispose(pool.SpecializeFor<int>());
                    pool.SpecializeFor<T>().Return(ref Caches);
                }
            }

            ContinuationCache<DiscretePair> discrete;
            ContinuationCache<LinearPair> linear;
            ContinuationCache<SubstepPair> substep;
            ContinuationCache<SubstepWithLinearPair> substepWithLinear;

            public CollisionCallbacks(int workerIndex, BufferPool pool, NarrowPhase<TCallbacks> narrowPhase)
            {
                this.pool = pool;
                this.workerIndex = workerIndex;
                this.narrowPhase = narrowPhase;
                discrete = new ContinuationCache<DiscretePair>(pool);
                linear = new ContinuationCache<LinearPair>(pool);
                substep = new ContinuationCache<SubstepPair>(pool);
                substepWithLinear = new ContinuationCache<SubstepWithLinearPair>(pool);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public ContinuationIndex AddDiscrete(ref CollidablePair pair, float speculativeMargin)
            {
                discrete.Allocate(pool, out var index).Initialize(ref pair, speculativeMargin);
                return new ContinuationIndex((int)ConstraintGeneratorType.Discrete, index, 0);
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int AddLinear(ref CollidablePair pair, float speculativeMargin)
            {
                linear.Allocate(pool, out var index).Initialize(ref pair, speculativeMargin);
                return index;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int AddSubsteps(ref CollidablePair pair, int substepCount, float speculativeMargin)
            {
                substep.Allocate(pool, out var index).Initialize(pool, substepCount, ref pair, speculativeMargin);
                return index;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int AddSubstepsWithLinear(ref CollidablePair pair, int substepCount, float speculativeMargin)
            {
                substepWithLinear.Allocate(pool, out var index).Initialize(pool, substepCount, ref pair, speculativeMargin);
                return index;
            }

            static class CCDFeatureIdOffsets
            {
                public const int LinearA = 1 << 16;
                public const int LinearB = 1 << 17;
                //Substeps are simply a series of discrete steps, so you don't want to offset them and make actual discrete contacts unshared.
            }

            //[MethodImpl(MethodImplOptions.AggressiveInlining)]
            //private unsafe void FillLinearManifoldSlotA(ref ContactManifold linearManifold, ContactManifold* manifold)
            //{
            //    //Note that linear A is always in slot 0, and linear B is always in slot 1.
            //    //This is allowed because a linear pair is guaranteed to have two contacts generated from CCD.
            //    //There is no separation limit on inner sphere pairs, and you never just create one sphere-collidable pair- it's always bilateral.
            //    //Also note that offsetB is only used from linear A, not linear B. They should be identical.                       
            //    Debug.Assert(manifold->ContactCount == 1);
            //    linearManifold.OffsetB = manifold->OffsetB;
            //    linearManifold.Offset0 = manifold->Offset0;
            //    linearManifold.Depth0 = manifold->Depth0;
            //    linearManifold.FeatureId0 = CCDFeatureIdOffsets.LinearA;
            //    linearManifold.Normal0 = manifold->Normal0;

            //}
            //[MethodImpl(MethodImplOptions.AggressiveInlining)]
            //private unsafe void FillLinearManifoldSlotB(ref ContactManifold linearManifold, ContactManifold* manifold)
            //{
            //    Debug.Assert(manifold->ContactCount == 1);
            //    linearManifold.Offset1 = manifold->Offset0;
            //    linearManifold.Depth1 = manifold->Depth0;
            //    linearManifold.FeatureId1 = CCDFeatureIdOffsets.LinearB;
            //    linearManifold.Normal1 = manifold->Normal0;
            //}                       

            //Generic pointers are not allowed, so we have to do a bit of hackery.
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            unsafe void OnPairCompleted<TManifold>(int pairId, void* manifoldPointer)
            {
                var todoTestCollisionCache = default(EmptyCollisionCache);
                ContinuationIndex continuationId = new ContinuationIndex(pairId);
                Debug.Assert(continuationId.Exists);
                var continuationIndex = continuationId.Index;
                switch ((ConstraintGeneratorType)continuationId.Type)
                {
                    case ConstraintGeneratorType.Discrete:
                        {
                            //Direct has no need for accumulating multiple reports; we can immediately dispatch.
                            ref var continuation = ref discrete.Caches[continuationIndex];                            
                            narrowPhase.UpdateConstraintsForPair<TManifold, EmptyCollisionCache>(workerIndex, ref continuation.Pair, manifoldPointer, ref todoTestCollisionCache);
                            discrete.Return(continuationIndex, pool);
                        }
                        break;
                    //TODO: Note that we could avoid a copy on all the multi-manifold continuations-
                    //rather than creating a manifold and passing it in, we could let the user 'allocate' from the continuation
                    //and then fill the appropriate slot. However, this complicates the api- the user would then have to say 'okay im done' to trigger the flush.
                    case ConstraintGeneratorType.Linear:
                        {
                            //TODO: Dataflow between collision batcher and CCD needs a rework.
                            //ref var continuation = ref linear.Caches[continuationIndex];
                            //Debug.Assert(continuation.ManifoldsReported < 2 || manifold->OffsetB == continuation.LinearManifold.OffsetB,
                            //    "The offset from A to B should be the same in both LinearA and linearB. This requires a guarantee on the part of work submission; did you break that?");
                            //switch (continuationId.InnerIndex)
                            //{
                            //    case 0:
                            //        //Discrete manifolds obey speculative margin limitations.
                            //        ReduceDistantContacts(manifold, continuation.SpeculativeMargin, (ContactManifold*)Unsafe.AsPointer(ref continuation.DiscreteManifold));
                            //        break;
                            //    case 1:
                            //        FillLinearManifoldSlotA(ref continuation.LinearManifold, manifold);
                            //        break;
                            //    case 2:
                            //        FillLinearManifoldSlotB(ref continuation.LinearManifold, manifold);
                            //        break;
                            //}
                            //++continuation.ManifoldsReported;

                            //if (continuation.ManifoldsReported == 3)
                            //{
                            //    var manifolds = (ContactManifold*)Unsafe.AsPointer(ref continuation.DiscreteManifold);
                            //    ContactManifold combinedManifold;
                            //    ResolveLinearManifold(manifolds, manifolds + 1, &combinedManifold);
                            //    narrowPhase.UpdateConstraintsForPair(workerIndex, ref continuation.Pair, &combinedManifold, ref todoTestCollisionCache);
                            //    linear.Return(continuationIndex, pool);
                            //}
                        }
                        break;
                    case ConstraintGeneratorType.Substep:
                        {
                            //TODO: Dataflow between collision batcher and CCD needs a rework.
                            //ref var continuation = ref substep.Caches[continuationId.Index];
                            //Debug.Assert(continuationId.InnerIndex >= 0 && continuationId.InnerIndex < continuation.Manifolds.Manifolds.Count);
                            ////Every substep manifold obeys speculative margin limitations. This ensures a decent substep is chosen when reducing substeps to a final manifold.
                            //ReduceDistantContacts(manifold, continuation.SpeculativeMargin, (ContactManifold*)Unsafe.AsPointer(ref continuation.Manifolds.Manifolds[continuationId.InnerIndex]));
                            //++continuation.ManifoldsReported;

                            //if (continuation.ManifoldsReported == continuation.Manifolds.Manifolds.Count)
                            //{
                            //    ContactManifold resolvedManifold;
                            //    ResolveSubstepManifold(ref continuation.Manifolds, &resolvedManifold);
                            //    narrowPhase.UpdateConstraintsForPair(workerIndex, ref continuation.Pair, &resolvedManifold, ref todoTestCollisionCache);
                            //    substep.Return(continuationIndex, pool);
                            //}
                        }
                        break;
                    case ConstraintGeneratorType.SubstepWithLinear:
                        {
                            //TODO: Dataflow between collision batcher and CCD needs a rework.
                            //ref var continuation = ref this.substepWithLinear.Caches[continuationId.Index];
                            //var innerIndex = continuationId.InnerIndex;
                            //switch (innerIndex)
                            //{
                            //    case 0:
                            //        FillLinearManifoldSlotA(ref continuation.LinearManifold, manifold);
                            //        break;
                            //    case 1:
                            //        FillLinearManifoldSlotB(ref continuation.LinearManifold, manifold);
                            //        break;
                            //    default:
                            //        var substepIndex = innerIndex - 2;
                            //        Debug.Assert(substepIndex >= 0 && substepIndex < continuation.SubstepManifolds.Manifolds.Count);
                            //        //Every substep manifold obeys speculative margin limitations. This ensures a decent substep is chosen when reducing substeps to a final manifold.
                            //        ReduceDistantContacts(manifold, continuation.SpeculativeMargin, (ContactManifold*)Unsafe.AsPointer(ref continuation.SubstepManifolds.Manifolds[substepIndex]));
                            //        break;
                            //}
                            //++continuation.ManifoldsReported;

                            //if (continuation.ManifoldsReported == continuation.SubstepManifolds.Manifolds.Count + 2)
                            //{
                            //    ContactManifold substepManifold, completeManifold;
                            //    ResolveSubstepManifold(ref continuation.SubstepManifolds, &substepManifold);
                            //    ResolveLinearManifold(&substepManifold, (ContactManifold*)Unsafe.AsPointer(ref continuation.LinearManifold), &completeManifold);
                            //    narrowPhase.UpdateConstraintsForPair(workerIndex, ref continuation.Pair, &completeManifold, ref todoTestCollisionCache);
                            //    substepWithLinear.Return(continuationIndex, pool);
                            //}
                        }
                        break;
                }

            }

            public unsafe void OnPairCompleted(int pairId, NonconvexContactManifold* manifold)
            {
                OnPairCompleted<NonconvexContactManifold>(pairId, manifold);
            }

            public unsafe void OnPairCompleted(int pairId, ConvexContactManifold* manifold)
            {
                OnPairCompleted<ConvexContactManifold>(pairId, manifold);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            CollidablePair GetCollidablePair(int pairId)
            {
                var continuation = new ContinuationIndex(pairId);
                Debug.Assert(continuation.Exists);
                var index = continuation.Index;
                switch ((ConstraintGeneratorType)continuation.Type)
                {
                    case ConstraintGeneratorType.Discrete:
                        return discrete.Caches[index].Pair;
                    case ConstraintGeneratorType.Linear:
                        return linear.Caches[index].Pair;
                    case ConstraintGeneratorType.Substep:
                        return substep.Caches[index].Pair;
                    case ConstraintGeneratorType.SubstepWithLinear:
                        return substepWithLinear.Caches[index].Pair;
                }
                Debug.Fail("Invalid collision continuation type. Corrupted data?");
                return new CollidablePair();

            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowCollisionTesting(int pairId, int childA, int childB)
            {
                return narrowPhase.Callbacks.AllowContactGeneration(workerIndex, GetCollidablePair(pairId), childA, childB);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe void OnChildPairCompleted(int pairId, int childA, int childB, ConvexContactManifold* manifold)
            {
                narrowPhase.Callbacks.ConfigureContactManifold(workerIndex, GetCollidablePair(pairId), childA, childB, manifold);
            }

            internal void Dispose()
            {
                discrete.Dispose(pool);
                linear.Dispose(pool);
                substep.Dispose(pool);
                substepWithLinear.Dispose(pool);
            }
        }


    }


}
