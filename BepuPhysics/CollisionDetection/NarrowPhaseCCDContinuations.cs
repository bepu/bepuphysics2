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
            /// Pair which samples a swept location for contacts and needs to be rewound to compute proper speculative depths in a post process.
            /// </summary>
            Continuous = 1,
        }

        public struct CollisionCallbacks : ICollisionCallbacks
        {
            int workerIndex;
            BufferPool pool;
            NarrowPhase<TCallbacks> narrowPhase;


            struct DiscretePair
            {
                public CollidablePair Pair;

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void Initialize(ref CollidablePair pair)
                {
                    Pair = pair;
                }
            }

            struct ContinuousPair
            {
                public CollidablePair Pair;
                public Vector3 RelativeLinearVelocity;
                public Vector3 AngularA;
                public Vector3 AngularB;
                public float T;

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void Initialize(ref CollidablePair pair, in Vector3 relativeLinearVelocity, in Vector3 angularVelocityA, in Vector3 angularVelocityB, float t)
                {
                    Pair = pair;
                    AngularA = angularVelocityA;
                    AngularB = angularVelocityB;
                    RelativeLinearVelocity = relativeLinearVelocity;
                    T = t;
                }
            }


            struct ContinuationCache<T> where T : unmanaged
            {
                public IdPool Ids;
                public Buffer<T> Caches;

                public ContinuationCache(BufferPool pool)
                {
                    Ids = new IdPool(32, pool);
                    pool.TakeAtLeast(128, out Caches);
                }

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public ref T Allocate(BufferPool pool, out int index)
                {
                    index = Ids.Take();
                    if (Caches.Length <= index)
                    {
                        pool.ResizeToAtLeast(ref Caches, index + 1, Caches.Length);
                    }
                    return ref Caches[index];
                }

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void Return(int index, BufferPool pool)
                {
                    Ids.Return(index, pool);
                }

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void Dispose(BufferPool pool)
                {
                    Ids.Dispose(pool);
                    pool.Return(ref Caches);
                }
            }

            ContinuationCache<DiscretePair> discrete;
            ContinuationCache<ContinuousPair> continuous;

            public CollisionCallbacks(int workerIndex, BufferPool pool, NarrowPhase<TCallbacks> narrowPhase)
            {
                this.pool = pool;
                this.workerIndex = workerIndex;
                this.narrowPhase = narrowPhase;
                discrete = new ContinuationCache<DiscretePair>(pool);
                continuous = new ContinuationCache<ContinuousPair>(pool);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public CCDContinuationIndex AddDiscrete(ref CollidablePair pair)
            {
                discrete.Allocate(pool, out var index).Initialize(ref pair);
                return new CCDContinuationIndex((int)ConstraintGeneratorType.Discrete, index);
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public CCDContinuationIndex AddContinuous(ref CollidablePair pair, in Vector3 relativeLinearVelocity, in Vector3 angularVelocityA, in Vector3 angularVelocityB, float t)
            {
                continuous.Allocate(pool, out var index).Initialize(ref pair, relativeLinearVelocity, angularVelocityA, angularVelocityB, t);
                return new CCDContinuationIndex((int)ConstraintGeneratorType.Continuous, index);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe void OnPairCompleted<TManifold>(int pairId, ref TManifold manifoldReference) where TManifold : unmanaged, IContactManifold<TManifold>
            {
                var todoTestCollisionCache = default(EmptyCollisionCache);
                CCDContinuationIndex continuationId = new CCDContinuationIndex(pairId);
                Debug.Assert(continuationId.Exists);
                var continuationIndex = continuationId.Index;
#if DEBUG && CHECKMATH
                //Check all contact data for invalid data early so that we don't end up spewing NaNs all over the engine and catching in the broad phase or some other highly indirect location.
                for (int i = 0; i < manifoldReference.Count; ++i)
                {
                    manifoldReference.GetDepth(ref manifoldReference, i).Validate();
                    ref var normal = ref manifoldReference.GetNormal(ref manifoldReference, i);
                    normal.Validate();
                    Debug.Assert(Math.Abs(normal.LengthSquared() - 1) < 1e-5f, "Normals should be unit length. Something's gone wrong!");
                    manifoldReference.GetOffset(ref manifoldReference, i).Validate();
                }
#endif
                switch ((ConstraintGeneratorType)continuationId.Type)
                {
                    case ConstraintGeneratorType.Discrete:
                        {
                            //Direct has no need for accumulating multiple reports; we can immediately dispatch.
                            ref var continuation = ref discrete.Caches[continuationIndex];
                            narrowPhase.UpdateConstraintsForPair(workerIndex, ref continuation.Pair, ref manifoldReference, ref todoTestCollisionCache);
                            discrete.Return(continuationIndex, pool);
                        }
                        break;
                    case ConstraintGeneratorType.Continuous:
                        {
                            ref var continuation = ref continuous.Caches[continuationIndex];
                            //The manifold we received is for a future point in time. We need to rewind it to the timestep start for consistency.
                            //Treat all the offsets as unchanged, but update the depths according to relative motion.
                            if (typeof(TManifold) == typeof(ConvexContactManifold))
                            {
                                ref var manifold = ref Unsafe.As<TManifold, ConvexContactManifold>(ref manifoldReference);
                                for (int i = 0; i < manifold.Count; ++i)
                                {
                                    ref var contact = ref Unsafe.Add(ref manifold.Contact0, i);
                                    var angularContributionA = Vector3.Cross(continuation.AngularA, contact.Offset);
                                    var angularContributionB = Vector3.Cross(continuation.AngularB, contact.Offset - manifold.OffsetB);
                                    var velocityAtContact = Vector3.Dot(angularContributionB - angularContributionA + continuation.RelativeLinearVelocity, manifold.Normal);
                                    contact.Depth -= velocityAtContact * continuation.T;
                                }
                            }
                            else
                            {
                                Debug.Assert(typeof(TManifold) == typeof(NonconvexContactManifold));
                                ref var manifold = ref Unsafe.As<TManifold, NonconvexContactManifold>(ref manifoldReference);
                                for (int i = 0; i < manifold.Count; ++i)
                                {
                                    ref var contact = ref Unsafe.Add(ref manifold.Contact0, i);
                                    var angularContributionA = Vector3.Cross(continuation.AngularA, contact.Offset);
                                    var angularContributionB = Vector3.Cross(continuation.AngularB, contact.Offset - manifold.OffsetB);
                                    var velocityAtContact = Vector3.Dot(angularContributionB - angularContributionA + continuation.RelativeLinearVelocity, contact.Normal);
                                    contact.Depth -= velocityAtContact * continuation.T;
                                }
                            }
                            narrowPhase.UpdateConstraintsForPair(workerIndex, ref continuation.Pair, ref manifoldReference, ref todoTestCollisionCache);
                            continuous.Return(continuationIndex, pool);
                        }
                        break;
                }

            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            CollidablePair GetCollidablePair(int pairId)
            {
                var continuation = new CCDContinuationIndex(pairId);
                Debug.Assert(continuation.Exists);
                var index = continuation.Index;
                switch ((ConstraintGeneratorType)continuation.Type)
                {
                    case ConstraintGeneratorType.Discrete:
                        return discrete.Caches[index].Pair;
                    case ConstraintGeneratorType.Continuous:
                        return continuous.Caches[index].Pair;
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
            public unsafe void OnChildPairCompleted(int pairId, int childA, int childB, ref ConvexContactManifold manifold)
            {
                narrowPhase.Callbacks.ConfigureContactManifold(workerIndex, GetCollidablePair(pairId), childA, childB, ref manifold);
            }

            internal void Dispose()
            {
                discrete.Dispose(pool);
                continuous.Dispose(pool);
            }
        }


    }


}
