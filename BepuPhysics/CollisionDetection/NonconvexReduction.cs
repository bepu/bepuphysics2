using BepuUtilities;
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
    public struct NonconvexReductionChild
    {
        public ConvexContactManifold Manifold;
        /// <summary>
        /// Offset from the origin of the first shape's parent to the child's location in world space. If there is no parent, this is the zero vector.
        /// </summary>
        public Vector3 OffsetA;
        public int ChildIndexA;
        /// <summary>
        /// Offset from the origin of the second shape's parent to the child's location in world space. If there is no parent, this is the zero vector.
        /// </summary>
        public Vector3 OffsetB;
        public int ChildIndexB;
    }

    public struct NonconvexReduction : ICollisionTestContinuation
    {
        public int ChildCount;
        public int CompletedChildCount;
        public Buffer<NonconvexReductionChild> Children;

        public void Create(int childManifoldCount, BufferPool pool)
        {
            ChildCount = childManifoldCount;
            CompletedChildCount = 0;
            pool.Take(childManifoldCount, out Children);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void AddContact(NonconvexContactManifold* manifold, ref NonconvexReductionChild sourceChild,
            ref Vector3 offset, float depth, ref Vector3 normal, int featureId)
        {
            ref var target = ref NonconvexContactManifold.Allocate(manifold);
            target.Offset = offset + sourceChild.OffsetA;
            target.Normal = normal;
            //Mix the convex-generated feature id with the child indices.
            target.FeatureId = (featureId ^ (sourceChild.ChildIndexA << 8)) ^ (sourceChild.ChildIndexB << 16);
            target.Depth = depth;
        }

        unsafe void ChooseBadly(NonconvexContactManifold* reducedManifold)
        {
            for (int i = 0; i < ChildCount; ++i)
            {
                ref var child = ref Children[i];
                ref var contactBase = ref child.Manifold.Contact0;
                for (int j = 0; j < child.Manifold.Count; ++j)
                {
                    ref var contact = ref Unsafe.Add(ref contactBase, j);
                    AddContact(reducedManifold, ref child, ref contact.Offset, contact.Depth, ref child.Manifold.Normal, contact.FeatureId);
                    if (reducedManifold->Count == 8)
                        break;
                }
                if (reducedManifold->Count == 8)
                    break;
            }

        }
        unsafe void ChooseDeepest(NonconvexContactManifold* manifold)
        {
            //Note that we're relying on stackalloc being efficient here. That's not actually a good idea unless you strip the stackalloc's init.
            var contactCountUpperBound = ChildCount * 4;
            int contactCount = 0;
            var depths = stackalloc float[contactCountUpperBound];
            var indices = stackalloc Int2[contactCountUpperBound];
            for (int i = 0; i < ChildCount; ++i)
            {
                ref var child = ref Children[i];
                ref var contactBase = ref child.Manifold.Contact0;
                for (int j = 0; j < child.Manifold.Count; ++j)
                {
                    var contactIndex = contactCount++;
                    ref var contactIndices = ref indices[contactIndex];
                    contactIndices.X = i;
                    contactIndices.Y = j;
                    depths[contactIndex] = Unsafe.Add(ref contactBase, j).Depth;
                }
            }

            var comparer = default(PrimitiveComparer<float>);
            QuickSort.Sort(ref *depths, ref *indices, 0, contactCount - 1, ref comparer);
            var newCount = contactCount < 8 ? contactCount : 8;
            var minimumIndex = contactCount - newCount;
            for (int i = contactCount - 1; i >= minimumIndex; --i)
            {
                ref var contactIndices = ref indices[i];
                ref var child = ref Children[contactIndices.X];
                ref var contact = ref Unsafe.Add(ref child.Manifold.Contact0, contactIndices.Y);
                AddContact(manifold, ref child, ref contact.Offset, contact.Depth, ref child.Manifold.Normal, contact.FeatureId);
            }


        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void FlushIfCompleted<TCallbacks>(int pairId, ref CollisionBatcher<TCallbacks> batcher) where TCallbacks : struct, ICollisionCallbacks
        {
            ++CompletedChildCount;
            Debug.Assert(ChildCount > 0);
            if (ChildCount == CompletedChildCount)
            {
                //This continuation is ready for processing. Find which contact manifold to report.
                int populatedChildManifolds = 0;
                //We cache an index in case there is only one populated manifold. Order of discovery doesn't matter- this value only gets used when there's one manifold.
                int samplePopulatedChildIndex = 0;
                for (int i = 0; i < ChildCount; ++i)
                {
                    if (Children[i].Manifold.Count > 0)
                    {
                        ++populatedChildManifolds;
                        samplePopulatedChildIndex = i;
                    }
                }
                var sampleChild = (NonconvexReductionChild*)Children.Memory + samplePopulatedChildIndex;
                if (populatedChildManifolds > 1)
                {
                    //There are multiple contributing child manifolds, so just assume that the resulting manifold is going to be nonconvex.
                    NonconvexContactManifold reducedManifold;
                    //We should assume that the stack memory backing the reduced manifold is uninitialized. We rely on the count, so initialize it manually.
                    reducedManifold.Count = 0;

                    //ChooseBadly(&reducedManifold);
                    ChooseDeepest(&reducedManifold);

                    //The manifold offsetB is the offset from shapeA origin to shapeB origin.
                    reducedManifold.OffsetB = sampleChild->Manifold.OffsetB - sampleChild->OffsetB + sampleChild->OffsetA;
                    batcher.Callbacks.OnPairCompleted(pairId, &reducedManifold);
                }
                else
                {
                    //Two possibilities here: 
                    //1) populatedChildManifolds == 1, and samplePopulatedChildIndex is the index of that sole populated manifold. We can directly report it.
                    //It's useful to directly report the convex child manifold for performance reasons- convex constraints do not require multiple normals and use a faster friction model.
                    //2) populatedChildManifolds == 0, and samplePopulatedChildIndex is 0. Given that we know this continuation is only used when there is at least one manifold expected
                    //and that we can only hit this codepath if all manifolds are empty, reporting manifold 0 is perfectly fine.
                    //The manifold offsetB is the offset from shapeA origin to shapeB origin.
                    sampleChild->Manifold.OffsetB = sampleChild->Manifold.OffsetB - sampleChild->OffsetB + sampleChild->OffsetA;
                    var contacts = &sampleChild->Manifold.Contact0;
                    for (int i = 0; i < sampleChild->Manifold.Count; ++i)
                    {
                        contacts[i].Offset += sampleChild->OffsetA;
                    }
                    batcher.Callbacks.OnPairCompleted(pairId, &sampleChild->Manifold);
                }
                batcher.Pool.ReturnUnsafely(Children.Id);
#if DEBUG
                //This makes it a little easier to detect invalid accesses that occur after disposal.
                this = new NonconvexReduction();
#endif
            }
        }
        public unsafe void OnChildCompleted<TCallbacks>(ref PairContinuation report, ConvexContactManifold* manifold, ref CollisionBatcher<TCallbacks> batcher)
            where TCallbacks : struct, ICollisionCallbacks
        {
            Children[report.ChildIndex].Manifold = *manifold;
            FlushIfCompleted(report.PairId, ref batcher);
        }

        public void OnChildCompletedEmpty<TCallbacks>(ref PairContinuation report, ref CollisionBatcher<TCallbacks> batcher) where TCallbacks : struct, ICollisionCallbacks
        {
            Children[report.ChildIndex].Manifold.Count = 0;
            FlushIfCompleted(report.PairId, ref batcher);
        }
    }
}
