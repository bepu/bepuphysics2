using BepuPhysics.CollisionDetection.CollisionTasks;
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
        unsafe static void AddContact(ref NonconvexReductionChild child, int contactIndexInChild, NonconvexContactManifold* targetManifold)
        {
            ref var contact = ref Unsafe.Add(ref child.Manifold.Contact0, contactIndexInChild);
            ref var target = ref NonconvexContactManifold.Allocate(targetManifold);
            target.Offset = contact.Offset;
            target.Depth = contact.Depth;
            target.Normal = child.Manifold.Normal;
            //Mix the convex-generated feature id with the child indices.
            target.FeatureId = contact.FeatureId ^ ((child.ChildIndexA << 8) ^ (child.ChildIndexB << 16));
        }
        struct RemainingCandidate
        {
            public int ChildIndex;
            public int ContactIndex;
            public float Distinctiveness;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static void UseContact(ref QuickList<RemainingCandidate> remainingCandidates, int index, ref Buffer<NonconvexReductionChild> children, NonconvexContactManifold* targetManifold)
        {
            ref var candidate = ref remainingCandidates[index];
            AddContact(ref children[candidate.ChildIndex], candidate.ContactIndex, targetManifold);
            remainingCandidates.FastRemoveAt(index);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static float ComputeDistinctiveness(in ConvexContact candidate, in Vector3 contactNormal, in NonconvexContact reducedContact, float distanceSquaredInterpolationMin, float inverseDistanceSquaredInterpolationSpan, float depthScale)
        {
            //The more distant a contact is from another contact, or the more different its normal is, the more distinct it is considered.
            //The goal is for distinctiveness to range from around 0 to 2. The exact values aren't extremely important- we just want a rough range
            //so that we can meaningfully blend in a depth heuristic.
            var normalDot = Vector3.Dot(contactNormal, reducedContact.Normal);
            const float normalInterpolationSpan = -0.99999f;
            //Normal dots above a threshold are considered completely redundant. acos(0.99999) is about 0.25 degrees.
            //Normals pointing at a 90 degree angle are given a value of ~1, while being completely opposed gives a value of ~2.
            var normalDistinctiveness = (normalDot - 0.99999f) * (1f / normalInterpolationSpan);

            //Below a threshold, the offset is considered completely redundant.
            var offsetDistinctiveness = ((reducedContact.Offset - candidate.Offset).LengthSquared() - distanceSquaredInterpolationMin) * inverseDistanceSquaredInterpolationSpan;

            //We use a three way max across the normal, offset, and normal * offset distinctiveness. Both normal and offset scores can go above 1 sometimes, so it's possible for normal * offset to be higher than the other two.
            //Note that a point in the exact same position but a significantly different normal is still considered distinct.
            var combinedDistinctiveness = normalDistinctiveness * offsetDistinctiveness;
            var distinctiveness = offsetDistinctiveness > normalDistinctiveness ? offsetDistinctiveness : normalDistinctiveness;
            if (distinctiveness < combinedDistinctiveness)
                distinctiveness = combinedDistinctiveness;

            if (distinctiveness <= 0)
            {
                distinctiveness = 0;
            }
            else
            {
                //Take into account the depth of the contact. Deeper contacts are more important to the manifold.
                var depthMultiplier = 1 + candidate.Depth * depthScale;
                if (depthMultiplier < 0.01f)
                    depthMultiplier = 0.01f;
                distinctiveness *= depthMultiplier;
            }
            return distinctiveness;
        }

        unsafe void ChooseMostDistinct(NonconvexContactManifold* manifold, BufferPool pool)
        {
            //The end goal of contact reduction is to choose a reasonably stable subset of contacts which offer the greatest degree of constraint.
            //Computing a globally optimal solution to this would be pretty expensive for any given scoring mechanism, so we'll make some simplifications:
            //1) Heuristically select a starting point that will be reasonably stable between frames. Add it to the reduced manifold.
            //2) Search for the contact in the unadded set which maximizes the constraint heuristic and add it. Remove it from the unadded set.
            //3) Repeat #2 until there is no room left in the reduced manifold or all remaining candidates are redundant with the reduced manifold.

            //We first compute some calibration data. Knowing the approximate center of the manifold and the maximum distance allows more informed thresholds.

            var extentAxis = new Vector3(0.280454652f, 0.55873544499f, 0.7804869574f);
            float minimumExtent = float.MaxValue;
            Vector3 minimumExtentPosition = default;
            for (int i = 0; i < ChildCount; ++i)
            {
                ref var child = ref Children[i];
                for (int j = 0; j < child.Manifold.Count; ++j)
                {
                    ref var position = ref Unsafe.Add(ref child.Manifold.Contact0, j).Offset;
                    var extent = Vector3.Dot(position, extentAxis);
                    if (extent < minimumExtent)
                    {
                        minimumExtent = extent;
                        minimumExtentPosition = position;
                    }
                }
            }
            float maximumDistanceSquared = 0;
            for (int i = 0; i < ChildCount; ++i)
            {
                ref var child = ref Children[i];
                for (int j = 0; j < child.Manifold.Count; ++j)
                {
                    var distanceSquared = (Unsafe.Add(ref child.Manifold.Contact0, j).Offset - minimumExtentPosition).LengthSquared();
                    if (distanceSquared > maximumDistanceSquared)
                        maximumDistanceSquared = distanceSquared;
                }
            }
            var maximumDistance = (float)Math.Sqrt(maximumDistanceSquared);
            float initialBestScore = -float.MaxValue;
            int initialBestScoreIndex = 0;
            var maximumAllocatedCandidateCount = ChildCount * 4;
            Buffer<RemainingCandidate> remainingContactsBuffer;
            const int heapAllocationThreshold = 8192;
            if (maximumAllocatedCandidateCount > heapAllocationThreshold)
            {
                pool.Take(maximumAllocatedCandidateCount, out remainingContactsBuffer);
            }
            else
            {
                var memory = stackalloc RemainingCandidate[maximumAllocatedCandidateCount];
                remainingContactsBuffer = new Buffer<RemainingCandidate>(memory, maximumAllocatedCandidateCount);
            }
            var remainingContacts = new QuickList<RemainingCandidate>(remainingContactsBuffer);
            var extremityScale = maximumDistance * 5e-3f;
            for (int childIndex = 0; childIndex < ChildCount; ++childIndex)
            {
                ref var child = ref Children[childIndex];
                ref var childContactsBase = ref child.Manifold.Contact0;
                for (int contactIndex = 0; contactIndex < child.Manifold.Count; ++contactIndex)
                {
                    ref var contact = ref Unsafe.Add(ref childContactsBase, contactIndex);
                    //Note that we only consider 'extreme' contacts that have positive depth to avoid selecting purely speculative contacts as a starting point.
                    //If there are no contacts with positive depth, it's fine to just rely on the 'deepest' speculative contact. 
                    //Feature id stability doesn't matter much if there is no stable contact.
                    float candidateScore;
                    if (contact.Depth >= 0)
                    {
                        //Note that we assume that the contact offsets have already been moved into the parent's space in compound pairs so that we can validly compare extents across manifolds.
                        var extent = Vector3.Dot(contact.Offset, extentAxis) - minimumExtent;
                        candidateScore = contact.Depth + extent * extremityScale;
                    }
                    else
                    {
                        //Speculative contact scores are simply based on depth.
                        candidateScore = contact.Depth;
                    }
                    if (candidateScore > initialBestScore)
                    {
                        initialBestScore = candidateScore;
                        initialBestScoreIndex = remainingContacts.Count;
                    }
                    ref var indices = ref remainingContacts.AllocateUnsafely();
                    indices.ChildIndex = childIndex;
                    indices.ContactIndex = contactIndex;
                    indices.Distinctiveness = float.MaxValue;
                }
            }

            Debug.Assert(remainingContacts.Count > 0, "This function should only be called when there are populated manifolds.");

            UseContact(ref remainingContacts, initialBestScoreIndex, ref Children, manifold);

            var distanceSquaredInterpolationMin = maximumDistanceSquared * (1e-3f * 1e-3f);
            var inverseDistanceSquaredInterpolationSpan = 1f / (maximumDistanceSquared);
            var depthScale = 400f / maximumDistance;

            //We now have a decent starting point. Now, incrementally search for contacts which expand the manifold as much as possible.
            //This is going to be a greedy and nonoptimal search, but being consistent and good enough is more important than true optimality.
            var reducedContacts = &manifold->Contact0;
            while (remainingContacts.Count > 0 && manifold->Count < NonconvexContactManifold.MaximumContactCount)
            {
                var lastContactIndex = manifold->Count - 1;
                var reducedContact = reducedContacts + lastContactIndex;

                float bestScore = -1;
                int bestScoreIndex = -1;
                //Note the order of the loop; we choose the best contact by index, and removals can modify indices after the removal index.
                //Can't reverse order (easily).
                for (int i = 0; i < remainingContacts.Count; ++i)
                {
                    ref var remainingContact = ref remainingContacts[i];
                    ref var childManifold = ref Children[remainingContact.ChildIndex].Manifold;
                    ref var childContact = ref Unsafe.Add(ref childManifold.Contact0, remainingContact.ContactIndex);
                    var distinctiveness = ComputeDistinctiveness(childContact, childManifold.Normal, *reducedContact, distanceSquaredInterpolationMin, inverseDistanceSquaredInterpolationSpan, depthScale);
                    if (distinctiveness <= 0)
                    {
                        //This contact is fully redundant.
                        remainingContacts.FastRemoveAt(i);
                        --i;
                    }
                    else
                    {
                        //The contact wasn't fully redundant. Update its uniqueness score; we choose the lowest uniqueness score across all tested contacts.
                        if (distinctiveness < remainingContact.Distinctiveness)
                            remainingContact.Distinctiveness = distinctiveness;
                        if (remainingContact.Distinctiveness > bestScore)
                        {
                            bestScore = remainingContact.Distinctiveness;
                            bestScoreIndex = i;
                        }
                    }
                }
                //We may have removed all the remaining contacts, so the bestScoreIndex might still be -1. In that case, we're done.
                if (bestScoreIndex >= 0)
                    UseContact(ref remainingContacts, bestScoreIndex, ref Children, manifold);
            }

            if (maximumAllocatedCandidateCount > heapAllocationThreshold)
            {
                remainingContacts.Dispose(pool);
            }
        }


        public unsafe void Flush<TCallbacks>(int pairId, ref CollisionBatcher<TCallbacks> batcher) where TCallbacks : struct, ICollisionCallbacks
        {
            Debug.Assert(ChildCount > 0);
            if (ChildCount == CompletedChildCount)
            {
                //This continuation is ready for processing. Find which contact manifold to report.
                int populatedChildManifolds = 0;
                //We cache an index in case there is only one populated manifold. Order of discovery doesn't matter- this value only gets used when there's one manifold.
                int samplePopulatedChildIndex = 0;
                int totalContactCount = 0;
                for (int i = 0; i < ChildCount; ++i)
                {
                    ref var child = ref Children[i];
                    var childManifoldCount = child.Manifold.Count;
                    if (childManifoldCount > 0)
                    {
                        totalContactCount += childManifoldCount;
                        ++populatedChildManifolds;
                        samplePopulatedChildIndex = i;
                        for (int j = 0; j < child.Manifold.Count; ++j)
                        {
                            //Push all contacts into the space of the parent object.
                            Unsafe.Add(ref child.Manifold.Contact0, j).Offset += child.OffsetA;
                        }
                    }
                }
                var sampleChild = Children.Memory + samplePopulatedChildIndex;

                if (populatedChildManifolds > 1)
                {
                    //There are multiple contributing child manifolds, so just assume that the resulting manifold is going to be nonconvex.
                    Unsafe.SkipInit(out NonconvexContactManifold reducedManifold);
                    //We should assume that the stack memory backing the reduced manifold is uninitialized. We rely on the count, so initialize it manually.
                    reducedManifold.Count = 0;

                    if (totalContactCount <= NonconvexContactManifold.MaximumContactCount)
                    {
                        //No reduction required; we can fit every contact.
                        //TODO: If you have any redundant contact removal, you'd have to do it before running this.
                        for (int i = 0; i < ChildCount; ++i)
                        {
                            ref var child = ref Children[i];
                            for (int j = 0; j < child.Manifold.Count; ++j)
                            {
                                AddContact(ref child, j, &reducedManifold);
                            }
                        }
                    }
                    else
                    {
                        ChooseMostDistinct(&reducedManifold, batcher.Pool);
                    }

                    //The manifold offsetB is the offset from shapeA origin to shapeB origin.
                    reducedManifold.OffsetB = sampleChild->Manifold.OffsetB - sampleChild->OffsetB + sampleChild->OffsetA;
                    batcher.Callbacks.OnPairCompleted(pairId, ref reducedManifold);
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
                    batcher.Callbacks.OnPairCompleted(pairId, ref sampleChild->Manifold);
                }
                batcher.Pool.ReturnUnsafely(Children.Id);
#if DEBUG
                //This makes it a little easier to detect invalid accesses that occur after disposal.
                this = new NonconvexReduction();
#endif
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void OnChildCompleted<TCallbacks>(ref PairContinuation report, ref ConvexContactManifold manifold, ref CollisionBatcher<TCallbacks> batcher)
            where TCallbacks : struct, ICollisionCallbacks
        {
            Children[report.ChildIndex].Manifold = manifold;
            ++CompletedChildCount;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void OnChildCompletedEmpty<TCallbacks>(ref PairContinuation report, ref CollisionBatcher<TCallbacks> batcher) where TCallbacks : struct, ICollisionCallbacks
        {
            Children[report.ChildIndex].Manifold.Count = 0;
            ++CompletedChildCount;
        }

        public bool TryFlush<TCallbacks>(int pairId, ref CollisionBatcher<TCallbacks> batcher) where TCallbacks : struct, ICollisionCallbacks
        {
            if (CompletedChildCount == ChildCount)
            {
                Flush(pairId, ref batcher);
                return true;
            }
            return false;
        }
    }
}
