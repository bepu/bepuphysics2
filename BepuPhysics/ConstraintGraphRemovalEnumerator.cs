using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics
{
    /// <summary>
    /// Enumerates the bodies attached to an active constraint and removes the constraint's handle from all of the connected body constraint reference lists.
    /// </summary>
    struct ActiveConstraintReferenceRemovalEnumerator : IForEach<int>
    {
        internal Bodies bodies;
        internal int constraintHandle;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void LoopBody(int bodyIndex)
        {
            bodies.ActiveSet.RemoveConstraint(bodyIndex, constraintHandle, bodies.minimumConstraintCapacityPerBody, bodies.pool);
        }
    }

    //Note that inactive constraints are handled differently because inactive constraint body references store *body handles*, not body indices.
    /// <summary>
    /// Enumerates the bodies attached to an inactive constraint and removes the constraint's handle from all of the connected body constraint reference lists.
    /// </summary>
    struct InactiveConstraintReferenceRemovalEnumerator : IForEach<int>
    {
        internal Bodies bodies;
        internal int constraintHandle;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void LoopBody(int bodyHandle)
        {
            ref var location = ref bodies.HandleToLocation[bodyHandle];
            bodies.Sets[location.SetIndex].RemoveConstraint(location.Index, constraintHandle, bodies.minimumConstraintCapacityPerBody, bodies.pool);
        }
    }
}
