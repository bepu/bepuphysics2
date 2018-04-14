using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics
{
    /// <summary>
    /// Enumerates the bodies attached to an active constraint and removes the constraint's handle from all of the connected body constraint reference lists.
    /// </summary>
    struct ConstraintGraphRemovalEnumerator : IForEach<int>
    {
        internal Bodies bodies;
        internal int constraintHandle;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void LoopBody(int bodyIndex)
        {
            //Note that this only looks in the active set. Directly removing inactive objects is unsupported- removals and adds activate all involved islands.
            bodies.RemoveConstraintReference(bodyIndex, constraintHandle);
        }
    }

}
