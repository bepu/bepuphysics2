using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics
{
    struct ConstraintGraphRemovalEnumerator : IForEach<int>
    {
        internal ConstraintGraph graph;
        internal int constraintHandle;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void LoopBody(int bodyIndex)
        {
            graph.RemoveConstraint(bodyIndex, constraintHandle);
        }
    }
}
