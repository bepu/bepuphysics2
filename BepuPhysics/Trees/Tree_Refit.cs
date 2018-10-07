using BepuUtilities;
using System.Diagnostics;
using System.Linq;
using System.Numerics;

namespace BepuPhysics.Trees
{
    partial struct Tree
    {
        //TODO: Recursive approach is a bit silly. Our earlier nonrecursive implementations weren't great, but we could do better.
        //This is especially true if we end up changing the memory layout. If we go back to a contiguous array per level, refit becomes trivial.
        //That would only happen if it turns out useful for other parts of the execution, though- optimizing refits at the cost of self-tests would be a terrible idea.
        unsafe void Refit(int nodeIndex, out Vector3 min, out Vector3 max)
        {
            Debug.Assert(leafCount >= 2);
            var node = nodes + nodeIndex;
            ref var a = ref node->A;
            if (node->A.Index >= 0)
            {
                Refit(a.Index, out a.Min, out a.Max);
            }
            ref var b = ref node->B;
            if (b.Index >= 0)
            {
                Refit(b.Index, out b.Min, out b.Max);
            }
            BoundingBox.CreateMerged(a.Min, a.Max, b.Min, b.Max, out min, out max);
        }
        
        public unsafe void Refit()
        {
            //No point in refitting a tree with no internal nodes!
            if (leafCount <= 2)
                return;
            Refit(0, out var rootMin, out var rootMax);
        }       
        


    }
}
