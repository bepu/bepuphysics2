using BepuPhysics.Constraints;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BepuPhysics
{
    /// <summary>
    /// Collects body handles associated with an active constraint.
    /// </summary>
    public unsafe struct ConstraintBodyHandleCollector : IForEach<int> 
    {
        public Bodies Bodies;
        public int* Handles;
        public int Index;

        public ConstraintBodyHandleCollector(Bodies bodies, int* handles)
        {
            Bodies = bodies;
            Handles = handles;
            Index = 0;
        }

        public void LoopBody(int bodyIndex)
        {
            Handles[Index++] = Bodies.ActiveSet.IndexToHandle[bodyIndex];
        }
    }

}
