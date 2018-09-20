using BepuPhysics.Constraints;
using BepuUtilities;
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
    public unsafe struct ActiveConstraintBodyHandleCollector : IForEach<int> 
    {
        public Bodies Bodies;
        public int* Handles;
        public int Index;

        public ActiveConstraintBodyHandleCollector(Bodies bodies, int* handles)
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

    public unsafe struct ReferenceCollector : IForEach<int>
    {
        public int* References;
        public int Index;

        public ReferenceCollector(int* references)
        {
            References = references;
            Index = 0;
        }

        public void LoopBody(int reference)
        {
            References[Index++] = reference;
        }
    }


    public unsafe struct FloatCollector : IForEach<float>
    {
        public float* Values;
        public int Index;

        public FloatCollector(float* values)
        {
            Values = values;
            Index = 0;
        }

        public void LoopBody(float value)
        {
            Values[Index++] = value;
        }
    }
}
