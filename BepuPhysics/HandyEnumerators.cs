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
    /// Collects body handles associated with an active constraint as integers.
    /// </summary>
    public unsafe struct ActiveConstraintBodyHandleCollector : IForEach<int>
    {
        public Bodies Bodies;
        public int* Handles;
        public int Count;

        public ActiveConstraintBodyHandleCollector(Bodies bodies, int* handles)
        {
            Bodies = bodies;
            Handles = handles;
            Count = 0;
        }

        public void LoopBody(int encodedBodyIndex)
        {
            //Note that this enumerator is used with prefiltered body indices and with raw body indices. A redundant & isn't much of a concern; lets us share more frequently.
            Handles[Count++] = Bodies.ActiveSet.IndexToHandle[encodedBodyIndex & Bodies.BodyReferenceMask].Value;
        }
    }

    /// <summary>
    /// Collects body handles associated with an active constraint as integers.
    /// </summary>
    public unsafe struct ActiveConstraintDynamicBodyHandleCollector : IForEach<int>
    {
        public Bodies Bodies;
        public int* Handles;
        public int Count;

        public ActiveConstraintDynamicBodyHandleCollector(Bodies bodies, int* handles)
        {
            Bodies = bodies;
            Handles = handles;
            Count = 0;
        }

        public void LoopBody(int encodedBodyIndex)
        {
            if (Bodies.IsEncodedDynamicReference(encodedBodyIndex))
            {
                //Note that this enumerator is used with prefiltered body indices and with raw body indices. A redundant & isn't much of a concern; lets us share more frequently.
                Handles[Count++] = Bodies.ActiveSet.IndexToHandle[encodedBodyIndex & Bodies.BodyReferenceMask].Value;
            }
        }
    }

    /// <summary>
    /// Collects body indices associated with an active constraint. Encoded metadata is stripped.
    /// </summary>
    public unsafe struct ActiveConstraintBodyIndexCollector : IForEach<int>
    {
        public int* Indices;
        public int Count;

        public ActiveConstraintBodyIndexCollector(int* indices)
        {
            Indices = indices;
            Count = 0;
        }

        public void LoopBody(int encodedBodyIndex)
        {
            Indices[Count++] = encodedBodyIndex & Bodies.BodyReferenceMask;
        }
    }

    /// <summary>
    /// Directly reports references as provided by whatever is being enumerated.
    /// For example, when used directly with the TypeProcessor's EnumerateConnectedRawBodyReferences, if the constraint is active, this will report encoded body indices. If the constraint is sleeping, this will report body handles.
    /// If used with an enumerator that does filtering, the filtered results will be reported unmodified.
    /// </summary>
    public unsafe struct PassthroughReferenceCollector : IForEach<int>
    {
        public int* References;
        public int Index;

        public PassthroughReferenceCollector(int* references)
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
