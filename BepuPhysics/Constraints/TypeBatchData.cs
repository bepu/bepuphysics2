using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Text;

namespace BepuPhysics.Constraints
{
    public struct ConstraintBatchData
    {
        //Note that we do not store the constraint handle set here. Deactivation does not require storing the handle set after initial creation,
        //because nothing is ever added to islands after their initial creation.
        //Further, because island creation (and removal from the space) is an all or nothing operation, 
        //constraint batch order we choose initially is known to be acceptable to the solver when we re-add the island.
        public QuickList<TypeBatchData, Buffer<TypeBatchData>> TypeBatches;
    }

    /// <summary>
    /// Stores the raw AOSOA formatted data associated with constraints in a type batch.
    /// </summary>
    public struct TypeBatchData
    {
        //Note the constraint data is all stored untyped. It is up to the user to read from these pointers correctly.
        public RawBuffer BodyReferences;
        public RawBuffer PrestepData;
        public RawBuffer AccumulatedImpulses;
        public Buffer<int> IndexToHandle;
        public int ConstraintCount;
        public int TypeId;
    }
}
