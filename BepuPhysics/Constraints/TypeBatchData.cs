using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
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
        //TODO: Note that we still include a projection buffer here- even though deactivated islands using this struct will never allocate space for it,
        //and even though we may end up not even persisting the allocation between frames. We may later pull this out and store it strictly ephemerally in the solver.
        public RawBuffer Projection;
        public Buffer<int> IndexToHandle;
        public int ConstraintCount;
        public int TypeId;

        public int BundleCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return BundleIndexing.GetBundleCount(ConstraintCount);
            }
        }

        public void Dispose(BufferPool pool)
        {
            pool.Return(ref Projection);
            pool.Return(ref BodyReferences);
            pool.Return(ref PrestepData);
            pool.Return(ref AccumulatedImpulses);
            pool.SpecializeFor<int>().Return(ref IndexToHandle);
        }
    }
}
