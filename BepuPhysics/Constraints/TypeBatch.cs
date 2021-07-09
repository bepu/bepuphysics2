using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.Constraints
{
    public struct BodyIntegrationFlags
    {
        /// <summary>
        /// Set of flags aligned with the constraints in the type batch which, if set, indicate that the constraint is the earliest one associated with the body.
        /// </summary>
        public IndexSet Early;
        /// <summary>
        /// Set of flags aligned with the constraints in the type batch which, if set, indicate that the constraint is the latest one associated with the body.
        /// </summary>
        public IndexSet Late;

    }
    /// <summary>
    /// Stores the raw AOSOA formatted data associated with constraints in a type batch.
    /// </summary>
    public struct TypeBatch
    {
        //Note the constraint data is all stored untyped. It is up to the user to read from these pointers correctly.
        public RawBuffer BodyReferences;
        public RawBuffer PrestepData;
        public RawBuffer AccumulatedImpulses;
        //TODO: Note that we still include a projection buffer here- even though sleeping islands using this struct will never allocate space for it,
        //and even though we may end up not even persisting the allocation between frames. We may later pull this out and store it strictly ephemerally in the solver.
        public RawBuffer Projection;
        public Buffer<ConstraintHandle> IndexToHandle;
        public Buffer<BodyIntegrationFlags> IntegrationFlags;
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
            pool.Return(ref IndexToHandle);
            for (int i = 0; i < IntegrationFlags.Length; ++i)
            {
                ref var flags = ref IntegrationFlags[i];
                flags.Early.Dispose(pool);
                flags.Late.Dispose(pool);
            }
            pool.Return(ref IntegrationFlags);
        }
    }
}
