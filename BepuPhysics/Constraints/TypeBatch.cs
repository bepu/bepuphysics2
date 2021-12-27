using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.Constraints
{
    /// <summary>
    /// Stores the raw AOSOA formatted data associated with constraints in a type batch.
    /// </summary>
    public struct TypeBatch
    {
        //Note the constraint data is all stored untyped. It is up to the user to read from these pointers correctly.
        public Buffer<byte> BodyReferences;
        public Buffer<byte> PrestepData;
        public Buffer<byte> AccumulatedImpulses;
        public Buffer<ConstraintHandle> IndexToHandle;
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
            pool.Return(ref BodyReferences);
            pool.Return(ref PrestepData);
            pool.Return(ref AccumulatedImpulses);
            pool.Return(ref IndexToHandle);
        }
    }
}
