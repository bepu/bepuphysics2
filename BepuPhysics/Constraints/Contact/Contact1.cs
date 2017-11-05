using BepuPhysics.CollisionDetection;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
namespace BepuPhysics.Constraints.Contact
{
    public struct Contact1 : IConstraintDescription<Contact1>
    {
        public ManifoldContactData Contact0;
        public Vector3 OffsetB;
        public float FrictionCoefficient;
        public Vector3 Normal;
        public SpringSettings SpringSettings;
        public float MaximumRecoveryVelocity;

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            //We assume a contiguous block of Vector<T> types, where T is a 32 bit type. It is unlikely that future runtime changes will introduce
            //packing on the fields, since each of them are a Vector<T> in size- which will tend to be 16, 32, or in the future, 64 bytes.
            //That said, relying on non-explicit memory layouts is still a risk.

            //TODO: Note that this is a maintenance nightmare. There's always going to be a bit of maintenance nightmare, but this is pretty much maximizing it.
            //We can only justify this by saying that contact manifolds are highly performance sensitive, but other constraints that don't undergo constant modification
            //should probably use a somewhat less gross option. For example, while it's still a nightmare, aligning the description's memory layout such that it matches a lane
            //(except the lane has a longer stride between elements) would allow a *relatively* clean and reusable helper that simply loops across the lane.
            //At the end of the day, the important thing is that this mapping is kept localized so that not every system needs to be aware of it.

            //Note that we use an unsafe cast.
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var lane = ref GatherScatter.Get(ref Buffer<Contact1PrestepData>.Get(ref batch.PrestepData, bundleIndex).OffsetA0.X, innerIndex);
            lane = Contact0.OffsetA.X;
            Unsafe.Add(ref lane, Vector<float>.Count) = Contact0.OffsetA.Y;
            Unsafe.Add(ref lane, 2 * Vector<float>.Count) = Contact0.OffsetA.Z;

            Unsafe.Add(ref lane, 3 * Vector<float>.Count) = OffsetB.X;
            Unsafe.Add(ref lane, 4 * Vector<float>.Count) = OffsetB.Y;
            Unsafe.Add(ref lane, 5 * Vector<float>.Count) = OffsetB.Z;

            Unsafe.Add(ref lane, 6 * Vector<float>.Count) = FrictionCoefficient;

            Unsafe.Add(ref lane, 7 * Vector<float>.Count) = Normal.X;
            Unsafe.Add(ref lane, 8 * Vector<float>.Count) = Normal.Y;
            Unsafe.Add(ref lane, 9 * Vector<float>.Count) = Normal.Z;

            Unsafe.Add(ref lane, 10 * Vector<float>.Count) = SpringSettings.NaturalFrequency;
            Unsafe.Add(ref lane, 11 * Vector<float>.Count) = SpringSettings.DampingRatio;
            Unsafe.Add(ref lane, 12 * Vector<float>.Count) = MaximumRecoveryVelocity;

            Unsafe.Add(ref lane, 13 * Vector<float>.Count) = Contact0.PenetrationDepth;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact1 description)
        {
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var lane = ref GatherScatter.Get(ref Buffer<Contact1PrestepData>.Get(ref batch.PrestepData, bundleIndex).OffsetA0.X, innerIndex);
            description.Contact0.OffsetA.X = lane;
            description.Contact0.OffsetA.Y = Unsafe.Add(ref lane, Vector<float>.Count);
            description.Contact0.OffsetA.Z = Unsafe.Add(ref lane, 2 * Vector<float>.Count);

            description.OffsetB.X = Unsafe.Add(ref lane, 3 * Vector<float>.Count);
            description.OffsetB.Y = Unsafe.Add(ref lane, 4 * Vector<float>.Count);
            description.OffsetB.Z = Unsafe.Add(ref lane, 5 * Vector<float>.Count);

            description.FrictionCoefficient = Unsafe.Add(ref lane, 6 * Vector<float>.Count);

            description.Normal.X = Unsafe.Add(ref lane, 7 * Vector<float>.Count);
            description.Normal.Y = Unsafe.Add(ref lane, 8 * Vector<float>.Count);
            description.Normal.Z = Unsafe.Add(ref lane, 9 * Vector<float>.Count);

            description.SpringSettings.NaturalFrequency = Unsafe.Add(ref lane, 10 * Vector<float>.Count);
            description.SpringSettings.DampingRatio = Unsafe.Add(ref lane, 11 * Vector<float>.Count);
            description.MaximumRecoveryVelocity = Unsafe.Add(ref lane, 12 * Vector<float>.Count);

            description.Contact0.PenetrationDepth = Unsafe.Add(ref lane, 13 * Vector<float>.Count);

        }

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return Contact1TypeBatch.BatchTypeId;
            }
        }

        public Type BatchType => typeof(Contact1TypeBatch);
    }

}
