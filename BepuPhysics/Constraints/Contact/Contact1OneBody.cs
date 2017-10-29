using BepuPhysics.CollisionDetection;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
namespace BepuPhysics.Constraints.Contact
{
    public struct Contact1OneBody : IConstraintDescription<Contact1OneBody>
    {
        public ManifoldContactData Contact0;
        public float FrictionCoefficient;
        public Vector3 Normal;
        public SpringSettings SpringSettings;
        public float MaximumRecoveryVelocity;

        public void ApplyDescription(ref TypeBatchData batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var lane = ref GatherScatter.Get(ref Buffer<Contact1OneBodyPrestepData>.Get(ref batch.PrestepData, bundleIndex).OffsetA0.X, innerIndex);
            lane = Contact0.OffsetA.X;
            Unsafe.Add(ref lane, Vector<float>.Count) = Contact0.OffsetA.Y;
            Unsafe.Add(ref lane, 2 * Vector<float>.Count) = Contact0.OffsetA.Z;

            Unsafe.Add(ref lane, 3 * Vector<float>.Count) = FrictionCoefficient;

            Unsafe.Add(ref lane, 4 * Vector<float>.Count) = Normal.X;
            Unsafe.Add(ref lane, 5 * Vector<float>.Count) = Normal.Y;
            Unsafe.Add(ref lane, 6 * Vector<float>.Count) = Normal.Z;

            Unsafe.Add(ref lane, 7 * Vector<float>.Count) = SpringSettings.NaturalFrequency;
            Unsafe.Add(ref lane, 8 * Vector<float>.Count) = SpringSettings.DampingRatio;
            Unsafe.Add(ref lane, 9 * Vector<float>.Count) = MaximumRecoveryVelocity;

            Unsafe.Add(ref lane, 10 * Vector<float>.Count) = Contact0.PenetrationDepth;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void BuildDescription(ref TypeBatchData batch, int bundleIndex, int innerIndex, out Contact1OneBody description)
        {
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var lane = ref GatherScatter.Get(ref Buffer<Contact1OneBodyPrestepData>.Get(ref batch.PrestepData, bundleIndex).OffsetA0.X, innerIndex);
            description.Contact0.OffsetA.X = lane;
            description.Contact0.OffsetA.Y = Unsafe.Add(ref lane, Vector<float>.Count);
            description.Contact0.OffsetA.Z = Unsafe.Add(ref lane, 2 * Vector<float>.Count);

            description.FrictionCoefficient = Unsafe.Add(ref lane, 3 * Vector<float>.Count);

            description.Normal.X = Unsafe.Add(ref lane, 4 * Vector<float>.Count);
            description.Normal.Y = Unsafe.Add(ref lane, 5 * Vector<float>.Count);
            description.Normal.Z = Unsafe.Add(ref lane, 6 * Vector<float>.Count);

            description.SpringSettings.NaturalFrequency = Unsafe.Add(ref lane, 7 * Vector<float>.Count);
            description.SpringSettings.DampingRatio = Unsafe.Add(ref lane, 8 * Vector<float>.Count);
            description.MaximumRecoveryVelocity = Unsafe.Add(ref lane, 9 * Vector<float>.Count);

            description.Contact0.PenetrationDepth = Unsafe.Add(ref lane, 10 * Vector<float>.Count);

        }

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return Contact1OneBodyTypeBatch.BatchTypeId;
            }
        }

        public Type BatchType => typeof(Contact1OneBodyTypeBatch);
    }

}
