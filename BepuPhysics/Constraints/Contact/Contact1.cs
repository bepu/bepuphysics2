using BepuPhysics.CollisionDetection;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Quaternion = BepuUtilities.Quaternion;
using static BepuPhysics.GatherScatter;
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
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<Contact1PrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            GetFirst(ref target.OffsetA0.X) = Contact0.OffsetA.X;
            GetFirst(ref target.OffsetA0.Y) = Contact0.OffsetA.Y;
            GetFirst(ref target.OffsetA0.Z) = Contact0.OffsetA.Z;

            GetFirst(ref target.OffsetB.X) = OffsetB.X;
            GetFirst(ref target.OffsetB.Y) = OffsetB.Y;
            GetFirst(ref target.OffsetB.Z) = OffsetB.Z;

            GetFirst(ref target.FrictionCoefficient) = FrictionCoefficient;

            GetFirst(ref target.Normal.X) = Normal.X;
            GetFirst(ref target.Normal.Y) = Normal.Y;
            GetFirst(ref target.Normal.Z) = Normal.Z;

            GetFirst(ref target.SpringSettings.NaturalFrequency) = SpringSettings.NaturalFrequency;
            GetFirst(ref target.SpringSettings.DampingRatio) = SpringSettings.DampingRatio;
            GetFirst(ref target.MaximumRecoveryVelocity) = MaximumRecoveryVelocity;

            GetFirst(ref target.PenetrationDepth0) = Contact0.PenetrationDepth;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact1 description)
        {
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<Contact1PrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            description.Contact0.OffsetA.X = GetFirst(ref source.OffsetA0.X);
            description.Contact0.OffsetA.Y = GetFirst(ref source.OffsetA0.Y);
            description.Contact0.OffsetA.Z = GetFirst(ref source.OffsetA0.Z);

            description.OffsetB.X = GetFirst(ref source.OffsetB.X);
            description.OffsetB.Y = GetFirst(ref source.OffsetB.Y);
            description.OffsetB.Z = GetFirst(ref source.OffsetB.Z);

            description.FrictionCoefficient = GetFirst(ref source.FrictionCoefficient);

            description.Normal.X = GetFirst(ref source.Normal.X);
            description.Normal.Y = GetFirst(ref source.Normal.Y);
            description.Normal.Z = GetFirst(ref source.Normal.Z);

            description.SpringSettings.NaturalFrequency = GetFirst(ref source.SpringSettings.NaturalFrequency);
            description.SpringSettings.DampingRatio = GetFirst(ref source.SpringSettings.DampingRatio);
            description.MaximumRecoveryVelocity = GetFirst(ref source.MaximumRecoveryVelocity);

            description.Contact0.PenetrationDepth = GetFirst(ref source.PenetrationDepth0);

        }

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact1TypeProcessor.BatchTypeId;
        }

        public Type BatchType => typeof(Contact1TypeProcessor);
    }

}
