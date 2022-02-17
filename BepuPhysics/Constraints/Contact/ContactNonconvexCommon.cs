using BepuPhysics.CollisionDetection;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using static BepuUtilities.GatherScatter;
using BepuUtilities;

namespace BepuPhysics.Constraints.Contact
{
    public struct NonconvexContactPrestepData
    {
        public Vector3Wide Offset;
        public Vector<float> Depth;
        public Vector3Wide Normal;
    }

    public interface INonconvexContactPrestep<TPrestep> : IContactPrestep<TPrestep> where TPrestep : struct, INonconvexContactPrestep<TPrestep>
    {
        ref NonconvexContactPrestepData GetContact(ref TPrestep prestep, int index);

    }

    public interface ITwoBodyNonconvexContactPrestep<TPrestep> : INonconvexContactPrestep<TPrestep> where TPrestep : struct, ITwoBodyNonconvexContactPrestep<TPrestep>
    {
        ref Vector3Wide GetOffsetB(ref TPrestep prestep);
    }


    public interface INonconvexContactAccumulatedImpulses<TAccumulatedImpulses> : IContactAccumulatedImpulses<TAccumulatedImpulses> where TAccumulatedImpulses : struct, INonconvexContactAccumulatedImpulses<TAccumulatedImpulses>
    {
        ref NonconvexAccumulatedImpulses GetImpulsesForContact(ref TAccumulatedImpulses impulses, int index);
    }


    static class NonconvexConstraintHelpers
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void CopyContactData(int contactCount, ref NonconvexConstraintContactData sourceContacts, ref NonconvexContactPrestepData targetContacts)
        {
            for (int i = 0; i < contactCount; ++i)
            {
                ref var sourceContact = ref Unsafe.Add(ref sourceContacts, i);
                ref var targetContact = ref Unsafe.Add(ref targetContacts, i);
                GetFirst(ref targetContact.Offset.X) = sourceContact.OffsetA.X;
                GetFirst(ref targetContact.Offset.Y) = sourceContact.OffsetA.Y;
                GetFirst(ref targetContact.Offset.Z) = sourceContact.OffsetA.Z;

                GetFirst(ref targetContact.Normal.X) = sourceContact.Normal.X;
                GetFirst(ref targetContact.Normal.Y) = sourceContact.Normal.Y;
                GetFirst(ref targetContact.Normal.Z) = sourceContact.Normal.Z;

                GetFirst(ref targetContact.Depth) = sourceContact.PenetrationDepth;
            }
        }
        //TODO: These could share even more, but... this already handles the 14
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyTwoBodyDescription<TDescription, TPrestep>(ref TDescription description, ref TypeBatch batch, int bundleIndex, int innerIndex)
              where TPrestep : unmanaged, ITwoBodyNonconvexContactPrestep<TPrestep>
              where TDescription : unmanaged, INonconvexTwoBodyContactConstraintDescription<TDescription>
        {
            Debug.Assert(batch.TypeId == description.ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<TPrestep>.Get(ref batch.PrestepData, bundleIndex), innerIndex);

            ref var sourceCommon = ref description.GetCommonProperties(ref description);
            ref var targetOffsetB = ref target.GetOffsetB(ref target);
            GetFirst(ref targetOffsetB.X) = sourceCommon.OffsetB.X;
            GetFirst(ref targetOffsetB.Y) = sourceCommon.OffsetB.Y;
            GetFirst(ref targetOffsetB.Z) = sourceCommon.OffsetB.Z;

            ref var targetMaterial = ref target.GetMaterialProperties(ref target);
            GetFirst(ref targetMaterial.FrictionCoefficient) = sourceCommon.FrictionCoefficient;
            GetFirst(ref targetMaterial.SpringSettings.AngularFrequency) = sourceCommon.SpringSettings.AngularFrequency;
            GetFirst(ref targetMaterial.SpringSettings.TwiceDampingRatio) = sourceCommon.SpringSettings.TwiceDampingRatio;
            GetFirst(ref targetMaterial.MaximumRecoveryVelocity) = sourceCommon.MaximumRecoveryVelocity;

            ref var sourceContacts = ref description.GetFirstContact(ref description);
            ref var targetContacts = ref target.GetContact(ref target, 0);
            CopyContactData(description.ContactCount, ref sourceContacts, ref targetContacts);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyOneBodyDescription<TDescription, TPrestep>(ref TDescription description, ref TypeBatch batch, int bundleIndex, int innerIndex)
              where TPrestep : unmanaged, INonconvexContactPrestep<TPrestep>
              where TDescription : unmanaged, INonconvexOneBodyContactConstraintDescription<TDescription>
        {
            Debug.Assert(batch.TypeId == description.ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<TPrestep>.Get(ref batch.PrestepData, bundleIndex), innerIndex);

            ref var sourceCommon = ref description.GetCommonProperties(ref description);
            ref var materialCommon = ref target.GetMaterialProperties(ref target);
            GetFirst(ref materialCommon.FrictionCoefficient) = sourceCommon.FrictionCoefficient;
            GetFirst(ref materialCommon.SpringSettings.AngularFrequency) = sourceCommon.SpringSettings.AngularFrequency;
            GetFirst(ref materialCommon.SpringSettings.TwiceDampingRatio) = sourceCommon.SpringSettings.TwiceDampingRatio;
            GetFirst(ref materialCommon.MaximumRecoveryVelocity) = sourceCommon.MaximumRecoveryVelocity;

            ref var sourceContacts = ref description.GetFirstContact(ref description);
            ref var targetContacts = ref target.GetContact(ref target, 0);
            CopyContactData(description.ContactCount, ref sourceContacts, ref targetContacts);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void CopyContactData(int contactCount, ref NonconvexContactPrestepData sourceContacts, ref NonconvexConstraintContactData targetContacts)
        {
            for (int i = 0; i < contactCount; ++i)
            {
                ref var source = ref Unsafe.Add(ref sourceContacts, i);
                ref var target = ref Unsafe.Add(ref targetContacts, i);
                target.OffsetA = new Vector3(GetFirst(ref source.Offset.X), GetFirst(ref source.Offset.Y), GetFirst(ref source.Offset.Z));
                target.Normal = new Vector3(GetFirst(ref source.Normal.X), GetFirst(ref source.Normal.Y), GetFirst(ref source.Normal.Z));
                target.PenetrationDepth = GetFirst(ref source.Depth);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void BuildTwoBodyDescription<TDescription, TPrestep>(ref TypeBatch batch, int bundleIndex, int innerIndex, out TDescription description)
              where TPrestep : unmanaged, ITwoBodyNonconvexContactPrestep<TPrestep>
              where TDescription : unmanaged, INonconvexTwoBodyContactConstraintDescription<TDescription>
        {
            Debug.Assert(batch.TypeId == default(TDescription).ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var prestep = ref GetOffsetInstance(ref Buffer<TPrestep>.Get(ref batch.PrestepData, bundleIndex), innerIndex);

            Vector3Wide.ReadFirst(prestep.GetOffsetB(ref prestep), out var offsetB);
            ref var materialSource = ref prestep.GetMaterialProperties(ref prestep);
            PairMaterialProperties material;
            material.FrictionCoefficient = GetFirst(ref materialSource.FrictionCoefficient);
            material.SpringSettings.AngularFrequency = GetFirst(ref materialSource.SpringSettings.AngularFrequency);
            material.SpringSettings.TwiceDampingRatio = GetFirst(ref materialSource.SpringSettings.TwiceDampingRatio);
            material.MaximumRecoveryVelocity = GetFirst(ref materialSource.MaximumRecoveryVelocity);

            //TODO: Replace with Unsafe.SkipInit?
            description = default;
            description.CopyManifoldWideProperties(ref offsetB, ref material);

            ref var descriptionContacts = ref description.GetFirstContact(ref description);
            ref var prestepContacts = ref prestep.GetContact(ref prestep, 0);
            CopyContactData(description.ContactCount, ref prestepContacts, ref descriptionContacts);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void BuildOneBodyDescription<TDescription, TPrestep>(ref TypeBatch batch, int bundleIndex, int innerIndex, out TDescription description)
              where TPrestep : unmanaged, INonconvexContactPrestep<TPrestep>
              where TDescription : unmanaged, INonconvexOneBodyContactConstraintDescription<TDescription>
        {
            Debug.Assert(batch.TypeId == default(TDescription).ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var prestep = ref GetOffsetInstance(ref Buffer<TPrestep>.Get(ref batch.PrestepData, bundleIndex), innerIndex);

            ref var materialSource = ref prestep.GetMaterialProperties(ref prestep);
            PairMaterialProperties material;
            material.FrictionCoefficient = GetFirst(ref materialSource.FrictionCoefficient);
            material.SpringSettings.AngularFrequency = GetFirst(ref materialSource.SpringSettings.AngularFrequency);
            material.SpringSettings.TwiceDampingRatio = GetFirst(ref materialSource.SpringSettings.TwiceDampingRatio);
            material.MaximumRecoveryVelocity = GetFirst(ref materialSource.MaximumRecoveryVelocity);

            //TODO: Replace with Unsafe.SkipInit?
            description = default;
            description.CopyManifoldWideProperties(ref material);

            ref var descriptionContacts = ref description.GetFirstContact(ref description);
            ref var prestepContacts = ref prestep.GetContact(ref prestep, 0);
            CopyContactData(description.ContactCount, ref prestepContacts, ref descriptionContacts);
        }
    }

    public struct NonconvexAccumulatedImpulses
    {
        public Vector2Wide Tangent;
        public Vector<float> Penetration;
    }

    public struct ContactNonconvexOneBodyFunctions<TPrestep, TAccumulatedImpulses> :
        IOneBodyConstraintFunctions<TPrestep, TAccumulatedImpulses>
        where TPrestep : struct, INonconvexContactPrestep<TPrestep>
        where TAccumulatedImpulses : struct
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateContactData(in Vector<float> dt, in BodyVelocityWide velocity, ref TPrestep prestep)
        {
            ref var prestepContactStart = ref prestep.GetContact(ref prestep, 0);
            for (int i = 0; i < prestep.ContactCount; ++i)
            {
                ref var prestepContact = ref Unsafe.Add(ref prestepContactStart, i);
                PenetrationLimitOneBody.UpdatePenetrationDepth(dt, prestepContact.Offset, prestepContact.Normal, velocity, ref prestepContact.Depth);
            }
        }

        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, ref TPrestep prestep, ref TAccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA)
        {
            ref var prestepMaterial = ref prestep.GetMaterialProperties(ref prestep);
            ref var prestepContactStart = ref prestep.GetContact(ref prestep, 0);
            ref var accumulatedImpulsesStart = ref Unsafe.As<TAccumulatedImpulses, NonconvexAccumulatedImpulses>(ref accumulatedImpulses);
            for (int i = 0; i < prestep.ContactCount; ++i)
            {
                ref var prestepContact = ref Unsafe.Add(ref prestepContactStart, i);
                Helpers.BuildOrthonormalBasis(prestepContact.Normal, out var x, out var z);
                ref var contactImpulse = ref Unsafe.Add(ref accumulatedImpulsesStart, i);
                TangentFrictionOneBody.WarmStart(x, z, prestepContact.Offset, inertiaA, contactImpulse.Tangent, ref wsvA);
                PenetrationLimitOneBody.WarmStart(inertiaA, prestepContact.Normal, prestepContact.Offset, contactImpulse.Penetration, ref wsvA);
            }
        }

        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, float dt, float inverseDt, ref TPrestep prestep, ref TAccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA)
        {
            //Note that, unlike convex manifolds, we simply solve every contact in sequence rather than tangent->penetration.
            //This is not for any principled reason- only simplicity. May want to reconsider later, but remember the significant change in access pattern.
            ref var prestepMaterial = ref prestep.GetMaterialProperties(ref prestep);
            ref var accumulatedImpulsesStart = ref Unsafe.As<TAccumulatedImpulses, NonconvexAccumulatedImpulses>(ref accumulatedImpulses);
            ref var prestepContactStart = ref prestep.GetContact(ref prestep, 0);
            SpringSettingsWide.ComputeSpringiness(prestepMaterial.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            var inverseDtWide = new Vector<float>(inverseDt);
            for (int i = 0; i < prestep.ContactCount; ++i)
            {
                ref var contact = ref Unsafe.Add(ref prestepContactStart, i);
                ref var contactImpulse = ref Unsafe.Add(ref accumulatedImpulsesStart, i);
                PenetrationLimitOneBody.Solve(inertiaA, contact.Normal, contact.Offset, contact.Depth,
                    positionErrorToVelocity, effectiveMassCFMScale, prestepMaterial.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref contactImpulse.Penetration, ref wsvA);
                Helpers.BuildOrthonormalBasis(contact.Normal, out var x, out var z);
                var maximumTangentImpulse = prestepMaterial.FrictionCoefficient * contactImpulse.Penetration;
                TangentFrictionOneBody.Solve(x, z, contact.Offset, inertiaA, maximumTangentImpulse, ref contactImpulse.Tangent, ref wsvA);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void UpdateForNewPose(
            in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in BodyVelocityWide wsvA,
            in Vector<float> dt, in TAccumulatedImpulses accumulatedImpulses, ref TPrestep prestep)
        {
            throw new System.NotImplementedException();
        }

        public bool RequiresIncrementalSubstepUpdates => true;
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, ref TPrestep prestep)
        {
            ref var prestepContactStart = ref prestep.GetContact(ref prestep, 0);
            for (int i = 0; i < prestep.ContactCount; ++i)
            {
                ref var prestepContact = ref Unsafe.Add(ref prestepContactStart, i);
                PenetrationLimitOneBody.UpdatePenetrationDepth(dt, prestepContact.Offset, prestepContact.Normal, wsvA, ref prestepContact.Depth);
            }
        }
    }

    public struct ContactNonconvexTwoBodyFunctions<TPrestep, TAccumulatedImpulses> :
        ITwoBodyConstraintFunctions<TPrestep, TAccumulatedImpulses>
        where TPrestep : struct, ITwoBodyNonconvexContactPrestep<TPrestep>
        where TAccumulatedImpulses : struct
    {
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref TPrestep prestep, ref TAccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ref var prestepMaterial = ref prestep.GetMaterialProperties(ref prestep);
            ref var prestepOffsetB = ref prestep.GetOffsetB(ref prestep);
            ref var prestepContactStart = ref prestep.GetContact(ref prestep, 0);
            ref var accumulatedImpulsesStart = ref Unsafe.As<TAccumulatedImpulses, NonconvexAccumulatedImpulses>(ref accumulatedImpulses);
            for (int i = 0; i < prestep.ContactCount; ++i)
            {
                ref var prestepContact = ref Unsafe.Add(ref prestepContactStart, i);
                Helpers.BuildOrthonormalBasis(prestepContact.Normal, out var x, out var z);
                Vector3Wide.Subtract(prestepContact.Offset, prestepOffsetB, out var contactOffsetB);
                ref var contactImpulse = ref Unsafe.Add(ref accumulatedImpulsesStart, i);
                TangentFriction.WarmStart(x, z, prestepContact.Offset, contactOffsetB, inertiaA, inertiaB, contactImpulse.Tangent, ref wsvA, ref wsvB);
                PenetrationLimit.WarmStart(inertiaA, inertiaB, prestepContact.Normal, prestepContact.Offset, contactOffsetB, contactImpulse.Penetration, ref wsvA, ref wsvB);
            }
        }

        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref TPrestep prestep, ref TAccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            //Note that, unlike convex manifolds, we simply solve every contact in sequence rather than tangent->penetration.
            //This is not for any principled reason- only simplicity. May want to reconsider later, but remember the significant change in access pattern.
            ref var prestepOffsetB = ref prestep.GetOffsetB(ref prestep);
            ref var prestepMaterial = ref prestep.GetMaterialProperties(ref prestep);
            ref var accumulatedImpulsesStart = ref Unsafe.As<TAccumulatedImpulses, NonconvexAccumulatedImpulses>(ref accumulatedImpulses);
            ref var prestepContactStart = ref prestep.GetContact(ref prestep, 0);
            SpringSettingsWide.ComputeSpringiness(prestepMaterial.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            var inverseDtWide = new Vector<float>(inverseDt);
            for (int i = 0; i < prestep.ContactCount; ++i)
            {
                ref var contact = ref Unsafe.Add(ref prestepContactStart, i);
                ref var contactImpulse = ref Unsafe.Add(ref accumulatedImpulsesStart, i);
                Vector3Wide.Subtract(contact.Offset, prestepOffsetB, out var contactOffsetB);
                PenetrationLimit.Solve(inertiaA, inertiaB, contact.Normal, contact.Offset, contactOffsetB, contact.Depth,
                    positionErrorToVelocity, effectiveMassCFMScale, prestepMaterial.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref contactImpulse.Penetration, ref wsvA, ref wsvB);
                Helpers.BuildOrthonormalBasis(contact.Normal, out var x, out var z);
                var maximumTangentImpulse = prestepMaterial.FrictionCoefficient * contactImpulse.Penetration;
                TangentFriction.Solve(x, z, contact.Offset, contactOffsetB, inertiaA, inertiaB, maximumTangentImpulse, ref contactImpulse.Tangent, ref wsvA, ref wsvB);
            }
        }


        public bool RequiresIncrementalSubstepUpdates => true;
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, ref TPrestep prestep)
        {
            ref var prestepOffsetB = ref prestep.GetOffsetB(ref prestep);
            ref var prestepContactStart = ref prestep.GetContact(ref prestep, 0);
            for (int i = 0; i < prestep.ContactCount; ++i)
            {
                ref var prestepContact = ref Unsafe.Add(ref prestepContactStart, i);
                PenetrationLimit.UpdatePenetrationDepth(dt, prestepContact.Offset, prestepOffsetB, prestepContact.Normal, wsvA, wsvB, ref prestepContact.Depth);
            }
        }
    }
}
