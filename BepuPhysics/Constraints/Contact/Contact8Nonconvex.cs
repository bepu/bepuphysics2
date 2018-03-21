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
    public struct NonconvexPrestepData
    {
        public Vector3Wide Offset;
        public Vector<float> Depth;
        public Vector3Wide Normal;
    }
    public struct NonconvexTwoBodyManifoldConstraintPropertiesWide
    {
        public Vector3Wide OffsetB;
        public Vector<float> FrictionCoefficient;
        public SpringSettingsWide SpringSettings;
        public Vector<float> MaximumRecoveryVelocity;
    }
    public struct NonconvexOneBodyManifoldConstraintPropertiesWide
    {
        public Vector<float> FrictionCoefficient;
        public SpringSettingsWide SpringSettings;
        public Vector<float> MaximumRecoveryVelocity;
    }
    public interface INonconvexTwoBodyContactPrestepWide<TPrestep> where TPrestep : struct, INonconvexTwoBodyContactPrestepWide<TPrestep>
    {
        ref NonconvexTwoBodyManifoldConstraintPropertiesWide GetCommonProperties(ref TPrestep prestep);
        ref NonconvexPrestepData GetFirstContact(ref TPrestep prestep);
    }
    public interface INonconvexOneBodyContactPrestepWide<TPrestep> where TPrestep : struct, INonconvexOneBodyContactPrestepWide<TPrestep>
    {
        ref NonconvexOneBodyManifoldConstraintPropertiesWide GetCommonProperties(ref TPrestep prestep);
        ref NonconvexPrestepData GetFirstContact(ref TPrestep prestep);
    }

    static class NonconvexConstraintHelpers
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void CopyContactData(int contactCount, ref NonconvexConstraintContactData sourceContacts, ref NonconvexPrestepData targetContacts)
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
              where TPrestep : struct, INonconvexTwoBodyContactPrestepWide<TPrestep>
              where TDescription : struct, INonconvexTwoBodyContactConstraintDescription<TDescription>
        {
            Debug.Assert(batch.TypeId == description.ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<TPrestep>.Get(ref batch.PrestepData, bundleIndex), innerIndex);

            ref var sourceCommon = ref description.GetCommonProperties(ref description);
            ref var targetCommon = ref target.GetCommonProperties(ref target);
            GetFirst(ref targetCommon.OffsetB.X) = sourceCommon.OffsetB.X;
            GetFirst(ref targetCommon.OffsetB.Y) = sourceCommon.OffsetB.Y;
            GetFirst(ref targetCommon.OffsetB.Z) = sourceCommon.OffsetB.Z;

            GetFirst(ref targetCommon.FrictionCoefficient) = sourceCommon.FrictionCoefficient;
            GetFirst(ref targetCommon.SpringSettings.NaturalFrequency) = sourceCommon.SpringSettings.NaturalFrequency;
            GetFirst(ref targetCommon.SpringSettings.DampingRatio) = sourceCommon.SpringSettings.DampingRatio;
            GetFirst(ref targetCommon.MaximumRecoveryVelocity) = sourceCommon.MaximumRecoveryVelocity;

            ref var sourceContacts = ref description.GetFirstContact(ref description);
            ref var targetContacts = ref target.GetFirstContact(ref target);
            CopyContactData(description.ContactCount, ref sourceContacts, ref targetContacts);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyOneBodyDescription<TDescription, TPrestep>(ref TDescription description, ref TypeBatch batch, int bundleIndex, int innerIndex)
              where TPrestep : struct, INonconvexOneBodyContactPrestepWide<TPrestep>
              where TDescription : struct, INonconvexOneBodyContactConstraintDescription<TDescription>
        {
            Debug.Assert(batch.TypeId == description.ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<TPrestep>.Get(ref batch.PrestepData, bundleIndex), innerIndex);

            ref var sourceCommon = ref description.GetCommonProperties(ref description);
            ref var targetCommon = ref target.GetCommonProperties(ref target);
            GetFirst(ref targetCommon.FrictionCoefficient) = sourceCommon.FrictionCoefficient;
            GetFirst(ref targetCommon.SpringSettings.NaturalFrequency) = sourceCommon.SpringSettings.NaturalFrequency;
            GetFirst(ref targetCommon.SpringSettings.DampingRatio) = sourceCommon.SpringSettings.DampingRatio;
            GetFirst(ref targetCommon.MaximumRecoveryVelocity) = sourceCommon.MaximumRecoveryVelocity;

            ref var sourceContacts = ref description.GetFirstContact(ref description);
            ref var targetContacts = ref target.GetFirstContact(ref target);
            CopyContactData(description.ContactCount, ref sourceContacts, ref targetContacts);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void CopyContactData(int contactCount, ref NonconvexPrestepData sourceContacts, ref NonconvexConstraintContactData targetContacts)
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
        public static void BuildTwoBodyDescription<TPrestep, TDescription>(ref TypeBatch batch, int bundleIndex, int innerIndex, out TDescription description)
              where TPrestep : struct, INonconvexTwoBodyContactPrestepWide<TPrestep>
              where TDescription : struct, INonconvexTwoBodyContactConstraintDescription<TDescription>
        {
            Debug.Assert(batch.TypeId == default(TDescription).ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var prestep = ref GetOffsetInstance(ref Buffer<TPrestep>.Get(ref batch.PrestepData, bundleIndex), innerIndex);

            ref var prestepCommon = ref prestep.GetCommonProperties(ref prestep);
            var offsetB = new Vector3(GetFirst(ref prestepCommon.OffsetB.X), GetFirst(ref prestepCommon.OffsetB.Y), GetFirst(ref prestepCommon.OffsetB.Z));
            PairMaterialProperties material;
            material.FrictionCoefficient = GetFirst(ref prestepCommon.FrictionCoefficient);
            material.SpringSettings.NaturalFrequency = GetFirst(ref prestepCommon.SpringSettings.NaturalFrequency);
            material.SpringSettings.DampingRatio = GetFirst(ref prestepCommon.SpringSettings.DampingRatio);
            material.MaximumRecoveryVelocity = GetFirst(ref prestepCommon.MaximumRecoveryVelocity);

            //TODO: Not ideal. We could avoid a default initialization with blittable...
            description = default;
            description.CopyManifoldWideProperties(ref offsetB, ref material);

            ref var descriptionContacts = ref description.GetFirstContact(ref description);
            ref var prestepContacts = ref prestep.GetFirstContact(ref prestep);
            CopyContactData(description.ContactCount, ref prestepContacts, ref descriptionContacts);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void BuildOneBodyDescription<TPrestep, TDescription>(ref TypeBatch batch, int bundleIndex, int innerIndex, out TDescription description)
              where TPrestep : struct, INonconvexOneBodyContactPrestepWide<TPrestep>
              where TDescription : struct, INonconvexOneBodyContactConstraintDescription<TDescription>
        {
            Debug.Assert(batch.TypeId == default(TDescription).ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var prestep = ref GetOffsetInstance(ref Buffer<TPrestep>.Get(ref batch.PrestepData, bundleIndex), innerIndex);

            ref var prestepCommon = ref prestep.GetCommonProperties(ref prestep);
            PairMaterialProperties material;
            material.FrictionCoefficient = GetFirst(ref prestepCommon.FrictionCoefficient);
            material.SpringSettings.NaturalFrequency = GetFirst(ref prestepCommon.SpringSettings.NaturalFrequency);
            material.SpringSettings.DampingRatio = GetFirst(ref prestepCommon.SpringSettings.DampingRatio);
            material.MaximumRecoveryVelocity = GetFirst(ref prestepCommon.MaximumRecoveryVelocity);

            //TODO: Not ideal. We could avoid a default initialization with blittable...
            description = default;
            description.CopyManifoldWideProperties(ref material);

            ref var descriptionContacts = ref description.GetFirstContact(ref description);
            ref var prestepContacts = ref prestep.GetFirstContact(ref prestep);
            CopyContactData(description.ContactCount, ref prestepContacts, ref descriptionContacts);
        }
    }


    public struct Contact8Nonconvex : INonconvexTwoBodyContactConstraintDescription<Contact8Nonconvex>
    {
        public NonconvexTwoBodyManifoldConstraintProperties Common;
        public NonconvexConstraintContactData Contact0;
        public NonconvexConstraintContactData Contact1;
        public NonconvexConstraintContactData Contact2;
        public NonconvexConstraintContactData Contact3;
        public NonconvexConstraintContactData Contact4;
        public NonconvexConstraintContactData Contact5;
        public NonconvexConstraintContactData Contact6;
        public NonconvexConstraintContactData Contact7;

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            NonconvexConstraintHelpers.ApplyTwoBodyDescription<Contact8Nonconvex, Contact8NonconvexPrestepData>(ref this, ref batch, bundleIndex, innerIndex);
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyManifoldWideProperties(ref Vector3 offsetB, ref PairMaterialProperties material)
        {
            Common.OffsetB = offsetB;
            Common.FrictionCoefficient = material.FrictionCoefficient;
            Common.SpringSettings = material.SpringSettings;
            Common.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyManifoldConstraintProperties GetCommonProperties(ref Contact8Nonconvex description)
        {
            return ref description.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexConstraintContactData GetFirstContact(ref Contact8Nonconvex description)
        {
            return ref description.Contact0;
        }

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact8NonconvexTypeProcessor.BatchTypeId;
        }

        public Type BatchType => typeof(Contact8NonconvexTypeProcessor);

        public int ContactCount => throw new NotImplementedException();
    }



    public struct Contact8NonconvexPrestepData : INonconvexTwoBodyContactPrestepWide<Contact8NonconvexPrestepData>
    {
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public NonconvexTwoBodyManifoldConstraintPropertiesWide Common;
        public NonconvexPrestepData Contact0;
        public NonconvexPrestepData Contact1;
        public NonconvexPrestepData Contact2;
        public NonconvexPrestepData Contact3;
        public NonconvexPrestepData Contact4;
        public NonconvexPrestepData Contact5;
        public NonconvexPrestepData Contact6;
        public NonconvexPrestepData Contact7;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyManifoldConstraintPropertiesWide GetCommonProperties(ref Contact8NonconvexPrestepData prestep)
        {
            return ref prestep.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexPrestepData GetFirstContact(ref Contact8NonconvexPrestepData prestep)
        {
            return ref prestep.Contact0;
        }
    }
    public struct NonconvexAccumulatedImpulses
    {
        public Vector2Wide Tangent;
        public Vector<float> Penetration;
    }

    public struct Contact8NonconvexAccumulatedImpulses
    {
        public NonconvexAccumulatedImpulses Contact0;
        public NonconvexAccumulatedImpulses Contact1;
        public NonconvexAccumulatedImpulses Contact2;
        public NonconvexAccumulatedImpulses Contact3;
        public NonconvexAccumulatedImpulses Contact4;
        public NonconvexAccumulatedImpulses Contact5;
        public NonconvexAccumulatedImpulses Contact6;
        public NonconvexAccumulatedImpulses Contact7;
    }

    public struct Contact8NonconvexProjection
    {
        public BodyInertias InertiaA;
        public BodyInertias InertiaB;
        public Vector<float> FrictionCoefficient;
        public TangentFriction.Projection Tangent0;
        public TangentFriction.Projection Tangent1;
        public TangentFriction.Projection Tangent2;
        public TangentFriction.Projection Tangent3;
        public TangentFriction.Projection Tangent4;
        public TangentFriction.Projection Tangent5;
        public TangentFriction.Projection Tangent6;
        public TangentFriction.Projection Tangent7;
        public PenetrationLimit1.Projection Penetration0;
        public PenetrationLimit1.Projection Penetration1;
        public PenetrationLimit1.Projection Penetration2;
        public PenetrationLimit1.Projection Penetration3;
        public PenetrationLimit1.Projection Penetration4;
        public PenetrationLimit1.Projection Penetration5;
        public PenetrationLimit1.Projection Penetration6;
        public PenetrationLimit1.Projection Penetration7;
    }

    public struct Contact8NonconvexFunctions :
        IConstraintFunctions<Contact8NonconvexPrestepData, Contact8NonconvexProjection, Contact8NonconvexAccumulatedImpulses>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Prestep(ref Vector3Wide normal, ref Vector3Wide offsetB, )
        {
            TangentFriction.Prestep(ref )
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count,
            float dt, float inverseDt, ref Contact8NonconvexPrestepData prestep, out Contact8NonconvexProjection projection)
        {
            bodies.GatherInertia(ref bodyReferences, count, out projection.InertiaA, out projection.InertiaB);
            Vector3Wide.Subtract(ref prestep.OffsetA0, ref prestep.OffsetB, out var offsetToManifoldCenterB);
            projection.FrictionCoefficient = prestep.FrictionCoefficient;
            Helpers.BuildOrthnormalBasis(ref prestep.Normal, out var x, out var z);
            TangentFriction.Prestep(ref x, ref z, ref prestep.OffsetA0, ref offsetToManifoldCenterB, ref projection.InertiaA, ref projection.InertiaB, out projection.Tangent);
            PenetrationLimit1.Prestep(ref projection.InertiaA, ref projection.InertiaB, ref prestep.Normal, ref prestep, dt, inverseDt, out projection.Penetration);
            //Single contact manifolds have no true surface area, so approximate twist friction lever arm using the penetration depth. 
            projection.PremultipliedTwistFrictionCoefficient = Vector.Max(Vector<float>.Zero, prestep.FrictionCoefficient * prestep.PenetrationDepth0);
            TwistFriction.Prestep(ref projection.InertiaA, ref projection.InertiaB, ref prestep.Normal, out projection.Twist);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities wsvA, ref BodyVelocities wsvB, ref Contact1Projection projection, ref Contact1AccumulatedImpulses accumulatedImpulses)
        {
            Helpers.BuildOrthnormalBasis(ref projection.Normal, out var x, out var z);
            TangentFriction.WarmStart(ref x, ref z, ref projection.Tangent, ref projection.InertiaA, ref projection.InertiaB, ref accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
            PenetrationLimit1.WarmStart(ref projection.Penetration, ref projection.InertiaA, ref projection.InertiaB,
                ref projection.Normal,
                ref accumulatedImpulses.Penetration0, ref wsvA, ref wsvB);
            TwistFriction.WarmStart(ref projection.Normal, ref projection.InertiaA, ref projection.InertiaB, ref accumulatedImpulses.Twist, ref wsvA, ref wsvB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities wsvA, ref BodyVelocities wsvB, ref Contact1Projection projection, ref Contact1AccumulatedImpulses accumulatedImpulses)
        {
            Helpers.BuildOrthnormalBasis(ref projection.Normal, out var x, out var z);
            var maximumTangentImpulse = projection.PremultipliedFrictionCoefficient * accumulatedImpulses.Penetration0;
            TangentFriction.Solve(ref x, ref z, ref projection.Tangent, ref projection.InertiaA, ref projection.InertiaB, ref maximumTangentImpulse, ref accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
            //Note that we solve the penetration constraints after the friction constraints. 
            //This makes the penetration constraints more authoritative at the cost of the first iteration of the first frame of an impact lacking friction influence.
            //It's a pretty minor effect either way.
            PenetrationLimit1.Solve(ref projection.Penetration, ref projection.InertiaA, ref projection.InertiaB, ref projection.Normal,
                ref accumulatedImpulses.Penetration0, ref wsvA, ref wsvB);
            var maximumTwistImpulse = projection.PremultipliedTwistFrictionCoefficient * accumulatedImpulses.Penetration0;
            TwistFriction.Solve(ref projection.Normal, ref projection.InertiaA, ref projection.InertiaB, ref projection.Twist, ref maximumTwistImpulse, ref accumulatedImpulses.Twist, ref wsvA, ref wsvB);
        }

    }

    /// <summary>
    /// Handles the solve iterations of a bunch of 8-contact nonconvex manifold constraints.
    /// </summary>
    public class Contact8NonconvexTypeProcessor :
        TwoBodyTypeProcessor<Contact8NonconvexPrestepData, Contact8NonconvexProjection, Contact8NonconvexAccumulatedImpulses, Contact8NonconvexFunctions>
    {
        public const int BatchTypeId = 4;
    }
}
